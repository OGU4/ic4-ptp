#!/usr/bin/env python3
"""PTP-synchronised multi-camera video capture.

This script arms PTP-synchronised cameras and records a fixed-length video clip
from each device. Frames are continuously queued, but writing to disk starts only
once each camera's device timestamp (or host monotonic timer) reaches the
scheduled start time, ensuring aligned clips without relying on trigger-per-frame
actions.

Requirements:
  * imagingcontrol4 >= 4.0 (QueueSink, Action Scheduler support)
  * Cameras configured for TriggerSource="Action" to respond to action commands

Note: The script assumes the cameras deliver BGR8 frames. Adjust PIXFORMAT and
conversion logic if a different pixel format is required.
"""

import datetime
import os
import time
import gc
from typing import Optional, Sequence

import imagingcontrol4 as ic4

SAVE_DIR       = "/data/ic4-ptp/videos"
DELAY_SEC      = 10.0                 # seconds from now to start recording
RECORD_SEC     = 10.0                 # duration of each video clip
PIXFORMAT      = ic4.PixelFormat.BGR8
DEFAULT_FPS    = 30.0

VIDEO_WRITER_CANDIDATES = [("mp4", ic4.VideoWriterType.MP4_H265)]
if hasattr(ic4.VideoWriterType, "MP4_H264"):
    VIDEO_WRITER_CANDIDATES.append(("mp4", ic4.VideoWriterType.MP4_H264))
if hasattr(ic4.VideoWriterType, "AVI_UNCOMPRESSED"):
    VIDEO_WRITER_CANDIDATES.append(("avi", ic4.VideoWriterType.AVI_UNCOMPRESSED))


# ── Helpers ───────────────────────────────────────────────────────────────────
def enum_set_contains(prop_enum: ic4.PropEnumeration, keyword: str) -> bool:
    for e in prop_enum.entries:
        if keyword.lower() in e.name.lower():
            prop_enum.value = e.name
            return True
    return False


def enum_set_on(prop_enum: ic4.PropEnumeration) -> bool:
    for key in ("On", "Enable", "Enabled", "True", "1"):
        for e in prop_enum.entries:
            if e.name == key:
                prop_enum.value = e.name
                return True
    return False


def get_prop(pm: ic4.PropertyMap, pid_name: str):
    if hasattr(ic4.PropId, pid_name):
        try:
            return pm[getattr(ic4.PropId, pid_name)]
        except Exception:
            return None
    return None


def find_first_prop(pm: ic4.PropertyMap, pid_names: Sequence[str]):
    for name in pid_names:
        p = get_prop(pm, name)
        if p is not None:
            return p
    return None


def try_call(obj, names: Sequence[str]) -> bool:
    for name in names:
        fn = getattr(obj, name, None)
        if callable(fn):
            try:
                fn()
                return True
            except Exception:
                continue
    return False


def get_device_time_and_freq(pm: ic4.PropertyMap):
    latch_cmd = find_first_prop(pm, ["TIMESTAMP_LATCH", "TIMESTAMPCONTROL_LATCH"])
    val_int   = find_first_prop(pm, ["TIMESTAMP_LATCH_VALUE", "TIMESTAMPCONTROL_LATCHVALUE",
                                     "DEVICE_TIMESTAMP_VALUE", "GevTimestampValue"])
    freq_int  = find_first_prop(pm, ["TIMESTAMP_TICK_FREQUENCY", "DEVICE_TIMESTAMP_TICK_FREQUENCY",
                                     "GevTimestampTickFrequency"])

    if isinstance(latch_cmd, ic4.PropCommand) and isinstance(val_int, ic4.PropInteger):
        latch_cmd.execute()
        now_ticks = int(val_int.value)
        tick_hz = int(freq_int.value) if isinstance(freq_int, ic4.PropInteger) else 1_000_000_000
        return now_ticks, tick_hz

    now_int = find_first_prop(pm, ["DEVICE_TIMESTAMP", "PTP_TIME", "CURRENT_TIMESTAMP"])
    if isinstance(now_int, ic4.PropInteger):
        return int(now_int.value), 1_000_000_000

    raise RuntimeError("Device timestamp/frequency properties not found. Consider pm.dump().")


def get_frame_rate(pm: ic4.PropertyMap) -> float:
    candidates = [
        "ACQUISITION_FRAME_RATE",
        "ACQUISITIONFRAMERATE",
        "ACQUISITION_FRAME_RATE_ABSOLUTE",
        "RESULTING_FRAME_RATE",
    ]
    prop = find_first_prop(pm, candidates)
    if isinstance(prop, ic4.PropFloat):
        try:
            return float(prop.value)
        except Exception:
            pass
    return DEFAULT_FPS


# ── Listener ──────────────────────────────────────────────────────────────────
class TimedVideoRecorder(ic4.QueueSinkListener):
    def __init__(self, serial: str, duration_sec: float, fps: float,
                 image_type: Optional[ic4.ImageType] = None):
        self.serial = serial
        self.duration_sec = duration_sec
        self.fps = fps
        self.image_type = image_type

        self.writer: Optional[ic4.VideoWriter] = None
        self.start_monotonic: Optional[float] = None
        self.stop_monotonic: Optional[float] = None
        self.target_monotonic: Optional[float] = None
        self.target_stop_monotonic: Optional[float] = None
        self.start_device_ticks: Optional[int] = None
        self.stop_device_ticks: Optional[int] = None
        self.tick_hz: Optional[int] = None
        self.finished = False
        self.saved_path: Optional[str] = None
        self.recording_started = False
        self.begin_error: Optional[str] = None
        self._init_attempts = 0

    def sink_connected(self, sink, image_type, min_buffers_required):
        return True

    def set_image_type(self, image_type: ic4.ImageType):
        self.image_type = image_type

    def arm(self, start_ticks: int, stop_ticks: int, tick_hz: int,
            start_monotonic: float, stop_monotonic: float):
        self.start_device_ticks = start_ticks
        self.stop_device_ticks = stop_ticks
        self.tick_hz = tick_hz
        self.target_monotonic = start_monotonic
        self.target_stop_monotonic = stop_monotonic

    def _begin_recording(self):
        if self.recording_started:
            return
        if self.image_type is None:
            raise RuntimeError("Image type not set before recording start.")
        os.makedirs(SAVE_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

        last_exc: Optional[Exception] = None
        tried: set = set()
        self._init_attempts += 1
        for ext, writer_type in VIDEO_WRITER_CANDIDATES:
            if writer_type in tried:
                continue
            tried.add(writer_type)
            try:
                writer = ic4.VideoWriter(writer_type)
                path = os.path.join(SAVE_DIR, f"{self.serial}_{ts}.{ext}")
                writer.begin_file(path, self.image_type, self.fps)
                self.writer = writer
                self.saved_path = path
                self.start_monotonic = time.monotonic()
                self.stop_monotonic = self.start_monotonic + self.duration_sec
                self.recording_started = True
                self.begin_error = None
                print(f"[{self.serial}] recording -> {path} (fps={self.fps:.2f})")
                break
            except Exception as exc:
                last_exc = exc
                print(f"[{self.serial}] VideoWriter {writer_type.name} failed: {exc}")
                continue

        if not self.recording_started:
            error_msg = f"{self.serial}: failed to initialise VideoWriter"
            if last_exc:
                error_msg += f" ({last_exc})"
            self.begin_error = error_msg
            if self._init_attempts >= 3:
                self.finished = True
            if self._init_attempts == 1:
                print(error_msg)

    def _finish_recording(self):
        if self.writer is not None:
            try:
                self.writer.finish_file()
            except Exception:
                pass
            print(f"[{self.serial}] recording stopped")
            self.writer = None
        self.finished = True
        self.recording_started = False

    def frames_queued(self, sink: ic4.QueueSink):
        while True:
            try:
                buf = sink.pop_output_buffer()
            except Exception as exc:
                code = exc.args[0] if getattr(exc, "args", None) else None
                if code == getattr(ic4.ErrorCode, "NoData", None):
                    break
                raise

            if buf is None:
                break
            try:
                if self.finished:
                    continue
                device_ts = getattr(buf, "timestamp_ns", None)
                now = time.monotonic()

                should_start = False
                if self.start_device_ticks is not None and isinstance(device_ts, (int, float)) and self.tick_hz:
                    should_start = device_ts >= self.start_device_ticks
                elif self.target_monotonic is not None:
                    should_start = now >= self.target_monotonic

                if should_start and not self.recording_started:
                    self._begin_recording()

                if not self.recording_started or self.writer is None:
                    continue

                self.writer.add_frame(buf)

                should_stop = False
                if self.stop_device_ticks is not None and isinstance(device_ts, (int, float)) and self.tick_hz:
                    should_stop = device_ts >= self.stop_device_ticks
                elif self.target_stop_monotonic is not None:
                    should_stop = now >= self.target_stop_monotonic

                if should_stop:
                    self._finish_recording()
            finally:
                del buf

        if self.finished and self.writer is not None:
            self._finish_recording()


# ── Main work ─────────────────────────────────────────────────────────────────
def _configure_for_action_trigger(g: ic4.Grabber):
    pm = g.device_property_map

    trig_mode = get_prop(pm, "TRIGGER_MODE")
    if isinstance(trig_mode, ic4.PropEnumeration):
        for entry in trig_mode.entries:
            if entry.name in ("Off", "Continuous", "Freerun"):
                trig_mode.value = entry.name
                break

    trig_src = get_prop(pm, "TRIGGER_SOURCE")
    if isinstance(trig_src, ic4.PropEnumeration):
        for entry in trig_src.entries:
            if entry.name in ("Software", "Line0", "Freerun", "Off"):
                trig_src.value = entry.name
                break


def _work():
    devs = ic4.DeviceEnum.devices()
    if len(devs) < 2:
        raise RuntimeError("At least two cameras are required.")

    grabbers = [ic4.Grabber(info) for info in devs]
    serials  = [g.device_info.serial for g in grabbers]

    sinks = []
    recorders: list[TimedVideoRecorder] = []

    for g in grabbers:
        _configure_for_action_trigger(g)

    try:
        for g, srl in zip(grabbers, serials):
            fps = get_frame_rate(g.device_property_map)
            recorder = TimedVideoRecorder(srl, RECORD_SEC, fps)
            sink = ic4.QueueSink(recorder, accepted_pixel_formats=[PIXFORMAT])
            g.stream_setup(sink)
            try_call(g, ("stream_start", "start_stream", "stream_enable", "start_acquisition"))
            try:
                image_type = sink.output_image_type
                recorder.set_image_type(image_type)
            except Exception:
                pass
            sinks.append((g, sink, recorder))
            recorders.append(recorder)

        host_start = time.monotonic() + DELAY_SEC
        host_stop = host_start + RECORD_SEC
        wall_start = datetime.datetime.now() + datetime.timedelta(seconds=DELAY_SEC)

        for g, recorder in zip(grabbers, recorders):
            pm = g.device_property_map
            now_ticks, tick_hz = get_device_time_and_freq(pm)
            start_ticks = now_ticks + int(DELAY_SEC * tick_hz)
            stop_ticks = start_ticks + int(RECORD_SEC * tick_hz)
            recorder.arm(start_ticks, stop_ticks, tick_hz, host_start, host_stop)
            print(f"[{g.device_info.serial}] armed start_ticks={start_ticks} stop_ticks={stop_ticks} hz={tick_hz}")

        print(f"Recording window: ~{wall_start.strftime('%H:%M:%S')} for {RECORD_SEC:.1f}s")

        timeout = time.time() + DELAY_SEC + RECORD_SEC + 5.0
        while time.time() < timeout:
            if all(rec.finished for rec in recorders):
                break
            time.sleep(0.05)

        incomplete = [rec.serial for rec in recorders if not rec.finished]
        begin_failures = [rec.begin_error for rec in recorders if rec.begin_error]

        if begin_failures:
            raise RuntimeError("/".join(begin_failures))
        if incomplete:
            raise RuntimeError(f"Recording timeout for: {incomplete}")

        print("All cameras recorded successfully.")

    finally:
        for g, sink, recorder in sinks:
            try:
                try_call(g, ("stream_stop", "stop_stream", "stream_disable", "stop_acquisition"))
            except Exception:
                pass
            if recorder.recording_started and not recorder.finished:
                recorder._finish_recording()
            g.device_close()
            del sink
            del recorder
        grabbers.clear()
        gc.collect()


def main():
    with ic4.Library.init_context(api_log_level=ic4.LogLevel.INFO,
                                  log_targets=ic4.LogTarget.STDERR):
        _work()
        gc.collect()


if __name__ == "__main__":
    main()

