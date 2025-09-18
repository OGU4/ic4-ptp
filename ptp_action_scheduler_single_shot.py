#!/usr/bin/env python3
import os, time, datetime, gc
import imagingcontrol4 as ic4

SAVE_DIR   = "~/data/ic4-ptp/captures"
DELAY_SEC  = 10.0                # seconds from now to fire the action
PIXFORMAT  = ic4.PixelFormat.BGR8

# ── Helpers ───────────────────────────────────────────────────────────────────
def enum_set_contains(prop_enum: ic4.PropEnumeration, keyword: str) -> bool:
    """Select the first enum entry whose name contains `keyword` (case-insensitive)."""
    for e in prop_enum.entries:
        if keyword.lower() in e.name.lower():
            prop_enum.value = e.name
            return True
    return False

def enum_set_on(prop_enum: ic4.PropEnumeration) -> bool:
    """Try to set a typical ON/ENABLE entry for trigger-like properties."""
    for key in ("On", "Enable", "Enabled", "True", "1"):
        for e in prop_enum.entries:
            if e.name == key:
                prop_enum.value = e.name
                return True
    return False

def get_prop(pm: ic4.PropertyMap, pid_name: str):
    """Fetch a property by PropId name; return None if missing."""
    if hasattr(ic4.PropId, pid_name):
        try:
            return pm[getattr(ic4.PropId, pid_name)]
        except Exception:
            return None
    return None

def find_first_prop(pm: ic4.PropertyMap, pid_names):
    """Return the first existing property among given PropId names; None if none exist."""
    for name in pid_names:
        p = get_prop(pm, name)
        if p is not None:
            return p
    return None

def get_device_time_and_freq(pm: ic4.PropertyMap):
    """
    Read current device time (ticks) and tick frequency (Hz).
    Standard path:
      - TIMESTAMP_LATCH (command) -> TIMESTAMP_LATCH_VALUE (integer)
      - TIMESTAMP_TICK_FREQUENCY (integer)
    Fallback:
      - DEVICE_TIMESTAMP / PTP_TIME / CURRENT_TIMESTAMP (integer), assume 1e9 Hz.
    """
    latch_cmd = find_first_prop(pm, ["TIMESTAMP_LATCH","TIMESTAMPCONTROL_LATCH"])
    val_int   = find_first_prop(pm, ["TIMESTAMP_LATCH_VALUE","TIMESTAMPCONTROL_LATCHVALUE",
                                     "DEVICE_TIMESTAMP_VALUE","GevTimestampValue"])
    freq_int  = find_first_prop(pm, ["TIMESTAMP_TICK_FREQUENCY","DEVICE_TIMESTAMP_TICK_FREQUENCY",
                                     "GevTimestampTickFrequency"])

    if isinstance(latch_cmd, ic4.PropCommand) and isinstance(val_int, ic4.PropInteger):
        latch_cmd.execute()
        now_ticks = int(val_int.value)
        tick_hz = int(freq_int.value) if isinstance(freq_int, ic4.PropInteger) else 1_000_000_000
        return now_ticks, tick_hz

    now_int = find_first_prop(pm, ["DEVICE_TIMESTAMP","PTP_TIME","CURRENT_TIMESTAMP"])
    if isinstance(now_int, ic4.PropInteger):
        return int(now_int.value), 1_000_000_000

    raise RuntimeError("Device timestamp/frequency properties not found. Consider pm.dump().")

# Listener that saves exactly one frame per camera
class OneShotSaveListener(ic4.QueueSinkListener):
    def __init__(self, serial: str):
        self.serial = serial
        self.saved  = False
    def sink_connected(self, sink, image_type, min_buffers_required):
        return True
    def frames_queued(self, sink: ic4.QueueSink):
        buf = sink.pop_output_buffer()
        if not self.saved:
            os.makedirs(SAVE_DIR, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            path = os.path.join(SAVE_DIR, f"{self.serial}_{ts}.png")
            buf.save_as_png(path)
            print(f"[{self.serial}] saved: {path}")
            self.saved = True

def _work():
    devs = ic4.DeviceEnum.devices()
    if len(devs) < 2:
        raise RuntimeError("At least two cameras are required.")

    grabbers = [ic4.Grabber(info) for info in devs]
    serials  = [g.device_info.serial for g in grabbers]  # keep plain strings

    # Show PTP status (optional, for visibility)
    def ptp_status(pm: ic4.PropertyMap):
        p = find_first_prop(pm, ["PTP_STATUS"])
        return p.value if isinstance(p, ic4.PropEnumeration) else "Unknown"

    print("=== PTP Status ===")
    for g, srl in zip(grabbers, serials):
        print(f"[{srl}] {ptp_status(g.device_property_map)}")

    # Configure Trigger -> Action, and set TriggerMode = On
    for g, srl in zip(grabbers, serials):
        pm = g.device_property_map

        trig_mode = get_prop(pm, "TRIGGER_MODE")
        if not isinstance(trig_mode, ic4.PropEnumeration) or not enum_set_on(trig_mode):
            raise RuntimeError(f"{srl}: failed to set TRIGGER_MODE=On")

        trig_src = get_prop(pm, "TRIGGER_SOURCE")
        if not isinstance(trig_src, ic4.PropEnumeration) or not enum_set_contains(trig_src, "Action"):
            raise RuntimeError(f"{srl}: TRIGGER_SOURCE with 'Action' not found")

        # Common keys for action scheduler
        for pid, val in (
            ("ACTION_SCHEDULER_GROUP_KEY",  0x00000001),
            ("ACTION_SCHEDULER_GROUP_MASK", 0xFFFFFFFF),
            ("ACTION_SCHEDULER_DEVICE_KEY", 0x00000001),
        ):
            p = get_prop(pm, pid)
            if isinstance(p, ic4.PropInteger):
                p.value = int(val)

        # If a selector exists, select "0" variant
        sel = get_prop(pm, "ACTION_SCHEDULER_SELECTOR")
        if isinstance(sel, ic4.PropEnumeration):
            for e in sel.entries:
                if e.name in ("0", "Action0", "Schedule0"):
                    sel.value = e.name
                    break

    # Connect sinks and configure streaming
    sinks = []
    for g, srl in zip(grabbers, serials):
        lst  = OneShotSaveListener(srl)
        sink = ic4.QueueSink(lst, accepted_pixel_formats=[PIXFORMAT])
        g.stream_setup(sink)
        sinks.append((g, srl, sink, lst))

    # Schedule a single action at device time "now + DELAY_SEC"
    fire_at_host = datetime.datetime.now() + datetime.timedelta(seconds=DELAY_SEC)
    for g, srl, _, _ in sinks:
        pm = g.device_property_map
        now_ticks, tick_hz = get_device_time_and_freq(pm)
        target_ticks = now_ticks + int(DELAY_SEC * tick_hz)

        tprop = get_prop(pm, "ACTION_SCHEDULER_TIME")
        if not isinstance(tprop, ic4.PropInteger):
            raise RuntimeError(f"{srl}: ACTION_SCHEDULER_TIME not found")
        tprop.value = int(target_ticks)

        iprop = get_prop(pm, "ACTION_SCHEDULER_INTERVAL")
        if isinstance(iprop, ic4.PropInteger):
            iprop.value = 0  # single-shot

        print(f"[{srl}] will fire ticks={target_ticks} (hz={tick_hz})")

    print(f"Scheduling single-shot at ~{fire_at_host.strftime('%H:%M:%S')} (device time).")

    # Commit on all cameras as simultaneously as possible
    for g, srl, _, _ in sinks:
        pm = g.device_property_map
        commit = get_prop(pm, "ACTION_SCHEDULER_COMMIT")
        if not isinstance(commit, ic4.PropCommand):
            raise RuntimeError(f"{srl}: ACTION_SCHEDULER_COMMIT not found")
        commit.execute()
        print(f"[{srl}] COMMIT issued")

    # Wait until all cameras have saved their single frame (timeout = DELAY + 5s)
    deadline = time.time() + DELAY_SEC + 5.0
    while time.time() < deadline and not all(lst.saved for (_,_,_,lst) in sinks):
        time.sleep(0.01)

    if all(lst.saved for (_,_,_,lst) in sinks):
        print("All cameras saved one frame.")
    else:
        missing = [srl for (_,srl,_,lst) in sinks if not lst.saved]
        raise RuntimeError(f"Missing saves on: {missing}")

    # Clean up completely (avoid “Library.init was not called” on GC after context exit)
    for g, _, sink, _ in sinks:
        try:
            g.stream_stop()
        except Exception:
            pass
        g.device_close()
        del sink
        del g

    sinks.clear()
    grabbers.clear()
    del devs
    del serials
    gc.collect()

def main():
    # Keep all IC4 objects inside the context and free them before leaving it
    with ic4.Library.init_context(api_log_level=ic4.LogLevel.INFO,
                                  log_targets=ic4.LogTarget.STDERR):
        _work()
        gc.collect()  # extra safety

if __name__ == "__main__":
    main()

