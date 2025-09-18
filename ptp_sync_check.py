# ptp_sync_check_multi.py
# 接続されている 2〜8 台のカメラで PTP を有効化し、収束（Master 1台 + 残り Slave）を確認して一覧表示

import time
import imagingcontrol4 as ic4

PTP_ENABLE_NAMES = ["PtpEnable", "GevIEEE1588Enable"]
PTP_STATUS_NAMES = ["PtpStatus", "GevIEEE1588Status"]

MAX_WAIT_SEC = 30.0   # 収束待ち最大時間
POLL_SEC = 0.5        # ポーリング間隔


def find_prop(pm: ic4.PropertyMap, names):
    """候補名を順に探して最初に見つかった Property を返す。なければ None。"""
    for name in names:
        try:
            return pm.find(name)
        except ic4.IC4Exception:
            continue
    return None


def ensure_ptp_enabled(pm: ic4.PropertyMap) -> bool:
    """PTP を有効化（可能なら）。成功/既に有効なら True、ノードが無ければ False。"""
    prop = find_prop(pm, PTP_ENABLE_NAMES)
    if prop is None:
        return False
    try:
        if prop.value is False:
            prop.value = True
        return True
    except ic4.IC4Exception:
        return False


def get_ptp_status(pm: ic4.PropertyMap) -> str | None:
    """PTP ステータスの文字列（例: 'Master', 'Slave'）を返す。取得できなければ None。"""
    prop = find_prop(pm, PTP_STATUS_NAMES)
    if prop is None:
        return None
    try:
        return f"{prop.value}"
    except ic4.IC4Exception:
        return None


def has_converged(statuses: list[str | None]) -> bool:
    """収束条件: ステータスが全て 'Master' か 'Slave' で、Master がちょうど 1、Slave が 1 以上。"""
    s = [st for st in statuses if st is not None]
    if len(s) < 2:
        return False
    if any(st not in {"Master", "Slave"} for st in s):
        return False
    return (s.count("Master") == 1) and (s.count("Slave") >= 1)


def main():
    devices = ic4.DeviceEnum.devices()

    if len(devices) < 2:
        print("⚠ カメラが2台未満です。接続を確認してください。")
        return
    if len(devices) > 8:
        print(f"ℹ 8台を想定していますが、現在 {len(devices)} 台接続されています。先頭8台のみ対象にします。")
        devices = devices[:8]

    # 対象カメラの一覧表示
    print("=== 接続カメラ一覧 ===")
    for i, dev in enumerate(devices):
        print(f"[{i}] {dev.model_name} ({dev.serial}) [{dev.interface.display_name}]")
    print()

    # Grabber を開く
    grabbers: list[ic4.Grabber] = []
    try:
        for dev in devices:
            grabbers.append(ic4.Grabber(dev))

        # 各カメラの PropertyMap
        propmaps = [g.device_property_map for g in grabbers]

        # 可能なら PTP を有効化
        enabled_flags = [ensure_ptp_enabled(pm) for pm in propmaps]

        # PTP 収束待ち
        print(f"PTP 収束待ち中…（最大 {int(MAX_WAIT_SEC)} 秒）")
        t0 = time.time()
        statuses: list[str | None] = [None] * len(propmaps)

        while time.time() - t0 < MAX_WAIT_SEC:
            statuses = [get_ptp_status(pm) for pm in propmaps]
            if has_converged(statuses):
                break
            time.sleep(POLL_SEC)

        # 結果表示
        print("\n=== 最終ステータス ===")
        masters = 0
        slaves = 0
        for i, (dev, st, en) in enumerate(zip(devices, statuses, enabled_flags)):
            st_disp = st or "Unknown"
            en_disp = "Enabled" if en else "N/A"   # ノードなし等は N/A
            print(f"[{i}] {dev.serial}: PTP={en_disp}, Status={st_disp}")
            if st == "Master":
                masters += 1
            elif st == "Slave":
                slaves += 1

        if has_converged(statuses):
            print(f"\n✅ PTP同期 OK: Master={masters}, Slave={slaves}")
        else:
            print("\n⚠️ PTP同期 未達")
            if masters == 0:
                print("  - Master がいません（全台が Unknown/Slave/その他）。")
            elif masters > 1:
                print(f"  - Master が {masters} 台います（同一ドメインでの衝突の可能性）。")
            # 追加の一般的な確認ポイント
            print("  - 同一セグメント接続 / PTPドメイン一致 / スイッチのPTP透過設定 / IGMP設定 を確認してください。")
            print("  - 機種によりノード名が異なる場合があります。必要なら pm.dump() でノード探索してください。")

    finally:
        # 後始末：ストリームは使っていないので stream_stop() は不要
        for g in grabbers:
            try:
                g.device_close()
            except Exception:
                pass


if __name__ == "__main__":
    with ic4.Library.init_context(api_log_level=ic4.LogLevel.INFO, log_targets=ic4.LogTarget.STDERR):
        main()

