#!/usr/bin/env python3
"""TM1 monitor — record UAV state before/during/after injection.

Run from the GCS UE network namespace:
  docker run -i --rm --network container:srsue_zmq2 dah-testbed-air \
    python3 - < testbed-4g/scripts/tm1_monitor.py

The monitor does not attack. It listens to the normal GCS downlink and records
state changes that prove whether TM1 injection had an operational effect.
"""
import argparse
import json
import sys
import time


DEFAULT_CONN = "udpin:0.0.0.0:14550"
mavutil = None


def iso_ts():
    return time.strftime("%Y-%m-%dT%H:%M:%S")


def mode_name(mav, heartbeat):
    try:
        return mavutil.mode_string_v10(heartbeat)
    except Exception:
        return str(getattr(heartbeat, "custom_mode", "?"))


def emit(rec, fp=None):
    line = json.dumps({"t": iso_ts(), **rec}, ensure_ascii=False)
    print(line, flush=True)
    if fp:
        fp.write(line + "\n")
        fp.flush()


def main():
    global mavutil
    ap = argparse.ArgumentParser(description="TM1 state monitor")
    ap.add_argument("--conn", default=DEFAULT_CONN,
                    help=f"MAVLink receive endpoint, default {DEFAULT_CONN}")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--jsonl", default=None,
                    help="optional path for JSONL evidence output")
    ap.add_argument("--request-streams", action="store_true",
                    help="request MAVLink streams after heartbeat")
    args = ap.parse_args()

    from pymavlink import mavutil as _mavutil
    mavutil = _mavutil

    fp = open(args.jsonl, "a", encoding="utf-8") if args.jsonl else None
    m = mavutil.mavlink_connection(args.conn)
    emit({"kind": "start", "conn": args.conn, "duration": args.duration}, fp)

    hb = m.wait_heartbeat(timeout=40)
    if hb is None:
        emit({"kind": "summary", "status": "FAIL",
              "detail": "no heartbeat before monitor timeout"}, fp)
        return 1

    target_system = m.target_system
    target_component = m.target_component
    emit({"kind": "heartbeat_initial", "sys": target_system,
          "comp": target_component, "mode": mode_name(m, hb),
          "armed": bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}, fp)

    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if args.request_streams:
        m.mav.request_data_stream_send(target_system, target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                       4, 1)

    state = {
        "mode": None,
        "armed": None,
        "lat": None,
        "lon": None,
        "relalt": None,
        "last_hb": time.time(),
        "hb_count": 0,
    }
    last_emit = 0.0
    end = time.time() + args.duration

    while time.time() < end:
        # Keep a GCS heartbeat present so the normal C2 path stays active.
        if int(time.time() * 2) % 2 == 0:
            try:
                m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                     0, 0, 0)
            except Exception:
                pass

        msg = m.recv_match(blocking=True, timeout=1)
        if not msg:
            if time.time() - state["last_hb"] > 5:
                emit({"kind": "heartbeat_gap",
                      "seconds": round(time.time() - state["last_hb"], 1)}, fp)
                state["last_hb"] = time.time()
            continue

        t = msg.get_type()
        if t == "BAD_DATA":
            continue

        if t == "HEARTBEAT" and msg.get_srcSystem() == target_system:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = mode_name(m, msg)
            state["last_hb"] = time.time()
            state["hb_count"] += 1
            if mode != state["mode"] or armed != state["armed"]:
                state["mode"] = mode
                state["armed"] = armed
                emit({"kind": "state_change", "mode": mode, "armed": armed,
                      "base_mode": msg.base_mode,
                      "custom_mode": msg.custom_mode}, fp)

        elif t == "GLOBAL_POSITION_INT":
            relalt = msg.relative_alt / 1000.0
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            changed = (
                state["relalt"] is None
                or abs(relalt - state["relalt"]) >= 0.5
            )
            state.update(lat=lat, lon=lon, relalt=relalt)
            if changed or time.time() - last_emit > 5:
                last_emit = time.time()
                emit({"kind": "position", "lat": round(lat, 7),
                      "lon": round(lon, 7), "relalt_m": round(relalt, 2)}, fp)

        elif t == "COMMAND_ACK":
            emit({"kind": "command_ack", "src_sys": msg.get_srcSystem(),
                  "src_comp": msg.get_srcComponent(), "command": msg.command,
                  "result": msg.result}, fp)

        elif t == "STATUSTEXT":
            emit({"kind": "statustext", "severity": msg.severity,
                  "text": getattr(msg, "text", "")}, fp)

        elif t == "SYS_STATUS":
            emit({"kind": "sys_status",
                  "battery_remaining": getattr(msg, "battery_remaining", None),
                  "errors": getattr(msg, "errors_count1", 0)}, fp)

    emit({"kind": "summary", "status": "DONE",
          "heartbeat_count": state["hb_count"],
          "last_mode": state["mode"], "last_armed": state["armed"],
          "last_relalt_m": state["relalt"]}, fp)
    if fp:
        fp.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[tm1_monitor] interrupted", file=sys.stderr)
        raise SystemExit(130)
