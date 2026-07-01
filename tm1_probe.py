#!/usr/bin/env python3
"""TM1 probe — verify the Rogue UE can reach the UAV MAVLink endpoint.

Run from the ROGUE UE network namespace:
  docker run -i --rm --network container:srsue_zmq3 dah-testbed-air \
    python3 - < testbed-4g/scripts/tm1_probe.py

This script is an evidence-gathering precheck, not the attack step. It sends a
GCS heartbeat and a benign capabilities request, then records whether the UAV
responds from the exposed MAVLink service.
"""
import argparse
import json
import sys
import time


DEFAULT_TARGET = "udpout:192.168.100.2:14550"
mavutil = None


def now():
    return time.strftime("%H:%M:%S")


def event(kind, **fields):
    rec = {"t": now(), "kind": kind, **fields}
    print(json.dumps(rec, ensure_ascii=False), flush=True)


def open_link(target, source_system, source_component):
    m = mavutil.mavlink_connection(
        target,
        source_system=source_system,
        source_component=source_component,
    )
    try:
        m.port.settimeout(0.2)
    except Exception:
        pass
    return m


def main():
    global mavutil
    ap = argparse.ArgumentParser(description="TM1 prerequisite probe")
    ap.add_argument("--target", default=DEFAULT_TARGET,
                    help=f"UAV MAVLink endpoint, default {DEFAULT_TARGET}")
    ap.add_argument("--source-system", type=int, default=255,
                    help="spoofed GCS system id used for the probe")
    ap.add_argument("--source-component", type=int, default=190,
                    help="spoofed GCS component id used for the probe")
    ap.add_argument("--duration", type=float, default=8.0,
                    help="seconds to wait for replies")
    args = ap.parse_args()

    from pymavlink import mavutil as _mavutil
    mavutil = _mavutil

    event("start", target=args.target, source_system=args.source_system,
          source_component=args.source_component)
    m = open_link(args.target, args.source_system, args.source_component)

    # A few heartbeats make ArduPilot learn this UDP sender as a GCS endpoint.
    for i in range(5):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                             0, 0, 0)
        event("sent_heartbeat", seq=i + 1)
        time.sleep(0.2)

    m.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, 1, 0, 0, 0, 0, 0, 0)
    event("sent_capabilities_request")

    seen = set()
    deadline = time.time() + args.duration
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue
        t = msg.get_type()
        if t == "BAD_DATA":
            continue
        seen.add(t)
        fields = {"msg_type": t, "src_sys": msg.get_srcSystem(),
                  "src_comp": msg.get_srcComponent()}
        if t == "COMMAND_ACK":
            fields.update(command=msg.command, result=msg.result)
        elif t == "AUTOPILOT_VERSION":
            fields.update(capabilities=getattr(msg, "capabilities", 0))
        elif t == "HEARTBEAT":
            fields.update(autopilot=msg.autopilot, vehicle_type=msg.type)
        event("rx", **fields)

    ok = bool(seen)
    event("summary", status="PASS" if ok else "FAIL",
          detail=("UAV responded to ROGUE-origin MAVLink"
                  if ok else "no MAVLink reply observed"),
          observed=sorted(seen))
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[tm1_probe] interrupted", file=sys.stderr)
        raise SystemExit(130)
