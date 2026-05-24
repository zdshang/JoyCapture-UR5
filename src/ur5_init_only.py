#!/usr/bin/env python3
"""
UR5 init-only helper.

Runs startup initialization without launching teleop:
1) close safety popup
2) unlock protective stop
3) power on
4) brake release
5) optional gripper activate
"""

from __future__ import annotations

import argparse
import socket
import time


DASHBOARD_PORT = 29999
URSCRIPT_PORT = 30002


def recv_line(sock: socket.socket, timeout: float) -> str:
    sock.settimeout(timeout)
    data = sock.recv(4096)
    return data.decode("utf-8", errors="replace").strip()


def dashboard_command(host: str, command: str, timeout: float) -> str:
    """Send one UR dashboard command and return the controller reply."""
    with socket.create_connection((host, DASHBOARD_PORT), timeout=timeout) as sock:
        _ = recv_line(sock, timeout)
        sock.sendall((command.strip() + "\n").encode("utf-8"))
        return recv_line(sock, timeout)


def wait_program_stopped(host: str, timeout_s: float, dashboard_timeout: float) -> None:
    """Poll dashboard state until no UR program is running."""
    deadline = time.monotonic() + max(0.1, timeout_s)
    while time.monotonic() < deadline:
        try:
            state = dashboard_command(host, "programState", dashboard_timeout)
        except Exception:
            time.sleep(0.2)
            continue
        if "stopped" in state.lower():
            return
        time.sleep(0.2)
    print("[init] warning: programState did not become STOPPED before timeout.")


def gripper_activate_urp(host: str, urp_path: str, timeout: float, settle_timeout: float) -> None:
    print(f"[init] gripper activate via URP: {urp_path}")
    reply = dashboard_command(host, f"load {urp_path}", timeout)
    print(f"[init] load -> {reply}")
    try:
        reply = dashboard_command(host, "play", timeout)
        print(f"[init] play -> {reply}")
    except TimeoutError:
        print("[init] play reply timeout; command sent.")
    wait_program_stopped(host, settle_timeout, timeout)
    try:
        reply = dashboard_command(host, "stop", timeout)
        print(f"[init] stop -> {reply}")
    except Exception as exc:
        print(f"[init] stop failed: {exc}")


def gripper_activate_socket(host: str, timeout: float, port: int, speed: int, force: int, pos: int) -> None:
    """Activate Robotiq through its socket interface without loading a URP."""
    print(f"[init] gripper activate via socket port {port}")
    cmds = [
        "SET ACT 1",
        "SET GTO 1",
        f"SET SPE {speed}",
        f"SET FOR {force}",
        f"SET POS {pos}",
    ]
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        for cmd in cmds:
            sock.sendall((cmd + "\n").encode("utf-8"))
            try:
                _ = sock.recv(128)
            except Exception:
                pass
    print("[init] gripper socket activate sent.")


def main() -> int:
    parser = argparse.ArgumentParser(description="UR5 init-only helper.")
    parser.add_argument("--host", required=True, help="Robot controller IP")
    parser.add_argument("--dashboard-timeout-s", type=float, default=1.5, help="Dashboard timeout")
    parser.add_argument("--startup-settle-timeout-s", type=float, default=8.0, help="Program settle timeout")
    parser.add_argument(
        "--mode",
        choices=("full", "brake_release_only", "gripper_activate_only"),
        default="full",
        help="Initialization scope: full startup sequence or brake-release-only",
    )
    parser.add_argument(
        "--gripper-activate-mode",
        choices=("auto", "urp", "robotiq_socket", "none"),
        default="auto",
        help="How to run gripper activation",
    )
    parser.add_argument("--gripper-activate-urp", default="/programs/gripper_activate.urp", help="Activation URP path")
    parser.add_argument("--robotiq-socket-port", type=int, default=63352, help="Robotiq socket port")
    parser.add_argument("--robotiq-speed", type=int, default=180, help="Robotiq speed [0..255]")
    parser.add_argument("--robotiq-force", type=int, default=120, help="Robotiq force [0..255]")
    parser.add_argument("--robotiq-init-pos", type=int, default=0, help="Robotiq init/open pos [0..255]")
    args = parser.parse_args()

    print(f"[init] target: {args.host}")
    if args.mode == "brake_release_only":
        startup_cmds = ("brake release",)
    elif args.mode == "gripper_activate_only":
        startup_cmds = ()
    else:
        startup_cmds = (
            "close safety popup",
            "unlock protective stop",
            "power on",
            "brake release",
        )
    for command in startup_cmds:
        try:
            reply = dashboard_command(args.host, command, args.dashboard_timeout_s)
            print(f"[init] {command} -> {reply}")
        except Exception as exc:
            print(f"[init] {command} failed: {exc}")
        time.sleep(0.15)

    if args.mode == "brake_release_only":
        print("[init] brake-release-only mode done.")
        return 0

    if args.gripper_activate_mode == "auto":
        # Try URP first (matches many existing setups), then socket fallback.
        try:
            gripper_activate_urp(
                args.host,
                args.gripper_activate_urp,
                args.dashboard_timeout_s,
                args.startup_settle_timeout_s,
            )
        except Exception as exc:
            print(f"[init] URP activate failed in auto mode: {exc}")
        try:
            gripper_activate_socket(
                args.host,
                args.dashboard_timeout_s,
                args.robotiq_socket_port,
                args.robotiq_speed,
                args.robotiq_force,
                args.robotiq_init_pos,
            )
        except Exception as exc:
            print(f"[init] socket activate failed in auto mode: {exc}")
    elif args.gripper_activate_mode == "urp":
        gripper_activate_urp(
            args.host,
            args.gripper_activate_urp,
            args.dashboard_timeout_s,
            args.startup_settle_timeout_s,
        )
    elif args.gripper_activate_mode == "robotiq_socket":
        gripper_activate_socket(
            args.host,
            args.dashboard_timeout_s,
            args.robotiq_socket_port,
            args.robotiq_speed,
            args.robotiq_force,
            args.robotiq_init_pos,
        )
    else:
        print("[init] gripper activate skipped.")

    wait_program_stopped(args.host, args.startup_settle_timeout_s, args.dashboard_timeout_s)
    print("[init] done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
