#!/usr/bin/env python3
"""End-to-end tests. Runs the real node and receiver as separate processes over real sockets,
with the simulated chip injecting earthquakes and hardware faults.

usage: system_test.py <build-dir>
"""
import json
import pathlib
import signal
import socket
import subprocess
import sys
import time

SPEED = 20


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def run(build_dir, duration, node_args):
    udp_port, tcp_port = free_port(), free_port()
    recv = subprocess.Popen(
        [str(build_dir / "seismo-recv"), "--udp-port", str(udp_port), "--tcp-port", str(tcp_port)],
        stdout=subprocess.PIPE,
        text=True,
    )
    assert "ready" in recv.stdout.readline()

    node = subprocess.run(
        [
            str(build_dir / "seismo-node"),
            "--sim",
            "--udp-port", str(udp_port),
            "--tcp-port", str(tcp_port),
            "--speed", str(SPEED),
            "--duration", str(duration),
            *node_args,
        ],
        capture_output=True,
        text=True,
        timeout=duration / SPEED + 30,
    )
    assert node.returncode == 0, node.stderr
    node_stats = json.loads(node.stdout.strip().splitlines()[-1])

    time.sleep(0.3)  # Give the receiver a moment to drain its socket buffers.
    recv.send_signal(signal.SIGTERM)
    out, _ = recv.communicate(timeout=10)
    lines = [json.loads(line) for line in out.splitlines()]
    recv_stats = lines[-1]
    assert recv_stats["type"] == "recv_stats"
    return lines[:-1], node_stats, recv_stats


def alerts_of(alerts, kind):
    return [a for a in alerts if a["type"] == kind]


def test_quiet(build_dir):
    alerts, node, recv = run(build_dir, 60, [])
    assert alerts == [], alerts
    assert node["overruns"] == 0
    assert recv["corrupt"] == 0


def test_earthquake(build_dir):
    alerts, _, _ = run(build_dir, 90, ["--quake-at", "40"])
    assert len(alerts_of(alerts, "event_start")) == 1, alerts
    end = alerts_of(alerts, "event_end")
    assert len(end) == 1, alerts
    assert 39.5 < end[0]["t_s"] < 41.0, end
    assert 2.5 < end[0]["dominant_hz"] < 3.5, end


def test_door_slam(build_dir):
    alerts, _, _ = run(build_dir, 60, ["--slam-at", "40"])
    assert len(alerts_of(alerts, "transient")) == 1, alerts
    assert alerts_of(alerts, "event_end") == [], alerts


def test_door_slam_does_not_hide_later_earthquake(build_dir):
    alerts, _, _ = run(build_dir, 110, ["--slam-at", "45", "--quake-at", "60"])
    assert len(alerts_of(alerts, "transient")) == 1, alerts
    starts = alerts_of(alerts, "event_start")
    assert len(starts) == 2, alerts
    # Caught on the weak P wave, not 3 s later on the S wave.
    assert 60.0 <= starts[1]["t_s"] < 61.5, starts


def test_bus_failure_then_earthquake(build_dir):
    alerts, node, _ = run(build_dir, 110, ["--bus-fault-at", "10", "--quake-at", "60"])
    assert len(alerts_of(alerts, "sensor_fault_bus")) == 1, alerts
    assert len(alerts_of(alerts, "sensor_recovered")) == 1, alerts
    assert len(alerts_of(alerts, "event_end")) == 1, alerts
    assert node["read_errors"] > 0


def test_stuck_sensor(build_dir):
    alerts, _, _ = run(build_dir, 30, ["--stuck-at", "10"])
    assert len(alerts_of(alerts, "sensor_fault_stuck")) >= 1, alerts
    assert len(alerts_of(alerts, "sensor_recovered")) >= 1, alerts


def test_packet_loss_is_measured(build_dir):
    _, node, recv = run(build_dir, 50, ["--drop-every", "10"])
    dropped = node["packets"] // 10
    assert recv["packets"] == node["packets"] - dropped, (node, recv)
    # Sequence numbers reveal a gap only when a later packet arrives, so if the very
    # last packet was the one dropped, it can't be counted.
    last_was_dropped = node["packets"] % 10 == 0
    assert recv["lost"] == dropped - (1 if last_was_dropped else 0), (node, recv)


def main():
    build_dir = pathlib.Path(sys.argv[1]).resolve()
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    failed = 0
    for test in tests:
        try:
            test(build_dir)
            print(f"PASS {test.__name__}")
        except Exception as error:  # Report every failure, not only the first.
            failed += 1
            print(f"FAIL {test.__name__}: {error!r}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
