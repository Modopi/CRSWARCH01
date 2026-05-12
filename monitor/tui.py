#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "rich>=13.7",
#   "pyserial>=3.5",
# ]
# ///
"""Smart Helmet — Impact Monitor TUI.

master.ino 가 시리얼로 흘려보내는 CSV 스트림을 받아 보기 좋게 그려준다.

실행:
    uv run monitor/tui.py                       # 시리얼 포트 자동 탐색
    uv run monitor/tui.py --port /dev/cu.usbserial-XXXX
    uv run monitor/tui.py --port COM5 --baud 9600

uv 없으면:
    python3 -m pip install rich pyserial
    python3 monitor/tui.py
"""

import argparse
import math
import random
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from threading import Lock, Thread

import serial
from rich.align import Align
from rich.console import Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from serial.tools import list_ports

FIELDS = [
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "temp", "light",
    "aMag", "aNet",
    "force", "impulse", "jerk",
    "roll", "pitch", "gyroMag",
    "peakG", "peakForce",
    "impact",
]


@dataclass
class State:
    last: dict = field(default_factory=dict)
    history: deque = field(default_factory=lambda: deque(maxlen=120))
    impacts: deque = field(default_factory=lambda: deque(maxlen=10))
    last_update: float = 0.0
    connected: bool = False
    port: str = ""
    frames: int = 0
    bad_lines: int = 0
    # RFID
    last_uid: str = ""
    last_uid_time: float = 0.0
    rfid_count: int = 0
    rfid_log: deque = field(default_factory=lambda: deque(maxlen=10))
    lock: Lock = field(default_factory=Lock)


state = State()


def find_port() -> str | None:
    """가장 그럴싸한 Arduino 시리얼 포트를 고른다 (macOS/Linux/Windows)."""
    ports = list(list_ports.comports())
    keys = ["usbserial", "wchusb", "usbmodem", "SLAB_USB", "CH340", "CP210", "Arduino"]

    def score(p):
        s = f"{p.device} {p.description or ''} {p.manufacturer or ''}".lower()
        return sum(1 for k in keys if k.lower() in s)

    ports.sort(key=score, reverse=True)
    return ports[0].device if ports else None


def parse_line(line: str):
    line = line.strip()
    if not line.startswith("IMU,"):
        return None
    parts = line[4:].split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    return dict(zip(FIELDS, values))


def parse_rfid(line: str) -> str | None:
    """RFID,<UID_HEX> 라인을 파싱. UID 문자열(대문자) 또는 None."""
    line = line.strip()
    if not line.startswith("RFID,"):
        return None
    uid = line[5:].strip().upper()
    if not uid or any(c not in "0123456789ABCDEF" for c in uid):
        return None
    return uid


def _record_rfid(uid: str) -> None:
    with state.lock:
        state.last_uid = uid
        state.last_uid_time = time.time()
        state.rfid_count += 1
        stamp = datetime.now().strftime("%H:%M:%S")
        state.rfid_log.appendleft((stamp, uid))


def reader_thread(port: str, baud: int) -> None:
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                with state.lock:
                    state.connected = True
                    state.port = port
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace")
                    uid = parse_rfid(line)
                    if uid is not None:
                        _record_rfid(uid)
                        continue
                    data = parse_line(line)
                    if data is None:
                        if line.strip() and not line.strip().startswith("#"):
                            with state.lock:
                                state.bad_lines += 1
                        continue
                    with state.lock:
                        state.last = data
                        state.history.append(data["aMag"])
                        state.last_update = time.time()
                        state.frames += 1
                        if data["impact"] >= 0.5:
                            stamp = datetime.now().strftime("%H:%M:%S")
                            state.impacts.appendleft(
                                (stamp, data["aMag"], data["force"])
                            )
        except (serial.SerialException, OSError):
            with state.lock:
                state.connected = False
            time.sleep(1.5)


def demo_thread() -> None:
    """하드웨어 없이 TUI 시각 확인용 가짜 데이터 생성기 (~10 Hz)."""
    with state.lock:
        state.connected = True
        state.port = "DEMO"
    t0 = time.time()
    peak_g = 0.0
    peak_f = 0.0
    next_impact = t0 + random.uniform(4, 8)
    next_rfid = t0 + random.uniform(5, 10)
    demo_uids = ["04A37B9F", "DEADBEEF", "12345678"]
    while True:
        t = time.time() - t0
        # 자연스러운 흔들림
        ax = 0.05 * math.sin(t * 1.3) + random.gauss(0, 0.02)
        ay = 0.05 * math.sin(t * 0.9 + 1) + random.gauss(0, 0.02)
        az = 1.0 + 0.04 * math.sin(t * 2.1) + random.gauss(0, 0.02)
        gx = 5 * math.sin(t * 0.7) + random.gauss(0, 1)
        gy = 5 * math.sin(t * 1.1) + random.gauss(0, 1)
        gz = 3 * math.sin(t * 1.5) + random.gauss(0, 1)

        # 가끔 충격 한 방
        if time.time() >= next_impact:
            ax += random.uniform(-3, 3)
            ay += random.uniform(-3, 3)
            az += random.uniform(-3, 3)
            gx += random.uniform(-200, 200)
            gy += random.uniform(-200, 200)
            gz += random.uniform(-200, 200)
            next_impact = time.time() + random.uniform(4, 9)

        a_mag = math.sqrt(ax * ax + ay * ay + az * az)
        a_net = abs(a_mag - 1.0)
        force = 5.0 * a_net * 9.807
        impulse = force * 0.1
        jerk = random.uniform(0, 5) + (50 if a_mag > 2.5 else 0)
        roll = math.atan2(ay, az) * 180 / math.pi
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180 / math.pi
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        peak_g = max(peak_g, a_mag)
        peak_f = max(peak_f, force)
        impact = 1.0 if a_mag >= 2.5 else 0.0

        light = int(max(0, min(1023, 500 + 300 * math.sin(t / 7) + random.gauss(0, 20))))
        data = {
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "temp": 24.0 + 0.3 * math.sin(t / 30),
            "light": light,
            "aMag": a_mag, "aNet": a_net,
            "force": force, "impulse": impulse, "jerk": jerk,
            "roll": roll, "pitch": pitch, "gyroMag": gyro_mag,
            "peakG": peak_g, "peakForce": peak_f,
            "impact": impact,
        }
        with state.lock:
            state.last = data
            state.history.append(a_mag)
            state.last_update = time.time()
            state.frames += 1
            if impact >= 0.5:
                stamp = datetime.now().strftime("%H:%M:%S")
                state.impacts.appendleft((stamp, a_mag, force))
        if time.time() >= next_rfid:
            _record_rfid(random.choice(demo_uids))
            next_rfid = time.time() + random.uniform(6, 12)
        time.sleep(0.1)


# ── 위젯 빌더들 ───────────────────────────────────────────

def signed_bar(value: float, lo: float, hi: float, width: int = 14) -> Text:
    """중앙 0 기준 좌/우로 채우는 가로 바."""
    center = width // 2
    bar = [" "] * width
    if value >= 0:
        frac = min(value / hi, 1.0) if hi > 0 else 0
        filled = int(frac * center)
        for i in range(filled):
            bar[center + i] = "█"
        for i in range(filled, center):
            bar[center + i] = "·"
        for i in range(center):
            bar[i] = "·"
    else:
        frac = min(-value / -lo, 1.0) if lo < 0 else 0
        filled = int(frac * center)
        for i in range(filled):
            bar[center - 1 - i] = "█"
        for i in range(filled, center):
            bar[center - 1 - i] = "·"
        for i in range(center, width):
            bar[i] = "·"
    color = "red" if abs(value) > (hi if value > 0 else -lo) * 0.75 else (
        "yellow" if abs(value) > (hi if value > 0 else -lo) * 0.4 else "green"
    )
    return Text("".join(bar), style=color)


def sparkline(values, width: int = 80) -> str:
    blocks = "▁▂▃▄▅▆▇█"
    if not values:
        return "·" * width
    data = list(values)[-width:]
    if len(data) < width:
        data = ["·"] * (width - len(data)) + data
    nums = [v for v in data if isinstance(v, float)]
    if not nums:
        return "·" * width
    lo = min(min(nums), 0.8)
    hi = max(max(nums), 1.6)
    span = hi - lo or 1.0
    out = []
    for v in data:
        if isinstance(v, str):
            out.append(" ")
            continue
        idx = int((v - lo) / span * (len(blocks) - 1))
        idx = max(0, min(len(blocks) - 1, idx))
        out.append(blocks[idx])
    return "".join(out)


def header_panel() -> Panel:
    with state.lock:
        connected = state.connected
        port = state.port or "(searching...)"
        frames = state.frames
        bad = state.bad_lines
        age = time.time() - state.last_update if state.last_update else 999
    if connected and age < 2:
        status = "[bold green]●[/] [green]LIVE[/]"
    elif connected:
        status = "[bold yellow]●[/] [yellow]stalled[/]"
    else:
        status = "[bold red]●[/] [red]disconnected[/]"
    title = Text.from_markup(
        f"[bold white]Smart Helmet[/]  —  Impact Monitor   "
        f"{status}    [dim]{port}   frames {frames}   bad {bad}[/]"
    )
    return Panel(Align.center(title), border_style="cyan", padding=(0, 1))


def accel_panel(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold cyan", width=4)
    t.add_column(justify="right", width=12)
    t.add_column(width=16)
    if d:
        for key in ("ax", "ay", "az"):
            v = d[key]
            t.add_row(key, f"{v:+.3f} g", signed_bar(v, -2, 2))
        t.add_row("", "", "")
        a_mag = d["aMag"]
        net = d["aNet"]
        mag_color = "red" if a_mag > 2.5 else "yellow" if a_mag > 1.5 else "white"
        t.add_row("|a|", f"[bold {mag_color}]{a_mag:.3f} g[/]", f"net [bold]{net:.3f}[/] g")
    else:
        t.add_row("--", "no data", "")
    return Panel(t, title="[bold]Acceleration[/]", border_style="blue")


def impact_panel(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold cyan", width=8)
    t.add_column(justify="right")
    if d:
        f_val = d["force"]
        f_color = "red" if f_val > 100 else "yellow" if f_val > 30 else "white"
        t.add_row("Force",   f"[{f_color}]{f_val:9.2f} N[/]")
        t.add_row("Impulse", f"{d['impulse']:9.4f} N·s")
        t.add_row("Jerk",    f"{d['jerk']:9.2f} g/s")
    return Panel(t, title="[bold magenta]Impact metrics[/]", border_style="magenta")


def orient_panel(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold cyan", width=6)
    t.add_column(justify="right", width=10)
    t.add_column(width=16)
    if d:
        t.add_row("Roll",  f"{d['roll']:+7.1f}°",  signed_bar(d["roll"],  -90, 90))
        t.add_row("Pitch", f"{d['pitch']:+7.1f}°", signed_bar(d["pitch"], -90, 90))
    return Panel(t, title="[bold]Orientation[/]", border_style="green")


def gyro_panel(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold cyan", width=4)
    t.add_column(justify="right", width=14)
    if d:
        for k in ("gx", "gy", "gz"):
            t.add_row(k, f"{d[k]:+9.1f} °/s")
        t.add_row("|ω|", f"[bold]{d['gyroMag']:9.1f} °/s[/]")
    return Panel(t, title="[bold]Angular velocity[/]", border_style="yellow")


def status_panel(d) -> Panel:
    t = Table.grid(padding=(0, 2))
    t.add_column(style="bold cyan")
    t.add_column(justify="right")
    if d:
        t.add_row("Temp",     f"{d['temp']:.1f} °C")
        lux = d.get("light", 0)
        lux_pct = max(0.0, min(1.0, lux / 1023.0)) * 100
        lux_color = "bright_yellow" if lux_pct > 60 else "yellow" if lux_pct > 25 else "dim"
        t.add_row("Light",    f"[{lux_color}]{int(lux):4d} ({lux_pct:5.1f}%)[/]")
        t.add_row("Peak |a|", f"[bold red]{d['peakG']:.3f} g[/]")
        t.add_row("Peak F",   f"[bold red]{d['peakForce']:.1f} N[/]")
    with state.lock:
        last_uid = state.last_uid
        uid_time = state.last_uid_time
        rfid_n = state.rfid_count
    if last_uid:
        age = time.time() - uid_time
        if age < 3:
            age_str = "[bold green]now[/]"
        elif age < 60:
            age_str = f"{age:.0f}s ago"
        elif age < 3600:
            age_str = f"{age/60:.0f}m ago"
        else:
            age_str = "long ago"
        t.add_row("RFID", f"[bold green]{last_uid}[/] [dim]({age_str}, n={rfid_n})[/]")
    else:
        t.add_row("RFID", "[dim](no scan yet)[/]")
    return Panel(t, title="[bold]Status[/]", border_style="cyan")


def history_panel() -> Panel:
    with state.lock:
        vals = list(state.history)
    spark = sparkline(vals, width=78)
    if vals:
        lo, hi = min(vals), max(vals)
        info = f"  min {lo:.2f} g    max {hi:.2f} g    n={len(vals)}"
    else:
        info = "  (waiting for data)"
    body = Group(
        Text(spark, style="bright_blue"),
        Text(info, style="dim"),
    )
    return Panel(body, title="[bold]|a| history[/]  (recent samples)", border_style="blue")


def impact_log_panel() -> Panel:
    with state.lock:
        rows = list(state.impacts)
    t = Table.grid(padding=(0, 2))
    t.add_column(style="dim", width=10)
    t.add_column(style="bold red")
    t.add_column(justify="right", style="red")
    if not rows:
        t.add_row("--:--:--", "(no impacts yet)", "")
    for ts, g, f_val in rows:
        t.add_row(ts, f"⚠ {g:.2f} g", f"{f_val:.1f} N")
    return Panel(t, title="[bold red]Impact log[/]", border_style="red")


def render() -> Layout:
    with state.lock:
        d = dict(state.last) if state.last else None

    layout = Layout()
    layout.split_column(
        Layout(header_panel(), name="header", size=3),
        Layout(name="body"),
        Layout(history_panel(), name="hist", size=5),
        Layout(name="footer", size=9),
    )
    layout["body"].split_row(
        Layout(name="left"),
        Layout(name="right"),
    )
    layout["left"].split_column(
        Layout(accel_panel(d)),
        Layout(impact_panel(d), size=7),
    )
    layout["right"].split_column(
        Layout(orient_panel(d)),
        Layout(gyro_panel(d), size=7),
    )
    layout["footer"].split_row(
        Layout(status_panel(d)),
        Layout(impact_log_panel(), ratio=2),
    )
    return layout


def main() -> None:
    ap = argparse.ArgumentParser(description="Smart Helmet TUI monitor")
    ap.add_argument("--port", help="시리얼 포트 (생략 시 자동 탐색)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--list", action="store_true", help="감지된 시리얼 포트 목록만 보고 종료")
    ap.add_argument("--demo", action="store_true", help="가짜 데이터로 TUI 동작 확인 (시리얼 미사용)")
    ap.add_argument("--raw", action="store_true", help="TUI 대신 시리얼 raw 출력 (디버그용)")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return

    if args.demo:
        Thread(target=demo_thread, daemon=True).start()
        try:
            with Live(render(), refresh_per_second=15, screen=True) as live:
                while True:
                    time.sleep(1 / 15)
                    live.update(render())
        except KeyboardInterrupt:
            pass
        return

    port = args.port or find_port()
    if not port:
        print("시리얼 포트를 찾지 못했어. --port 로 직접 지정해.", file=sys.stderr)
        print("사용 가능한 포트:", file=sys.stderr)
        for p in list_ports.comports():
            print(f"  {p.device}\t{p.description}", file=sys.stderr)
        sys.exit(1)

    if args.raw:
        print(f"raw dump from {port} @ {args.baud}  (Ctrl+C to quit)")
        try:
            with serial.Serial(port, args.baud, timeout=1) as ser:
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    parsed = parse_line(line)
                    tag = "OK " if parsed else "BAD"
                    n_fields = len(line.split(",")) if "," in line else 0
                    starts = line[:4]
                    print(f"[{tag}] len={len(line):3d} fields={n_fields:2d} start={starts!r:7s} | {line}")
        except KeyboardInterrupt:
            pass
        return

    Thread(target=reader_thread, args=(port, args.baud), daemon=True).start()

    try:
        with Live(render(), refresh_per_second=15, screen=True) as live:
            while True:
                time.sleep(1 / 15)
                live.update(render())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
