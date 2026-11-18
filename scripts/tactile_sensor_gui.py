#!/usr/bin/env python3
"""Live GUI for M2020 tactile sensors, with optional single-sensor selection."""

from __future__ import annotations

import argparse
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import serial


def calculate_lrc(data: bytes) -> int:
    return ((~(sum(data) & 0xFF)) + 1) & 0xFF


def signed_u8(value: int) -> int:
    return int(value) if value <= 127 else int(value) - 256


def request_frame(length: int = 27, addr: int = 1) -> bytes:
    payload = bytes(
        [
            0x55,
            0xAA,
            0x09,
            0x00,
            addr & 0xFF,
            0x00,
            0xFB,
            0x0E,
            0x04,
            0x00,
            0x00,
            length & 0xFF,
            (length >> 8) & 0xFF,
        ]
    )
    return payload + bytes([calculate_lrc(payload)])


def parse_response(response: bytes, expected_len: int = 27, data_start: int = 14) -> list[tuple[float, float, float]]:
    if len(response) < data_start + expected_len:
        raise RuntimeError(f"short response: {len(response)} bytes")
    if response[:2] != b"\xaa\x55":
        raise RuntimeError(f"bad header: {response[:2].hex()}")
    body_len = int.from_bytes(response[2:4], byteorder="little")
    lrc_pos = 4 + body_len
    if lrc_pos >= len(response):
        raise RuntimeError(f"incomplete lrc: pos={lrc_pos} len={len(response)}")
    expected_lrc = calculate_lrc(response[:lrc_pos])
    actual_lrc = response[lrc_pos]
    if actual_lrc != expected_lrc:
        raise RuntimeError(f"bad lrc: expected={expected_lrc:02X} got={actual_lrc:02X}")

    payload = response[data_start : data_start + expected_len]
    points: list[tuple[float, float, float]] = []
    for index in range(0, len(payload), 3):
        chunk = payload[index : index + 3]
        points.append((signed_u8(chunk[0]) * 0.1, signed_u8(chunk[1]) * 0.1, int(chunk[2]) * 0.1))
    return points


def read_response(ser: serial.Serial, min_len: int = 42, timeout: float = 0.08) -> bytes:
    data = b""
    deadline = time.monotonic() + max(0.02, timeout)
    last_rx = time.monotonic()
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting > 0:
            data += ser.read(waiting)
            last_rx = time.monotonic()
            if len(data) >= min_len:
                break
        elif data and time.monotonic() - last_rx > 0.01:
            break
        time.sleep(0.0005)
    if not data:
        raise RuntimeError("empty response")
    return data


def open_sensor_serial(port: str, *, timeout: float = 0.08) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=921600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
        inter_byte_timeout=0.001,
        xonxoff=False,
        rtscts=False,
    )
    ser.reset_input_buffer()
    return ser


@dataclass
class SensorPanel:
    root: tk.Widget
    name: str
    port: str
    select_callback: Callable[[str], None] | None = None
    ser: serial.Serial | None = None

    def __post_init__(self) -> None:
        self.frame = tk.LabelFrame(self.root, text=f"{self.name}  {self.port}", padx=10, pady=8)
        self.status = tk.StringVar(value="正在测试串口…")
        self.contact = tk.StringVar(value="contact: --")
        self.cells = [tk.StringVar(value="--") for _ in range(9)]
        self._selection_button: tk.Button | None = None
        self._health_widgets: list[tk.Widget] = []

        status_label = tk.Label(self.frame, textvariable=self.status, anchor="w")
        status_label.grid(row=0, column=0, columnspan=3, sticky="ew")
        contact_label = tk.Label(self.frame, textvariable=self.contact, font=("Sans", 14, "bold"))
        contact_label.grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(2, 8)
        )
        self._health_widgets.extend((status_label, contact_label))
        for i, var in enumerate(self.cells):
            label = tk.Label(
                self.frame,
                textvariable=var,
                width=16,
                height=5,
                relief="ridge",
                borderwidth=2,
                font=("Sans", 13),
            )
            label.grid(row=2 + i // 3, column=i % 3, padx=4, pady=4, sticky="nsew")
            self._health_widgets.append(label)
        for index in range(3):
            self.frame.columnconfigure(index, weight=1)
        if self.select_callback is not None:
            self._selection_button = tk.Button(
                self.frame,
                text=f"选择 {self.name}（等待测试）",
                command=lambda: self.select_callback(self.port),
                state="disabled",
            )
            self._selection_button.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self._set_health(None)

    def grid(self, **kwargs) -> None:
        self.frame.grid(**kwargs)

    def open(self) -> None:
        if self.ser is not None and self.ser.is_open:
            return
        self.ser = open_sensor_serial(self.port, timeout=0.08)

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def _set_health(self, healthy: bool | None) -> None:
        """Make the serial preflight result obvious before a port can be selected."""

        if healthy is True:
            color = "#d9f7de"
            if self._selection_button is not None:
                self._selection_button.configure(state="normal", text=f"选择 {self.name}（测试通过）")
        elif healthy is False:
            color = "#ffdede"
            if self._selection_button is not None:
                self._selection_button.configure(state="disabled", text=f"选择 {self.name}（测试失败）")
        else:
            color = "#f2f2f2"
        self.frame.configure(background=color)
        for widget in self._health_widgets:
            widget.configure(background=color)

    def update(self) -> None:
        try:
            self.open()
            assert self.ser is not None
            self.ser.reset_input_buffer()
            self.ser.write(request_frame())
            points = parse_response(read_response(self.ser))
            contact = sum(max(0.0, point[2] - 0.1) for point in points)
            self._set_health(True)
            self.status.set("✓ 测试通过：9 个 taxel 可读")
            self.contact.set(f"contact: {contact:.3f} N")
            for index, (fx, fy, fz) in enumerate(points):
                self.cells[index].set(
                    f"p{index:02d}\n"
                    f"Fx {fx:+.2f}  Fy {fy:+.2f}\n"
                    f"Fz {fz:+.2f} N"
                )
        except Exception as exc:
            self._set_health(False)
            self.status.set(f"✗ 测试失败：{type(exc).__name__}: {exc}")
            self.contact.set("contact: --")
            for cell in self.cells:
                cell.set("--")
            self.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", action="append", default=[], help="Serial port. Repeat for multiple sensors.")
    parser.add_argument("--period-ms", type=int, default=80)
    parser.add_argument(
        "--select-one",
        action="store_true",
        help="Show a select button on every sensor panel, write the chosen port, then close.",
    )
    parser.add_argument(
        "--selection-file",
        default="",
        help="File receiving the selected serial-port path when --select-one is used.",
    )
    args = parser.parse_args()
    if args.select_one and not str(args.selection_file).strip():
        parser.error("--select-one requires --selection-file")
    return args


def main() -> int:
    args = parse_args()
    ports = args.port or ["/dev/ttyACM0", "/dev/ttyACM1"]
    root = tk.Tk()
    root.title("Tactile Sensor Selector" if args.select_one else "Tactile Sensor Live Viewer")

    def on_close() -> None:
        for panel in panels:
            panel.close()
        root.destroy()

    selection_path = Path(str(args.selection_file)).expanduser() if args.select_one else None

    def select_port(port: str) -> None:
        assert selection_path is not None
        selection_path.parent.mkdir(parents=True, exist_ok=True)
        selection_path.write_text(str(port).strip() + "\n", encoding="utf-8")
        root.after(0, on_close)

    panels = [
        SensorPanel(root, f"tactile #{index + 1}", port, select_callback=select_port if args.select_one else None)
        for index, port in enumerate(ports)
    ]
    for index, panel in enumerate(panels):
        panel.grid(row=0, column=index, sticky="nsew", padx=8, pady=8)
        root.columnconfigure(index, weight=1)
    root.rowconfigure(0, weight=1)

    def tick() -> None:
        for panel in panels:
            panel.update()
        root.after(max(20, int(args.period_ms)), tick)

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
