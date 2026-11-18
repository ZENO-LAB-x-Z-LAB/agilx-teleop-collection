"""Bring the selected SocketCAN interfaces up before starting teleoperation."""

import json
import os
import shlex
import subprocess
from dataclasses import dataclass
from typing import Mapping


def configured_bitrate() -> int:
    value = os.environ.get("NERO_CAN_BITRATE", "1000000")
    try:
        bitrate = int(value)
    except ValueError:
        bitrate = 0
    if bitrate <= 0:
        raise ValueError(f"NERO_CAN_BITRATE 需要填写正整数（bit/s），当前值为 {value!r}")
    return bitrate


@dataclass(frozen=True)
class CanLink:
    name: str
    up: bool
    virtual: bool
    bitrate: int | None


def read_can_link(name: str, expected_bitrate: int) -> CanLink:
    result = subprocess.run(
        ["ip", "-json", "-details", "link", "show", "dev", name],
        capture_output=True, text=True,
    )
    if result.returncode:
        raise RuntimeError(f"无法读取接口 {name}，请核对 CAN 配置和接线。{result.stderr.strip()}")
    links = json.loads(result.stdout)
    if not links or links[0].get("link_type") != "can":
        raise RuntimeError(f"{name} 不是 SocketCAN 接口，请核对 CAN 配置。")
    link = links[0]
    info = link.get("linkinfo", {})
    data = info.get("info_data", {})
    # iproute2 reports controllers with a fixed list of bitrates separately.
    bitrate = data.get("bittiming", {}).get("bitrate", data.get("bittiming_bitrate"))
    up = "UP" in link["flags"]
    if up and data.get("state") in {"BUS-OFF", "STOPPED", "SLEEPING"}:
        raise RuntimeError(f"{name} 的 CAN 控制器为 {data['state']}，请检查总线并恢复接口。")
    if up and bitrate and bitrate != expected_bitrate:
        raise RuntimeError(
            f"{name} 正以 {bitrate} bit/s 运行，配置为 {expected_bitrate} bit/s。"
            "请核对配置。如需切换速率，先停用该接口再重试。"
        )
    return CanLink(link["ifname"], up, info.get("info_kind") in {"vcan", "vxcan"}, bitrate)


def ensure_can_up(interfaces: Mapping[str, str], bitrate: int) -> None:
    # Resolve every selected role before writing to any interface.
    selected: dict[str, tuple[str, CanLink]] = {}
    for role, name in interfaces.items():
        link = read_can_link(name, bitrate)
        if link.name in selected:
            previous_role, _ = selected[link.name]
            raise ValueError(f"{role} 和 {previous_role} 同时映射到 {link.name}，请分别配置接口。")
        selected[link.name] = role, link

    for role, link in selected.values():
        if not link.up:
            command = ["ip", "link", "set", "dev", link.name, "up"]
            if not link.virtual:
                command.extend(["type", "can", "bitrate", str(bitrate)])
            if os.geteuid() != 0:
                command = ["sudo", "--", *command]
            print(f"[CAN] {role}  {shlex.join(command)}", flush=True)
            result = subprocess.run(command)
            # Another terminal may have brought the same interface up first.
            link = read_can_link(link.name, bitrate)
            if not link.up:
                raise RuntimeError(f"{role} {link.name} 仍为 DOWN（命令退出码 {result.returncode}）。")
        if link.virtual:
            detail = "虚拟 CAN"
        elif link.bitrate:
            detail = f"{link.bitrate} bit/s"
        else:
            detail = "适配器未报告波特率"
        print(f"[CAN] {role}  {link.name}  UP（{detail}）", flush=True)
