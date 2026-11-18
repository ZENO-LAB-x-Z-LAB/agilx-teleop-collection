"""Exercise the CAN menu and privileged command selection without hardware."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from pure_teleop.internal import can_setup, pure_teleop_tui as tui


def link_reply(name, *, up=False, bitrate=None, kind="can", state=None):
    data = {"state": state or ("ERROR-ACTIVE" if up else "STOPPED")}
    if bitrate is not None:
        data["bittiming"] = {"bitrate": bitrate}
    link = {
        "ifname": name, "link_type": "can" if kind == "can" else "ether",
        "flags": ["UP"] if up else [], "operstate": "UNKNOWN",
        "linkinfo": {"info_kind": kind, "info_data": data},
    }
    return subprocess.CompletedProcess([], 0, json.dumps([link]), "")


def commands(run):
    return [call.args[0] for call in run.call_args_list]


class CanSetupTest(unittest.TestCase):
    def setUp(self):
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))

    def test_only_change_down_link_and_use_configured_bitrate(self):
        up = link_reply("slave", up=True, bitrate=500000)
        replies = [link_reply("master"), up, subprocess.CompletedProcess([], 0),
                   link_reply("master", up=True, bitrate=500000)]
        with patch.object(os, "geteuid", return_value=1000), patch.object(
            subprocess, "run", side_effect=replies,
        ) as run:
            can_setup.ensure_can_up({"右主臂": "master", "右从臂": "slave"}, 500000)
        writes = [cmd for cmd in commands(run) if "set" in cmd]
        self.assertEqual(writes, [["sudo", "--", "ip", "link", "set", "dev", "master",
                                   "up", "type", "can", "bitrate", "500000"]])

    def test_preflight_errors_leave_entire_selection_untouched(self):
        missing = subprocess.CompletedProcess([], 1, "", "Device does not exist")
        for other, reason in (
            (missing, "无法读取"), (link_reply("eth0", kind="ether"), "SocketCAN"),
            (link_reply("slave", up=True, bitrate=250000), "250000"),
            (link_reply("slave", up=True, state="BUS-OFF"), "BUS-OFF"),
            (link_reply("master"), "同时映射"),
        ):
            with self.subTest(reason=reason), patch.object(
                subprocess, "run", side_effect=[link_reply("master"), other],
            ) as run:
                with self.assertRaisesRegex((RuntimeError, ValueError), reason):
                    can_setup.ensure_can_up({"主臂": "master", "从臂": "other"}, 1000000)
                self.assertTrue(all("show" in cmd for cmd in commands(run)))

    def test_readback_after_set_and_retry(self):
        targets = {"主臂": "master", "从臂": "slave"}
        master_up = link_reply("master", up=True, bitrate=1000000)
        slave_down = link_reply("slave")
        ok = subprocess.CompletedProcess([], 0)
        with patch.object(subprocess, "run", side_effect=[
            link_reply("master"), slave_down, ok, master_up, ok, slave_down,
        ]):
            with self.assertRaisesRegex(RuntimeError, "slave.*DOWN"):
                can_setup.ensure_can_up(targets, 1000000)
        with patch.object(subprocess, "run", side_effect=[
            master_up, slave_down, ok, link_reply("slave", up=True, bitrate=1000000),
        ]) as retry:
            can_setup.ensure_can_up(targets, 1000000)
        self.assertEqual([cmd[cmd.index("dev") + 1] for cmd in commands(retry) if "set" in cmd], ["slave"])

    def test_teleop_start_prepares_only_the_selected_role(self):
        for failure in (None, RuntimeError("interface missing")):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp, patch.dict(
                os.environ, {"NERO_RIGHT_LEADER_CAN": "leader7", "NERO_CAN_BITRATE": "500000"},
            ), patch("builtins.input", side_effect=["2", "2", "", ""]), patch.object(
                can_setup, "ensure_can_up", side_effect=failure,
            ) as ensure, patch.object(tui, "run_interactive", return_value=0) as launch:
                actions = Mock()
                actions.attach_mock(ensure, "can")
                actions.attach_mock(launch, "launch")
                tui.option_activation(tui.default_state(), Path(tmp) / "state.json")
                ensure.assert_called_once_with({"右主臂": "leader7"}, 500000)
                self.assertEqual([call[0] for call in actions.mock_calls],
                                 ["can", "launch"] if failure is None else ["can"])

    def test_can_menu_allows_return_before_any_change(self):
        with patch("builtins.input", side_effect=["2", "1", "b"]), patch.object(
            can_setup, "ensure_can_up",
        ) as ensure:
            tui.option_can(tui.default_state())
        ensure.assert_not_called()


if __name__ == "__main__":
    unittest.main()
