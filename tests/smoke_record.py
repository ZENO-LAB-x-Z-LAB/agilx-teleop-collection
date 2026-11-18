#!/usr/bin/env python3
"""Record synthetic joint state, action and JPEG through the normal keyboard entry point."""
from collections import Counter
import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage, JointState


ROOT = Path(__file__).resolve().parents[1]
PREFIX = f"/nero_smoke_{os.getpid()}"
STATE, ACTION, CAMERA = (f"{PREFIX}/{name}" for name in ("state", "action", "camera/compressed"))
NAMES = [f"joint{i}" for i in range(1, 8)] + ["gripper"]
MEASURED = [float(i) / 10 for i in range(8)]
COMMANDED = [float(i) / 20 for i in range(8)]
PIXELS = np.full((24, 32, 3), 96, dtype=np.uint8)


def publish_samples(node, finished):
    state_pub = node.create_publisher(JointState, STATE, 10)
    action_pub = node.create_publisher(JointState, ACTION, 10)
    camera_pub = node.create_publisher(CompressedImage, CAMERA, 10)
    measured = JointState(name=NAMES, position=MEASURED)
    commanded = JointState(name=NAMES, position=COMMANDED)
    ok, jpeg = cv2.imencode(".jpg", PIXELS)
    assert ok
    frame = CompressedImage(format="jpeg", data=jpeg.tobytes())
    while not finished.is_set():
        stamp = node.get_clock().now().to_msg()
        for publisher, msg in ((state_pub, measured), (action_pub, commanded), (camera_pub, frame)):
            msg.header.stamp = stamp
            publisher.publish(msg)
        finished.wait(0.05)


def wait_for_message(master, marker, log, timeout=50):
    start = len(log)
    deadline = time.monotonic() + timeout
    while marker.encode() not in log[start:]:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([master], [], [], remaining)[0]:
            raise RuntimeError(f"Recorder timed out waiting for {marker}")
        try:
            chunk = os.read(master, 65536)
        except OSError as exc:
            if exc.errno != errno.EIO:
                raise
            chunk = b""
        if not chunk:
            raise RuntimeError(f"Recorder exited before {marker}")
        log.extend(chunk)


def record_episodes(directory):
    command = [str(ROOT / "scripts" / "record_current_pure_teleop_rosbag2.sh"),
               "--skip-camera", "--skip-tactile", "--no-rviz", "--bag-root", str(directory),
               "--name", "smoke", "--", "--min-free-gb", "0", "--wait-timeout-sec", "20",
               "--exclude-regex", f"^(?!{PREFIX}/)"]
    for topic in (STATE, ACTION, CAMERA):
        command.extend(["--required-topic", topic])
    master, slave = pty.openpty()
    log = bytearray()
    proc = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
    os.close(slave)
    try:
        wait_for_message(master, "Next episode: episode_001", log)
        for number in (1, 2):
            os.write(master, b" ")
            wait_for_message(master, f"[record] START episode_{number:03d}", log)
            time.sleep(1.4)
            os.write(master, b" ")
            wait_for_message(master, f"[record] next=episode_{number + 1:03d}", log)
        os.write(master, b"q")
        if proc.wait(timeout=15) != 0:
            raise RuntimeError("Recorder exited with an error")
    except BaseException:
        print(log.decode(errors="replace"), flush=True)
        raise
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
        os.close(master)


def verify_episode(path):
    meta = json.loads((path / "metadata.json").read_text())
    assert meta["stop_reason"] == "user_stop" and meta["rosbag_info_ok"], meta
    assert set(meta["recorded_topics"]) == {STATE, ACTION, CAMERA}, meta["recorded_topics"]
    bag = path / "episode"
    assert (bag / "metadata.yaml").is_file() and list(bag.glob("*.mcap"))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    counts = Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == CAMERA:
            msg = deserialize_message(data, CompressedImage)
            actual = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            assert msg.format == "jpeg" and np.array_equal(actual, PIXELS)
        else:
            msg = deserialize_message(data, JointState)
            assert list(msg.name) == NAMES
            assert list(msg.position) == (MEASURED if topic == STATE else COMMANDED)
        assert msg.header.stamp.sec > 0
        counts[topic] += 1
    assert set(counts) == {STATE, ACTION, CAMERA}, counts
    print(f"{path.name}: state={counts[STATE]} action={counts[ACTION]} jpeg={counts[CAMERA]}")


def main():
    with tempfile.TemporaryDirectory(prefix="nero-smoke-") as temp:
        directory = Path(temp)
        config = directory / "hardware.env"
        config.touch()
        os.environ["NERO_CONFIG"] = str(config)
        os.environ["ROS_DOMAIN_ID"] = str(40 + os.getpid() % 40)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ.pop("ROS_STATIC_PEERS", None)
        os.environ["ROS_HOME"] = str(directory / "ros")
        rclpy.init()
        node = rclpy.create_node(f"nero_smoke_{os.getpid()}")
        finished = threading.Event()
        publisher = threading.Thread(target=publish_samples, args=(node, finished))
        publisher.start()
        try:
            record_episodes(directory)
            for number in (1, 2):
                verify_episode(directory / "smoke" / f"episode_{number:03d}")
        finally:
            finished.set()
            publisher.join()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
