"""Check that replaying a recorded flight keeps its timestamps and transforms.

Runs in the ROS 2 container on a run folder holding a bag (./ironlark check --record):

    ./ironlark check-replay runs/<run>

Plays the bag (its own /clock carries Webots time: the player publishes no clock of its own) while
recording what it publishes, then compares the two bags topic by topic: the same messages, byte for
byte, in the same order. A topic missing from either bag is a difference. Writes replay_check.json
beside the bag; exits nonzero on a difference.
"""
import hashlib
import json
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

TOPICS = ["/clock", "/imu/data", "/lidar/points", "/camera/image_raw", "/camera/camera_info", "/tf_static"]


def digests(bag):
    """Per topic: a digest of each message as stored (serialized), in order. Digests, not the
    messages: a forest flight's images alone are gigabytes once decompressed."""
    import rosbag2_py
    from bag_reader import open_bag

    reader = open_bag(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out.setdefault(topic, []).append(hashlib.sha256(data).digest())
    return out


def compare(original, replayed):
    """Per topic, whether the replay holds the original's messages in order; a topic either bag
    lacks fails."""
    report = {"topics": {}}
    for topic in TOPICS:
        a, b = original.get(topic, []), replayed.get(topic, [])
        entry = {"recorded": len(a), "replayed": len(b), "equal": bool(a) and a == b}
        if a != b:
            entry["first_difference"] = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
        report["topics"][topic] = entry
    report["passed"] = all(entry["equal"] for entry in report["topics"].values())
    return report


def main():
    run = Path(sys.argv[1])
    replay = run / "replay"
    shutil.rmtree(replay, ignore_errors=True)
    qos = "/project/ros/record_qos.yaml"  # reliable, deep: the check measures the bag, not a lagging recorder
    recorder = subprocess.Popen(["ros2", "bag", "record", "-o", str(replay), "--qos-profile-overrides-path", qos,
                                 *TOPICS],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)  # the recorder subscribes before the player starts
    # The delay lets the recorder match every topic before the first message goes out.
    subprocess.run(["ros2", "bag", "play", str(run / "bag"), "--delay", "3", "--qos-profile-overrides-path", qos,
                    "--topics", *TOPICS], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    recorder.send_signal(signal.SIGINT)
    recorder.wait(timeout=30)

    report = compare(digests(run / "bag"), digests(replay))
    (run / "replay_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    sys.exit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
