"""Open a recorded rosbag2 for reading, compressed (zstd per message, as ./ironlark records them) or not, and count
the messages a bag being recorded holds so far (no ROS needed)."""
import sqlite3
import time
from pathlib import Path


def open_bag(uri):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    compressed = rosbag2_py.Info().read_metadata(str(uri), "sqlite3").compression_mode not in ("", "NONE", "none")
    reader = rosbag2_py.SequentialCompressionReader() if compressed else rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def messages_written(bag):
    """The messages in a bag's database files, while the recorder writes them (--max-cache-size 0: each one as it
    arrives); 0 before the first file exists."""
    count = 0
    for db in sorted(Path(bag).glob("*.db3")):
        with sqlite3.connect(f"file:{db}?mode=ro", uri=True, timeout=5) as connection:
            count += connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
    return count


def wait_for_messages(bag, expected, timeout_s=60):
    """Wait until a bag being recorded holds the expected messages; how many it holds."""
    deadline = time.monotonic() + timeout_s
    while (count := messages_written(bag)) < expected and time.monotonic() < deadline:
        time.sleep(0.1)
    return count
