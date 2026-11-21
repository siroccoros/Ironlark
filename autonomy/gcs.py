"""Lightweight Supervisory Ground Control Station (GCS) HTTP server (zero external deps).

Serves autonomy/gcs.html on http://127.0.0.1:8080 and exposes live sensor-derived state,
the drone's own 2D occupancy map slice, onboard camera frames, and high-level supervisory
commands (goto, explore, mission, hold, rth, land) — never ground truth and never joysticks.
"""
import base64
import json
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HTML_PATH = Path(__file__).with_name("gcs.html")


def encode_rgb_png(rgb_u8):
    """Encode an (H, W, 3) uint8 RGB image to PNG bytes using stdlib zlib."""
    h, w, _ = rgb_u8.shape
    raw = b"".join(b"\x00" + rgb_u8[y].tobytes() for y in range(h))

    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, level=1))
            + chunk(b"IEND", b""))


def _any_z(mask, z0, z1):
    """Union of mask[:, :, z0:z1] over z; plane-by-plane ORs beat np.any on a short strided axis 3x."""
    out = np.zeros(mask.shape[:2], dtype=bool)
    for k in range(z0, z1):
        out |= mask[:, :, k]
    return out


def _dilate_xy(mask, disk):
    """2D binary dilation as the union of mask translated by each structure offset (same cells as
    scipy.ndimage.binary_dilation with zero borders, ~5x faster at this size)."""
    r0, r1 = disk.shape[0] // 2, disk.shape[1] // 2
    n0, n1 = mask.shape
    pad = np.zeros((n0 + 2 * r0, n1 + 2 * r1), dtype=bool)
    pad[r0:r0 + n0, r1:r1 + n1] = mask
    out = np.zeros_like(mask)
    for i, j in np.argwhere(disk):
        out |= pad[2 * r0 - i:2 * r0 - i + n0, 2 * r1 - j:2 * r1 - j + n1]
    return out


def occupancy_slice_2d(occ_map, z_range=(1.1, 3.2)):
    """Project the drone's 3D OccupancyMap across flyable altitudes into a compact 2D grid.

    Cell codes: 0 = unknown, 1 = free, 2 = inflated safety margin, 3 = occupied obstacle.
    """
    nz = occ_map.shape[2]
    z0 = max(0, int((z_range[0] - occ_map.origin[2]) / occ_map.resolution))
    z1 = min(nz, int((z_range[1] - occ_map.origin[2]) / occ_map.resolution) + 1)
    occupied = occ_map.occupied
    occ = _any_z(occupied, z0, z1)
    free = _any_z(occ_map.free, z0, z1)
    st = occ_map._struct
    if not occ_map._inflate_dirty or not (st == st[:, :, :1]).all():
        inf = _any_z(occ_map.inflated_obstacles(), z0, z1)
    else:
        # A fresh snapshot's inflation is stale, and computing it dilates the whole 3D grid (~20 ms
        # holding the GIL on every map refresh). For a structure that is one xy disk repeated over
        # z, the union over the slice of that 3D dilation is exactly the xy dilation of the union
        # over the slice widened by the z radius: the same cells for ~1/50 of the work.
        rz = st.shape[2] // 2
        inf = _dilate_xy(_any_z(occupied, max(0, z0 - rz), min(nz, z1 + rz)), st[:, :, 0]) if z1 > z0 else occ
    cells = np.zeros(occ.shape, dtype=np.uint8)
    cells[free] = 1
    cells[inf] = 2
    cells[occ] = 3
    return {
        "nx": int(cells.shape[0]),
        "ny": int(cells.shape[1]),
        "origin": [float(occ_map.origin[0]), float(occ_map.origin[1])],
        "resolution": float(occ_map.resolution),
        "data_b64": base64.b64encode(cells.tobytes()).decode("ascii"),
    }


def start_gcs_server(executive, host="127.0.0.1", port=8080):
    """Start the GCS HTTP server in a daemon thread; returns (server, url)."""
    html_bytes = HTML_PATH.read_bytes()
    map_body = [None, b"", ""]  # (slice dict it was encoded from, JSON bytes, ETag)
    map_lock = threading.Lock()

    def encoded_map():
        """JSON for the executive's cached map slice, encoded once per new slice rather than per poll."""
        m = executive.gcs_map()
        with map_lock:
            if m is not map_body[0]:
                body = json.dumps(m).encode()
                map_body[:] = [m, body, '"%08x"' % (zlib.crc32(body) & 0xFFFFFFFF)]
            return map_body[1], map_body[2]

    class Handler(BaseHTTPRequestHandler):
        # Keep-alive: the page polls several times a second, so one connection (and handler thread)
        # per browser socket instead of a new TCP connection and thread per request.
        protocol_version = "HTTP/1.1"
        timeout = 30  # idle kept-alive sockets close instead of parking a thread forever

        def log_message(self, fmt, *args):
            return  # keep terminal clean

        def _send(self, code, content_type, body, headers=()):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in headers:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path, _, query = self.path.partition("?")
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", html_bytes)
            elif path == "/api/state":
                self._send(200, "application/json", json.dumps(executive.gcs_state()).encode())
            elif path == "/api/map":
                body, etag = encoded_map()
                if self.headers.get("If-None-Match") == etag:
                    self._send(304, "application/json", b"", (("ETag", etag),))
                else:
                    self._send(200, "application/json", body, (("ETag", etag),))
            elif path == "/api/camera.raw":
                after = 0
                for part in query.split("&"):
                    if part.startswith("after="):
                        try:
                            after = int(part.split("=", 1)[1])
                        except ValueError:
                            pass
                frame = executive.gcs_camera_frame(after_seq=after, timeout=0.15)
                if frame and frame[0] > after:
                    seq, w, h, raw = frame
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(len(raw)))
                    self.send_header("X-Seq", str(seq))
                    self.send_header("X-Width", str(w))
                    self.send_header("X-Height", str(h))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(raw)
                else:
                    self._send(204, "application/octet-stream", b"")
            else:
                self._send(404, "text/plain", b"not found")

        def do_POST(self):
            # Read the body before any reply so a kept-alive connection stays in sync.
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")))
            if self.path.split("?", 1)[0] != "/api/command":
                self._send(404, "text/plain", b"not found")
                return
            try:
                cmd = json.loads(raw.decode())
                ack = executive.submit_command(cmd)
                self._send(200, "application/json", json.dumps(ack).encode())
            except Exception as exc:
                self._send(400, "application/json", json.dumps({"ok": False, "error": str(exc)}).encode())

    class ReuseServer(ThreadingHTTPServer):
        allow_reuse_address = True

    server = ReuseServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://{host}:{port}"
