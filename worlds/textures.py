"""Generate tileable procedural PBR PNGs (zero Pillow, zero Blender).

Writes PNGs using NumPy + stdlib zlib (~2 s cold, cached in .cache/textures-<VERSION>/): 1024 px surfaces
for what the camera meets up close (rock, sandstone, basalt, asphalt, slab concrete), 512 px for props.
Every map tiles seamlessly; colours are sRGB-encoded albedo in the range real materials have once linearised (asphalt ~0.09,
concrete ~0.33, basalt ~0.09, granite ~0.2), so lighting rather than paint sets the brightness.
"""
import hashlib
import struct
import zlib
from pathlib import Path

import numpy as np

VERSION = "v3"


def _write_png(path, rgb_u8):
    """Write an (H, W, 3) uint8 array as an RGB PNG using stdlib zlib only."""
    h, w, _ = rgb_u8.shape
    raw = b"".join(b"\x00" + rgb_u8[y].tobytes() for y in range(h))

    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, level=6))
           + chunk(b"IEND", b""))
    path.write_bytes(png)


def _tile_noise(size, octaves=(4, 8, 16, 32), seed=7, gain=0.5):
    """Smooth periodic 2D fBm value noise in [0, 1], seamless across tile edges."""
    rng = np.random.default_rng(seed)
    acc = np.zeros((size, size), dtype=np.float32)
    amp, total = 1.0, 0.0
    t = np.arange(size, dtype=np.float32)
    for freq in octaves:
        grid = rng.random((freq, freq), dtype=np.float32)
        f = t * freq / size
        i0 = np.floor(f).astype(np.int32) % freq
        i1 = (i0 + 1) % freq
        s = f - np.floor(f)
        s = s * s * (3.0 - 2.0 * s)
        # Separable bilinear lookup: rows (y) then columns (x).
        g0 = grid[i0][:, i0] * (1 - s)[None, :] + grid[i0][:, i1] * s[None, :]
        g1 = grid[i1][:, i0] * (1 - s)[None, :] + grid[i1][:, i1] * s[None, :]
        acc += amp * (g0 * (1 - s)[:, None] + g1 * s[:, None])
        total += amp
        amp *= gain
    return acc / total


def _ridged(size, octaves, seed, width=0.03):
    """Periodic crack mask in [0, 1]: thin lines where the fBm crosses its median, `width` sets thickness."""
    n = _tile_noise(size, octaves, seed=seed)
    return np.clip(1.0 - np.abs(n - 0.5) / width, 0.0, 1.0)


def _normal_from_height(height, strength=3.0):
    """Tangent-space normal map (uint8 RGB) from a periodic 2D height field."""
    dx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * strength
    dy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * strength
    n = np.dstack([-dx, dy, np.ones_like(height)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return np.clip((n * 0.5 + 0.5) * 255.0, 0, 255).astype(np.uint8)


def _to_u8(arr):
    return np.clip(arr * 255.0, 0, 255).astype(np.uint8)


def _gray(v, size):
    return _to_u8(np.repeat(np.clip(v, 0, 1)[..., None], 3, axis=2)) if np.ndim(v) else \
        np.full((size, size, 3), int(v * 255), dtype=np.uint8)



def ensure_textures(cache_dir):
    """Generate and cache all procedural PBR PNGs; return the texture directory path."""
    out = Path(cache_dir) / f"textures-{VERSION}"
    marker = out / ".ready"
    sig = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:12]
    if marker.is_file() and marker.read_text().strip() == sig:
        return out
    out.mkdir(parents=True, exist_ok=True)
    L, M = 1024, 512
    yy, xx = np.mgrid[0:L, 0:L]

    # 1. Weathered granite boulder: warm-grey grain, darker crevices, ochre & sage lichen patches.
    g1 = _tile_noise(L, (4, 8, 16, 32, 64, 128), seed=71)
    g2 = _tile_noise(L, (8, 16, 32, 64, 128, 256), seed=72, gain=0.6)
    cracks = _ridged(L, (4, 8, 16, 32), seed=73, width=0.012)
    lichen = np.clip((_tile_noise(L, (8, 16, 32, 64, 128), seed=74) - 0.64) * 6.0, 0, 1)
    lichen2 = np.clip((_tile_noise(L, (6, 12, 24, 48, 96), seed=75) - 0.66) * 6.0, 0, 1)
    rock = np.array([0.47, 0.45, 0.41]) * (0.78 + 0.34 * g1[..., None]) * (0.86 + 0.22 * g2[..., None])
    rock = rock * (1 - lichen[..., None] * 0.55) + np.array([0.56, 0.50, 0.30]) * lichen[..., None] * 0.55
    rock = rock * (1 - lichen2[..., None] * 0.45) + np.array([0.42, 0.47, 0.38]) * lichen2[..., None] * 0.45
    grains = _tile_noise(L, (256, 512), seed=76, gain=0.9)
    rock *= (1.0 - 0.32 * cracks[..., None]) * (0.86 + 0.28 * grains[..., None])
    _write_png(out / "rock.png", _to_u8(rock))
    _write_png(out / "rock_normal.png",
               _normal_from_height(g1 * 0.6 + g2 * 0.4 - cracks * 0.35, strength=5.0))

    # 2. Red-rock sandstone: horizontal strata along v (assets map v to world height), cross-bedding,
    #    pale bleached bands and dark desert-varnish streaks running down the face.
    sn = _tile_noise(L, (4, 8, 16, 32, 64), seed=91)
    sn2 = _tile_noise(L, (16, 32, 64, 128, 256), seed=92, gain=0.6)
    warp = _tile_noise(L, (2, 4, 8), seed=93)
    band_line = _tile_noise(L, (24, 48, 96, 192), seed=97, gain=0.65)[:, 0]
    rows = (yy + (warp - 0.5) * 60.0 + (sn - 0.5) * 18.0).astype(np.int32) % L
    strata = np.clip((band_line[rows] - 0.32) * 3.2, 0, 1)
    fine = _tile_noise(L, (128, 256, 512), seed=98, gain=0.8)
    pal_lo = np.array([0.56, 0.34, 0.22])
    pal_hi = np.array([0.67, 0.47, 0.33])
    bleach = np.clip((_tile_noise(L, (2, 4, 8), seed=94) - 0.55) * 4, 0, 1) * np.clip((strata - 0.6) * 3, 0, 1)
    sand = pal_lo * (1 - strata[..., None]) + pal_hi * strata[..., None]
    sand = sand * (0.84 + 0.24 * fine[..., None]) * (0.88 + 0.22 * sn2[..., None])
    sand = sand * (1 - bleach[..., None] * 0.5) + np.array([0.80, 0.66, 0.50]) * bleach[..., None] * 0.5
    # Varnish streaks: x-periodic noise stretched along y.
    streak_cols = _tile_noise(L, (16, 32, 64, 128), seed=95)[0]
    streak = np.clip((streak_cols - 0.55) * 4.0, 0, 1)[None, :] * (0.4 + 0.6 * _tile_noise(L, (2, 4), seed=96))
    sand *= (1.0 - 0.15 * streak[..., None])
    _write_png(out / "sandstone.png", _to_u8(sand))
    _write_png(out / "sandstone_normal.png",
               _normal_from_height(sn2 * 0.5 + strata * 0.35 + fine * 0.15, strength=4.0))

    # 3. Mine basalt: dark blue-grey with fine crystalline speckle and pale calcite veins.
    bn = _tile_noise(L, (4, 8, 16, 32, 64), seed=101)
    bs = _tile_noise(L, (64, 128, 256), seed=102, gain=0.7)
    veins = _ridged(L, (3, 6, 12, 24), seed=103, width=0.008)
    basalt = np.array([0.34, 0.33, 0.34]) * (0.74 + 0.40 * bn[..., None]) * (0.85 + 0.30 * bs[..., None])
    basalt = basalt * (1 - veins[..., None] * 0.22) + np.array([0.52, 0.50, 0.47]) * veins[..., None] * 0.22
    _write_png(out / "basalt.png", _to_u8(basalt))
    _write_png(out / "basalt_normal.png", _normal_from_height(bn * 0.7 + bs * 0.3 - veins * 0.2, strength=5.0))

    # 4. Aged asphalt: light-grey aggregate in a worn binder, oil stains, sealed cracks.
    an = _tile_noise(L, (4, 8, 16, 32), seed=111)
    agg = _tile_noise(L, (128, 256, 512), seed=112, gain=0.8)
    stones = np.clip((agg - 0.62) * 5.0, 0, 1)
    oil = np.clip((_tile_noise(L, (3, 6, 12), seed=113) - 0.62) * 3.5, 0, 1)
    seal = _ridged(L, (3, 6, 12, 24), seed=114, width=0.006) > 0.3
    asph = np.full((L, L, 3), 0.30, dtype=np.float32) * (0.85 + 0.30 * an[..., None])
    asph += stones[..., None] * np.array([0.16, 0.16, 0.15])
    asph *= (1.0 - 0.35 * oil[..., None])
    asph[seal] = asph[seal] * 0.72
    _write_png(out / "asphalt.png", _to_u8(asph))
    _write_png(out / "asphalt_normal.png",
               _normal_from_height(agg * 0.7 + an * 0.3 - seal.astype(np.float32) * 0.2, strength=3.5))

    # 5. Power-floated slab concrete (interior floors): saw-cut joints on a 2 m grid (tile = 4 m),
    #    trowel mottling, faint tyre & scuff marks.
    cn = _tile_noise(L, (4, 8, 16, 32), seed=121)
    cf = _tile_noise(L, (64, 128, 256), seed=122, gain=0.7)
    scuff = np.clip((_tile_noise(L, (2, 4, 8, 16), seed=123) - 0.6) * 3.0, 0, 1)
    joint = ((xx % (L // 2)) < 3) | ((yy % (L // 2)) < 3)
    slab = np.array([0.63, 0.62, 0.59]) * (0.86 + 0.20 * cn[..., None]) * (0.95 + 0.08 * cf[..., None])
    slab *= (1.0 - 0.18 * scuff[..., None])
    slab[joint] *= 0.55
    _write_png(out / "concrete.png", _to_u8(slab))
    _write_png(out / "concrete_normal.png",
               _normal_from_height(cf * 0.4 - joint.astype(np.float32) * 0.6, strength=2.0))

    # 6. Facade glazing ribbon: mullion grid (one module per texture tile), light anodised frames,
    #    deep blue-grey glass with a soft sky-to-ground gradient; separate roughness map.
    gy, gx = np.mgrid[0:M, 0:M] / M
    frame = (gx < 0.05) | (gx > 0.95) | (gy < 0.09) | (gy > 0.91) | (np.abs(gy - 0.5) < 0.025)
    glass = np.array([0.10, 0.14, 0.18]) + (1 - gy)[..., None] * np.array([0.10, 0.12, 0.14])
    glass += _tile_noise(M, (2, 4), seed=131)[..., None] * 0.03
    glaz = np.where(frame[..., None], np.array([0.62, 0.63, 0.62]), glass)
    _write_png(out / "glazing.png", _to_u8(glaz))
    _write_png(out / "glazing_roughness.png", _gray(np.where(frame, 0.45, 0.06), M))

    # 7. Cargo crate (pine boards, darker frame & diagonal brace) and kraft cardboard.
    crn = _tile_noise(M, (8, 16, 32, 64), seed=8)
    cy, cx = np.mgrid[0:M, 0:M]
    grain = 0.5 + 0.5 * np.sin(cy * 0.35 + _tile_noise(M, (4, 8), seed=81) * 12.0)
    boards = (cy % (M // 4)) < 3
    crate = np.array([0.58, 0.44, 0.27]) * (0.84 + 0.14 * grain[..., None]) * (0.9 + 0.2 * crn[..., None])
    frame = (cx < M * 0.08) | (cx > M * 0.92) | (cy < M * 0.08) | (cy > M * 0.92) | (np.abs(cx - cy) < M * 0.04)
    crate[frame] *= 0.78
    crate[boards & ~frame] *= 0.6
    _write_png(out / "crate.png", _to_u8(crate))

    # 8. Helipad: painted concrete disc, white perimeter ring, amber 'H' with thin white keyline.
    hy, hx = np.mgrid[0:M, 0:M]
    dx, dy = (hx - M / 2) / (M / 2), (hy - M / 2) / (M / 2)
    r = np.sqrt(dx * dx + dy * dy)
    pn = _tile_noise(M, (8, 16, 32, 64), seed=141)
    pad = np.array([0.30, 0.31, 0.32]) * (0.88 + 0.22 * pn[..., None])
    h_shape = ((np.abs(dx + 0.26) < 0.08) & (np.abs(dy) < 0.44)) | ((np.abs(dx - 0.26) < 0.08) & (np.abs(dy) < 0.44)) \
        | ((np.abs(dx) < 0.26) & (np.abs(dy) < 0.08))
    h_line = ((np.abs(dx + 0.26) < 0.10) & (np.abs(dy) < 0.46)) | ((np.abs(dx - 0.26) < 0.10) & (np.abs(dy) < 0.46)) \
        | ((np.abs(dx) < 0.28) & (np.abs(dy) < 0.10))
    pad[(r > 0.80) & (r < 0.88)] = [0.86, 0.86, 0.84]
    pad[h_line] = [0.86, 0.86, 0.84]
    pad[h_shape] = [0.92, 0.66, 0.14]
    _write_png(out / "helipad.png", _to_u8(pad * (0.94 + 0.08 * pn[..., None])))

    # 9. Yellow/black hazard stripes with chipped paint.
    stripe = ((hx + hy) % 128) < 64
    haz = np.where(stripe[..., None], np.array([0.86, 0.66, 0.10]), np.array([0.10, 0.10, 0.11])).astype(np.float32)
    chips = _tile_noise(M, (32, 64, 128), seed=151) > 0.7
    haz[chips] = haz[chips] * 0.6 + np.array([0.45, 0.45, 0.43]) * 0.4
    _write_png(out / "hazard.png", _to_u8(haz * (0.92 + 0.12 * crn[..., None])))

    marker.write_text(sig + "\n")
    return out
