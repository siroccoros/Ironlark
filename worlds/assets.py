"""Hybrid lightweight visual assets & official Webots R2025a PROTO helpers.

Combines Webots R2025a's official PBR PROTOs and HDRI skyboxes (auto-cached locally on first load)
with compact procedural meshes (~80-tri displaced boulders, ~2k-tri perimeter ElevationGrid rims,
steel trusses, shipping containers, pallet racking, and timber mine arches) for rich, diverse
environments at smooth 60 FPS on integrated / entry-level GPUs.
"""
import math
import random

WEBOTS_CDN = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects"
# Official R2025a appearance textures, referenced directly when a material needs different PBR values
# than its PROTO allows (e.g. painted, non-metallic corrugated steel).
WEBOTS_TEXTURES = f"{WEBOTS_CDN}/appearances/protos/textures"

# Official Webots R2025a EXTERNPROTO catalog.
PROTO_URLS = {
    # HDRI skyboxes & matching directional/IBL lighting
    "TexturedBackground": f"{WEBOTS_CDN}/objects/backgrounds/protos/TexturedBackground.proto",
    # PBR material appearances
    "Asphalt": f"{WEBOTS_CDN}/appearances/protos/Asphalt.proto",
    "CorrugatedMetal": f"{WEBOTS_CDN}/appearances/protos/CorrugatedMetal.proto",
    "DryMud": f"{WEBOTS_CDN}/appearances/protos/DryMud.proto",
    "FormedConcrete": f"{WEBOTS_CDN}/appearances/protos/FormedConcrete.proto",
    "Grass": f"{WEBOTS_CDN}/appearances/protos/Grass.proto",
    "OldSteel": f"{WEBOTS_CDN}/appearances/protos/OldSteel.proto",
    "Pavement": f"{WEBOTS_CDN}/appearances/protos/Pavement.proto",
    "RedBricks": f"{WEBOTS_CDN}/appearances/protos/RedBricks.proto",
    "Roughcast": f"{WEBOTS_CDN}/appearances/protos/Roughcast.proto",
    "RoughConcrete": f"{WEBOTS_CDN}/appearances/protos/RoughConcrete.proto",
    "RoughOak": f"{WEBOTS_CDN}/appearances/protos/RoughOak.proto",
    "RustyMetal": f"{WEBOTS_CDN}/appearances/protos/RustyMetal.proto",
    "SandyGround": f"{WEBOTS_CDN}/appearances/protos/SandyGround.proto",
    "ThreadMetalPlate": f"{WEBOTS_CDN}/appearances/protos/ThreadMetalPlate.proto",
    # Foliage
    "Pine": f"{WEBOTS_CDN}/objects/trees/protos/Pine.proto",
    "Oak": f"{WEBOTS_CDN}/objects/trees/protos/Oak.proto",
    "Cypress": f"{WEBOTS_CDN}/objects/trees/protos/Cypress.proto",
    # Industrial, warehouse, urban & street props
    "OilBarrel": f"{WEBOTS_CDN}/objects/obstacles/protos/OilBarrel.proto",
    "WoodenPalletStack": f"{WEBOTS_CDN}/objects/factory/pallet/protos/WoodenPalletStack.proto",
    "WoodenBox": f"{WEBOTS_CDN}/objects/factory/containers/protos/WoodenBox.proto",
    "CardboardBox": f"{WEBOTS_CDN}/objects/factory/containers/protos/CardboardBox.proto",
    "PlasticCrate": f"{WEBOTS_CDN}/objects/factory/containers/protos/PlasticCrate.proto",
    "PipeSection": f"{WEBOTS_CDN}/objects/factory/pipes/protos/PipeSection.proto",
    "ConveyorBelt": f"{WEBOTS_CDN}/objects/factory/conveyors/protos/ConveyorBelt.proto",
    "WorkBarrier": f"{WEBOTS_CDN}/objects/traffic/protos/WorkBarrier.proto",
    "TrafficCone": f"{WEBOTS_CDN}/objects/traffic/protos/TrafficCone.proto",
    "StreetLight": f"{WEBOTS_CDN}/objects/traffic/protos/StreetLight.proto",
    "UrbanFence": f"{WEBOTS_CDN}/objects/street_furniture/protos/UrbanFence.proto",
    "DoubleFluorescentLamp": f"{WEBOTS_CDN}/objects/lights/protos/DoubleFluorescentLamp.proto",
    "EmergencyExitSign": f"{WEBOTS_CDN}/objects/hospital/protos/EmergencyExitSign.proto",
    "Warehouse": f"{WEBOTS_CDN}/objects/buildings/protos/Warehouse.proto",
    "Silo": f"{WEBOTS_CDN}/objects/buildings/protos/Silo.proto",
}


def extern_protos(names):
    """Render EXTERNPROTO declarations for IronlarkDrone plus the requested Webots R2025a PROTOs."""
    lines = ['EXTERNPROTO "../protos/IronlarkDrone.proto"']
    for name in sorted(set(names)):
        lines.append(f'EXTERNPROTO "{PROTO_URLS[name]}"')
    return "\n".join(lines)


def app_proto(name, scale=None, color=None, extra=None, ibl=None):
    """Build a Webots PBR appearance PROTO block (e.g. FormedConcrete, Grass, CorrugatedMetal).

    `ibl` scales the sky's image-based light: interiors take less of it, as a roof would.
    """
    parts = []
    if ibl is not None:
        parts.append(f"IBLStrength {ibl:.2f}")
    if color is not None:
        parts.append(f"colorOverride {color[0]:.2f} {color[1]:.2f} {color[2]:.2f}")
    if scale is not None:
        parts.append(f"textureTransform TextureTransform {{ scale {scale[0]:.2f} {scale[1]:.2f} }}")
    if extra:
        parts.append(extra)
    inner = " ".join(parts)
    return f"{name} {{ {inner} }}" if inner else f"{name} {{ }}"


def pbr(tex_url, texture=None, normal=None, color=(1.0, 1.0, 1.0), scale=(1, 1),
        roughness=0.9, metalness=0.0, emissive=None, emissive_intensity=1.6, roughness_map=None,
        ibl=None, normal_factor=None):
    """PBRAppearance; a texture name containing '://' is used as a full URL (official Webots assets)."""
    def url(name):
        return name if "://" in name else f"{tex_url}/{name}"

    parts = [f"baseColor {color[0]:.2f} {color[1]:.2f} {color[2]:.2f}",
             f"roughness {roughness:.2f}", f"metalness {metalness:.2f}"]
    if texture:
        parts.append(f'baseColorMap ImageTexture {{ url "{url(texture)}" }}')
    if roughness_map:
        parts.append(f'roughnessMap ImageTexture {{ url "{url(roughness_map)}" }}')
    if normal:
        parts.append(f'normalMap ImageTexture {{ url "{url(normal)}" }}')
    if normal_factor is not None:
        parts.append(f"normalMapFactor {normal_factor:.2f}")
    if ibl is not None:
        parts.append(f"IBLStrength {ibl:.2f}")
    if texture and scale != (1, 1):
        parts.append(f"textureTransform TextureTransform {{ scale {scale[0]} {scale[1]} }}")
    if emissive:
        parts.append(f"emissiveColor {emissive[0]:.2f} {emissive[1]:.2f} {emissive[2]:.2f} "
                     f"emissiveIntensity {emissive_intensity:.2f}")
    return "PBRAppearance { " + " ".join(parts) + " }"


def box_solid(name, center, size, appearance, yaw=0.0, collide=True, cast_shadows=True):
    """Box Solid; cast_shadows=False for flat decals and liners, whose shadow volume would only cost
    stencil fill and self-shadow at grazing sun angles."""
    cx, cy, cz = center
    sx, sy, sz = size
    rot = f"\n  rotation 0 0 1 {yaw:.3f}" if abs(yaw) > 1e-6 else ""
    bo = f"\n  boundingObject Box {{ size {sx:.3f} {sy:.3f} {sz:.3f} }}" if collide else ""
    shadow = "" if cast_shadows else "\n      castShadows FALSE"
    return f"""Solid {{
  translation {cx:.3f} {cy:.3f} {cz:.3f}{rot}
  name "{name}"
  children [
    Shape {{
      appearance {appearance}
      geometry Box {{ size {sx:.3f} {sy:.3f} {sz:.3f} }}{shadow}
    }}
  ]{bo}
}}"""


def cyl_solid(name, center, radius, height, appearance, rot=None, collide=True, subdiv=14):
    cx, cy, cz = center
    r_line = f"\n  rotation {rot[0]:.3f} {rot[1]:.3f} {rot[2]:.3f} {rot[3]:.3f}" if rot else ""
    bo = f"\n  boundingObject Cylinder {{ height {height:.2f} radius {radius:.2f} }}" if collide else ""
    return f"""Solid {{
  translation {cx:.2f} {cy:.2f} {cz:.2f}{r_line}
  name "{name}"
  children [
    Shape {{
      appearance {appearance}
      geometry Cylinder {{ height {height:.2f} radius {radius:.2f} subdivision {subdiv} }}
    }}
  ]{bo}
}}"""


def _icosphere(levels):
    """Return (vertices, faces) of an icosahedron subdivided `levels` times onto the unit sphere."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    raw = [
        (-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
        (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
        (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1),
    ]
    verts = []
    for x, y, z in raw:
        n = math.sqrt(x * x + y * y + z * z)
        verts.append((x / n, y / n, z / n))
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    for _ in range(levels):
        mid_cache = {}

        def midpoint(a, b):
            key = (a, b) if a < b else (b, a)
            if key not in mid_cache:
                va, vb = verts[a], verts[b]
                mx, my, mz = (va[0] + vb[0]) * 0.5, (va[1] + vb[1]) * 0.5, (va[2] + vb[2]) * 0.5
                n = math.sqrt(mx * mx + my * my + mz * mz)
                mid_cache[key] = len(verts)
                verts.append((mx / n, my / n, mz / n))
            return mid_cache[key]

        sub = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            sub.extend([(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)])
        faces = sub
    return verts, faces


_ICO_VERTS, _ICO_FACES = _icosphere(2)  # 162 vertices, 320 triangles


def faceted_boulder(name, center, radii, appearance, seed=0, yaw=0.0, collide=True, tile=4.0):
    """320-triangle angular rock crag (IndexedFaceSet) with flat-bottom embedding & fast Box collider.

    Texture coordinates are box-projected per face along the face's dominant axis, in metres / `tile`,
    so every face samples the texture at the same density (sandstone strata stay horizontal) and no
    face gets the degenerate, stretched mapping Webots generates when texCoord is absent.
    """
    rng = random.Random(seed)
    rx, ry, rz = radii
    # Deterministic harmonic facets + per-vertex jitter for a crisp natural silhouette; the same
    # low-frequency envelope as the 80-triangle crag, with a finer octave for chipped faces.
    p1, p2, p3, p4 = (rng.uniform(0, 6.28) for _ in range(4))
    pts = []
    for vx, vy, vz in _ICO_VERTS:
        disp = (0.86
                + 0.14 * math.sin(vx * 3.1 + p1) * math.cos(vy * 2.7 + p2)
                + 0.10 * math.cos(vz * 3.7 + p3)
                + 0.035 * math.sin(vx * 9.0 + vy * 7.0 + vz * 8.0 + p4)
                + rng.uniform(-0.04, 0.04))
        # Flatten bottom slightly so the rock sits firmly embedded in the terrain.
        z_eff = max(-0.55, vz) * rz * disp
        pts.append((vx * rx * disp, vy * ry * disp, z_eff))

    tex, tex_idx = [], []
    for a, b, c in _ICO_FACES:
        pa, pb, pc = pts[a], pts[b], pts[c]
        ux, uy, uz = (pb[i] - pa[i] for i in range(3))
        wx, wy, wz = (pc[i] - pa[i] for i in range(3))
        n = (abs(uy * wz - uz * wy), abs(uz * wx - ux * wz), abs(ux * wy - uy * wx))
        axis = n.index(max(n))
        base = len(tex)
        for p in (pa, pb, pc):
            u, v = ((p[1], p[2]), (p[0], p[2]), (p[0], p[1]))[axis]
            tex.append((u / tile, v / tile))
        tex_idx.append(f"{base} {base + 1} {base + 2} -1")

    coord_str = ", ".join(f"{x:.3f} {y:.3f} {z:.3f}" for x, y, z in pts)
    idx_str = " ".join(f"{a} {b} {c} -1" for a, b, c in _ICO_FACES)
    tex_str = ", ".join(f"{u:.3f} {v:.3f}" for u, v in tex)
    cx, cy, cz = center
    rot = f"\n  rotation 0 0 1 {yaw:.3f}" if abs(yaw) > 1e-6 else ""
    bo = ""
    if collide:
        bo = f"\n  boundingObject Box {{ size {rx * 1.65:.2f} {ry * 1.65:.2f} {rz * 1.55:.2f} }}"
    return f"""Solid {{
  translation {cx:.2f} {cy:.2f} {cz:.2f}{rot}
  name "{name}"
  children [
    Shape {{
      appearance {appearance}
      geometry IndexedFaceSet {{
        coord Coordinate {{ point [ {coord_str} ] }}
        texCoord TextureCoordinate {{ point [ {tex_str} ] }}
        coordIndex [ {idx_str} ]
        texCoordIndex [ {" ".join(tex_idx)} ]
        creaseAngle 0.5
      }}
    }}
  ]{bo}
}}"""


def rim_height(x, y, inner_box, max_height, seed=7, style="alpine"):
    """Height (m) of the terrain_rim surface at (x, y): -0.08 inside inner_box (under the ground)."""
    x_min, x_max, y_min, y_max = inner_box
    rng = random.Random(seed + 101)
    ph1, ph2, ph3 = rng.uniform(0, 6.28), rng.uniform(0, 6.28), rng.uniform(0, 6.28)
    # Distance outside the playable rectangle.
    ox = max(0.0, x_min - x, x - x_max)
    oy = max(0.0, y_min - y, y - y_max)
    d = math.hypot(ox, oy)
    if d <= 0.5:
        return -0.08
    ramp = min(1.0, (d - 0.5) / (18.0 if style == "alpine" else 11.0))
    ramp = ramp * ramp * (3.0 - 2.0 * ramp)
    wave = (0.58
            + 0.24 * math.sin(x * 0.085 + ph1) * math.cos(y * 0.08 + ph2)
            + 0.16 * math.sin((x - y) * 0.13 + ph3)
            + 0.08 * math.cos((x + y) * 0.21))
    if style == "canyon":
        # Stepped mesa/cliff walls flanking north/south and towering behind the mine portal (+X).
        cliff = min(1.0, ramp * 1.45)
        stepped = round(cliff * 4.0) / 4.0 * 0.35 + cliff * 0.65
        return max_height * stepped * max(0.35, wave)
    return max_height * ramp * max(0.25, wave)


def terrain_rim(name, inner_box, outer_half, max_height, appearance, seed=7, style="alpine", nx=34, ny=34):
    """Sculpted perimeter ElevationGrid framing the flat playable arena with real hills/cliffs.

    Pure visual backdrop (no collision) kept slightly below z=0 inside inner_box so the arena floor stays
    perfectly flat while surrounding ridges, canyon cliffs, or mountain foothills enclose the horizon.
    """
    x0, y0 = -outer_half, -outer_half
    dx = (2.0 * outer_half) / (nx - 1)
    dy = (2.0 * outer_half) / (ny - 1)
    heights = [round(rim_height(x0 + i * dx, y0 + j * dy, inner_box, max_height, seed, style), 2)
               for j in range(ny) for i in range(nx)]
    h_str = " ".join(f"{h:.2f}" for h in heights)
    return f"""Solid {{
  translation {x0:.2f} {y0:.2f} 0
  name "{name}_rim"
  children [
    Shape {{
      appearance {appearance}
      geometry ElevationGrid {{
        xDimension {nx}
        yDimension {ny}
        xSpacing {dx:.3f}
        ySpacing {dy:.3f}
        height [ {h_str} ]
      }}
      castShadows FALSE
    }}
  ]
}}"""


def shipping_container(name, center, yaw, color_rgb, length=6.0, width=2.45, height=2.6):
    """Painted corrugated shipping container with corner frame trim & exact Box collider.

    Container steel is painted, so the corrugation keeps the official CorrugatedMetal relief and
    roughness but shades as a dielectric: bare-metal reflections of the ground read as black bands.
    """
    tex = f"{WEBOTS_TEXTURES}/corrugated_metal/corrugated_metal"
    body_app = (f"PBRAppearance {{ baseColor {color_rgb[0]:.2f} {color_rgb[1]:.2f} {color_rgb[2]:.2f} "
                f'baseColorMap ImageTexture {{ url "{tex}_base_color.jpg" }} '
                f'roughnessMap ImageTexture {{ url "{tex}_roughness.jpg" }} metalness 0 '
                f'normalMap ImageTexture {{ url "{tex}_normal.jpg" }} '
                f"textureTransform TextureTransform {{ scale 2.5 1.2 }} }}")
    trim = tuple(c * 0.62 for c in color_rgb)
    trim_app = pbr("", color=trim, roughness=0.62)
    cx, cy, cz = center
    rot = f"\n  rotation 0 0 1 {yaw:.3f}" if abs(yaw) > 1e-6 else ""
    hx, hy, hz = length / 2.0, width / 2.0, height / 2.0
    posts = []
    for sx in (-hx + 0.08, hx - 0.08):
        for sy in (-hy + 0.08, hy - 0.08):
            posts.append(f"""    Pose {{
      translation {sx:.2f} {sy:.2f} 0
      children [ Shape {{ appearance {trim_app} geometry Box {{ size 0.18 0.18 {height + 0.02:.2f} }} }} ]
    }}""")
    posts_txt = "\n".join(posts)
    return f"""Solid {{
  translation {cx:.2f} {cy:.2f} {cz:.2f}{rot}
  name "{name}"
  children [
    Shape {{
      appearance {body_app}
      geometry Box {{ size {length:.2f} {width:.2f} {height:.2f} }}
    }}
{posts_txt}
  ]
  boundingObject Box {{ size {length:.2f} {width:.2f} {height:.2f} }}
}}"""


def pallet_rack_bay(name, center, length=4.4, depth=1.35, height=3.8, yaw=0.0, tex_url="textures"):
    """Industrial steel warehouse pallet racking with orange crossbeams & stacked wooden/cardboard cargo."""
    # Powder-coated steel: a paint finish, so dielectric.
    upright_app = pbr(tex_url, color=(0.12, 0.27, 0.46), roughness=0.45)
    beam_app = pbr(tex_url, color=(0.90, 0.42, 0.10), roughness=0.42)
    crate_app = pbr(tex_url, "crate.png", scale=(1, 1), roughness=0.84)
    card_app = pbr(tex_url, "crate.png", color=(0.76, 0.62, 0.44), scale=(1, 1), roughness=0.90)
    cx, cy, cz = center
    rot = f"\n  rotation 0 0 1 {yaw:.3f}" if abs(yaw) > 1e-6 else ""
    hx, hy = length / 2.0, depth / 2.0
    parts = []
    # 6 vertical steel uprights (3 pairs along X)
    for px in (-hx + 0.06, 0.0, hx - 0.06):
        for py in (-hy + 0.06, hy - 0.06):
            parts.append(f"""    Pose {{
      translation {px:.2f} {py:.2f} {height / 2:.2f}
      children [ Shape {{ appearance {upright_app} geometry Box {{ size 0.10 0.10 {height:.2f} }} }} ]
    }}""")
    # Horizontal shelf beams + cargo boxes on 3 tiers
    for tier, z_beam in enumerate((0.16, 1.48, 2.76)):
        for py in (-hy + 0.05, hy - 0.05):
            parts.append(f"""    Pose {{
      translation 0 {py:.2f} {z_beam:.2f}
      children [ Shape {{ appearance {beam_app} geometry Box {{ size {length:.2f} 0.08 0.12 }} }} ]
    }}""")
        for slot, sx in enumerate((-length * 0.26, length * 0.26)):
            box_h = 0.96 if tier < 2 else 0.80
            app = crate_app if (tier + slot) % 2 == 0 else card_app
            parts.append(f"""    Pose {{
      translation {sx:.2f} 0 {z_beam + 0.06 + box_h / 2:.2f}
      children [ Shape {{ appearance {app} geometry Box {{ size {length * 0.40:.2f} {depth * 0.84:.2f} {box_h:.2f} }} }} ]
    }}""")
    parts_txt = "\n".join(parts)
    return f"""Solid {{
  translation {cx:.2f} {cy:.2f} {cz:.2f}{rot}
  name "{name}"
  children [
{parts_txt}
  ]
  boundingObject Pose {{
    translation 0 0 {height / 2:.2f}
    children [ Box {{ size {length:.2f} {depth:.2f} {height:.2f} }} ]
  }}
}}"""


def concrete_column(name, x, y, width=1.1, height=4.8, tex_url="textures", hazard_band=True):
    """Architectural reinforced concrete column with chamfered plinth, capital, and safety bumper."""
    col_app = app_proto("FormedConcrete", scale=(1.5, 3.0))
    haz_app = pbr(tex_url, "hazard.png", scale=(3, 1), roughness=0.72)
    cap_app = app_proto("FormedConcrete", scale=(2.0, 1.0))
    parts = [
        f"""    Pose {{
      translation 0 0 {height / 2:.2f}
      children [ Shape {{ appearance {col_app} geometry Box {{ size {width:.2f} {width:.2f} {height:.2f} }} }} ]
    }}""",
        f"""    Pose {{
      translation 0 0 0.22
      children [ Shape {{ appearance {cap_app} geometry Box {{ size {width + 0.16:.2f} {width + 0.16:.2f} 0.44 }} }} ]
    }}""",
        f"""    Pose {{
      translation 0 0 {height - 0.18:.2f}
      children [ Shape {{ appearance {cap_app} geometry Box {{ size {width + 0.24:.2f} {width + 0.24:.2f} 0.36 }} }} ]
    }}""",
    ]
    if hazard_band:
        parts.append(f"""    Pose {{
      translation 0 0 1.25
      children [ Shape {{ appearance {haz_app} geometry Box {{ size {width + 0.04:.2f} {width + 0.04:.2f} 0.45 }} }} ]
    }}""")
    parts_txt = "\n".join(parts)
    return f"""Solid {{
  translation {x:.2f} {y:.2f} 0
  name "{name}"
  children [
{parts_txt}
  ]
  boundingObject Pose {{
    translation 0 0 {height / 2:.2f}
    children [ Box {{ size {width + 0.16:.2f} {width + 0.16:.2f} {height:.2f} }} ]
  }}
}}"""


def roof_truss(name, x, y_span, z_height, tex_url="textures"):
    """Visual steel ceiling truss beam running across Y (no collision; stays above z=4.6 m)."""
    steel = app_proto("OldSteel", color=(0.30, 0.33, 0.36))
    return box_solid(name, (x, 0.0, z_height), (0.28, y_span, 0.32), steel, collide=False)


def roof_truss_y(name, y, x_span, z_height, tex_url="textures"):
    """Visual steel ceiling truss beam running across X at given Y."""
    steel = app_proto("OldSteel", color=(0.30, 0.33, 0.36))
    return box_solid(name, (0.0, y, z_height), (x_span, 0.28, 0.32), steel, collide=False)


def mine_arch(name, x, y_span, height, tex_url="textures"):
    """Heavy timber/steel mine support portal arch framing subterranean cavern drifts (visual ribs)."""
    timber = app_proto("RoughOak", scale=(1.0, 4.0), color=(0.55, 0.42, 0.30))
    hy = y_span / 2.0
    return f"""Solid {{
  translation {x:.2f} 0 0
  name "{name}_arch"
  children [
    Pose {{
      translation 0 {-hy + 0.22:.2f} {height / 2:.2f}
      children [ Shape {{ appearance {timber} geometry Box {{ size 0.38 0.38 {height:.2f} }} }} ]
    }}
    Pose {{
      translation 0 {hy - 0.22:.2f} {height / 2:.2f}
      children [ Shape {{ appearance {timber} geometry Box {{ size 0.38 0.38 {height:.2f} }} }} ]
    }}
    Pose {{
      translation 0 0 {height - 0.18:.2f}
      children [ Shape {{ appearance {timber} geometry Box {{ size 0.42 {y_span:.2f} 0.36 }} }} ]
    }}
    Pose {{
      translation 0 {-hy + 0.95:.2f} {height - 0.85:.2f}
      rotation 1 0 0 0.785
      children [ Shape {{ appearance {timber} geometry Box {{ size 0.30 1.65 0.26 }} }} ]
    }}
    Pose {{
      translation 0 {hy - 0.95:.2f} {height - 0.85:.2f}
      rotation 1 0 0 -0.785
      children [ Shape {{ appearance {timber} geometry Box {{ size 0.30 1.65 0.26 }} }} ]
    }}
  ]
}}"""


def building_block(name, center, size, wall_app, roof_app, yaw=0.0, tex_url="textures"):
    """Multi-story urban building massing with parapet roof trim & window bands (fast Box collider)."""
    cx, cy, cz = center
    sx, sy, sz = size
    rot = f"\n  rotation 0 0 1 {yaw:.3f}" if abs(yaw) > 1e-6 else ""
    # Glazing ribbon: mullion grid texture, glass as a smooth dielectric (reflects by Fresnel only).
    modules = max(2, round(max(sx, sy) / 1.25))
    win_app = pbr(tex_url, "glazing.png", roughness_map="glazing_roughness.png", scale=(modules, 1),
                  roughness=0.1)
    trim_app = app_proto("FormedConcrete", scale=(2.0, 1.0))
    bands = []
    # Recessed dark-glass horizontal window ribbon(s) for architectural scale.
    for fz in (2.3, 4.9):
        if fz + 0.6 < sz:
            bands.append(f"""    Pose {{
      translation 0 0 {fz - sz / 2:.2f}
      children [ Shape {{ appearance {win_app} geometry Box {{ size {sx + 0.04:.2f} {sy + 0.04:.2f} 1.05 }} }} ]
    }}""")
    bands_txt = "\n".join(bands)
    return f"""Solid {{
  translation {cx:.2f} {cy:.2f} {cz:.2f}{rot}
  name "{name}"
  children [
    Shape {{
      appearance {wall_app}
      geometry Box {{ size {sx:.2f} {sy:.2f} {sz:.2f} }}
    }}
    Pose {{
      translation 0 0 {sz / 2 + 0.14:.2f}
      children [ Shape {{ appearance {roof_app} geometry Box {{ size {sx + 0.26:.2f} {sy + 0.26:.2f} 0.28 }} }} ]
    }}
    Pose {{
      translation 0 0 {-sz / 2 + 0.20:.2f}
      children [ Shape {{ appearance {trim_app} geometry Box {{ size {sx + 0.14:.2f} {sy + 0.14:.2f} 0.40 }} }} ]
    }}
{bands_txt}
  ]
  boundingObject Box {{ size {sx + 0.14:.2f} {sy + 0.14:.2f} {sz + 0.28:.2f} }}
}}"""


def overhead_lamp(idx, x, y, z_ceil, color=(1.0, 0.94, 0.84), intensity=1.7, radius=13.0, tex_url="textures"):
    """Fluorescent/industrial overhead luminaire + warm PointLight."""
    lamp_app = pbr(tex_url, color=color, roughness=0.2, emissive=color, emissive_intensity=2.0)
    housing = pbr(tex_url, color=(0.24, 0.26, 0.28), roughness=0.6, metalness=0.4)
    return "\n".join([
        f"""Solid {{
  translation {x:.2f} {y:.2f} {z_ceil - 0.06:.2f}
  name "interior_lamp_{idx}"
  children [
    Shape {{ appearance {housing} geometry Box {{ size 0.44 2.35 0.08 }} }}
    Pose {{
      translation 0 0 -0.04
      children [ Shape {{ appearance {lamp_app} geometry Box {{ size 0.30 2.15 0.04 }} }} ]
    }}
  ]
}}""",
        f"""PointLight {{
  attenuation 0 0 1
  color {color[0]:.2f} {color[1]:.2f} {color[2]:.2f}
  intensity {intensity * 6.0:.2f}
  location {x:.2f} {y:.2f} {z_ceil - 0.32:.2f}
  radius {radius:.1f}
  castShadows FALSE
}}""",
    ])


def portal_beacons(x, y_half, height, tex_url="textures", color=(0.95, 0.45, 0.08),
                   facing="-x", prefix="entrance"):
    """Glowing doorway marker columns + hazard lintel header framing a GPS-denied portal.

    `x` is the wall plane coordinate along X (for facing '-x'/'+'x') or along Y (for facing '-y'/'+'y').
    """
    beacon_app = pbr(tex_url, color=color, roughness=0.30, emissive=color, emissive_intensity=1.9)
    haz_app = pbr(tex_url, "hazard.png", scale=(6, 1), roughness=0.72)
    if facing in ("-x", "+x"):
        s = -1.0 if facing == "-x" else 1.0
        px = x + s * 0.42
        return [
            box_solid(f"{prefix}_marker_l", (px, y_half, height / 2), (0.14, 0.22, height), beacon_app),
            box_solid(f"{prefix}_marker_r", (px, -y_half, height / 2), (0.14, 0.22, height), beacon_app),
            box_solid(f"{prefix}_hazard_hdr", (px + s * 0.02, 0.0, height + 0.18),
                      (0.16, y_half * 2.0 + 0.30, 0.34), haz_app, collide=False),
        ]
    s = -1.0 if facing == "-y" else 1.0
    py = x + s * 0.42
    return [
        box_solid(f"{prefix}_marker_l", (y_half, py, height / 2), (0.22, 0.14, height), beacon_app),
        box_solid(f"{prefix}_marker_r", (-y_half, py, height / 2), (0.22, 0.14, height), beacon_app),
        box_solid(f"{prefix}_hazard_hdr", (0.0, py + s * 0.02, height + 0.18),
                  (y_half * 2.0 + 0.30, 0.16, 0.34), haz_app, collide=False),
    ]
