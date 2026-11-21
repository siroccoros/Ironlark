"""Generate the drone PROTO (from config/sensors.json) and diverse hybrid Webots R2025a worlds.

Flagship unified open world (`mission`, default for `./ironlark run`):
  Single 84x84 m multi-biome hub radiating 4 distinct outdoor + GPS-denied interior sectors from (0, 0):
    - EAST  (+X): Alpine Forest -> Industrial Bunker
    - NORTH (+Y): Urban Brick/Stucco Compound -> Covered Concrete Parking Deck (column slalom)
    - SOUTH (-Y): Logistics Shipping Yard -> Multi-Aisle Distribution Warehouse (pallet racking)
    - WEST  (-X): Red-Rock Desert Canyon -> Subterranean Basalt Mine (timber arches & rockfalls)

Standalone focused worlds (`warehouse`, `canyon`, `compound`, `forest`, `empty`) are also available via `--world`.
"""
import math
import random

from worlds import assets

# Iris motor layout: (x, y, z) thrust point, spin sign (+1 CCW, -1 CW), upstream propeller mesh.
MOTORS = [
    ((0.13, -0.22, 0.023), 1, "iris_prop_ccw.dae"),
    ((-0.13, 0.20, 0.023), 1, "iris_prop_ccw.dae"),
    ((0.13, 0.22, 0.023), -1, "iris_prop_cw.dae"),
    ((-0.13, -0.20, 0.023), -1, "iris_prop_cw.dae"),
]

MISSION_WORLDS = ("mission", "warehouse", "canyon", "compound")


def _pose(sensor):
    t, r = sensor["translation"], sensor["rotation"]
    return f"translation {t[0]} {t[1]} {t[2]}\n        rotation {r[0]} {r[1]} {r[2]} {r[3]}"


def drone_proto(config, meshes_url):
    """Build IronlarkDrone.proto: ArduPilot Iris flight model + config/sensors.json sensor rig."""
    imu, lidar, camera = config["imu"], config["lidar"], config["camera"]
    propellers = []
    for i, ((x, y, z), sign, mesh) in enumerate(MOTORS, 1):
        color = "0.95 0.42 0.08" if x > 0 else "0.14 0.16 0.19"
        propellers.append(f"""Propeller {{
        shaftAxis 0 0 1
        centerOfThrust {x} {y} {z}
        thrustConstants {0.0012 * sign} 0
        torqueConstants 5.0e-04 0
        device RotationalMotor {{
          name "m{i}_motor"
          maxVelocity 100
          maxTorque 30
          multiplier {sign}
        }}
        slowHelix Solid {{
          translation {x} {y} {z}
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor {color} roughness 0.35 metalness 0.2 }}
              geometry Mesh {{ url "{meshes_url}/{mesh}" }}
            }}
          ]
        }}
      }}""")
    props = "\n      ".join(propellers)
    lt, ct = lidar["translation"], camera["translation"]
    # Keep sensor housings strictly inside each sensor's near plane (0.05 m) so they never self-occlude.
    return f"""#VRML_SIM R2025a utf8
# Generated from config/sensors.json (worlds/generate.py): Iris dynamics + Ironlark sensor rig.

PROTO IronlarkDrone [
  field  SFVec3f     translation     0 0 0
  field  SFRotation  rotation        0 0 1 0
  field  SFString    name            "Ironlark"
  field  SFString    controller      "<none>"
  field  MFString    controllerArgs  []
  field  SFBool      supervisor      FALSE
  field  SFBool      synchronization TRUE
]
{{
  Robot {{
    translation IS translation
    rotation IS rotation
    controller IS controller
    controllerArgs IS controllerArgs
    supervisor IS supervisor
    synchronization IS synchronization
    name IS name
    children [
      Shape {{
        appearance PBRAppearance {{ baseColor 0.10 0.11 0.13 roughness 0.5 metalness 0.25 }}
        geometry Mesh {{ url "{meshes_url}/iris.dae" }}
        castShadows FALSE
      }}
      Pose {{
        translation {lt[0]} {lt[1]} {lt[2] - 0.025}
        children [
          Shape {{
            appearance PBRAppearance {{ baseColor 0.06 0.06 0.07 roughness 0.3 metalness 0.6 }}
            geometry Cylinder {{ height 0.05 radius 0.03 }}
          }}
        ]
      }}
      Pose {{
        translation {ct[0] - 0.025} {ct[1]} {ct[2]}
        children [
          Shape {{
            appearance PBRAppearance {{ baseColor 0.1 0.1 0.12 roughness 0.4 metalness 0.4 }}
            geometry Box {{ size 0.025 0.036 0.028 }}
          }}
        ]
      }}
      {props}
      Accelerometer {{ }}
      GPS {{ }}
      Gyro {{ }}
      InertialUnit {{ }}
      Pose {{
        {_pose(imu)}
        children [
          Accelerometer {{ name "{imu['name']} accelerometer" }}
          Gyro {{ name "{imu['name']} gyro" }}
          InertialUnit {{ name "{imu['name']} inertial unit" }}
        ]
      }}
      Lidar {{
        {_pose(lidar)}
        name "{lidar['name']}"
        horizontalResolution {lidar['horizontal_resolution']}
        fieldOfView {lidar['field_of_view']}
        verticalFieldOfView {lidar['vertical_field_of_view']}
        tiltAngle {lidar['tilt_angle']}
        numberOfLayers {lidar['render_layers']}
        near 0.05
        minRange {lidar['min_range']}
        maxRange {lidar['max_range']}
        noise {lidar['noise']}
        projection "cylindrical"
        type "fixed"
      }}
      Camera {{
        {_pose(camera)}
        name "{camera['name']}"
        fieldOfView {camera['field_of_view']}
        width {camera['width']}
        height {camera['height']}
        near {camera['near']}
      }}
    ]
    boundingObject Box {{ size 0.47 0.47 0.11 }}
    physics Physics {{
      density -1
      mass 1.5
      centerOfMass [ 0 0 0 ]
      inertiaMatrix [ 2.9125e-02 2.9125e-02 5.5225e-02 0 0 0 ]
    }}
  }}
}}
"""


# Art direction shared by every world: a clear, cloud-dotted late-morning sky over open country. The
# HDRI's lower hemisphere is sunlit grass, so down-facing surfaces and reflections pick up green-gold
# bounce light instead of the near-black floor of the "mountains" sky (black window bands, black
# ceilings, black reflections on metal).
SKY = "noon_cloudy_countryside"
# Sun: from the HDRI's glare (north, a little west of it, elevation ~45 deg), warm white. Raking enough
# to model rocks and facades; the sky's irradiance keeps shadows readable rather than black.
SUN = {"direction": (0.27, -0.65, -0.71), "color": (1.0, 0.95, 0.86), "intensity": 2.6}
FOG = {"color": (0.74, 0.80, 0.86), "range": 240.0}
# Ground quads are grouped per material per cell of this grid (m): the interiors (|x| or |y| in
# 20..41) each fall in one cell, so a lamp's light pass re-shades its own room's floor, not the world.
GROUND_CELLS = (-48.0, -20.0, 20.0, 48.0)
# Interior lamps reach 13 m: across their room (1/d^2 falloff leaves <8% at the edge) but not the
# outdoor scene, so each lamp's extra lighting pass covers the room's surfaces only.
LAMP_RADIUS = 13.0


def _ground(extent, base, layers):
    """Flat ground at z=0 as world-UV quads partitioned among materials: no two surfaces overlap.

    `base` is (appearance, tile_m); `layers` is a list of (appearance, tile_m, x0, x1, y0, y1),
    later entries painting over earlier ones. The partition replaces stacked slabs millimetres apart,
    which z-fight (coplanar ones flicker between materials as the camera moves). Quads sample
    textures in world metres, so a material continues seamlessly across quads. One Shape per
    material per GROUND_CELLS cell keeps per-light passes local; collision stays one Box. Ground
    never casts shadows: it only receives them.
    """
    mats = [base] + [(app, tile) for app, tile, *_ in layers]
    rects = [(x0, x1, y0, y1) for _, _, x0, x1, y0, y1 in layers]

    def clip(v):
        return min(extent, max(-extent, v))

    xs = sorted({-extent, extent, *(clip(v) for r in rects for v in r[:2])})
    ys = sorted({-extent, extent, *(clip(v) for r in rects for v in r[2:])})
    xs, ys = sorted({*xs, *GROUND_CELLS}), sorted({*ys, *GROUND_CELLS})

    def owner(x, y):
        for k in range(len(rects) - 1, -1, -1):
            x0, x1, y0, y1 = rects[k]
            if x0 <= x <= x1 and y0 <= y <= y1:
                return k + 1
        return 0

    def cell(v):
        return sum(v >= b for b in GROUND_CELLS)

    # Row spans of equal material, then merge identical spans of consecutive rows.
    spans = []
    for j in range(len(ys) - 1):
        cy = 0.5 * (ys[j] + ys[j + 1])
        i = 0
        while i < len(xs) - 1:
            k = owner(0.5 * (xs[i] + xs[i + 1]), cy)
            i1 = i + 1
            while i1 < len(xs) - 1 and owner(0.5 * (xs[i1] + xs[i1 + 1]), cy) == k and \
                    cell(xs[i1]) == cell(xs[i]):
                i1 += 1
            spans.append([xs[i], xs[i1], ys[j], ys[j + 1], k])
            i = i1
    merged = []
    for sp in spans:
        for m in merged:
            if m[0] == sp[0] and m[1] == sp[1] and m[4] == sp[4] and m[3] == sp[2] and cell(m[2]) == cell(sp[2]):
                m[3] = sp[3]
                break
        else:
            merged.append(sp)
    groups = {}
    outer = (0, len(GROUND_CELLS))
    for x0, x1, y0, y1, k in merged:
        cx, cy = cell(x0), cell(y0)
        key = (k, -1, -1) if cx in outer or cy in outer else (k, cx, cy)  # one group beyond the cells
        groups.setdefault(key, []).append((x0, x1, y0, y1))
    shapes = []
    for (k, _, _), quads in sorted(groups.items(), key=lambda kv: (kv[0][0], str(kv[0][1:]))):
        app, tile = mats[k]
        pts, uvs, idx = [], [], []
        for x0, x1, y0, y1 in quads:
            n = len(pts)
            for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                pts.append(f"{x:.3f} {y:.3f} 0")
                uvs.append(f"{x / tile:.4f} {y / tile:.4f}")
            idx.append(f"{n} {n + 1} {n + 2} {n + 3} -1")
        shapes.append(f"""    Shape {{
      appearance {app}
      geometry IndexedFaceSet {{
        coord Coordinate {{ point [ {", ".join(pts)} ] }}
        texCoord TextureCoordinate {{ point [ {", ".join(uvs)} ] }}
        coordIndex [ {" ".join(idx)} ]
      }}
      castShadows FALSE
    }}""")
    body = "\n".join(shapes)
    return f"""Solid {{
  children [
{body}
  ]
  name "ground"
  boundingObject Pose {{
    translation 0 0 -0.1
    children [ Box {{ size {2 * extent:.0f} {2 * extent:.0f} 0.2 }} ]
  }}
}}"""


def _world_header(title, seed, ground, tex_url, protos, sky=SKY, sun=SUN, fog=FOG, luminosity=1.0,
                  exposure=1.0, shadows=True):
    """Shared VRML header: official Webots R2025a HDRI TexturedBackground, a sun matched to it, haze
    fog, the partitioned ground (see _ground) & helipad.

    Viewpoint near is 0.1 m (default 0.05): the chase camera stays >= 0.9 m from what it frames, and
    the doubled depth precision keeps 1 cm ground markings stable out past the fog.
    """
    cast = "TRUE" if shadows else "FALSE"
    d, c = sun["direction"], sun["color"]
    pad = assets.pbr(tex_url, "helipad.png", roughness=0.7)
    proto_hdr = assets.extern_protos(["TexturedBackground", *protos])
    return f"""#VRML_SIM R2025a utf8

{proto_hdr}

WorldInfo {{
  title "{title}"
  basicTimeStep 2
  FPS 60
  randomSeed {seed}
}}
DEF IRONLARK_VIEW Viewpoint {{
  orientation -0.174545 0.356882 0.917698 0.979312
  position -6.5 -3.5 3.6
  near 0.1
  exposure {exposure:.2f}
  ambientOcclusionRadius 1.2
}}
TexturedBackground {{
  texture "{sky}"
  luminosity {luminosity:.2f}
}}
DirectionalLight {{
  ambientIntensity 1
  direction {d[0]:.3f} {d[1]:.3f} {d[2]:.3f}
  color {c[0]:.2f} {c[1]:.2f} {c[2]:.2f}
  intensity {sun["intensity"]:.2f}
  castShadows {cast}
}}
Fog {{
  color {fog["color"][0]:.2f} {fog["color"][1]:.2f} {fog["color"][2]:.2f}
  visibilityRange {fog["range"]:.0f}
  fogType "EXPONENTIAL2"
}}
{ground}
Solid {{
  translation 0 0 0.012
  children [
    Shape {{
      appearance {pad}
      geometry Cylinder {{ height 0.008 radius 1.25 subdivision 48 }}
      castShadows FALSE
    }}
  ]
  name "home pad"
}}"""


def _world_footer():
    return """DEF IRONLARK_DRONE IronlarkDrone {
  translation 0 0 0.09
  name "Ironlark"
  controller "flight_bridge"
}
Robot {
  name "flight evaluator"
  supervisor TRUE
  controller "flight_observer"
}
"""


def _webots_tree(kind, idx, x, y, yaw, z=0.0):
    """Place an official Webots R2025a tree PROTO (Pine, Cypress, or Oak) with full collision."""
    return f"""{kind} {{
  translation {x:.2f} {y:.2f} {z:.2f}
  rotation 0 0 1 {yaw:.3f}
  name "tree_{idx}"
}}"""


def _enclosure_shell_dir(x0, x1, y0, y1, height, door_half, facing, wall_app, roof_app, floor,
                         ceiling_app, lintel_z=4.75, prefix="cave"):
    """Build a watertight 4-wall + roof GPS-denied building shell with portal facing -x/+x/-y/+y.

    Returns (solids, floor_layer): the floor is a _ground layer (appearance, tile_m) over the footprint,
    flush with the ground rather than a slab stacked on it. A 2 cm ceiling liner under the roof carries
    the interior ceiling material, which takes less sky light than the roof's outside.
    """
    dx, dy = x1 - x0, y1 - y0
    xc, yc = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    lintel_h = max(0.35, height - lintel_z + 0.25)
    solids = [
        assets.box_solid(f"{prefix}_roof", (xc, yc, height + 0.2), (dx + 1.2, dy + 1.4, 0.4), roof_app),
        assets.box_solid(f"{prefix}_ceiling", (xc, yc, height - 0.01), (dx, dy, 0.02), ceiling_app,
                         collide=False, cast_shadows=False),
    ]
    if facing in ("-x", "+x"):
        x_door = x0 if facing == "-x" else x1
        x_back = (x1 + 0.2) if facing == "-x" else (x0 - 0.2)
        side_len = 0.5 * dy - door_half
        side_off = door_half + 0.5 * side_len
        solids.extend([
            assets.box_solid(f"{prefix}_front_left", (x_door, yc + side_off, height / 2),
                             (0.8, side_len + 0.4, height), wall_app),
            assets.box_solid(f"{prefix}_front_right", (x_door, yc - side_off, height / 2),
                             (0.8, side_len + 0.4, height), wall_app),
            assets.box_solid(f"{prefix}_lintel", (x_door, yc, height - lintel_h / 2),
                             (0.85, door_half * 2.0 + 0.4, lintel_h), wall_app),
            assets.box_solid(f"{prefix}_wall_left", (xc, y1 + 0.2, height / 2), (dx + 0.8, 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_wall_right", (xc, y0 - 0.2, height / 2), (dx + 0.8, 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_wall_back", (x_back, yc, height / 2), (0.8, dy + 1.2, height), wall_app),
        ])
    else:
        y_door = y0 if facing == "-y" else y1
        y_back = (y1 + 0.2) if facing == "-y" else (y0 - 0.2)
        side_len = 0.5 * dx - door_half
        side_off = door_half + 0.5 * side_len
        solids.extend([
            assets.box_solid(f"{prefix}_front_left", (xc + side_off, y_door, height / 2),
                             (side_len + 0.4, 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_front_right", (xc - side_off, y_door, height / 2),
                             (side_len + 0.4, 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_lintel", (xc, y_door, height - lintel_h / 2),
                             (door_half * 2.0 + 0.4, 0.85, lintel_h), wall_app),
            assets.box_solid(f"{prefix}_wall_left", (x1 + 0.2, yc, height / 2), (0.8, dy + 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_wall_right", (x0 - 0.2, yc, height / 2), (0.8, dy + 0.8, height), wall_app),
            assets.box_solid(f"{prefix}_wall_back", (xc, y_back, height / 2), (dx + 1.2, 0.8, height), wall_app),
        ])
    return solids, (floor[0], floor[1], x0 - 0.2, x1 + 0.2, y0 - 0.2, y1 + 0.2)


def _enclosure_shell(x_front, x_back, y_half, height, door_half, wall_app, roof_app, floor, ceiling_app,
                     lintel_z=4.75, prefix="cave"):
    """Build a watertight 4-wall + roof GPS-denied building shell with front portal facing -x."""
    return _enclosure_shell_dir(x_front, x_back, -y_half, y_half, height, door_half, "-x",
                                wall_app, roof_app, floor, ceiling_app, lintel_z=lintel_z, prefix=prefix)


def forest_world(seed=7, tex_url="textures", trees=46, rocks=18, half_extent=48.0, clearing=7.5):
    """Standalone alpine forest with official Webots Pine/Cypress/Oak trees, faceted rocks & mountain rim."""
    rng = random.Random(seed)
    placed = []
    rock_app = assets.pbr(tex_url, "rock.png", "rock_normal.png", scale=(2, 2), roughness=0.9)
    grass = (assets.app_proto("Grass", color=(0.90, 0.93, 0.84), extra='type "mossy"'), 4.5)
    rim_app = assets.app_proto("Grass", scale=(24, 24), color=(0.86, 0.90, 0.80), extra='type "maintained"')

    def pick_xy(min_gap):
        for _ in range(80):
            x, y = rng.uniform(-half_extent, half_extent), rng.uniform(-half_extent, half_extent)
            if math.hypot(x, y) < clearing:
                continue
            if all(math.hypot(x - px, y - py) >= min_gap for px, py in placed):
                placed.append((x, y))
                return x, y
        return None

    obstacles = [
        assets.terrain_rim("alpine", (-half_extent - 2, half_extent + 2, -half_extent - 2, half_extent + 2),
                           half_extent + 28.0, 20.0, rim_app, seed=seed, style="alpine"),
    ]
    species = ("Pine", "Pine", "Cypress", "Oak")
    for i in range(trees):
        pos = pick_xy(6.4)
        if pos is None:
            break
        obstacles.append(_webots_tree(rng.choice(species), i, pos[0], pos[1], rng.uniform(0, math.tau)))
    for i in range(rocks):
        pos = pick_xy(4.5)
        if pos is None:
            break
        rx, ry, rz = rng.uniform(0.9, 2.0), rng.uniform(0.9, 2.0), rng.uniform(0.8, 2.0)
        obstacles.append(assets.faceted_boulder(f"rock_{i}", (pos[0], pos[1], rz * 0.45), (rx, ry, rz),
                                                rock_app, seed=seed * 31 + i, yaw=rng.uniform(0, math.tau)))

    protos = ["Grass", "Pine", "Cypress", "Oak"]
    return "\n".join([
        _world_header(f"Ironlark — alpine forest (seed {seed})", seed,
                      _ground(half_extent + 32.0, grass, []), tex_url, protos),
        *obstacles,
        _world_footer(),
    ])


def mission_world(seed=7, tex_url="textures"):
    """Unified 84x84 m 4-biome open world radiating from central helipad hub (0, 0):

      - EAST  (+X): Alpine Forest -> GPS-Denied Industrial Bunker (x in [21, 39.2])
      - NORTH (+Y): Urban Brick/Stucco Compound -> GPS-Denied Covered Parking Deck (y in [22, 40])
      - SOUTH (-Y): Logistics Shipping Yard -> GPS-Denied Multi-Aisle Warehouse (y in [-40, -22])
      - WEST  (-X): Red-Rock Desert Canyon -> GPS-Denied Subterranean Basalt Mine (x in [-40, -22])
    """
    rng = random.Random(seed)
    solids = []
    tree_idx = 0

    def add_tree(kind, x, y, yaw):
        nonlocal tree_idx
        solids.append(_webots_tree(kind, tree_idx, x, y, yaw))
        tree_idx += 1

    # Shared PBR appearances across the 4 biomes. Ground materials carry no textureTransform: _ground
    # maps them in world metres (tile size beside each), seamless across quads.
    grass = (assets.app_proto("Grass", color=(0.84, 0.86, 0.72), extra='type "maintained"'), 4.5)
    rim_app = assets.app_proto("Grass", scale=(30, 30), color=(0.80, 0.84, 0.72), extra='type "maintained"')
    paving = (assets.app_proto("Pavement", color=(0.96, 0.94, 0.90), extra='type "slate"'), 4.0)
    asphalt = (assets.pbr(tex_url, "asphalt.png", "asphalt_normal.png", roughness=0.86), 6.0)
    sand = (assets.app_proto("SandyGround", color=(1.0, 0.93, 0.84)), 5.0)
    dirt = (assets.app_proto("DryMud"), 3.0)
    slab = (assets.pbr(tex_url, "concrete.png", "concrete_normal.png", roughness=0.55, ibl=0.5), 4.0)
    mine_floor = (assets.app_proto("SandyGround", color=(0.62, 0.56, 0.50), ibl=0.45), 3.0)
    rock_app = assets.pbr(tex_url, "rock.png", "rock_normal.png", scale=(2, 2), roughness=0.9)
    sand_rock = assets.pbr(tex_url, "sandstone.png", "sandstone_normal.png", scale=(2, 2), roughness=0.92)
    basalt_wall = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 2), roughness=0.9)
    basalt_rock = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", color=(1.0, 0.9, 0.78), scale=(2, 2),
                             roughness=0.9, ibl=0.6)
    cavern_roof = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 6), roughness=0.95)
    cavern_ceiling = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 6), roughness=0.95, ibl=0.3)
    concrete_wall = assets.app_proto("FormedConcrete", scale=(4, 1.5))
    deck_wall = assets.app_proto("RoughConcrete", scale=(10, 3), color=(0.92, 0.91, 0.88))
    concrete_ceiling = assets.app_proto("RoughConcrete", scale=(5, 5), color=(0.86, 0.85, 0.82), ibl=0.3)
    metal_wall = assets.app_proto("CorrugatedMetal", scale=(8, 2), color=(0.80, 0.82, 0.84))
    metal_roof = assets.app_proto("CorrugatedMetal", scale=(5, 5), color=(0.56, 0.60, 0.63))
    metal_ceiling = assets.app_proto("CorrugatedMetal", scale=(5, 5), color=(0.78, 0.80, 0.82), ibl=0.3)
    brick_app = assets.app_proto("RedBricks", scale=(5, 2.5))
    stucco_app = assets.app_proto("Roughcast", scale=(4, 2), color=(0.93, 0.88, 0.80))
    deck_roof = assets.app_proto("FormedConcrete", scale=(4, 4))
    lane_app = assets.pbr(tex_url, color=(0.93, 0.74, 0.13), roughness=0.74)

    # Sculpted alpine mountain ridges enclosing the entire 84x84 m open world.
    rim_box = (-44.0, 44.0, -44.0, 44.0)
    solids.append(assets.terrain_rim("world", rim_box, 92.0, 22.0,
                                     rim_app, seed=seed, style="alpine", nx=56, ny=56))
    # A tree line along the foot of the ridges (north, east, south) and rock outcrops on their slopes:
    # granite above the green sectors, sandstone above the western desert. All beyond the 40 m
    # LiDAR reach of the outdoor flight areas.
    rim_rng = random.Random(seed * 97 + 5)
    for side, (ax, ay) in (("n", (0, 1)), ("e", (1, 0)), ("s", (0, -1))):
        for k in range(6):
            along = -36.0 + k * 14.4 + rim_rng.uniform(-3.0, 3.0)
            out = 46.5 + rim_rng.uniform(0.0, 6.0)
            x, y = (along, ay * out) if ay else (ax * out, along)
            z = assets.rim_height(x, y, rim_box, 22.0, seed) - 0.15
            solids.append(_webots_tree(rim_rng.choice(("Pine", "Pine", "Cypress", "Oak")), f"rim_{side}{k}", x, y,
                                       rim_rng.uniform(0, math.tau), z=z))
    for idx, (x, y, r, app) in enumerate((
        (18.0, 58.0, 5.5, rock_app), (-22.0, 60.0, 6.5, rock_app), (60.0, 26.0, 6.0, rock_app),
        (58.0, -30.0, 5.0, rock_app), (8.0, -60.0, 6.0, rock_app),
        (-58.0, 22.0, 6.5, sand_rock), (-62.0, -6.0, 7.0, sand_rock), (-57.0, -32.0, 5.5, sand_rock),
    )):
        rz = r * rim_rng.uniform(0.7, 1.0)
        z = assets.rim_height(x, y, rim_box, 22.0, seed) - 0.3 * rz
        solids.append(assets.faceted_boulder(f"rim_outcrop_{idx}", (x, y, z), (r, r * rim_rng.uniform(0.7, 1.0), rz),
                                             app, seed=seed * 61 + idx, yaw=rim_rng.uniform(0, math.tau),
                                             collide=False, tile=r * 1.5))

    # =========================================================================
    # CENTER HUB (0, 0): ground partition (later layers paint over earlier ones; see _ground)
    # =========================================================================
    ground_layers = [
        (*sand, -22.2, -5.6, -15.0, 15.0),            # West desert wash
        (*asphalt, -15.0, 15.0, 5.6, 22.2),           # North street (takes the NW corner)
        (*asphalt, -15.0, 15.0, -22.2, -5.6),         # South yard (takes the SW corner)
        (*dirt, -21.8, -5.6, -1.8, 1.8),              # West trail to the mine portal
        (*paving, 16.0, 21.2, -4.3, 4.3),             # East bunker apron
        (*paving, -5.6, 5.6, -5.6, 5.6),              # Hub plaza
    ]

    # =========================================================================
    # 1. EAST SECTOR (+X): Alpine Forest -> GPS-Denied Industrial Bunker
    # =========================================================================
    placed_east = [(0.0, 0.0), (19.0, 0.0)]
    solids.append('WorkBarrier { translation 17.0 4.3 0 rotation 0 0 1 0.35 name "bunker_barrier_l" }')
    solids.append('WorkBarrier { translation 17.0 -4.3 0 rotation 0 0 1 -0.35 name "bunker_barrier_r" }')
    solids.append('TrafficCone { translation 16.5 3.2 0 name "bunker_cone_l" }')
    solids.append('TrafficCone { translation 16.5 -3.2 0 name "bunker_cone_r" }')

    # Two deliberate faceted rock crags blocking straight-line flight from hub to bunker.
    for idx, (x, y, rx, ry, rz) in enumerate([
        (9.5, 0.4, 1.45, 1.65, 2.7),
        (14.8, -2.1, 1.35, 1.50, 2.5),
    ]):
        placed_east.append((x, y))
        solids.append(assets.faceted_boulder(f"approach_rock_{idx}", (x, y, rz * 0.45), (rx, ry, rz),
                                             rock_app, seed=seed * 17 + idx + 1, yaw=idx * 1.1))

    # Alpine Pine / Cypress / Oak forest framing the East approach.
    # Full-canopy Webots Pine/Oak trees (~2.2 m foliage radius) need >=6.2 m spacing so real
    # leaf meshes + C-space inflation never pinch shut around the approach crags.
    species = ("Pine", "Pine", "Cypress", "Oak")
    for _ in range(14):
        for _ in range(80):
            x, y = rng.uniform(6.5, 18.2), rng.uniform(-13.5, 13.5)
            if math.hypot(x, y) < 6.8 or math.hypot(x - 19.0, y) < 5.8:
                continue
            if abs(y) > 5.8 and x < 14.8:
                continue
            # Keep wide leafy canopies off the tight rock-slalom centerline (|y| < 4.5).
            if abs(y) < 4.5 and 7.0 < x < 17.5:
                continue
            if all(math.hypot(x - px, y - py) >= 6.2 for px, py in placed_east):
                placed_east.append((x, y))
                add_tree(rng.choice(species), x, y, rng.uniform(0, math.tau))
                break

    for i in range(4):
        for _ in range(50):
            x, y = rng.uniform(6.5, 16.8), rng.uniform(-11.0, 11.0)
            if math.hypot(x, y) < 6.2 or math.hypot(x - 19.0, y) < 5.2:
                continue
            if abs(y) > 5.5 and x < 14.0:
                continue
            if all(math.hypot(x - px, y - py) >= 4.4 for px, py in placed_east):
                placed_east.append((x, y))
                rx, ry, rz = rng.uniform(0.8, 1.25), rng.uniform(0.8, 1.25), rng.uniform(0.7, 1.35)
                solids.append(assets.faceted_boulder(f"forest_rock_{i}", (x, y, rz * 0.42), (rx, ry, rz),
                                                     rock_app, seed=seed * 53 + i))
                break

    # Enclosed East Industrial Bunker: x in [21, 39.2], y in [-10, 10], z in [0, 5.2], facing -x.
    shell, floor = _enclosure_shell_dir(21.0, 39.2, -10.0, 10.0, 5.2, 2.85, "-x", concrete_wall, metal_roof, slab,
                                        concrete_ceiling, lintel_z=4.65, prefix="cave")
    solids.extend(shell)
    ground_layers.append(floor)
    solids.extend(assets.portal_beacons(21.0, 2.95, 4.4, tex_url=tex_url,
                                        color=(0.95, 0.45, 0.08), facing="-x", prefix="bunker"))
    for idx, tx in enumerate((24.5, 28.5, 32.5, 36.5)):
        solids.append(assets.roof_truss(f"bunker_truss_{idx}", tx, 19.6, 5.02, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((25.0, 0.0), (34.5, 4.5), (34.5, -4.0))):
        solids.append(assets.overhead_lamp(f"bunker_{idx}", lx, ly, 5.14, color=(1.0, 0.94, 0.84),
                                           intensity=1.75, tex_url=tex_url))
    solids.append(assets.concrete_column("cave_pillar_center", 29.5, 0.4, width=1.9, height=5.15, tex_url=tex_url))
    solids.append(assets.box_solid("cave_divider_left", (33.5, 6.8, 2.55), (0.8, 6.0, 5.1), concrete_wall))
    solids.append('WoodenPalletStack { translation 34.5 -5.2 0 rotation 0 0 1 0.15 name "bunker_pallets" palletNumber 6 }')
    solids.append('WoodenBox { translation 34.5 -5.2 1.75 size 1.5 1.6 1.7 name "bunker_crate_hi" }')
    solids.append('OilBarrel { translation 36.4 -7.5 0.44 name "bunker_barrel_0" }')
    solids.append('OilBarrel { translation 37.3 -7.1 0.44 name "bunker_barrel_1" }')
    solids.append('OilBarrel { translation 23.8 7.8 0.44 name "bunker_barrel_2" }')
    solids.append('WoodenPalletStack { translation 24.4 -7.6 0 name "bunker_pallets_entry" palletNumber 4 }')
    solids.append('PipeSection { translation 38.3 2.5 2.0 rotation 1 0 0 1.5708 name "bunker_pipe_0" height 6.0 }')
    solids.append('EmergencyExitSign { translation 21.55 3.6 3.7 rotation 0 0 1 3.14159 name "bunker_exit_sign" }')

    # =========================================================================
    # 2. NORTH SECTOR (+Y): Urban Brick/Stucco Compound -> Covered Parking Deck
    # =========================================================================
    dx_u = rng.uniform(-0.7, 0.7)
    urban_buildings = [
        ("urban_bldg_center", (0.5 + dx_u, 12.2, 2.9), (4.6, 4.0, 5.8), brick_app),
        ("urban_bldg_west", (-9.6, 13.5, 2.8), (4.8, 6.2, 5.6), stucco_app),
        ("urban_bldg_east", (9.8, 13.5, 3.0), (4.8, 6.0, 6.0), stucco_app),
    ]
    for bname, bpos, bsize, bapp in urban_buildings:
        solids.append(assets.building_block(bname, bpos, bsize, bapp, deck_roof, tex_url=tex_url))

    for cx, cy, yaw in ((-13.0, 8.2, 0.4), (13.0, 8.2, 2.1)):
        add_tree("Cypress", cx, cy, yaw)
    solids.append('StreetLight { translation -5.8 17.8 0 name "urban_lamp_w" }')
    solids.append('StreetLight { translation 5.8 17.8 0 rotation 0 0 1 3.14159 name "urban_lamp_e" }')
    solids.append('WorkBarrier { translation -4.2 18.8 0 rotation 0 0 1 1.35 name "deck_barrier_w" }')
    solids.append('WorkBarrier { translation 4.2 18.8 0 rotation 0 0 1 1.79 name "deck_barrier_e" }')
    solids.append('TrafficCone { translation -3.4 17.8 0 name "deck_cone_w" }')
    solids.append('TrafficCone { translation 3.4 17.8 0 name "deck_cone_e" }')

    # Enclosed North Parking Deck: x in [-10.5, 10.5], y in [22.0, 40.0], z in [0, 4.9], facing -y.
    shell, floor = _enclosure_shell_dir(-10.5, 10.5, 22.0, 40.0, 4.9, 3.2, "-y", deck_wall, deck_roof, slab,
                                        concrete_ceiling, lintel_z=4.4, prefix="deck")
    solids.extend(shell)
    ground_layers.append(floor)
    solids.extend(assets.portal_beacons(22.0, 3.3, 4.2, tex_url=tex_url,
                                        color=(0.15, 0.75, 0.96), facing="-y", prefix="deck"))
    for idx, ty in enumerate((25.0, 28.5, 32.0, 35.5, 38.5)):
        solids.append(assets.roof_truss_y(f"deck_beam_{idx}", ty, 20.6, 4.72, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((0.0, 25.5), (-5.2, 33.5), (5.2, 33.5))):
        solids.append(assets.overhead_lamp(f"deck_{idx}", lx, ly, 4.84, color=(0.94, 0.98, 1.0),
                                           intensity=1.85, radius=LAMP_RADIUS, tex_url=tex_url))
    # 2x3 hazard-banded concrete column slalom grid.
    for ci, cx in enumerate((-5.2, 0.0, 5.2)):
        for cj, cy in enumerate((28.0, 34.5)):
            solids.append(assets.concrete_column(f"deck_col_{ci}_{cj}", cx, cy, width=1.10, height=4.85,
                                                 tex_url=tex_url, hazard_band=True))
    for sy in (25.0, 28.5, 32.0, 35.5, 38.2):
        solids.append(assets.box_solid(f"deck_stall_w_{sy:.0f}_lane", (-8.6, sy, 0.006), (3.2, 0.12, 0.008),
                                       lane_app, collide=False, cast_shadows=False))
        solids.append(assets.box_solid(f"deck_stall_e_{sy:.0f}_lane", (8.6, sy, 0.006), (3.2, 0.12, 0.008),
                                       lane_app, collide=False, cast_shadows=False))
    solids.append('WoodenPalletStack { translation -8.2 38.2 0 name "deck_service_pallets" palletNumber 5 }')
    solids.append('OilBarrel { translation 8.4 38.2 0.44 name "deck_service_barrel" }')
    solids.append('EmergencyExitSign { translation 3.9 22.55 3.6 rotation 0 0 1 -1.5708 name "deck_exit_sign" }')

    # =========================================================================
    # 3. SOUTH SECTOR (-Y): Logistics Shipping Yard -> Multi-Aisle Warehouse
    # =========================================================================
    for x_lane in (-2.4, 2.4):
        solids.append(assets.box_solid(f"yard_stripe_{x_lane:+.1f}_lane", (x_lane, -13.5, 0.006),
                                       (0.14, 15.0, 0.008), lane_app, collide=False, cast_shadows=False))
    for idx, fy in enumerate((-8.5, -16.5)):
        solids.append(f'UrbanFence {{ translation -14.2 {fy:.1f} 0 rotation 0 0 1 1.5708 name "yard_fence_w_{idx}" }}')
        solids.append(f'UrbanFence {{ translation 14.2 {fy:.1f} 0 rotation 0 0 1 1.5708 name "yard_fence_e_{idx}" }}')
    solids.append('StreetLight { translation -12.6 -10.0 0 name "yard_light_w" }')
    solids.append('StreetLight { translation 12.6 -10.0 0 rotation 0 0 1 3.14159 name "yard_light_e" }')

    dx_s = rng.uniform(-0.8, 0.8)
    yard_containers = [
        ("yard_container_0", (0.6 + dx_s, -11.6, 1.3), rng.uniform(-0.10, 0.10), (0.14, 0.42, 0.56)),
        ("yard_container_1", (-8.8, -14.8, 1.3), 1.57 + rng.uniform(-0.10, 0.10), (0.72, 0.28, 0.16)),
        ("yard_container_2", (9.0, -14.6, 1.3), 1.57 + rng.uniform(-0.10, 0.10), (0.20, 0.46, 0.34)),
    ]
    for cname, cpos, cyaw, ccol in yard_containers:
        solids.append(assets.shipping_container(cname, cpos, cyaw, ccol))

    solids.append('WoodenPalletStack { translation -5.4 -8.2 0 name "yard_pallets_0" palletNumber 5 }')
    solids.append('OilBarrel { translation 5.4 -18.5 0.44 name "yard_barrel_0" }')
    solids.append('OilBarrel { translation 6.1 -19.0 0.44 name "yard_barrel_1" }')
    solids.append('WorkBarrier { translation -4.2 -18.8 0 rotation 0 0 1 -1.35 name "wh_barrier_w" }')
    solids.append('WorkBarrier { translation 4.2 -18.8 0 rotation 0 0 1 -1.79 name "wh_barrier_e" }')
    solids.append('TrafficCone { translation -3.4 -17.8 0 name "wh_cone_w" }')
    solids.append('TrafficCone { translation 3.4 -17.8 0 name "wh_cone_e" }')

    # Enclosed South Warehouse: x in [-10.5, 10.5], y in [-40.0, -22.0], z in [0, 5.6], facing +y.
    shell, floor = _enclosure_shell_dir(-10.5, 10.5, -40.0, -22.0, 5.6, 3.1, "+y", metal_wall, metal_roof, slab,
                                        metal_ceiling, lintel_z=4.85, prefix="wh")
    solids.extend(shell)
    ground_layers.append(floor)
    solids.extend(assets.portal_beacons(-22.0, 3.2, 4.6, tex_url=tex_url,
                                        color=(0.96, 0.68, 0.08), facing="+y", prefix="wh"))
    for idx, ty in enumerate((-25.0, -28.5, -32.0, -35.5, -38.5)):
        solids.append(assets.roof_truss_y(f"wh_truss_{idx}", ty, 20.6, 5.40, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((0.0, -25.5), (-5.5, -33.5), (5.5, -33.5))):
        solids.append(assets.overhead_lamp(f"wh_{idx}", lx, ly, 5.54, color=(0.94, 0.97, 1.0),
                                           intensity=1.9, radius=LAMP_RADIUS, tex_url=tex_url))
    # 4 heavy steel pallet racking bays aligned North-South along Y with wide center & cross aisles.
    for rname, rpos in (
        ("wh_rack_nw", (-3.8, -27.2, 0.0)),
        ("wh_rack_ne", (3.8, -27.2, 0.0)),
        ("wh_rack_sw", (-3.8, -34.8, 0.0)),
        ("wh_rack_se", (3.8, -34.8, 0.0)),
    ):
        solids.append(assets.pallet_rack_bay(rname, rpos, length=4.0, depth=1.35, height=3.8,
                                             yaw=1.5708, tex_url=tex_url))
    solids.append('ConveyorBelt { translation 9.2 -31.0 0 rotation 0 0 1 1.5708 size 11.0 1.0 0.85 speed 0 name "wh_conveyor" }')
    solids.append('WoodenPalletStack { translation -8.2 -38.2 0 name "wh_pallets_back" palletNumber 6 }')
    solids.append('CardboardBox { translation 8.2 -38.2 0.45 size 0.9 0.9 0.9 name "wh_card_0" }')
    solids.append('EmergencyExitSign { translation 3.9 -22.55 3.9 rotation 0 0 1 1.5708 name "wh_exit_sign" }')

    # =========================================================================
    # 4. WEST SECTOR (-X): Red-Rock Desert Canyon -> Subterranean Basalt Mine
    # =========================================================================
    placed_west = [(0.0, 0.0), (-19.5, 0.0)]
    for idx, (x, y, rx, ry, rz) in enumerate([
        (-9.2, -0.6, 1.50, 1.65, 3.3),
        (-14.6, 2.3, 1.40, 1.50, 3.0),
    ]):
        placed_west.append((x, y))
        solids.append(assets.faceted_boulder(f"canyon_spire_{idx}", (x, y, rz * 0.45), (rx, ry, rz),
                                             sand_rock, seed=seed * 19 + idx + 3, yaw=idx * 0.9))

    for i in range(9):
        for _ in range(70):
            x, y = rng.uniform(-19.0, -7.2), rng.uniform(-12.5, 12.5)
            if math.hypot(x, y) < 6.4 or math.hypot(x + 19.5, y) < 4.8:
                continue
            if abs(y) > 5.8 and x > -14.5:
                continue
            if all(math.hypot(x - px, y - py) >= 4.6 for px, py in placed_west):
                placed_west.append((x, y))
                rx, ry, rz = rng.uniform(1.0, 2.0), rng.uniform(1.0, 2.0), rng.uniform(1.2, 3.4)
                solids.append(assets.faceted_boulder(f"desert_rock_{i}", (x, y, rz * 0.44), (rx, ry, rz),
                                                     sand_rock, seed=seed * 41 + i, yaw=rng.uniform(0, math.tau)))
                break

    solids.append('OilBarrel { translation -18.0 4.2 0.44 name "mine_barrel_0" }')
    solids.append('OilBarrel { translation -18.8 4.6 0.44 name "mine_barrel_1" }')
    solids.append('WoodenBox { translation -17.8 -4.4 0.55 size 1.3 1.1 1.1 name "mine_supply_crate" }')
    solids.append('WorkBarrier { translation -17.4 -3.6 0 rotation 0 0 1 0.4 name "mine_barrier" }')
    solids.append('PipeSection { translation -20.2 5.8 0.55 rotation 0 1 0 1.5708 name "mine_vent_duct" height 3.4 }')

    # Enclosed West Basalt Mine: x in [-40.0, -22.0], y in [-10.5, 10.5], z in [0, 5.2], facing +x.
    shell, floor = _enclosure_shell_dir(-40.0, -22.0, -10.5, 10.5, 5.2, 3.0, "+x", basalt_wall, cavern_roof,
                                        mine_floor, cavern_ceiling, lintel_z=4.65, prefix="mine")
    solids.extend(shell)
    ground_layers.append(floor)
    solids.extend(assets.portal_beacons(-22.0, 3.1, 4.4, tex_url=tex_url,
                                        color=(1.0, 0.56, 0.12), facing="+x", prefix="mine"))
    for idx, ax in enumerate((-22.7, -26.8, -31.0, -35.2, -39.0)):
        solids.append(assets.mine_arch(f"mine_rib_{idx}", ax, 20.4, 5.05, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((-25.5, 0.0), (-33.0, 5.5), (-33.0, -5.5))):
        solids.append(assets.overhead_lamp(f"mine_{idx}", lx, ly, 5.14, color=(1.0, 0.78, 0.45),
                                           intensity=1.95, radius=LAMP_RADIUS, tex_url=tex_url))
    solids.append(assets.faceted_boulder("mine_pillar_center", (-30.5, 0.5, 2.3), (1.9, 2.1, 3.4),
                                         basalt_rock, seed=seed * 7 + 11))
    solids.append(assets.faceted_boulder("mine_buttress_n", (-34.8, 7.2, 2.1), (1.7, 2.5, 3.2),
                                         basalt_rock, seed=seed * 7 + 22))
    solids.append(assets.faceted_boulder("mine_rockfall_s", (-34.2, -5.6, 1.5), (1.8, 2.0, 2.5),
                                         basalt_rock, seed=seed * 7 + 33))
    solids.append(assets.faceted_boulder("mine_outcrop_nw", (-25.0, 8.6, 1.6), (1.5, 1.5, 2.7),
                                         basalt_rock, seed=seed * 7 + 44))
    solids.append(assets.faceted_boulder("mine_outcrop_se", (-38.4, -8.2, 1.7), (1.7, 1.7, 2.7),
                                         basalt_rock, seed=seed * 7 + 55))
    solids.append('OilBarrel { translation -25.4 -8.0 0.44 name "mine_int_barrel_0" }')
    solids.append('WoodenBox { translation -38.0 7.6 0.65 size 1.3 1.3 1.3 name "mine_int_crate" }')

    # =========================================================================
    # OUTER CORNER LANDMARKS (NE / SE / NW / SW visual continuity)
    # =========================================================================
    # NE corner: alpine pine grove linking East forest and North urban district.
    for x, y, sp, yaw in (
        (19.5, 18.5, "Pine", 0.5), (25.5, 22.0, "Oak", 1.8), (31.0, 17.5, "Pine", 3.1),
        (21.5, 29.0, "Cypress", 0.9), (32.0, 28.5, "Pine", 2.4),
    ):
        add_tree(sp, x, y, yaw)
    # SE corner: industrial silos + pine transition.
    solids.append('Silo { translation 25.0 -24.0 0 name "se_silo_0" }')
    solids.append('Silo { translation 32.0 -24.0 0 name "se_silo_1" }')
    add_tree("Pine", 19.5, -18.5, 1.2)
    # NW corner: urban-to-desert transition warehouse & sandstone crags.
    solids.append('Warehouse { translation -28.0 27.0 0 rotation 0 0 1 -0.25 name "nw_warehouse" }')
    solids.append(assets.faceted_boulder("nw_crag_0", (-21.0, 17.5, 1.6), (2.2, 2.0, 3.6), sand_rock, seed=seed + 91))
    # SW corner: red-rock crags & mining silo.
    solids.append('Silo { translation -26.0 -25.0 0 name "sw_mine_silo" }')
    solids.append(assets.faceted_boulder("sw_crag_0", (-20.5, -17.5, 1.8), (2.4, 2.2, 3.8), sand_rock, seed=seed + 92))
    solids.append(assets.faceted_boulder("sw_crag_1", (-33.0, -21.0, 1.9), (2.5, 2.1, 4.0), sand_rock, seed=seed + 93))

    protos = [
        "Grass", "FormedConcrete", "RoughConcrete", "CorrugatedMetal", "Pavement", "OldSteel",
        "RedBricks", "Roughcast", "SandyGround", "DryMud", "RoughOak",
        "Pine", "Cypress", "Oak",
        "WorkBarrier", "TrafficCone", "OilBarrel", "WoodenPalletStack", "WoodenBox",
        "CardboardBox", "PipeSection", "ConveyorBelt", "StreetLight", "UrbanFence",
        "EmergencyExitSign", "Warehouse", "Silo",
    ]
    wbt = "\n".join([
        _world_header(f"Ironlark — unified 4-biome world (seed {seed})", seed,
                      _ground(100.0, grass, ground_layers), tex_url, protos),
        *solids,
        _world_footer(),
    ])
    bunker_gps = {"x": [20.5, 40.5], "y": [-11.0, 11.0], "z": [-1.0, 6.0]}
    deck_gps = {"x": [-11.5, 11.5], "y": [21.5, 41.0], "z": [-1.0, 5.6]}
    wh_gps = {"x": [-11.5, 11.5], "y": [-41.0, -21.5], "z": [-1.0, 6.2]}
    mine_gps = {"x": [-41.0, -21.5], "y": [-11.5, 11.5], "z": [-1.0, 6.0]}

    bunker_int = {"x": [22.0, 38.8], "y": [-9.6, 9.6], "z": [0.0, 4.8]}
    deck_int = {"x": [-9.9, 9.9], "y": [22.8, 39.4], "z": [0.0, 4.6]}
    wh_int = {"x": [-9.9, 9.9], "y": [-39.4, -22.8], "z": [0.0, 5.2]}
    mine_int = {"x": [-39.4, -22.8], "y": [-9.9, 9.9], "z": [0.0, 4.8]}

    sim_meta = {
        "world": "mission",
        "seed": seed,
        # Primary East Bunker boxes kept for mission_eval.py compatibility;
        # all 4 GPS-denied interiors are enforced simultaneously by flight_observer.
        "gps_denied_box": bunker_gps,
        "interior_box": bunker_int,
        "gps_denied_boxes": [bunker_gps, deck_gps, wh_gps, mine_gps],
        "interior_boxes": [bunker_int, deck_int, wh_int, mine_int],
        "mission_brief": {
            "home": [0.0, 0.0, 2.2],
            "entrance_approach": [18.2, 0.0, 2.2],
            "interior_entry": [24.5, 0.0, 2.2],
            "explore_bounds": {"x": [23.5, 36.8], "y": [-7.8, 7.8], "z": [1.8, 2.7]},
        },
    }
    return wbt, sim_meta


def warehouse_world(seed=7, tex_url="textures"):
    """2. Urban Logistics Yard -> Multi-Aisle Distribution Warehouse (tight racking & container slalom)."""
    rng = random.Random(seed)
    solids = []

    asphalt = (assets.pbr(tex_url, "asphalt.png", "asphalt_normal.png", roughness=0.86), 6.0)
    wall_app = assets.app_proto("CorrugatedMetal", scale=(8, 2), color=(0.80, 0.82, 0.84))
    lower_wall_app = assets.app_proto("FormedConcrete", scale=(6, 1.5))
    roof_app = assets.app_proto("CorrugatedMetal", scale=(6, 6), color=(0.52, 0.56, 0.60))
    ceiling_app = assets.app_proto("CorrugatedMetal", scale=(6, 6), color=(0.78, 0.80, 0.82), ibl=0.3)
    slab = (assets.pbr(tex_url, "concrete.png", "concrete_normal.png", roughness=0.55, ibl=0.5), 4.0)
    lane_app = assets.pbr(tex_url, color=(0.92, 0.72, 0.12), roughness=0.75)

    # Distant industrial skyline (non-blocking background buildings outside the arena).
    solids.append('Warehouse { translation -32.0 24.0 0 rotation 0 0 1 -0.25 name "bg_warehouse_nw" }')
    solids.append('Silo { translation -26.0 -26.0 0 name "bg_silo_sw_0" }')
    solids.append('Silo { translation -19.0 -26.0 0 name "bg_silo_sw_1" }')

    # Yard perimeter fencing & street lights.
    for idx, fx in enumerate((-14.0, -6.0, 2.0, 10.0, 18.0)):
        solids.append(f'UrbanFence {{ translation {fx:.1f} 17.5 0 name "fence_n_{idx}" }}')
        solids.append(f'UrbanFence {{ translation {fx:.1f} -17.5 0 name "fence_s_{idx}" }}')
    for idx, (sx, sy, rot) in enumerate(((-12.0, 15.8, -1.57), (4.0, 15.8, -1.57), (-12.0, -15.8, 1.57), (4.0, -15.8, 1.57))):
        solids.append(f'StreetLight {{ translation {sx:.1f} {sy:.1f} 0 rotation 0 0 1 {rot:.2f} name "yard_light_{idx}" }}')

    # Painted yard lane guide lines leading toward the loading dock.
    for y_lane in (-2.4, 2.4):
        solids.append(assets.box_solid(f"yard_lane_{y_lane:+.1f}", (10.5, y_lane, 0.006), (16.5, 0.14, 0.008),
                                       lane_app, collide=False, cast_shadows=False))

    # Outdoor shipping-container slalom (seeded jitter so each seed shifts the yard layout!).
    y_shift = rng.uniform(-1.0, 1.0)
    containers = [
        ("container_0", (8.8, 0.8 + y_shift, 1.3), rng.uniform(-0.12, 0.12), (0.14, 0.42, 0.56)),
        ("container_1", (13.6, -5.0 + y_shift * 0.5, 1.3), rng.uniform(-0.18, 0.18), (0.72, 0.28, 0.16)),
        ("container_2", (12.8, 7.6, 1.3), 1.57 + rng.uniform(-0.1, 0.1), (0.20, 0.46, 0.34)),
        ("container_3", (-6.8, 6.2, 1.3), 0.25, (0.68, 0.48, 0.14)),
        ("container_4", (-7.5, -7.2, 1.3), -0.35, (0.26, 0.32, 0.44)),
    ]
    for cname, pos, yaw, col in containers:
        solids.append(assets.shipping_container(cname, pos, yaw, col))

    # Loading-dock staging props outside the wide bay door.
    solids.append('WoodenPalletStack { translation 5.5 -6.5 0 name "yard_pallets_0" palletNumber 5 }')
    solids.append('OilBarrel { translation 17.8 4.8 0.44 name "yard_barrel_0" }')
    solids.append('OilBarrel { translation 18.6 5.2 0.44 name "yard_barrel_1" }')
    solids.append('WorkBarrier { translation 17.4 4.1 0 rotation 0 0 1 0.2 name "dock_barrier_l" }')
    solids.append('WorkBarrier { translation 17.4 -4.1 0 rotation 0 0 1 -0.2 name "dock_barrier_r" }')
    solids.append('TrafficCone { translation 16.4 3.4 0 name "dock_cone_l" }')
    solids.append('TrafficCone { translation 16.4 -3.4 0 name "dock_cone_r" }')

    # Large GPS-denied logistics warehouse: x in [21, 41], y in [-11, 11], z in [0, 5.6].
    shell, floor = _enclosure_shell(21.0, 41.0, 11.0, 5.6, 3.1, wall_app, roof_app, slab, ceiling_app,
                                    lintel_z=4.85, prefix="wh")
    solids.extend(shell)
    # Reinforced lower dado band along inside/outside base for architectural realism (visual).
    solids.append(assets.box_solid("wh_dado_n", (31.0, 10.75, 0.75), (19.6, 0.12, 1.5), lower_wall_app, collide=False))
    solids.append(assets.box_solid("wh_dado_s", (31.0, -10.75, 0.75), (19.6, 0.12, 1.5), lower_wall_app, collide=False))
    solids.extend(assets.portal_beacons(21.0, 3.2, 4.6, tex_url=tex_url, color=(0.95, 0.65, 0.08)))

    # Steel trusses & crisp cool-white warehouse overhead lighting.
    for idx, tx in enumerate((24.0, 28.0, 32.0, 36.0, 39.5)):
        solids.append(assets.roof_truss(f"wh_truss_{idx}", tx, 21.4, 5.40, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((25.0, 0.0), (31.0, 6.2), (31.0, -6.2), (37.5, 0.0))):
        solids.append(assets.overhead_lamp(idx, lx, ly, 5.54, color=(0.94, 0.97, 1.0),
                                           intensity=1.9, radius=LAMP_RADIUS, tex_url=tex_url))

    # Multi-aisle heavy pallet racking: 2 rows (y=+4.3, y=-4.3) x 2 bays (x=27.2, x=34.8)
    # -> leaves generous 4.2 m center aisle, 4.6 m outer aisles, and 3.4 m cross-aisle at x=31.0!
    rack_specs = [
        ("rack_nw", (27.2, 4.3, 0.0)),
        ("rack_sw", (27.2, -4.3, 0.0)),
        ("rack_ne", (34.8, 4.3, 0.0)),
        ("rack_se", (34.8, -4.3, 0.0)),
    ]
    for rname, rpos in rack_specs:
        solids.append(assets.pallet_rack_bay(rname, rpos, length=4.2, depth=1.35, height=3.8, tex_url=tex_url))

    # Conveyor line along the north wall + staging crates/pallets + exit sign.
    solids.append('ConveyorBelt { translation 31.0 9.6 0 size 12.0 1.0 0.85 speed 0 name "wh_conveyor" }')
    solids.append('WoodenPalletStack { translation 39.2 -7.8 0 name "wh_pallets_back" palletNumber 6 }')
    solids.append('WoodenBox { translation 39.2 -7.8 1.55 size 1.1 1.1 1.2 name "wh_box_back" }')
    solids.append('CardboardBox { translation 38.8 7.6 0.45 size 0.9 0.9 0.9 name "wh_card_0" }')
    solids.append('EmergencyExitSign { translation 21.55 3.9 3.9 rotation 0 0 1 3.14159 name "wh_exit_sign" }')

    protos = [
        "CorrugatedMetal", "FormedConcrete", "OldSteel",
        "Warehouse", "Silo", "UrbanFence", "StreetLight",
        "WorkBarrier", "TrafficCone", "OilBarrel", "WoodenPalletStack",
        "WoodenBox", "CardboardBox", "ConveyorBelt", "EmergencyExitSign",
    ]
    wbt = "\n".join([
        _world_header(f"Ironlark — logistics warehouse (seed {seed})", seed,
                      _ground(80.0, asphalt, [floor]), tex_url, protos),
        *solids,
        _world_footer(),
    ])
    sim_meta = {
        "world": "warehouse",
        "seed": seed,
        "gps_denied_box": {"x": [20.5, 42.0], "y": [-12.0, 12.0], "z": [-1.0, 6.2]},
        "interior_box": {"x": [22.0, 40.4], "y": [-10.4, 10.4], "z": [0.0, 5.2]},
        "mission_brief": {
            "home": [0.0, 0.0, 2.2],
            "entrance_approach": [18.2, 0.0, 2.2],
            "interior_entry": [24.5, 0.0, 2.2],
            "explore_bounds": {"x": [23.5, 38.6], "y": [-8.8, 8.8], "z": [1.8, 2.7]},
        },
    }
    return wbt, sim_meta


def canyon_world(seed=7, tex_url="textures"):
    """3. Red-Rock Desert Canyon -> Branching Subterranean Mine / Cavern (DARPA SubT Tunnel/Cave style)."""
    rng = random.Random(seed)
    solids = []
    placed = [(0.0, 0.0), (19.0, 0.0)]

    sand = (assets.app_proto("SandyGround", color=(1.0, 0.93, 0.84)), 5.0)
    cliff_app = assets.pbr(tex_url, "sandstone.png", "sandstone_normal.png", scale=(14, 14), roughness=0.95)
    sand_rock = assets.pbr(tex_url, "sandstone.png", "sandstone_normal.png", scale=(2, 2), roughness=0.92)
    basalt_wall = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 2), roughness=0.9)
    basalt_rock = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", color=(1.0, 0.9, 0.78), scale=(2, 2),
                             roughness=0.9, ibl=0.6)
    mine_floor = (assets.app_proto("SandyGround", color=(0.62, 0.56, 0.50), ibl=0.45), 3.0)
    cavern_roof = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 6), roughness=0.95)
    cavern_ceiling = assets.pbr(tex_url, "basalt.png", "basalt_normal.png", scale=(6, 6), roughness=0.95, ibl=0.3)

    # Dramatic stepped red-sandstone canyon cliffs framing the desert wash & mine cliffside.
    solids.append(assets.terrain_rim("canyon_cliffs", (-18.0, 43.0, -17.0, 17.0), 72.0, 17.0,
                                     cliff_app, seed=seed, style="canyon"))

    # Two deliberate sandstone rock spires on the outdoor approach.
    for idx, (x, y, rx, ry, rz) in enumerate([
        (8.6, -0.6, 1.55, 1.70, 3.4),
        (14.2, 2.4, 1.45, 1.55, 3.1),
    ]):
        placed.append((x, y))
        solids.append(assets.faceted_boulder(f"canyon_spire_{idx}", (x, y, rz * 0.45), (rx, ry, rz),
                                             sand_rock, seed=seed * 19 + idx + 3, yaw=idx * 0.9))

    # Scattered red-rock boulders & spires across the desert canyon floor.
    for i in range(18):
        for _ in range(70):
            x, y = rng.uniform(-15.5, 17.0), rng.uniform(-16.0, 16.0)
            if math.hypot(x, y) < 5.2 or math.hypot(x - 19.0, y) < 4.8:
                continue
            if all(math.hypot(x - px, y - py) >= 4.0 for px, py in placed):
                placed.append((x, y))
                rx = rng.uniform(1.0, 2.1)
                ry = rng.uniform(1.0, 2.1)
                rz = rng.uniform(1.2, 3.6)
                solids.append(assets.faceted_boulder(f"desert_rock_{i}", (x, y, rz * 0.44), (rx, ry, rz),
                                                     sand_rock, seed=seed * 41 + i, yaw=rng.uniform(0, math.tau)))
                break

    # Mine-site staging props outside the portal.
    solids.append('OilBarrel { translation 17.6 4.2 0.44 name "mine_barrel_0" }')
    solids.append('OilBarrel { translation 18.4 4.5 0.44 name "mine_barrel_1" }')
    solids.append('WoodenBox { translation 17.4 -4.5 0.55 size 1.3 1.1 1.1 name "mine_supply_crate" }')
    solids.append('WorkBarrier { translation 16.8 -3.6 0 rotation 0 0 1 -0.4 name "mine_barrier" }')
    solids.append('PipeSection { translation 19.8 5.8 0.55 rotation 0 1 0 1.5708 name "mine_vent_duct" height 3.4 }')

    # Subterranean basalt cavern / mine: x in [21, 41], y in [-11, 11], z in [0, 5.2].
    shell, floor = _enclosure_shell(21.0, 41.0, 11.0, 5.2, 3.0, basalt_wall, cavern_roof, mine_floor,
                                    cavern_ceiling, lintel_z=4.65, prefix="mine")
    solids.extend(shell)
    solids.extend(assets.portal_beacons(21.0, 3.1, 4.4, tex_url=tex_url, color=(1.0, 0.55, 0.12)))

    # Heavy timber mine support arches & warm sodium work lamps along the cavern drift.
    for idx, ax in enumerate((21.7, 26.2, 30.7, 35.2, 39.4)):
        solids.append(assets.mine_arch(f"mine_rib_{idx}", ax, 20.8, 5.05, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((25.0, 0.0), (31.5, 6.2), (31.5, -6.2), (37.5, 0.0))):
        solids.append(assets.overhead_lamp(idx, lx, ly, 5.14, color=(1.0, 0.78, 0.45),
                                           intensity=1.9, radius=LAMP_RADIUS, tex_url=tex_url))

    # Organic cavern obstacles: central basalt pillar splitting the main drift + wall outcrops + rockfalls.
    solids.append(assets.faceted_boulder("cavern_pillar_center", (29.8, 0.5, 2.3), (2.0, 2.2, 3.4),
                                         basalt_rock, seed=seed * 7 + 11))
    solids.append(assets.faceted_boulder("cavern_buttress_n", (34.5, 7.2, 2.1), (1.7, 2.6, 3.2),
                                         basalt_rock, seed=seed * 7 + 22))
    solids.append(assets.faceted_boulder("cavern_rockfall_s", (33.8, -5.6, 1.5), (1.8, 2.0, 2.5),
                                         basalt_rock, seed=seed * 7 + 33))
    solids.append(assets.faceted_boulder("cavern_outcrop_nw", (24.5, 8.8, 1.6), (1.6, 1.6, 2.8),
                                         basalt_rock, seed=seed * 7 + 44))
    solids.append(assets.faceted_boulder("cavern_outcrop_se", (38.8, -8.5, 1.8), (1.8, 1.8, 2.8),
                                         basalt_rock, seed=seed * 7 + 55))
    solids.append('OilBarrel { translation 25.2 -8.2 0.44 name "mine_int_barrel_0" }')
    solids.append('WoodenBox { translation 38.4 7.8 0.65 size 1.3 1.3 1.3 name "mine_int_crate" }')

    protos = [
        "SandyGround", "RoughOak",
        "OilBarrel", "WoodenBox", "WorkBarrier", "PipeSection",
    ]
    wbt = "\n".join([
        _world_header(f"Ironlark — desert canyon mine (seed {seed})", seed,
                      _ground(80.0, sand, [floor]), tex_url, protos, sky="dusk",
                      sun={"direction": (-0.44, -0.88, -0.22), "color": (1.0, 0.84, 0.64), "intensity": 2.2},
                      fog={"color": (0.80, 0.64, 0.52), "range": 200.0}),
        *solids,
        _world_footer(),
    ])
    sim_meta = {
        "world": "canyon",
        "seed": seed,
        "gps_denied_box": {"x": [20.5, 42.0], "y": [-12.0, 12.0], "z": [-1.0, 6.0]},
        "interior_box": {"x": [22.0, 40.4], "y": [-10.4, 10.4], "z": [0.0, 4.8]},
        "mission_brief": {
            "home": [0.0, 0.0, 2.2],
            "entrance_approach": [18.2, 0.0, 2.2],
            "interior_entry": [24.5, 0.0, 2.2],
            "explore_bounds": {"x": [23.5, 38.5], "y": [-8.4, 8.4], "z": [1.8, 2.7]},
        },
    }
    return wbt, sim_meta


def compound_world(seed=7, tex_url="textures"):
    """4. Urban Industrial Compound -> Covered Concrete Parking Deck (alleys + column-grid slalom)."""
    rng = random.Random(seed)
    solids = []

    asphalt = (assets.pbr(tex_url, "asphalt.png", "asphalt_normal.png", roughness=0.86), 6.0)
    paving = (assets.app_proto("Pavement", color=(0.96, 0.94, 0.90), extra='type "slate"'), 4.0)
    brick_app = assets.app_proto("RedBricks", scale=(5, 2.5))
    stucco_app = assets.app_proto("Roughcast", scale=(4, 2), color=(0.93, 0.88, 0.80))
    roof_app = assets.app_proto("FormedConcrete", scale=(4, 4))
    ceiling_app = assets.app_proto("RoughConcrete", scale=(5, 5), color=(0.86, 0.85, 0.82), ibl=0.3)
    deck_wall = assets.app_proto("RoughConcrete", scale=(10, 3), color=(0.92, 0.91, 0.88))
    deck_floor = (assets.pbr(tex_url, "concrete.png", "concrete_normal.png", roughness=0.55, ibl=0.5), 4.0)
    stripe_app = assets.pbr(tex_url, color=(0.94, 0.76, 0.14), roughness=0.72)

    # Courtyard plaza around launch pad & distant urban skyline.
    solids.append('Warehouse { translation -32.0 -20.0 0 rotation 0 0 1 0.15 name "bg_compound_wh" }')
    solids.append('Silo { translation -26.0 24.0 0 name "bg_compound_silo" }')

    # Urban street buildings forming navigable alleys around a central block (seeded slight y-offset).
    dy = rng.uniform(-0.8, 0.8)
    buildings = [
        ("bldg_center", (9.2, 0.8 + dy, 3.2), (4.6, 5.8, 6.4), brick_app),
        ("bldg_south", (10.5, -10.5, 2.8), (6.4, 5.2, 5.6), stucco_app),
        ("bldg_north", (10.2, 11.2, 3.0), (6.0, 5.0, 6.0), stucco_app),
        ("bldg_nw", (-8.5, 8.8, 2.6), (5.8, 5.2, 5.2), brick_app),
        ("bldg_sw", (-8.8, -9.2, 2.8), (6.0, 5.4, 5.6), stucco_app),
    ]
    for bname, bpos, bsize, bapp in buildings:
        solids.append(assets.building_block(bname, bpos, bsize, bapp, roof_app, tex_url=tex_url))

    # Courtyard Cypress trees, street lights, barriers, and cones.
    for idx, (tx, ty) in enumerate(((-4.2, 12.0), (-4.2, -12.0), (3.5, 11.5), (3.5, -11.5))):
        solids.append(_webots_tree("Cypress", idx, tx, ty, idx * 1.3))
    for idx, (sx, sy, rot) in enumerate(((-4.5, 5.8, -1.57), (15.5, 6.0, -1.57), (15.5, -6.0, 1.57))):
        solids.append(f'StreetLight {{ translation {sx:.1f} {sy:.1f} 0 rotation 0 0 1 {rot:.2f} name "street_lamp_{idx}" }}')
    solids.append('WorkBarrier { translation 17.5 4.2 0 rotation 0 0 1 0.25 name "deck_barrier_l" }')
    solids.append('WorkBarrier { translation 17.5 -4.2 0 rotation 0 0 1 -0.25 name "deck_barrier_r" }')
    solids.append('TrafficCone { translation 16.5 3.4 0 name "deck_cone_l" }')
    solids.append('TrafficCone { translation 16.5 -3.4 0 name "deck_cone_r" }')

    # GPS-denied Covered Concrete Parking Deck: x in [21, 40], y in [-10.5, 10.5], z in [0, 4.9].
    shell, floor = _enclosure_shell(21.0, 40.0, 10.5, 4.9, 3.2, deck_wall, roof_app, deck_floor, ceiling_app,
                                    lintel_z=4.4, prefix="deck")
    solids.extend(shell)
    solids.extend(assets.portal_beacons(21.0, 3.3, 4.2, tex_url=tex_url, color=(0.15, 0.72, 0.95)))

    # Overhead waffle concrete ceiling beams & parking-deck fluorescent strips.
    for idx, tx in enumerate((24.2, 27.5, 30.8, 34.0, 37.2)):
        solids.append(assets.roof_truss(f"deck_beam_{idx}", tx, 20.6, 4.72, tex_url=tex_url))
    for idx, (lx, ly) in enumerate(((24.5, 0.0), (30.8, 5.0), (30.8, -5.0), (37.2, 0.0))):
        solids.append(assets.overhead_lamp(idx, lx, ly, 4.84, color=(0.95, 0.98, 1.0),
                                           intensity=1.85, radius=LAMP_RADIUS, tex_url=tex_url))

    # Classic 2x3 reinforced concrete column grid with yellow/black bumper bands (5.2 m bay spacing!).
    for ci, cx in enumerate((27.5, 34.0)):
        for cj, cy in enumerate((-5.2, 0.0, 5.2)):
            solids.append(assets.concrete_column(f"deck_col_{ci}_{cj}", cx, cy, width=1.15, height=4.85,
                                                 tex_url=tex_url, hazard_band=True))

    # Painted parking stall markings along the north & south bays + corner service props.
    for sx in (24.0, 27.5, 31.0, 34.5, 37.5):
        solids.append(assets.box_solid(f"stall_n_{sx:.0f}", (sx, 8.6, 0.006), (0.12, 3.2, 0.008), stripe_app,
                                       collide=False, cast_shadows=False))
        solids.append(assets.box_solid(f"stall_s_{sx:.0f}", (sx, -8.6, 0.006), (0.12, 3.2, 0.008), stripe_app,
                                       collide=False, cast_shadows=False))
    solids.append('WoodenPalletStack { translation 38.2 -8.2 0 name "deck_service_pallets" palletNumber 5 }')
    solids.append('OilBarrel { translation 38.4 8.2 0.44 name "deck_service_barrel" }')
    solids.append('EmergencyExitSign { translation 21.55 3.9 3.6 rotation 0 0 1 3.14159 name "deck_exit_sign" }')

    protos = [
        "Pavement", "RedBricks", "Roughcast", "FormedConcrete", "RoughConcrete", "OldSteel",
        "Warehouse", "Silo", "Cypress", "StreetLight",
        "WorkBarrier", "TrafficCone", "WoodenPalletStack", "OilBarrel", "EmergencyExitSign",
    ]
    wbt = "\n".join([
        _world_header(f"Ironlark — urban compound & parking deck (seed {seed})", seed,
                      _ground(80.0, asphalt, [(*paving, -4.75, 4.75, -4.75, 4.75), floor]), tex_url, protos),
        *solids,
        _world_footer(),
    ])
    sim_meta = {
        "world": "compound",
        "seed": seed,
        "gps_denied_box": {"x": [20.5, 41.0], "y": [-11.5, 11.5], "z": [-1.0, 5.6]},
        "interior_box": {"x": [22.0, 39.4], "y": [-9.9, 9.9], "z": [0.0, 4.6]},
        "mission_brief": {
            "home": [0.0, 0.0, 2.2],
            "entrance_approach": [18.2, 0.0, 2.2],
            "interior_entry": [24.2, 0.0, 2.2],
            "explore_bounds": {"x": [23.2, 37.8], "y": [-8.2, 8.2], "z": [1.8, 2.6]},
        },
    }
    return wbt, sim_meta


WORLD_BUILDERS = {
    "mission": mission_world,
    "warehouse": warehouse_world,
    "canyon": canyon_world,
    "compound": compound_world,
}


def build_mission_world(world, seed=7, tex_url="textures"):
    """Dispatch any supported GPS-denied mission world by name; returns (wbt_text, sim_meta)."""
    if world not in WORLD_BUILDERS:
        raise ValueError(f"Unknown mission world '{world}'. Choose from: {', '.join(MISSION_WORLDS)}")
    return WORLD_BUILDERS[world](seed=seed, tex_url=tex_url)
