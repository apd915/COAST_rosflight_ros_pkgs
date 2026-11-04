#!/usr/bin/env python3
"""Generate static-forest benchmark environments matching SANDO's Sec. IX-C protocol.

WHY THESE EXACT NUMBERS
-----------------------
Built to mirror the SANDO paper's static-forest benchmark so our results can sit next to
Table VI rather than needing a caveat about a different environment:

    cylinders   radius 1.0-1.5 m, height 6 m, placed randomly
    field       100 m x 40 m
    difficulty  fraction of field area occupied -- easy 5%, medium 10%, hard 20%
    route       start (0, 0, 3) -> goal (105, 0, 3)   [+z up; NED here is z = -3]
    protocol    10 runs per difficulty level

Trunks are 6 m and cruise is 3 m, so overflight is not a shortcut -- the benchmark measures
horizontal avoidance, as theirs does.

RANDOM PLACEMENT, NOT A LATTICE
-------------------------------
An earlier version used a jittered lattice because it gives a closed-form minimum gap. That is
reproducible but it is NOT what SANDO did, and the difference matters: a lattice has bounded
clustering, so it never produces the locally-impassable pockets that force a genuine detour.
Matching their protocol means accepting random clustering and averaging over seeds, exactly as
they do with 10 runs per level.

What we add is a CONNECTIVITY CHECK. Random placement can wall off the route outright, and a
run that fails in an impassable forest measures nothing about the planner. --check flood-fills
the inflated free space from start to goal and reports whether a route exists at all, so a
failure can be attributed rather than guessed at. Report the check alongside the success rate.

DYNAMIC LIMITS DIFFER FROM SANDO
--------------------------------
They ran this benchmark at v_max 5.0 m/s, a_max 20.0 m/s^2, j_max 100 m/s^3. Our defaults are
v_max 3.0 and a_max 2.0 -- an order of magnitude less aggressive on acceleration. Travel time
T_trav is therefore NOT comparable to their Table VI unless the limits are matched. Either
raise ours and say so, or report T_trav only against our own baselines. Computation time,
success rate and constraint-violation rates remain comparable.

USAGE
  python3 generate_forest.py --all --out-dir ../params          # seed 0 of each level
  python3 generate_forest.py --level hard --seed 3 --out ../params/forest_hard_s3.yaml
  python3 generate_forest.py --all --seeds 10 --out-dir ../params/forests   # full protocol
  python3 generate_forest.py --all --check                      # stats + passability, no writes
"""

import argparse
import math
import os
import random
import zlib

# Planner voxel resolution. route_exists MUST rasterise at this, not finer -- see its docstring.
VOXEL_RES = 0.5

# --- SANDO Sec. IX-C environment definition -------------------------------------------------
FIELD_X = (0.0, 100.0)      # m, along the route
FIELD_Y = (-20.0, 20.0)     # m, 40 m wide
RADIUS_RANGE = (1.0, 1.5)   # m
TRUNK_HEIGHT = 6.0          # m
START = (0.0, 0.0)          # m, cruise altitude applied separately
GOAL = (105.0, 0.0)         # m, just beyond the field
CRUISE_ALT = 3.0            # m above ground (NED z = -CRUISE_ALT)
CLEAR_RADIUS = 5.0          # m, obstacle-free ball around each waypoint

AREA_FRACTION = {"easy": 0.05, "medium": 0.10, "hard": 0.20}

# --density overrides the level's target so a set can be generated at an arbitrary occupancy.
# The named levels stay as they are -- they mirror SANDO Sec. IX-C and are what the published
# comparison is against; this exists to characterise the band in between.
DENSITY_OVERRIDE = None

# Planner clearance: a gap admits a corridor only if it exceeds 2*(hull + margin).
DEFAULT_HULL = 0.49
DEFAULT_MARGIN = 0.30


def generate(level, seed):
    """Return (cylinders, stats). Cylinder = (x, y, z_center, radius, half_height)."""
    # zlib.crc32, NOT hash(). Python salts hash() on strings per process (PYTHONHASHSEED
    # randomisation, on by default since 3.3), so the same level and seed produced a DIFFERENT
    # forest on every invocation -- the files were reproducible once written, but not
    # regenerable, which is fatal for a benchmark anyone else has to reproduce. crc32 is stable
    # across processes, machines and Python versions.
    rng = random.Random(zlib.crc32(level.encode()) * 1000 + seed)

    field_area = (FIELD_X[1] - FIELD_X[0]) * (FIELD_Y[1] - FIELD_Y[0])
    frac = DENSITY_OVERRIDE if DENSITY_OVERRIDE is not None else AREA_FRACTION[level]
    target_area = frac * field_area

    half_h = TRUNK_HEIGHT / 2.0
    z_center = -half_h                    # NED: trunk base sits on the ground plane z = 0

    cylinders, placed_area = [], 0.0
    # Cap the attempts so an over-dense target cannot spin forever.
    for _ in range(200000):
        if placed_area >= target_area:
            break
        r = rng.uniform(*RADIUS_RANGE)
        x = rng.uniform(*FIELD_X)
        y = rng.uniform(*FIELD_Y)

        # Never block a waypoint.
        if math.hypot(x - START[0], y - START[1]) < CLEAR_RADIUS + r:
            continue
        if math.hypot(x - GOAL[0], y - GOAL[1]) < CLEAR_RADIUS + r:
            continue

        cylinders.append((x, y, z_center, r, half_h))
        placed_area += math.pi * r * r

    stats = {
        "level": level,
        "seed": seed,
        "count": len(cylinders),
        "area_fraction": placed_area / field_area,
        "target_fraction": frac,
        "stems_per_ha": len(cylinders) / (field_area / 10000.0),
    }
    return cylinders, stats


def min_surface_gap(cylinders):
    """Smallest gap between any two trunk surfaces (negative where they overlap)."""
    best = float("inf")
    for i, a in enumerate(cylinders):
        for b in cylinders[i + 1:]:
            best = min(best, math.hypot(a[0] - b[0], a[1] - b[1]) - a[3] - b[3])
    return best


def route_bottleneck(cylinders, lo=0.0, hi=3.0, tol=0.02):
    """Largest inflation at which a route still exists -- the tightest point on the best route.

    "A route exists" is a weak property for a horizon-limited planner. Connectivity can hold
    globally while the only way through is a pocket the planner commits into and cannot back out
    of, because it plans 15 m ahead with a greedy A* and does not see the trap until it is in it.
    Measured: every seed reports "route OK" at BOTH margin 0.20 and 0.30, yet 0.30 stalled the
    vehicle -- the environments were never the problem, the local tightness was.

    This reports the number that actually characterises an environment: the widest bottleneck any
    route must pass. Compare it against the planner's effective inflation. A bottleneck barely
    above it means the seed is passable only through a gap with no room for error, which is a
    property of the environment worth reporting rather than a planner failure to explain away.
    """
    if not route_exists(cylinders, lo):
        return 0.0
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if route_exists(cylinders, mid):
            lo = mid
        else:
            hi = mid
    return lo


def route_exists(cylinders, inflation, res=VOXEL_RES):
    """Flood-fill the free space from start to goal, AS THE PLANNER SEES IT.

    A failure in a forest with no route through it says nothing about the planner, so this
    separates "the planner could not find the way" from "there was no way".

    THIS MUST MIRROR THE PLANNER'S RASTERISATION, NOT AN EXACT CIRCLE. It used to inflate each
    trunk by the exact continuous radius on a 0.25 m grid, while the planner marks the voxels a
    trunk occupies and then stamps a BOX of +/-(half_res + inflation) around each of them on a
    0.5 m grid. The planner's footprint is therefore quantised UP to whole voxels and is square
    rather than round, so it needs a wider gap than the exact test predicts:

        margin 0.20 -> check said 1.38 m needed, planner actually needs 2.00 m   (1.45x)
        margin 0.30 -> check said 1.58 m needed, planner actually needs 3.00 m   (1.90x)

    Certifying routes the planner cannot fly is worse than not certifying at all: it sends a
    failure to be attributed to the planner when the environment was never passable for it.
    Raising obstacle_margin 0.20 -> 0.30 crossed a voxel boundary, jumped the required gap from
    2.00 m to 3.00 m, and stalled the vehicle in gaps this function had marked passable.
    """
    pad = 10.0
    x0, x1 = FIELD_X[0] - pad, GOAL[0] + pad
    y0, y1 = FIELD_Y[0] - pad, FIELD_Y[1] + pad
    nx = int((x1 - x0) / res) + 1
    ny = int((y1 - y0) / res) + 1

    # 1. Which voxels does a trunk occupy? Cell centres inside the raw circle, as the LiDAR
    #    hits and the voxel grid record them.
    occupied = set()
    for (cx, cy, _z, r, _hh) in cylinders:
        i0 = max(0, int((cx - r - x0) / res))
        i1 = min(nx - 1, int((cx + r - x0) / res))
        j0 = max(0, int((cy - r - y0) / res))
        j1 = min(ny - 1, int((cy + r - y0) / res))
        for i in range(i0, i1 + 1):
            dx = x0 + (i + 0.5) * res - cx
            for j in range(j0, j1 + 1):
                dy = y0 + (j + 0.5) * res - cy
                if dx * dx + dy * dy <= r * r:
                    occupied.add((i, j))

    # 2. Stamp the inflation as the planner does -- a BOX of +/-(half + inflation) around each
    #    occupied cell, rasterised by floor(). This is what quantises the margin to whole voxels.
    half = res / 2.0
    reach = int(math.floor((half + inflation) / res)) + 1
    blocked = bytearray(nx * ny)
    for (i, j) in occupied:
        for di in range(-reach, reach + 1):
            ii = i + di
            if ii < 0 or ii >= nx:
                continue
            for dj in range(-reach, reach + 1):
                jj = j + dj
                if 0 <= jj < ny:
                    blocked[ii * ny + jj] = 1

    def idx(px, py):
        return (int((px - x0) / res), int((py - y0) / res))

    si, sj = idx(*START)
    gi, gj = idx(*GOAL)
    if blocked[si * ny + sj] or blocked[gi * ny + gj]:
        return False

    seen = bytearray(nx * ny)
    stack = [(si, sj)]
    seen[si * ny + sj] = 1
    while stack:
        i, j = stack.pop()
        if i == gi and j == gj:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny:
                k = a * ny + b
                if not seen[k] and not blocked[k]:
                    seen[k] = 1
                    stack.append((a, b))
    return False


def to_yaml(cylinders, stats, hull, margin, passable, gap):
    t = 2.0 * (hull + margin)
    lines = [
        f"# Static forest benchmark -- {stats['level'].upper()}, seed {stats['seed']}",
        "#",
        "# GENERATED by tools/generate_forest.py. Edit the generator, not this file.",
        "# Mirrors SANDO Sec. IX-C: 100x40 m field, cylinders r in [1.0, 1.5] m, height 6 m,",
        "# difficulty by fraction of field area occupied, route (0,0,3) -> (105,0,3).",
        "#",
        f"#   trunks            {stats['count']}  ({stats['stems_per_ha']:.0f} stems/ha)",
        f"#   area occupied     {stats['area_fraction'] * 100:.1f}% "
        f"(target {stats['target_fraction'] * 100:.0f}%)",
        f"#   min surface gap   {gap:.2f} m",
        f"#   route exists      {'YES' if passable else 'NO'} "
        f"at inflation {hull + margin:.2f} m (hull {hull:.2f} + margin {margin:.2f})",
        "#",
        "# A corridor exists only where a gap exceeds 2*(hull + obstacle_margin) = "
        f"{t:.2f} m.",
        "# Random placement means local pockets can be impassable even when a route exists --",
        "# that is intentional and is what forces detours. Re-run with --check before treating a",
        "# failure as a planner bug.",
        "#",
        "# NOTE: keys are omitted rather than written as empty lists. ROS 2 cannot infer the type",
        "# of an empty YAML sequence, so `boxes: []` aborts the node at startup with",
        "#   InvalidParameterValueException: ... 'boxes': No parameter value set",
        "# Omitting the key leaves the node's declared default in force. Do not add them back.",
        "",
        "/mock_environment:",
        "  ros__parameters:",
        "    # Sensor sampling is part of the benchmark definition, not a rendering detail.",
        "    #",
        "    # Free space is only marked where a beam PASSES THROUGH, so the map's usable",
        "    # resolution is bounded by beam spacing, not voxel size. At the 3.0 deg default the",
        "    # rings are 0.79 m apart at 15 m -- wider than the 0.5 m voxel -- so the volume ahead",
        "    # stays UNKNOWN however long the vehicle flies, A* pays unknown_traversal_cost to",
        "    # cross it, and the search is pushed sideways and backwards into already-mapped space.",
        "    #",
        "    # 1.5 deg puts the rings 0.39 m apart at 15 m and keeps coverage contiguous out to",
        "    # 19 m, past the planning horizon. It also lands nearer the real sensor: 9440",
        "    # points/scan against a MID-360's ~20000, where 3.0 deg gives only 2360.",
        "    # The raycast is brute-force O(primitives) per beam, but mock_environment_node is its",
        "    # own process, so this costs the planner nothing.",
        "    # No ground plane. Cruise is 3 m among 6 m trunks, so the floor never constrains a",
        "    # corridor -- A* is held above it by min_altitude, independently of the map. But the",
        "    # -7 deg beams sweep it continuously, and every ground return becomes an occupied",
        "    # voxel that the per-scan rebuild then stamps with a 5x5x5 inflation box. It is pure",
        "    # cost for the planner and pure cost for the benchmark. Set true if you specifically",
        "    # want terrain in the picture.",
        "    ground_enabled: false",
        "",
        "    lidar_h_res_deg: 1.5",
        "    lidar_v_res_deg: 1.5",
        "",
        "    # [cx, cy, cz, radius, half_height] per trunk, NED metres.",
    ]
    if cylinders:
        lines.append("    cylinders:")
        for (x, y, z, r, hh) in cylinders:
            lines.append(f"      - {x:.3f}\n      - {y:.3f}\n      - {z:.3f}\n"
                         f"      - {r:.3f}\n      - {hh:.3f}")
    lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", choices=sorted(AREA_FRACTION))
    ap.add_argument("--density", type=float,
                    help="override the level's area fraction, e.g. 0.15. Names the files by "
                         "density so a 15%% set cannot be mistaken for the 20%% 'hard' set.")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, default=1,
                    help="generate seeds 0..N-1 (SANDO uses 10 runs per level)")
    ap.add_argument("--out")
    ap.add_argument("--out-dir")
    ap.add_argument("--check", action="store_true",
                    help="report stats and passability only, write nothing")
    ap.add_argument("--hull", type=float, default=DEFAULT_HULL)
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN)
    args = ap.parse_args()
    if args.density is not None:
        globals()['DENSITY_OVERRIDE'] = args.density

    inflation = args.hull + args.margin
    levels = sorted(AREA_FRACTION, key=lambda k: AREA_FRACTION[k]) \
        if (args.all or not args.level) else [args.level]
    seeds = range(args.seeds) if args.seeds > 1 else [args.seed]

    print(f"inflation {inflation:.2f} m (hull {args.hull:.2f} + margin {args.margin:.2f}); "
          f"a gap must exceed {2 * inflation:.2f} m\n")

    for level in levels:
        for seed in seeds:
            cylinders, stats = generate(level, seed)
            gap = min_surface_gap(cylinders)
            passable = route_exists(cylinders, inflation)
            # The tightest point on the best route. "Route OK" alone is too weak: every seed
            # passes at both margin 0.20 and 0.30 while 0.30 stalls the vehicle, because global
            # connectivity says nothing about whether a horizon-limited planner can get through.
            bottleneck = route_bottleneck(cylinders)
            head = max(0.0, bottleneck - inflation)

            print(f"  {level:<7} seed {seed:<2} {stats['count']:>4} trunks  "
                  f"{stats['area_fraction'] * 100:4.1f}% area  "
                  f"{stats['stems_per_ha']:>4.0f} stems/ha  "
                  f"min gap {gap:6.2f} m  "
                  f"bottleneck {bottleneck:4.2f} m (headroom {head:+5.2f})  "
                  f"route {'OK' if passable else 'BLOCKED'}")

            if args.check:
                continue

            if args.out and len(levels) == 1 and len(list(seeds)) == 1:
                path = args.out
            elif args.out_dir:
                os.makedirs(args.out_dir, exist_ok=True)
                # Name by DENSITY when overridden, so a 15% set can never be mistaken for the
                # named "hard" set at 20% -- the filename is the only thing a run records.
                tag = (level if args.density is None
                       else f"d{int(round(args.density * 100)):02d}")
                name = (f"forest_{tag}.yaml" if args.seeds == 1
                        else f"forest_{tag}_s{seed}.yaml")
                path = os.path.join(args.out_dir, name)
            else:
                continue

            with open(path, "w") as f:
                f.write(to_yaml(cylinders, stats, args.hull, args.margin, passable, gap))
            print(f"            -> {path}")


if __name__ == "__main__":
    main()
