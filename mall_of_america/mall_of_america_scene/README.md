# Mall of America inspired CARLA + Scenic starter scene

This is a **manually authored approximation**, inspected against Google Maps'
satellite view at 44.8548, -93.2427. It is not a surveyed reconstruction or an
automatic satellite-to-simulation model. No Google imagery, tiles, or textures
are distributed in this bundle.

## Quick start

Use a Python environment compatible with your CARLA server. The static checks
here used **Scenic 3.1.1 and the CARLA 0.9.16 Python API**. Your CARLA Python
package must match your server version; compatibility with other builds has not
been verified.

```bash
python -m pip install scenic==3.1.1 carla==0.9.16
python run.py --check
```

Start your CARLA 0.9.16 server separately, then run from this folder:

```bash
python run.py --seconds 60
```

The launcher replaces the server's current world with `mall.xodr`. It places
the spectator above the mall. The server viewport should show the top-down
view; `--render` additionally opens Scenic's ego-view window. Environment
actors are removed when the episode ends. The generated road world remains.

For a denser north lot or a different ego approach:

```bash
python run.py --seconds 90 --parked-stride 1 --ego-speed 4 --trigger-distance 20
```

For another server, add `--host ADDRESS --port 2000`. Stop with Ctrl+C.

## What the default scene contains

* A connected, two-way, rounded rectangular access loop: approximately 600 m
  east-west by 450 m north-south, with 3.5 m driving lanes and 2 m sidewalks.
* A 340 x 340 m central mall block and two 80 x 340 m garage blocks.
* A north entrance wing and a north surface lot with 41 fixed parked cars by
  default (164 with `--parked-stride 1`).
* An ego car, two moving NPC cars, and one pedestrian at the south entrance.
* A pedestrian who waits until the ego approaches within `trigger_distance`,
  then crosses at 1.3 m/s. The included ego uses a simple lane follower and a
  pedestrian braking rule. It is a demonstration controller, not a validated
  safe planner or your learned ego agent.

### Default graphics: block mode

`--geometry blocks` builds the massing from roughly 500 static cube props,
including a flat ground surface. This relies on the installed CARLA package
containing `/Engine/BasicShapes/Cube.Cube` and the `static.prop.mesh` blueprint.
The launcher checks for an empty mesh bounding box and stops with instructions
if the cube appears unavailable. It does not silently claim a complete scene.

The blocks are gray, have no storefront textures, and do not include painted
parking stalls. The entrance wing is rounded from 30 m to 40 m deep in this
mode. There is no building interior or garage ramp simulation. Several hundred
props can also be slower than one imported static mesh.

If this mode fails on your package, either use the imported mesh below or
`python run.py --geometry roads-only` to check roads and dynamic actors. In
roads-only mode there are no mall buildings and no supporting surface beneath
the north lot; the parked cars are fixed visual actors. It is a diagnostic mode.

### Better graphics: import the supplied environment once

`mall_environment.obj` and `mall_environment.mtl` contain the full combined
environment: simple colored buildings, ground, painted parking bays, and a
zebra crossing. This is the geometry shown in `preview.png`; that preview is
an authored plan, not a CARLA screenshot.

1. In a source/editor build of CARLA, import `mall_environment.obj` under
   `/Game/Mall/`, with materials. It is already in **Unreal coordinates and
   centimeters**; use import scale 1 and preserve its origin.
2. Import/combine it as one static mesh. In its collision settings choose
   **Use Complex Collision As Simple**; a single convex hull over the entire
   environment would incorrectly block the roads. Keep it a static object.
3. Save/cook the asset into the CARLA build you will run. An arbitrary local
   OBJ filename cannot be loaded as an Unreal mesh by the Python client.
4. Run, substituting the exact asset path shown in your content browser:

```bash
python run.py --geometry mesh --mesh-path /Game/Mall/mall_environment.mall_environment
```

This import path has not been exercised in an Unreal editor here. Mesh naming,
material import and collision must be checked in your CARLA build.

## Files and editing

| File | Purpose |
| --- | --- |
| `mall.scenic` | Actor placement, ego and NPC behavior, approach-triggered crossing |
| `mall.xodr` | Drivable road geometry and connected lane graph |
| `mall_support.py` | CARLA world generation, static geometry and BEV spectator |
| `run.py` | Launcher and offline sampling check |
| `layout.json` | Human-readable building dimensions and parking positions |
| `mall_environment.obj/.mtl` | Optional combined static environment for editor import |
| `preview.png` | Authored top-down layout, not a simulator result |
| `build_scene.py` | Regenerates map, layout JSON and OBJ/MTL from authored dimensions |
| `validate.py` | Offline map connectivity, mesh and Scenic placement checks |
| `VALIDATION.txt` | Checks actually run when preparing the bundle |

Run `python build_scene.py` after changing the dimensions in that script.
This overwrites generated XODR, JSON and OBJ/MTL files. Modify `mall.scenic`
to change behavior or replace `EntranceDrive` with your own ego integration.

The coordinates in authored files are x east, y north, z up in meters;
`mall_support.py` maps y to -y for CARLA. The OBJ export is already converted.
The map has a local origin and **is not georeferenced to an accurate survey**.

## Scope and evidence

The satellite image motivated the central mall / east-west garages / north
surface parking arrangement. Dimensions, curvatures, entrance locations,
parking rows and lane connectivity were authored, not extracted or verified
against ground truth. Nearby highways, hotels, transit, public-road junctions,
garage interiors and ramps are omitted. The north lot has no navigable lane
connection to the loop; its cars are visual context. This is not a parking
planner benchmark or a faithful Mall of America digital twin.

The crossing's start depends on observed ego distance, so it does not rely on
one absolute timestamp. That is only a first reactive rule: it does not ensure
a specified TTC, conflict angle, near-miss distance or challenge severity across
different ego policies. No collision-rate or planner-performance claims are made.

Offline checks passed, including CARLA's map parser and Scenic scene sampling.
**No CARLA server or Unreal editor was run here.** Graphics, collision shapes,
pedestrian execution, vehicle control and end-to-end runtime remain unverified.

## References

* [Google Maps satellite view inspected](https://www.google.com/maps/@44.8548,-93.2427,16z/data=!3m1!1e3)
* [Mall of America parking information](https://www.mallofamerica.com/parking)
* [CARLA OpenDRIVE standalone generation](https://carla.readthedocs.io/en/latest/adv_opendrive/)
* [CARLA static mesh props](https://carla.readthedocs.io/en/latest/content_authoring_props/)
* [Scenic CARLA model](https://docs.scenic-lang.org/en/latest/modules/scenic.simulators.carla.model.html)

Authored code, map and mesh in this bundle: CC0-1.0. Third-party tools and source
imagery retain their own licenses. No rights to third-party imagery are granted.
