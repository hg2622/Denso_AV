# carla_ws — CARLA 0.9.15 + Bench2Drive

> Start with **SETUP.txt** (repo map + dependencies) and **COMMANDS.txt** (the manual). This README keeps the original
> machine-setup notes (hybrid GPU / Vulkan fix, python 3.10 instead of 3.8).


Set up 2026-09-28 on Ubuntu 24.04 / RTX 5070 Ti Laptop (12 GB) / driver 595.

```
carla/            CARLA 0.9.15 + AdditionalMaps (Town01-07, 10HD, 11, 12, 13, 15)  ~44 GB
                  CarlaUE4.sh is a wrapper that pins Vulkan to the NVIDIA ICD (see below);
                  original launcher is CarlaUE4.sh.orig
Bench2Drive/      github.com/Thinklab-SJTU/Bench2Drive (leaderboard + scenario_runner vendored)
Bench2DriveZoo/   branch tcp/admlp — baseline agents; linked into Bench2Drive/leaderboard/team_code
data/Bench2Drive-mini/   10 clips of the offline dataset (2.7 GB, tar.gz each)
data/ckpts/tcp_b2d.ckpt  TCP baseline checkpoint
run_tcp_eval.sh   closed-loop eval runner (see below)
logs/
```

## Env
conda env `b2d` (Python 3.10, NOT the 3.8 the README asks for — the RTX 5070 Ti is Blackwell
and needs torch>=2.7/cu128, which has no py3.8 build). `carla==0.9.15` from PyPI wheel instead of
the tarball .egg. Consequences handled:
- `Element.getchildren()` (removed in py3.9) patched to `list(elem)` in 3 files (see `git diff` in Bench2Drive)
- ancient pins relaxed: numpy 1.26, shapely 1.8, networkx 2.8, opencv 4.11, setuptools<70 (for pkg_resources)

## Gotcha: hybrid GPU
UE4 enumerates the Intel iGPU first → CARLA renders at ~7 fps on the iGPU and NVIDIA sits idle.
Fix is baked into `carla/CarlaUE4.sh`: `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`.
With it: ~88 fps at 800x600 sync mode, ~6 GB VRAM.

## Plain CARLA
```
cd ~/carla_ws/carla && ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
~/miniconda3/envs/b2d/bin/python -c "import carla; c=carla.Client('localhost',2000); print(c.get_world().get_map().name)"
```

## Bench2Drive closed-loop episode (TCP agent)
```
cd ~/carla_ws && bash run_tcp_eval.sh leaderboard/data/drivetransformer_bench2drive_dev10.xml 25378 ./eval_dev10
```
args: routes xml, route id (or `25378,2091` / omit for all), save dir (relative to Bench2Drive/).
dev10 route ids: 3514 3255 26405 25381 25378 25424 2091 27494 17569 28198. Full benchmark: bench2drive220.xml.
The evaluator launches CARLA itself on port 30000 (TM 50000). Output: `<save>/RouteScenario_*/{rgb,rgb_front,bev,meta}`
and `<save>/result.json`. Sim ratio ~0.075x (≈13 s wall per sim second).
Clean up stuck servers: `bash Bench2Drive/tools/clean_carla.sh`.

## Offline dataset
`tar xzf data/Bench2Drive-mini/<clip>.tar.gz` → anno/*.json.gz, camera/rgb_{front,...}/*.jpg, lidar/*.laz, radar/*.h5
Format doc: Bench2Drive/docs/anno.md. Larger splits: HF `rethinklab/Bench2Drive` (Base, 334 GB), `rethinklab/Bench2Drive-V0.0.4`.

## Free driving quick ref
```
bash carla_server.sh                                   # terminal 1
bash drive.sh --map Town12 --res 1920x1080 --filter vehicle.dodge.charger_2020   # terminal 2
bash traffic.sh 150 80 --car-lights-on                 # terminal 3 (Ctrl+C to remove NPCs)
```
Maps: Town12/13 (10x10 km, highway ring), Town04 (highway loop), Town06 (long highways), Town10HD (city).
## Human Bench2Drive episode
```
bash run_human_eval.sh 25378 | random | random220     # scored; bash show_score.sh ./eval_human
```
