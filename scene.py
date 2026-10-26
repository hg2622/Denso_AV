"""One command for the whole scene workflow: capture -> edit (2D) -> drive -> test the models.
  python scene.py --route 24211 --name myscene              capture (if needed), edit, then drive and record on save
  python scene.py --name myscene --edit-only                just edit the saved scene
  python scene.py --name myscene --drive-only               skip editing, drive the scene as saved
  python scene.py --name myscene --recapture                throw the saved scene away and capture a fresh one
  python scene.py --name myscene --models "carl bridgedrive simlingo"    ... and run the models on the recording afterwards
  python scene.py --name myscene --drive-only --driver bridgedrive    LIVE: BridgeDrive drives the ego while you drive your car
Phases: scene_capture.py (CARLA ~1 min) -> scene_edit.py (no simulator) -> scene_drive.py (CARLA, you drive) -> invader_runs.sh."""
import argparse, json, os, subprocess, sys
WS = '/home/all/carla_ws'; PY = '/home/all/miniconda3/envs/b2d/bin/python'
ap = argparse.ArgumentParser(); ap.add_argument('--name', required=True); ap.add_argument('--route', default='24211')
ap.add_argument('--edit-only', action='store_true'); ap.add_argument('--drive-only', action='store_true'); ap.add_argument('--recapture', action='store_true')
ap.add_argument('--models', default=''); ap.add_argument('--settle', type=float, default=12.0); ap.add_argument('--horizon', type=float, default=8.0)
ap.add_argument('--set', default=''); ap.add_argument('--track-name', default=''); ap.add_argument('--driver', default='carl', choices=['carl', 'bridgedrive', 'simlingo'], help='model that drives the ego live while you drive your car'); a = ap.parse_args()
scene_file = f'{WS}/scenes/{a.name}.json'; track_name = a.track_name or f'{a.name}_run'
def run(cmd, what):
    print(f'\n=== {what} ===', flush=True); r = subprocess.run(cmd, cwd=WS)
    if r.returncode not in (0, 124): sys.exit(f'{what} failed (exit {r.returncode})')
    return r.returncode
if a.recapture and os.path.exists(scene_file): os.remove(scene_file)
if not os.path.exists(scene_file) and not a.drive_only:
    run([PY, 'scene_capture.py', '--route', a.route, '--name', a.name, '--settle', str(a.settle)] + (['--set', a.set] if a.set else []), 'CAPTURE: reading the episode scene (CARLA starts, ~1 min)')
if not a.drive_only:
    print('\nEDIT: click select | drag move | +/- speed | [ ] heading | right-click or DEL remove | DOUBLE-CLICK a car = the one you drive | S saves and continues', flush=True)
    run([PY, 'scene_edit.py', '--scene', a.name, '--horizon', str(a.horizon)], 'EDIT: 2D scene editor (no simulator)')
if a.edit_only:
    S = json.load(open(scene_file)); d = next((x for x in S['actors'] if x.get('driven')), None)
    print(f"\nsaved: {len(S['actors'])} actors, you drive {d['label'] if d else '(none selected)'}"); sys.exit(0)
S = json.load(open(scene_file))
if not any(x.get('driven') for x in S['actors']):
    sys.exit('\nNo car is marked as the one you drive. Re-run and DOUBLE-CLICK a car in the editor (it gets a yellow ring and [YOU]).')
run([PY, 'scene_drive.py', '--scene', a.name, '--name', track_name, '--driver', a.driver], f'DRIVE: {a.driver} drives the ego live, you drive your car (press any key in the window to start, then W/A/S/D; ESC saves)')
track = f'{WS}/npc_tracks/{track_name}.json'
if not os.path.exists(track): sys.exit('nothing was recorded')
print(f'\nrecorded -> npc_tracks/{track_name}.json', flush=True)
if a.models:
    run(['bash', 'invader_runs.sh', a.models], f'TEST: {a.models} on your scene')
    print(f'\nvideos: selected_runs/custom/*_{S["route_id"]}_Track_{track_name}.mp4', flush=True)
else:
    print(f'\ntest the models with:\n  TRACK=npc_tracks/{track_name}.json bash invader_runs.sh "carl bridgedrive simlingo"', flush=True)
