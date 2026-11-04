"""Summarise a RIFT / CRAFT evaluation: driving score per route plus how the controlled background vehicles behaved.
    python rift_report.py [TAG ...]        TAG = <ego>-<cbv>, e.g. pdm_lite-rift_pluto (default: every run found)
With several tags it prints a comparison table, which is the point: the same ego against differently fine-tuned NPCs.
"""
import json, sys, glob, os

ROOTS = ['/home/all/carla_ws/RIFT/log/eval', '/home/all/carla_ws/CRAFT/log/eval']


def load(tag_filter):
    runs = {}
    for root in ROOTS:
        for d in sorted(glob.glob(os.path.join(root, '*'))):
            tag = os.path.basename(d)
            if tag_filter and not any(t in tag for t in tag_filter):
                continue
            f = os.path.join(d, 'simulation_results.json')
            if os.path.exists(f):
                runs[tag] = json.load(open(f))['_checkpoint']
    return runs


def per_route(cp):
    for r in cp['records']:
        m, s = r['meta'], r['scores']
        parts = r['save_name'].split('_')
        town = next((p for p in parts if p.startswith('Town')), '?')
        yield town, r['route_id'], s['score_composed'], s['score_route'], m


def main():
    runs = load(sys.argv[1:])
    if not runs:
        print('no evaluation results found'); return
    for tag, cp in runs.items():
        done, total = cp['progress']
        print(f"\n=== {tag}   ({done}/{total} routes) ===")
        print(f"{'town':<9}{'route':<26}{'DS':>5}{'RC':>5}{'NPCs':>6}{'coll':>6}{'goal':>6}{'uncomf%':>9}{'progress':>10}")
        agg = {}
        for town, rid, ds, rc, m in per_route(cp):
            unc = 100 * m['cbv_uncomfortable_game_time'] / max(m['cbv_total_game_time'], 1e-9)
            print(f"{town:<9}{rid[:25]:<26}{ds:>5.0f}{rc:>5.0f}{m['cbv_count']:>6}"
                  f"{m['cbv_collision_count']:>6}{m['cbv_reach_goal_count']:>6}{unc:>8.0f}%{m['cbv_progress']:>10.1f}")
            for k in ('cbv_count', 'cbv_collision_count', 'cbv_reach_goal_count',
                      'cbv_total_game_time', 'cbv_uncomfortable_game_time', 'cbv_off_road_game_time'):
                agg[k] = agg.get(k, 0) + m[k]
            agg['ds'] = agg.get('ds', 0) + ds
            agg['n'] = agg.get('n', 0) + 1
        if agg.get('n'):
            print(f"\n  mean driving score {agg['ds'] / agg['n']:.1f}  |  NPCs taken over {agg['cbv_count']}"
                  f"  |  collisions they caused {agg['cbv_collision_count']}"
                  f"  |  reached their goal {agg['cbv_reach_goal_count']}")
            print(f"  NPC control time {agg['cbv_total_game_time']:.0f} s, of which uncomfortable "
                  f"{100 * agg['cbv_uncomfortable_game_time'] / max(agg['cbv_total_game_time'], 1e-9):.0f}%, "
                  f"off-road {agg['cbv_off_road_game_time']:.1f} s")
    if len(runs) > 1:
        print(f"\n=== comparison ===\n{'run':<34}{'routes':>7}{'mean DS':>9}{'NPCs':>6}{'coll':>6}{'coll/NPC':>10}")
        for tag, cp in runs.items():
            rows = list(per_route(cp))
            if not rows:
                continue
            ds = sum(r[2] for r in rows) / len(rows)
            n = sum(r[4]['cbv_count'] for r in rows)
            c = sum(r[4]['cbv_collision_count'] for r in rows)
            print(f"{tag[:33]:<34}{len(rows):>7}{ds:>9.1f}{n:>6}{c:>6}{(c / n if n else 0):>10.2f}")


if __name__ == '__main__':
    main()
