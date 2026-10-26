"""Write the VLM's sentence every time it changes (ignoring the "in N meters" number, which jitters every tick), with agent time,
speed, desired speed, brake and the distance it quoted.   python text_changes.py --trace trace.jsonl --out text.txt
Works on any AGENT_TRACE jsonl with a 'text' field (SimLingo, Drive-pi0/MoE); annotate_run.py calls write_changes() for every video."""
import argparse, json, re
_NUM = re.compile(r'\s*in\s+[0-9.]+\s*meters?')
def write_changes(trace, out, src=''):
    last = None; n = 0
    with open(out, 'w') as f:
        f.write(f'# {src} sentence changes ("in N meters" ignored for change detection): agent time s | speed km/h | desired km/h | brake | quoted distance | sentence\n')
        for r in trace:
            txt = (r.get('text') or '').strip(); key = _NUM.sub('', txt); m = re.search(r'in\s+([0-9.]+)\s*meters?', txt)
            if key != last:
                last = key; n += 1; des = r.get('desired'); des = f'{des * 3.6:5.1f}' if isinstance(des, (int, float)) and des == des else '  n/a'
                f.write(f"{r['t']:7.2f} | {r['speed'] * 3.6:5.1f} | {des} | {r.get('brake', 0):.1f} | {(m.group(1) + ' m') if m else '':>7} | {key}\n")
    return n
if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--trace', required=True); ap.add_argument('--out', required=True); a = ap.parse_args()
    tr = [json.loads(l) for l in open(a.trace) if l.strip()]; n = write_changes(tr, a.out, a.trace); print(f'{n} sentence changes over {len(tr)} ticks -> {a.out}')
