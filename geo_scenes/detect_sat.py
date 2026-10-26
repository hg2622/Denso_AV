"""SAM3 text-prompt detection of parked cars and trees on the site's satellite image (run in the `sam3` env):
   detect_sat.py SITE_DIR  -> SITE_DIR/detections.json (+ sat_detections.png overlay). Frame: x east, y south (CARLA), metres from the image centre."""
import sys, json, math, torch, numpy as np, cv2
from PIL import Image
sys.path.insert(0, '/home/all/Glass_killer/sam3')
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor
D = sys.argv[1]; meta = json.load(open(f'{D}/sat.json')); mpp = meta['m_per_px']
img = Image.open(f'{D}/sat.png').convert('RGB'); W, H = img.size
model = build_sam3_image_model(checkpoint_path='/home/all/Glass_killer/sam3/sam3.pt').cuda().eval(); proc = Sam3Processor(model, confidence_threshold=0.35)
T, S, UP = 600, 500, 2   # 600 px tiles (~136 m at z19), upsampled 2x so a 4.5 m car is ~40 px
results = {'car': [], 'tree': []}
for prompt in results:
    for ty in range(0, H, S):
        for tx in range(0, W, S):
            tile = img.crop((tx, ty, min(tx + T, W), min(ty + T, H))); tw, th = tile.size; tile_up = tile.resize((tw * UP, th * UP), Image.BICUBIC)
            with torch.no_grad(): st = proc.set_image(tile_up); out = proc.set_text_prompt(prompt=prompt, state=st)
            masks = out.get('masks'); scores = out.get('scores')
            if masks is None or masks.numel() == 0: continue
            m = masks[:, 0] if masks.ndim == 4 else masks; m = m.cpu().numpy().astype(np.uint8); sc = scores.cpu().numpy().tolist() if scores is not None else [1.0] * m.shape[0]
            for k in range(m.shape[0]):
                mk = cv2.resize(m[k], (tw, th), interpolation=cv2.INTER_NEAREST); cs, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not cs: continue
                c = max(cs, key=cv2.contourArea); area = cv2.contourArea(c); (cx, cy), (bw, bh), ang = cv2.minAreaRect(c)
                results[prompt].append({'px': [float(cx + tx), float(cy + ty)], 'size_px': [float(bw), float(bh)], 'angle': float(ang), 'area_px': float(area), 'score': float(sc[k])})
    print(prompt, 'raw detections', len(results[prompt]), flush=True)
final = {}
for prompt, dets in results.items():
    keep = []
    for d in sorted(dets, key=lambda d: -d['score']):
        if any(math.hypot(d['px'][0] - k['px'][0], d['px'][1] - k['px'][1]) * mpp < 1.5 for k in keep): continue
        keep.append(d)
    for d in keep:
        d['x'] = (d['px'][0] - W / 2) * mpp; d['y'] = (d['px'][1] - H / 2) * mpp; bw, bh = d['size_px']
        d['length_m'] = max(bw, bh) * mpp; d['width_m'] = min(bw, bh) * mpp; d['yaw'] = d['angle'] + (90.0 if bh > bw else 0.0)
    final[prompt] = keep; print(prompt, 'kept', len(keep))
json.dump({'m_per_px': mpp, 'size_px': [W, H], 'detections': final}, open(f'{D}/detections.json', 'w'))
vis = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
for d in final['car']: cv2.drawContours(vis, [cv2.boxPoints(((d['px'][0], d['px'][1]), (d['size_px'][0], d['size_px'][1]), d['angle'])).astype(np.int32)], 0, (0, 255, 255), 1)
for d in final['tree']: cv2.circle(vis, (int(d['px'][0]), int(d['px'][1])), int(max(d['size_px']) / 2), (0, 200, 0), 1)
cv2.imwrite(f'{D}/sat_detections.png', vis); print('wrote detections.json + sat_detections.png')
# region prompts on the whole image (lots that OSM does not map, grass, water, building masses) -> polygons in metres
regions = {}; proc_r = Sam3Processor(model, confidence_threshold=0.3)
for prompt in ('parking lot', 'grass', 'water', 'building'):
    with torch.no_grad(): st = proc_r.set_image(img); out = proc_r.set_text_prompt(prompt=prompt, state=st)
    masks = out.get('masks'); scores = out.get('scores'); polys = []
    if masks is not None and masks.numel():
        m = (masks[:, 0] if masks.ndim == 4 else masks).cpu().numpy().astype(np.uint8); sc = scores.cpu().numpy().tolist()
        for k in range(m.shape[0]):
            mk = cv2.resize(m[k], (W, H), interpolation=cv2.INTER_NEAREST); cs, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cs:
                if cv2.contourArea(c) * mpp * mpp < 60: continue
                c = cv2.approxPolyDP(c, 2.0, True)[:, 0, :]
                polys.append({'score': float(sc[k]), 'area_m2': float(cv2.contourArea(c) * mpp * mpp), 'poly': [[float((x - W / 2) * mpp), float((y - H / 2) * mpp)] for x, y in c]})
    regions[prompt] = polys; print(prompt, 'regions', len(polys))
json.dump({'m_per_px': mpp, 'size_px': [W, H], 'frame': 'x east, y south (CARLA), metres from image centre', 'regions': regions}, open(f'{D}/regions.json', 'w'))
vis = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
for prompt, col in (('parking lot', (0, 255, 255)), ('grass', (0, 200, 0)), ('water', (255, 100, 0)), ('building', (0, 0, 255))):
    for p in regions[prompt]: cv2.polylines(vis, [np.array([[int(x / mpp + W / 2), int(y / mpp + H / 2)] for x, y in p['poly']])], True, col, 2)
cv2.imwrite(f'{D}/sat_regions.png', vis); print('wrote regions.json + sat_regions.png')
