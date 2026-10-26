# Real-world site to CARLA scene: how the reconstruction works

A place is chosen on a map, and about one minute later it is a drivable CARLA scene with the real road network, building layout,
parking lots, parked cars and trees. Figures `01` to `14` in this folder show each stage; this file is the companion text.

## Inputs (all free, no Google key)
- **OpenStreetMap** via Overpass: roads (class, name, lanes), building footprints with tags (levels, height, shop, layer, building:part), parking polygons, footways, bus stops, trees.
- **ESRI World Imagery** tiles at zoom 19 (0.23 m per pixel), stitched to a 240 to 560 m square.
- **SAM3** (text-prompt segmentation) run on that image: "car", "tree", "parking lot", "grass", "water", "building".
- **CARLA 0.9.15** with its own cooked meshes (buildings, trees, street furniture), spawned by asset path.

## Stages
1. **Pick the site** (`01`): web app `geo_scenes/site_app.py`. Search or click, choose the square size, press BUILD.
2. **Common frame**: everything is projected to UTM metres with the square's centre as origin, x east, y south = CARLA's frame.
   Satellite pixels, OSM vectors and the generated road network therefore overlay each other; measured registration error is under 5 m.
3. **Satellite image** (`02`): fetched and cropped; used only for detection.
4. **OSM vectors** (`03`): roads, footprints, lots, footways; multipolygon relations assembled into rings.
5. **SAM3 objects** (`04`): cars as rotated boxes (pose, length, width), trees as crown diameters; tiles upsampled 2x so a car is ~40 px.
6. **SAM3 regions** (`05`): lot masks pave lots OSM does not map; building masks are roofs and only feed height estimation.
7. **Road network** (`06`): OSM to OpenDRIVE with CARLA's Osm2Odr, including service roads and parking aisles.
   A sanitizer fixes what crashes the server (roads under 1.5 m, crosswalk objects past the road end) and rewrites SUMO's
   "restricted" lanes to driving lanes. Lot polygons are covered with 9 m "parking" lane strips so they render as asphalt.
8. **Asset catalogue** (`07`): CARLA meshes measured once (size, pivot) into `geo_assets.json`.
9. **Buildings** (`08`): every footprint at its tagged height.
   - Rectangles: a grid of a tag-matched mesh (church, apartments, houses) or of storey modules (retail, commercial, offices), stacked to the height.
   - Irregular outlines: storey modules laid along the exact polygon edges, interior filled, so curved contours are kept.
   - `building:part`: podium plus towers with their base levels. `layer>=2` with aisles underneath: columns, deck, store on top.
   - Underground buildings skipped. Untagged heights fall back to a parallax estimate (needs 3 tagged buildings) or type defaults.
10. **Populate**: trees at detected crowns (mesh scaled to the crown), parked cars at detected poses off the lanes, generated bays under decks,
    shop-front props on facades facing a road, bus stops, bins, bike racks, lamps on main roads, Traffic Manager cars, pedestrians on footways.
    Markings: centre-line dashes from the lane graph; parking-bay lines at each detected car and completed along its row (2.7 m pitch), as real
    lane-marking meshes (`St_Road_TileRoad_LaneMarkingBroken`), so they shade like paint instead of glowing like debug lines.
11. **Result** (`09` to `13`): aerial comparison, street views, the driving window with a satellite minimap.

## Timing (measured on a fresh 340 m site)
| Stage | Time |
|---|---|
| OSM fetch | 5 to 20 s |
| imagery | ~10 s |
| SAM3 (incl. model load) | ~35 s |
| OpenDRIVE build | ~3 s |
| load into CARLA | 7 to 10 s |
| driving tick rate | 18 to 40 ticks/s |

## Known limits (`14`)
- Off-nadir imagery displaces tall roofs from their ground footprints (parallax). CARLA builds at the footprint, which is correct, but it does not overlay the roof in the picture.
- OSM coverage decides building detail: outlines without `building:part` lose their towers; untagged towers get default heights unless `heights.json` is supplied.
- Lanes come from OSM centre lines and lane counts, not from the image; widths default to 3.5 m.
- No grass or water material can be spawned, so those areas stay flat beige. Roof shapes are generic. Lane and bay markings are painted meshes; they follow OSM lane counts and detected cars, not the image's own paint.

## Commands
```
python geo_scenes/site_app.py                       # web app: search -> BEV -> BUILD -> DRIVE
python geo_scenes/build_site.py NAME LAT LON        # same build from the shell
~/miniconda3/envs/b2d/bin/python geo_scene.py --site geo_scenes/NAME
python geo_scenes/make_pipeline_figures.py geo_scenes/NAME denso_presentaiton/geo_pipeline   # regenerate these figures
```
