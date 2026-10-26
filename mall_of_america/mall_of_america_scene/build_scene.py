"""Regenerate the authored, approximate mall layout; no imagery is downloaded."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parent

def make_map():
    root = E.Element('OpenDRIVE')
    E.SubElement(root, 'header', revMajor='1', revMinor='4', name='Mall approximation',
                 version='1.00', date='2026-10-16', north='450', south='-270', east='340', west='-340')
    # Eight connected roads: four straights and four 30 m radius corners.
    segments = [( -270,-225,0,540,0), (270,-225,0,math.pi*15,1/30),
                (300,-195,math.pi/2,390,0), (300,195,math.pi/2,math.pi*15,1/30),
                (270,225,math.pi,540,0), (-270,225,math.pi,math.pi*15,1/30),
                (-300,195,3*math.pi/2,390,0), (-300,-195,3*math.pi/2,math.pi*15,1/30)]
    for i,(x,y,h,length,curvature) in enumerate(segments):
        road = E.SubElement(root,'road',name=f'Access loop {i+1}',length=str(length),id=str(i+1),junction='-1')
        link = E.SubElement(road,'link')
        E.SubElement(link,'predecessor',elementType='road',elementId=str((i-1)%8+1),contactPoint='end')
        E.SubElement(link,'successor',elementType='road',elementId=str((i+1)%8+1),contactPoint='start')
        typ=E.SubElement(road,'type',s='0',type='town'); E.SubElement(typ,'speed',max='20',unit='mph')
        geo=E.SubElement(E.SubElement(road,'planView'),'geometry',s='0',x=str(x),y=str(y),hdg=str(h),length=str(length))
        E.SubElement(geo,'arc',curvature=str(curvature)) if curvature else E.SubElement(geo,'line')
        E.SubElement(E.SubElement(road,'elevationProfile'),'elevation',s='0',a='0',b='0',c='0',d='0')
        lanes=E.SubElement(road,'lanes'); section=E.SubElement(lanes,'laneSection',s='0')
        for side,sign in [('left',1),('center',0),('right',-1)]:
            group=E.SubElement(section,side)
            ids=[0] if sign==0 else [sign,sign*2]
            for lid in ids:
                lane=E.SubElement(group,'lane',id=str(lid),type=('none' if lid==0 else 'driving' if abs(lid)==1 else 'sidewalk'),level='false')
                if lid:
                    ll=E.SubElement(lane,'link')
                    E.SubElement(ll,'predecessor',id=str(lid)); E.SubElement(ll,'successor',id=str(lid))
                    E.SubElement(lane,'width',sOffset='0',a='3.5' if abs(lid)==1 else '2',b='0',c='0',d='0')
                E.SubElement(lane,'roadMark',sOffset='0',type='solid',weight='standard',color='yellow' if lid==0 else 'white',width='0.12')
    E.indent(root)
    E.ElementTree(root).write(ROOT/'mall.xodr',encoding='utf-8',xml_declaration=True)

def layout():
    return dict(
        description='Manually authored Mall of America inspired massing; dimensions approximate.',
        units='meters',coordinates='x east, y north, z up; CARLA y = -y',
        source='https://www.google.com/maps/@44.8548,-93.2427,16z/data=!3m1!1e3',
        buildings=[dict(name='Main mall',x=0,y=0,w=340,d=340,h=20,material='mall'),
                   dict(name='West garage',x=-230,y=0,w=80,d=340,h=20,material='garage'),
                   dict(name='East garage',x=230,y=0,w=80,d=340,h=20,material='garage'),
                   dict(name='North entrance wing',x=0,y=185,w=280,d=30,h=20,material='mall')],
        parked_cars=[dict(x=x,y=y,heading=0 if row%2==0 else 180)
                     for row,y in enumerate([290,306,350,366]) for x in range(-240,241,12)],
        crossing=dict(x=40,y=-225,width=6),
        omitted=['Garage interiors and ramps','Actual entrances and public-road junctions',
                 'Navigable connection to north surface lot','Exact lane and building survey geometry'])

def make_obj(data):
    # Export CARLA/Unreal coordinates, centimeters, one combined mesh.
    # Detailed mesh is optional: it needs one-time import into the CARLA editor.
    verts=[]; faces=[]
    def box(x,y,z,w,d,h,mat):
        n=len(verts)
        verts.extend([(100*(x+sx*w/2),-100*(y+sy*d/2),100*(z+sz*h/2))
                      for sx,sy,sz in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                       (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
        # Reflection in y reverses winding relative to standard right-handed boxes.
        for f in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            faces.append((mat,[n+i+1 for i in reversed(f)]))
    box(0,80,-0.12,680,720,0.2,'ground')
    for b in data['buildings']:
        box(b['x'],b['y'],b['h']/2,b['w'],b['d'],b['h'],b['material'])
    box(0,5,20.03,160,150,0.04,'glass')
    for x in [-230,230]:
        for y in range(-155,161,12):box(x,y,20.02,64,0.12,0.02,'stripe')
    # North surface lot marking. These are visual strips, not separate roads.
    for row_y in [290,306,350,366]:
        for x in range(-246,247,3): box(x,row_y,0.005,0.10,5,0.01,'stripe')
    # South entrance zebra crossing, spanning both lanes and both sidewalks.
    for x in range(37,44): box(x,-225,0.025,0.5,11,0.01,'stripe')
    with (ROOT/'mall_environment.obj').open('w') as f:
        f.write('# Approximate authored mall, Unreal coordinates in centimeters\nmtllib mall_environment.mtl\no MallEnvironment\n')
        for v in verts:f.write('v %g %g %g\n'%v)
        for mat,face in faces:f.write('usemtl '+mat+'\nf '+' '.join(map(str,face))+'\n')
    (ROOT/'mall_environment.mtl').write_text(''.join(f'newmtl {name}\nKd {color}\nKa 0.1 0.1 0.1\n\n' for name,color in
        [('ground','0.16 0.18 0.19'),('mall','0.65 0.60 0.52'),('garage','0.46 0.49 0.52'),
         ('glass','0.48 0.65 0.70'),('stripe','0.95 0.95 0.88')]))

if __name__=='__main__':
    make_map(); data=layout()
    (ROOT/'layout.json').write_text(json.dumps(data,indent=2)+'\n')
    make_obj(data)
    print('Generated mall.xodr, layout.json, mall_environment.obj and .mtl')
