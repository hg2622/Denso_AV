"""Render an original authored plan, not a Google or CARLA screenshot."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as E
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parent

def main():
    data=json.loads((ROOT/'layout.json').read_text())
    fig,ax=plt.subplots(figsize=(10,10),facecolor='#f4f3ef')
    ax.set_facecolor('#e2e7dc')
    ax.add_patch(Rectangle((-340,-280),680,720,color='#b4b8b7'))
    for g in E.parse(ROOT/'mall.xodr').findall('.//geometry'):
        x,y,h,l=[float(g.get(k)) for k in ['x','y','hdg','length']]
        s=np.linspace(0,l,max(2,int(l/2)))
        if g.find('arc') is None: xs=x+s*np.cos(h);ys=y+s*np.sin(h)
        else:
            k=float(g.find('arc').get('curvature'))
            xs=x+(np.sin(h+k*s)-np.sin(h))/k
            ys=y-(np.cos(h+k*s)-np.cos(h))/k
        ax.plot(xs,ys,color='#e9e1cf',lw=17,solid_capstyle='butt')
        ax.plot(xs,ys,color='#444e55',lw=11,solid_capstyle='butt')
        ax.plot(xs,ys,color='#e4c85d',lw=1,ls='--')
    for b in data['buildings']:
        ax.add_patch(Rectangle((b['x']-b['w']/2,b['y']-b['d']/2),b['w'],b['d'],
                     facecolor='#bbac96' if b['material']=='mall' else '#82929e',edgecolor='#33444e',lw=1.2))
        if 'garage' in b['name'].lower():
            for y in range(-155,161,12):ax.plot([b['x']-32,b['x']+32],[y,y],color='#dae0e1',lw=.6)
            ax.text(b['x'],0,b['name'],ha='center',va='center',rotation=90,color='#172a36',fontsize=11,weight='bold')
        elif b['name']=='Main mall':
            ax.add_patch(Rectangle((-80,-70),160,150,facecolor='#9db4ba',edgecolor='#647e86',lw=1))
            ax.text(0,20,'CENTRAL MALL',ha='center',fontsize=15,weight='bold',color='#253947')
            ax.text(0,-10,'Approximate 340 × 340 m massing',ha='center',fontsize=9,color='#253947')
        else: ax.text(0,185,'ENTRANCE WING',ha='center',va='center',fontsize=8,color='#253947')
    for row_y in [290,306,350,366]:
        for x in range(-246,247,3):ax.plot([x,x],[row_y-2.5,row_y+2.5],color='white',lw=.5)
    for p in data['parked_cars'][::4]:
        ax.add_patch(Rectangle((p['x']-1,p['y']-2.25),2,4.5,facecolor='#405a6b'))
    ax.text(0,402,'NORTH SURFACE PARKING',ha='center',fontsize=11,weight='bold',color='#253947')
    ax.text(0,384,'Visual context • no lane connection in this starter',ha='center',fontsize=8,color='#253947')
    for x in range(37,44):ax.add_patch(Rectangle((x,-230.5),.5,11,color='white'))
    ax.scatter([-80,40],[-226.75,-232],s=[55,35],c=['#198978','#df693c'],zorder=9,edgecolor='white')
    ax.annotate('Ego →',(-80,-226.75),xytext=(-145,-258),fontsize=10,color='#126c5e',weight='bold')
    ax.annotate('Approach-triggered crossing',(40,-225),xytext=(65,-260),fontsize=9,color='#ac4928',
                arrowprops={'arrowstyle':'-','color':'#ac4928'})
    ax.annotate('N',(-312,390),xytext=(-312,340),ha='center',fontsize=12,
                arrowprops={'arrowstyle':'-|>','color':'#253947'},color='#253947')
    ax.plot([160,260],[-280,-280],color='#253947',lw=2)
    ax.text(210,-295,'100 m',ha='center',fontsize=9,color='#253947')
    ax.set(xlim=(-365,365),ylim=(-315,450),aspect='equal',xlabel='Local east (m)',ylabel='Local north (m)')
    ax.spines[['top','right']].set_visible(False)
    ax.tick_params(colors='#64727a')
    fig.suptitle('Mall of America inspired CARLA starter',fontsize=19,weight='bold',y=.97,color='#253947')
    ax.set_title('Authored layout preview • approximate geometry • not a CARLA render',fontsize=10,pad=15,color='#64727a')
    fig.text(.5,.025,'Central mall + two garages + access loop + parked cars + reactive pedestrian',ha='center',fontsize=10,color='#4d626f')
    fig.tight_layout(rect=[0,.045,1,.95])
    fig.savefig(ROOT/'preview.png',dpi=160)

if __name__=='__main__':main()
