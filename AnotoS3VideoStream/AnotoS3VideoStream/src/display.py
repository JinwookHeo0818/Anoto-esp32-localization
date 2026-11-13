"""Clean causal display. PC draws replies from either PC C++ or the MCU.

The large image is the present replay frame. A yellow 8x8 outline is drawn ONLY
on its own measured frame (small panel). No delayed box on a different image.
Six numeric fields: observation time, X, Y, heading, MCU compute, sample age.
"""
from __future__ import annotations
import argparse
from bisect import bisect_right
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np
from common import load_manifest


class Dashboard:
    def __init__(self,prepared,m,setup,hardware='UNKNOWN'):
        self.m=m;self.setup=setup;self.hardware=hardware
        self.w=m['width'];self.h=m['height']
        self.width=1080;self.height=720
        self.raw=np.memmap(Path(prepared)/'frames.gray',dtype=np.uint8,mode='r',
                           shape=(m['frame_count'],self.h,self.w))
        self.times=[f['time_us']/1e6 for f in m['frames']]
        self.cursor=0;self.latest=None;self.last_point=None;self.valid_count=0
        self.path=deque(maxlen=4000)
        self.yellow=(0,211,244)
        self.bg=(23,26,32);self.card=(34,39,47)
        self.ink=(239,242,246);self.muted=(151,163,179)
        self._map_cache=None;self._map_cursor=-1

    def consume(self,t,records):
        while self.cursor<len(records) and records[self.cursor]['available_wall_s']<=t+1e-9:
            r=records[self.cursor];self.cursor+=1;self.latest=r
            if r['valid']:
                if self.last_point is not None:
                    dt=r['source_time_s']-self.last_point['source_time_s']
                    d=math.hypot(r['x_mm']-self.last_point['x_mm'],r['y_mm']-self.last_point['y_mm'])
                    if 0<dt<=.5 and d<=50:
                        self.path.append(((self.last_point['x_mm'],self.last_point['y_mm']),
                                          (r['x_mm'],r['y_mm'])))
                self.last_point=r;self.valid_count+=1
            else:
                self.last_point=None

    def state_at(self,t):
        if self.latest is None:return 'WAITING'
        r=self.latest
        if not r['valid']:return 'LOST'
        age=(t-r['source_time_s'])*1000.
        return 'FRESH' if r['accepted_live'] and 0<=age<=self.setup['max_age_ms'] else 'STALE'

    def _image(self,canvas,gray,rect,box=None,reference=False):
        x,y,w,h=rect
        canvas[y:y+h,x:x+w]=self.card
        scale=min(w/gray.shape[1],h/gray.shape[0])
        sw=max(1,round(gray.shape[1]*scale));sh=max(1,round(gray.shape[0]*scale))
        image=cv2.cvtColor(cv2.resize(gray,(sw,sh),interpolation=cv2.INTER_NEAREST),cv2.COLOR_GRAY2BGR)
        if box is not None:
            p=np.asarray(box,float)
            if p.shape==(4,2) and np.isfinite(p).all() and np.max(abs(p))<1e6:
                p=np.rint((p+.5)*[sw/gray.shape[1],sh/gray.shape[0]]-.5).astype(np.int32)
                cv2.polylines(image,[p],True,self.yellow,2,cv2.LINE_AA)
        if reference:
            cx,cy=(sw-1)//2,(sh-1)//2
            cv2.drawMarker(image,(cx,cy),(15,19,22),cv2.MARKER_CROSS,19,4)
            cv2.drawMarker(image,(cx,cy),self.yellow,cv2.MARKER_CROSS,19,2)
            a=math.radians(self.setup['settings']['forward_image_deg'])
            cv2.arrowedLine(image,(cx,cy),(round(cx+25*math.sin(a)),round(cy-25*math.cos(a))),
                            self.yellow,2,cv2.LINE_AA,tipLength=.25)
        x+=max(0,(w-sw)//2);y+=max(0,(h-sh)//2)
        canvas[y:y+sh,x:x+sw]=image

    def _map(self,fresh):
        # Redraw path only when observations change, not for every screen refresh.
        key=(self.cursor,fresh)
        if self._map_cursor==key:return self._map_cache
        w,h=368,184
        image=np.full((h,w,3),self.card,np.uint8)
        for x in range(0,w,46):cv2.line(image,(x,0),(x,h-1),(44,50,60),1)
        for y in range(0,h,46):cv2.line(image,(0,y),(w-1,y),(44,50,60),1)
        points=[p for pair in self.path for p in pair]
        if self.last_point is not None:points.append((self.last_point['x_mm'],self.last_point['y_mm']))
        if points:
            pts=np.asarray(points,float);lo=pts.min(0);hi=pts.max(0);mid=(lo+hi)/2
            factor=min((w-24)/max(10.,hi[0]-lo[0])*0.85,(h-24)/max(10.,hi[1]-lo[1])*0.85)
            def mp(p):return tuple(np.rint((np.asarray(p)-mid)*factor+[w/2,h/2]).astype(int))
            for a,b in self.path:cv2.line(image,mp(a),mp(b),self.yellow,1,cv2.LINE_AA)
            r=self.latest
            if fresh and r:
                p=mp((r['x_mm'],r['y_mm']));cv2.circle(image,p,4,self.yellow,-1,cv2.LINE_AA)
                a=math.radians(r['heading_deg'])
                q=(round(p[0]+18*math.sin(a)),round(p[1]-18*math.cos(a)))
                cv2.arrowedLine(image,p,q,self.yellow,2,cv2.LINE_AA,tipLength=.3)
        self._map_cursor=key;self._map_cache=image
        return image

    def draw(self,t,records):
        self.consume(t,records)
        idx=max(0,min(len(self.times)-1,bisect_right(self.times,t)-1))
        c=np.full((self.height,self.width,3),self.bg,np.uint8)
        def text(s,x,y,size=.5,color=None,weight=1):
            cv2.putText(c,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,
                        self.ink if color is None else color,weight,cv2.LINE_AA)
        state=self.state_at(t);fresh=state=='FRESH'
        decoded=self.latest is not None and self.latest['valid']
        status_color={'FRESH':(115,219,151),'WAITING':self.muted,'STALE':(78,186,246),'LOST':(110,124,247)}[state]
        title='ANOTO  /  '+self.setup.get('backend','MCU')
        if 'HOST' in self.hardware:title='HOST SIMULATION  /  NOT ESP32'
        text(title,24,40,.74,weight=2)
        text(f'{max(0,t):.2f} / {self.m["source_duration_s"]:.2f} s',720,40,.62)
        cv2.rectangle(c,(922,14),(1056,49),self.card,-1)
        text(state,939,38,.58,status_color,2)
        # Progress has no effect on frame choice/measurement.
        p=max(0,min(1,t/self.m['source_duration_s']))
        cv2.line(c,(24,62),(1056,62),self.card,3)
        cv2.line(c,(24,62),(24+round(1032*p),62),self.yellow,3)
        text('LIVE INPUT',24,90,.46,self.muted)
        self._image(c,self.raw[idx],(24,100,640,422),reference=True)
        text('LAST MEASURED FRAME',688,90,.46,self.muted)
        r=self.latest
        if r is not None:
            box=r.get('box') if decoded else None
            self._image(c,self.raw[r['frame']],(688,100,368,210),box=box)
        else:
            c[100:310,688:1056]=self.card
            text('Waiting for '+self.setup.get('backend','MCU')+'...',710,211,.52,self.muted)
        text('MEASURED PATH',688,330,.46,self.muted)
        c[338:522,688:1056]=self._map(decoded)
        values=['--']*6
        if r is not None:
            values[0]=f"{r['source_time_s']:.3f}"
            values[4]=f"{r['compute_ms']:.2f}"
            values[5]=f"{max(0.,(t-r['source_time_s'])*1000):.1f}"
            if decoded:
                values[1:4]=[f"{r['x_mm']:.3f}",f"{r['y_mm']:.3f}",f"{r['heading_deg']:.2f}"]
        for k,(label,val) in enumerate(zip(['OBS TIME  s','X  mm','Y  mm','HEADING  deg',self.setup.get('backend','MCU')+'  ms','AGE  ms'],values)):
            x=24+k*174
            cv2.rectangle(c,(x,546),(x+162,660),self.card,-1)
            text(label,x+12,572,.42,self.muted)
            size=.82 if len(val)<9 else .66
            text(val,x+12,625,size,status_color if decoded and k in (1,2,3) else self.ink,2)
        text('Coordinates = LAST decoded observation, not current pose. STALE stays visible.',24,688,.43,self.muted)
        text('Q / ESC  stop',933,688,.4,self.muted)
        return c

    def close(self):
        raw=getattr(self,'raw',None)
        if raw is not None and getattr(raw,'_mmap',None) is not None:raw._mmap.close()
        self.raw=None


class LivePreview:
    def __init__(self,prepared,m,setup,hello):
        self.dash=Dashboard(prepared,m,setup,hello.get('hardware','UNKNOWN'))
        self.name='Anoto '+setup.get('backend','MCU')+' | X  Y  Heading'
        cv2.namedWindow(self.name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.name,self.dash.width,self.dash.height)
    def show(self,t,records):
        cv2.imshow(self.name,self.dash.draw(t,records))
        key=cv2.waitKey(1)&255
        try:visible=cv2.getWindowProperty(self.name,cv2.WND_PROP_VISIBLE)>=1
        except cv2.error:visible=False
        return visible and key not in (ord('q'),ord('Q'),27)
    def close(self):
        self.dash.close();cv2.destroyAllWindows()



def render_video(prepared,out,m,setup,hello,records):
    """After-replay export; no files required beyond the prepared input and results."""
    prepared=Path(prepared);out=Path(out)
    dash=Dashboard(prepared,m,setup,hello.get('hardware','UNKNOWN'))
    writer=None;dest=None
    for name,codec in [('annotated.mp4','mp4v'),('annotated.avi','MJPG')]:
        dest=out/name
        writer=cv2.VideoWriter(str(dest),cv2.VideoWriter_fourcc(*codec),m['source_fps'],(dash.width,dash.height))
        if writer.isOpened():break
        writer.release();writer=None
        if dest.exists():dest.unlink()
    if writer is None:
        dash.close();raise RuntimeError('Cannot create MP4/AVI. CSV is preserved.')
    total=max(1,math.ceil(m['source_duration_s']*m['source_fps']-1e-3))
    try:
        for k in range(total):
            writer.write(dash.draw(k/m['source_fps'],records))
    finally:
        writer.release();dash.close()
    return dest
