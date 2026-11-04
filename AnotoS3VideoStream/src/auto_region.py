"""One-time PC input framing, NOT Anoto address decoding.

Estimate a dotted lattice's apparent pitch and select enough visible cells while
keeping their displacement resolvable after an aspect-preserving reduction.
The crop is fixed for the ENTIRE video. No frame-dependent zoom, no sharpening,
no synthesized dots, no decoded coordinates and no MCU validation bypass.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import cv2
import numpy as np

VERSION = '6.1-auto-region'

@dataclass
class DotCloud:
    points: np.ndarray
    pitch: np.ndarray
    diameter: np.ndarray
    coherence: np.ndarray
    method: str


def gray_image(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image


def _neighbours(points, k=7):
    p = np.ascontiguousarray(points, np.float32)
    if len(p) < 3:
        return np.empty((0, 0), int), np.empty((0, 0), float)
    # Only a startup framing hint: no per-frame search or output pose on PC.
    index = cv2.flann_Index(p, dict(algorithm=1, trees=2))
    ix, d2 = index.knnSearch(p, min(k, len(p)), params=dict(checks=48))
    return ix[:, 1:], np.sqrt(np.maximum(d2[:, 1:], 0.))


def detect_cloud(image, max_side=1600):
    gray = gray_image(image)
    h, w = gray.shape
    scale = min(1., max_side / max(h, w))
    if scale < 1:
        work = cv2.resize(gray, (max(1, round(w*scale)), max(1, round(h*scale))), interpolation=cv2.INTER_AREA)
    else:
        work = gray
    hh, ww = work.shape
    sx, sy = ww/w, hh/h
    _, global_mask = cv2.threshold(work, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    masks = [('otsu', global_mask)]
    block = max(15, min(201, round(min(hh, ww)*.12))) | 1
    if min(hh, ww) >= 15:
        masks.append(('local', cv2.adaptiveThreshold(work, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                                  cv2.THRESH_BINARY_INV, block, 7)))
    best = None
    for method, mask in masks:
        n, _, stats, centres = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n < 10:
            continue
        b = stats[1:]
        area = b[:, cv2.CC_STAT_AREA].astype(float)
        aspect = b[:, 2] / np.maximum(b[:, 3], 1)
        fill = area / np.maximum(b[:, 2]*b[:, 3], 1)
        keep = ((area >= 2) & (aspect > .30) & (aspect < 3.34) & (fill > .28)
                & (b[:, 2] < ww*.20) & (b[:, 3] < hh*.20)
                & (b[:, 0] > 0) & (b[:, 1] > 0)
                & (b[:, 0]+b[:, 2] < ww-1) & (b[:, 1]+b[:, 3] < hh-1))
        p = centres[1:][keep]
        a = area[keep]
        if len(p) < 9 or len(p) > 30000:
            continue
        ix, dist = _neighbours(p)
        pitch = np.median(dist[:, :4], axis=1)
        vectors = p[ix[:, :4]] - p[:, None, :]
        theta = np.arctan2(vectors[:, :, 1], vectors[:, :, 0])
        coherence = np.abs(np.mean(np.exp(4j*theta), axis=1))
        relative = a / np.maximum(pitch*pitch, 1)
        diameter = 2*np.sqrt(a/np.pi)
        keep = ((pitch > 3) & (relative > .002) & (relative < .38) & (coherence > .24))
        # Remove isolated speckles with a very different size/spacing from neighbours.
        median_d = np.median(diameter[ix[:, :4]], axis=1)
        keep &= (diameter > median_d*.35) & (diameter < median_d*2.85)
        if keep.sum() < 9:
            continue
        cloud = DotCloud(p[keep]/[sx,sy], pitch[keep]/math.sqrt(sx*sy),
                         diameter[keep]/math.sqrt(sx*sy), coherence[keep], method)
        score = len(cloud.points) * float(np.median(cloud.coherence))
        if best is None or score > best[0]:
            best = (score, cloud)
    return best[1] if best else DotCloud(np.empty((0,2)), np.empty(0), np.empty(0), np.empty(0), 'none')


def output_size(crop, max_width=240, max_height=180, max_side=0):
    _, _, w, h = crop
    scale = min(1., max_width/w, max_height/h)
    if max_side:
        scale = min(scale, max_side/max(w,h))
    return max(1, int(math.floor(w*scale+1e-8))), max(1, int(math.floor(h*scale+1e-8)))


def prepare_patch(frame, crop, max_width=240, max_height=180, max_side=0):
    x,y,w,h = map(int,crop)
    gray = gray_image(frame[y:y+h,x:x+w])
    ow,oh = output_size(crop,max_width,max_height,max_side)
    if gray.shape != (oh,ow):
        gray = cv2.resize(gray,(ow,oh),interpolation=cv2.INTER_AREA)
    return gray


def _crop_report(crop, cloud, ow, oh, offset_ratio):
    x,y,w,h = crop
    mask = ((cloud.points[:,0]>=x) & (cloud.points[:,0]<x+w)
            & (cloud.points[:,1]>=y) & (cloud.points[:,1]<y+h))
    count = int(mask.sum())
    info = dict(source_crop_xywh=list(map(int,crop)),output_size=[ow,oh],dot_candidates=count,
                estimate_only=True,coordinate_decode_on_pc=False,warnings=[])
    if count < 9:
        info.update(status='UNKNOWN', source_pitch_px=None, output_pitch_px=None,
                    output_dot_diameter_px=None, expected_offset_px=None, approx_cells_short_side=None)
        info['warnings'].append('Too few dot-like candidates to estimate scale; address validity is UNKNOWN.')
        return info
    pitch = float(np.median(cloud.pitch[mask]))
    diameter = float(np.median(cloud.diameter[mask]))
    out_pitch = pitch*min(ow/w,oh/h)
    out_diameter = diameter*min(ow/w,oh/h)
    cells = min(w,h)/pitch
    coherence = float(np.median(cloud.coherence[mask]))
    density = count*pitch*pitch/(w*h)
    info.update(source_pitch_px=pitch,output_pitch_px=out_pitch,output_dot_diameter_px=out_diameter,
                expected_offset_px=out_pitch*offset_ratio,approx_cells_short_side=cells,
                lattice_direction_coherence=coherence,relative_dot_density=density)
    if count < 64 or cells < 8.5:
        info['warnings'].append('Few visible cells; a complete verified 8x8 may not fit. Widen the source ROI.')
    if out_pitch < 9 or out_pitch*offset_ratio < 1.4:
        info['warnings'].append('Predicted displacement is small after scaling. Select a smaller source ROI.')
    if out_diameter < 2:
        info['warnings'].append('Predicted dots are below 2 pixels; focus/source sampling may be insufficient.')
    if coherence < .38 or density < .45:
        info['warnings'].append('Weak/partial lattice estimate; inspect the input preview. No decoding guarantee.')
    info['status'] = 'CHECK_INPUT' if info['warnings'] else 'SAMPLING_ESTIMATE_OK'
    return info


def inspect_crop(frame,crop,max_width=240,max_height=180,max_side=0,offset_ratio=1/6):
    x,y,w,h = map(int,crop)
    cloud = detect_cloud(frame[y:y+h,x:x+w])
    cloud.points += [x,y]
    ow,oh = output_size(crop,max_width,max_height,max_side)
    info = _crop_report(crop,cloud,ow,oh,offset_ratio)
    actual = detect_cloud(prepare_patch(frame,crop,max_width,max_height,max_side))
    info['output_detected_dot_candidates'] = len(actual.points)
    if len(actual.pitch):
        info['measured_output_pitch_px'] = float(np.median(actual.pitch))
        info['measured_output_diameter_px'] = float(np.median(actual.diameter))
    else:
        info['measured_output_pitch_px'] = info['measured_output_diameter_px'] = None
    if info['dot_candidates'] >= 64 and len(actual.points) < 64:
        info['warnings'].append('Fewer than 64 dot candidates remain after reduction; inspect contrast/focus/ROI.')
        info['status']='CHECK_INPUT'
    return info


def automatic_crop(frame,max_width=240,max_height=180,target_cells=12.,offset_ratio=1/6):
    """Choose ONE fixed image rectangle. Dots are framing hints, NOT decoded symbols."""
    if not 9 <= target_cells <= 18:
        raise ValueError('--target-cells must be between 9 and 18.')
    h,w = frame.shape[:2]
    cloud = detect_cloud(frame)
    if len(cloud.points) < 64:
        raise ValueError('자동 영역 선택: 규칙적인 점 후보 64개를 찾지 못했습니다. '
                         '--select-time 1 로 다른 시점을 쓰거나 --roi 로 약 10~14줄을 선택하세요. '
                         '이 버전은 선택한 큰 ROI도 자동으로 축소합니다.')
    centre = np.array([(w-1)/2,(h-1)/2])
    # Geometrically different anchors, all snapped into observed dot regions.
    anchors=[centre,np.median(cloud.points,axis=0)]
    for yy in (.2,.5,.8):
        for xx in (.2,.5,.8):
            anchor=np.array([xx*w,yy*h])
            distances=np.sum((cloud.points-anchor)**2,axis=1)
            anchors.append(cloud.points[int(np.argmin(distances))])
    candidates=[(0,0,w,h)]
    for anchor in anchors:
        distances=np.sum((cloud.points-anchor)**2,axis=1)
        ids=np.argsort(distances)[:min(100,len(distances))]
        pitch=float(np.median(cloud.pitch[ids]))
        for aspect in (max_width/max_height, 1., min(1.,w/h)):
            for cells in (target_cells,target_cells+1.5,max(9.,target_cells-1.5)):
                ch=cells*pitch;cw=ch*aspect
                fit=min(1.,w/cw,h/ch);cw=max(32,round(cw*fit));ch=max(32,round(ch*fit))
                x=max(0,min(w-cw,round(anchor[0]-cw/2)))
                y=max(0,min(h-ch,round(anchor[1]-ch/2)))
                candidates.append((x,y,cw,ch))
    best = None
    for crop in dict.fromkeys(candidates):
        x,y,cw,ch=crop
        ow,oh=output_size(crop,max_width,max_height)
        info=_crop_report(crop,cloud,ow,oh,offset_ratio)
        if info['dot_candidates']<64 or info['source_pitch_px'] is None:continue
        if info['approx_cells_short_side']<8.5 or info['output_pitch_px']<8:continue
        if info['lattice_direction_coherence']<.35 or info['relative_dot_density']<.4:continue
        # Sampling estimate, NOT an address verification or physical accuracy test.
        density=info['relative_dot_density']
        score=(4*min(1.,density)+3*info['lattice_direction_coherence']
               -.22*abs(info['approx_cells_short_side']-target_cells)
               -.7*max(0.,2.5-info['output_dot_diameter_px'])
               -.8*float(np.linalg.norm(np.array([x+cw/2,y+ch/2])-centre))/max(w,h))
        if best is None or score>best[0]:best=(score,crop,info)
    if best is None:
        raise ValueError('자동 영역 선택 실패: 충분한 점 개수와 출력 해상도를 동시에 확보하지 못했습니다. '
                         '--roi 로 약 10~14줄을 선택하세요. 원본에 4x4만 보이면 더 넓게 촬영해야 합니다.')
    crop=best[1]
    report=inspect_crop(frame,crop,max_width,max_height,offset_ratio=offset_ratio)
    report.update(selection='automatic_dot_pitch',target_cells_short_side=float(target_cells))
    return crop,report
