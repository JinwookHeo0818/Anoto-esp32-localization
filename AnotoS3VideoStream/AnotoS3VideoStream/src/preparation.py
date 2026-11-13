"""Prepare fixed-ROI GRAY8 frames. No Anoto address decoding in this module.

Cache is tied to source bytes AND preparation options. Only one prepared video
is retained. Original scale is kept on PC; MCU uses v6.1 auto-region/resizing.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import zlib
import cv2
import numpy as np
from common import load_manifest, write_json

VERSION = 'organized-2026-10-30'


def digest_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def save_png(path: Path, image: np.ndarray) -> None:
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise OSError(f'PNG 저장 실패: {path}')
    data.tofile(str(path))


def validate_crop(crop, width, height):
    x, y, w, h = map(int, crop)
    if min(x, y) < 0 or min(w, h) < 32 or x + w > width or y + h > height:
        raise ValueError('선택 영역은 원본 안에 있어야 하고 가로·세로가 32픽셀 이상이어야 합니다.')
    return x, y, w, h


def choose_roi(frame: np.ndarray) -> tuple[int, int, int, int]:
    h, w = frame.shape[:2]
    scale = min(1.0, 1100/w, 750/h)
    small = cv2.resize(frame, (round(w*scale), round(h*scale))) if scale < 1 else frame
    box = cv2.selectROI('Select about 10-14 dot rows/columns. ENTER=OK, ESC=cancel', small, False, False)
    cv2.destroyAllWindows()
    x, y, cw, ch = [int(round(v/scale)) for v in box]
    if min(cw, ch) < 1:
        raise ValueError('영역 선택을 취소했습니다.')
    return x, y, min(cw, w-x), min(ch, h-y)


def at_time(source: Path, seconds: float, shape: tuple[int, int]) -> np.ndarray | None:
    cap = cv2.VideoCapture(str(source))
    try:
        cap.set(cv2.CAP_PROP_POS_MSEC, seconds*1000)
        ok, image = cap.read()
        return image if ok and image is not None and image.shape[:2] == shape else None
    finally:
        cap.release()


def make_preview(frame, crop, data, quality):
    """Display fits on screen; scaling here never changes the transmitted pixels."""
    canvas = np.full((610, 1120, 3), 28, np.uint8)
    x,y,w,h=crop
    view=frame.copy()
    cv2.rectangle(view,(x,y),(x+w-1,y+h-1),(0,210,240),max(2,round(max(frame.shape[:2])/650)))
    for im, x0, label in [(view,18,'SOURCE / FIXED REGION'),(cv2.cvtColor(data,cv2.COLOR_GRAY2BGR),578,'ACTUAL INPUT / DISPLAY ONLY')]:
        scale=min(524/im.shape[1],430/im.shape[0])
        size=(max(1,round(im.shape[1]*scale)),max(1,round(im.shape[0]*scale)))
        interp=cv2.INTER_NEAREST if x0==578 else cv2.INTER_AREA
        im=cv2.resize(im,size,interpolation=interp)
        yy=68+(430-size[1])//2;xx=x0+(524-size[0])//2
        canvas[yy:yy+size[1],xx:xx+size[0]]=im
        cv2.putText(canvas,label,(x0,38),cv2.FONT_HERSHEY_SIMPLEX,.56,(235,235,235),1,cv2.LINE_AA)
    lines=[f'Selected {w} x {h}  ->  decoder {data.shape[1]} x {data.shape[0]} pixels',
           'Fixed crop for the full video. No per-frame recentering or invented dots.']
    if quality and quality.get('output_pitch_px') is not None:
        lines.append(f"Estimated pitch {quality['output_pitch_px']:.2f}px | dot {quality['output_dot_diameter_px']:.2f}px | offset {quality['expected_offset_px']:.2f}px")
    for i,text in enumerate(lines):
        cv2.putText(canvas,text,(18,530+27*i),cv2.FONT_HERSHEY_SIMPLEX,.52,(235,235,235),1,cv2.LINE_AA)
    return canvas


def prepare_input(source: Path, args, root: Path, mode: str) -> Path:
    """Returns .cache/input. A failed preparation is never reused as valid input."""
    source=source.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f'동영상이 없습니다: {source}')
    cache=root/'.cache';cache.mkdir(exist_ok=True)
    target=cache/'input'
    request=dict(version=VERSION,mode=mode,source_sha256=digest_file(source),
        roi=args.roi,crop=args.crop,auto=args.auto,full_frame=args.full_frame,
        select_time=args.select_time,max_side=args.max_side,source_fps=args.source_fps,
        target_cells=args.target_cells,offset_ratio=args.offset_mm/args.pitch_mm)
    if target.exists() and not args.reprepare:
        try:
            m=load_manifest(target)
            if m.get('cache_request')==request:
                print(f"입력 재사용 | {m['width']}x{m['height']} | {m['source_duration_s']:.3f}s")
                return target
        except (OSError, ValueError, KeyError):
            pass
    temp=cache/'input_building'
    if temp.exists():
        shutil.rmtree(temp)
    cap=cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise ValueError('영상을 열 수 없습니다. 확장자만 바꾸지 말고 정상 MP4/H.264로 내보내세요.')
    try:
        fps=args.source_fps or float(cap.get(cv2.CAP_PROP_FPS))
        if not math.isfinite(fps) or not .1<=fps<=1000:
            raise ValueError('촬영 FPS를 확인할 수 없습니다. 실제 값으로 --source-fps를 지정하세요.')
        reported=max(0,int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        ok,frame=cap.read()
        if not ok or frame is None:
            raise ValueError('동영상 첫 프레임을 읽을 수 없습니다.')
        sh,sw=frame.shape[:2]
        selection=at_time(source,args.select_time,(sh,sw)) if args.select_time>0 else frame
        if selection is None:
            raise ValueError('--select-time이 영상 범위 안인지 확인하세요.')
        mw,mh=(240,180) if mode=='MCU' else (4096,4096)
        automatic=args.auto or (mode=='MCU' and not (args.roi or args.crop or args.full_frame))
        selected_time=args.select_time
        if args.crop:
            crop=tuple(args.crop)
        elif args.roi:
            crop=choose_roi(selection)
        elif automatic:
            from auto_region import automatic_crop
            try:
                crop,_=automatic_crop(selection,mw,mh,args.target_cells,args.offset_mm/args.pitch_mm)
            except ValueError as exc:
                if args.select_time>0:
                    raise ValueError(str(exc)+' --roi로 점무늬 영역을 직접 선택하세요.') from exc
                found=False
                for sec in (.2,.5,1.,2.):
                    test=at_time(source,sec,(sh,sw))
                    if test is None:
                        continue
                    try:
                        crop,_=automatic_crop(test,mw,mh,args.target_cells,args.offset_mm/args.pitch_mm)
                    except ValueError:
                        continue
                    selection=test;selected_time=sec;found=True;break
                if not found:
                    raise ValueError('자동으로 점무늬 영역을 찾지 못했습니다. --roi --reprepare로 직접 선택하세요.') from exc
        else:
            crop=(0,0,sw,sh)
        crop=validate_crop(crop,sw,sh)
        x,y,cw,ch=crop
        scale=min(1.,mw/cw,mh/ch)
        if args.max_side:
            scale=min(scale,args.max_side/max(cw,ch))
        if mode=='PC' and scale<1 and not args.max_side:
            raise ValueError('PC 입력 한 변은 최대 4096픽셀입니다. --roi로 영역을 줄이거나 --max-side를 명시하세요.')
        w,h=int(math.floor(cw*scale+1e-8)),int(math.floor(ch*scale+1e-8))
        if min(w,h)<32:
            raise ValueError('선택 영역의 종횡비가 너무 큽니다. 가로·세로가 32픽셀 이상이 되게 선택하세요.')
        def transform(im):
            gray=cv2.cvtColor(im[y:y+ch,x:x+cw],cv2.COLOR_BGR2GRAY)
            if gray.shape!=(h,w):
                gray=cv2.resize(gray,(w,h),interpolation=cv2.INTER_AREA)
            return np.ascontiguousarray(gray,dtype=np.uint8)
        quality=None
        if mode=='MCU' or automatic:
            from auto_region import inspect_crop
            quality=inspect_crop(selection,crop,mw,mh,args.max_side,args.offset_mm/args.pitch_mm)
            for message in quality.get('warnings',[]):
                print('입력 확인:',message)
        expected=reported*w*h
        free=shutil.disk_usage(cache).free
        if expected and expected+50*1024*1024>free:
            raise OSError(f'입력 캐시에 약 {expected/1024**2:.0f} MiB가 필요합니다. ROI 또는 --max-side를 사용하세요.')
        temp.mkdir()
        save_png(temp/'input_check.png',make_preview(selection,crop,transform(selection),quality))
        save_png(temp/'frame_000.png',transform(frame))
        print(f'영상 준비 | 전체 길이 | {sw}x{sh} -> ROI {cw}x{ch} -> {w}x{h} 흑백')
        print('준비는 입력 변환만 수행합니다. 좌표 계산은 '+('PC C++' if mode=='PC' else 'MCU')+'에서 수행합니다.')
        frames=[];i=0;previous=-1.;first_pts=None;prev_us=-1;fallbacks=0;written=0;last_print=time.perf_counter()
        with (temp/'frames.gray').open('wb') as f:
            while ok and frame is not None:
                if frame.shape[:2]!=(sh,sw):
                    raise ValueError('영상 중간에 프레임 크기가 바뀝니다.')
                pts=float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000.
                time_method='backend_pts'
                if args.source_fps:
                    t=i/fps;time_method='explicit_fps'
                elif math.isfinite(pts) and pts>=0:
                    if first_pts is None:first_pts=pts
                    t=pts-first_pts
                    if i>0 and t<=previous:
                        t=previous+1/fps;fallbacks+=1;time_method='fps_fallback'
                else:
                    t=0. if not i else previous+1/fps;fallbacks+=1;time_method='fps_fallback'
                us=max(prev_us+1,round(t*1e6));previous=us/1e6;prev_us=us
                data=transform(frame).tobytes();f.write(data);written+=len(data)
                frames.append(dict(index=i,source_index=i,time_us=us,crc32=zlib.crc32(data),time_method=time_method))
                if time.perf_counter()-last_print>2:
                    print(f'  준비 {i+1}장 / {reported or "?"} | {previous:.1f}s',flush=True);last_print=time.perf_counter()
                i+=1;ok,frame=cap.read()
        if not frames:
            raise ValueError('준비된 프레임이 없습니다.')
        dt=np.diff([a['time_us'] for a in frames])/1e6
        manifest=dict(format='ANOTO_GRAY8_V1',source_file=str(source),source_size=[sw,sh],
            source_fps=fps,width=w,height=h,crop_xywh=list(crop),scale_xy=[w/cw,h/ch],
            preparation_version=VERSION,cache_request=request,input_sampling_report=quality,
            native_crop_no_resize=(w==cw and h==ch),selection_time_s=selected_time,
            pixel_operations=['decode_video','integer_fixed_crop','BGR_to_GRAY']+(['INTER_AREA_resize'] if (w,h)!=(cw,ch) else []),
            frame_count=len(frames),bytes_per_frame=w*h,bytes_total=written,frames=frames,
            source_duration_s=frames[-1]['time_us']/1e6+1/fps,source_timestamp_fallbacks=fallbacks,
            variable_intervals_detected=bool(len(dt)>1 and np.max(abs(dt-1/fps))>.2/fps),
            full_video_requested=True,possibly_truncated=bool(reported>len(frames)+1),
            reference='centre of prepared cropped image, not full source image or robot centre')
        write_json(temp/'manifest.json',manifest)
        # Our dedicated cache ONLY. Never remove user-specified video/result paths.
        if target.exists():shutil.rmtree(target)
        temp.rename(target)
        print(f"입력 완료 | {len(frames)}장 | {manifest['source_duration_s']:.3f}s | 확인: {target/'input_check.png'}")
        if fallbacks:print(f'주의: {fallbacks}개 촬영시각에 FPS 기반 추정 사용.')
        if manifest['possibly_truncated']:print('주의: 보고된 프레임 수보다 적게 읽었습니다. 원본 파일 손상 여부를 확인하세요.')
        return target
    except BaseException:
        if temp.exists():shutil.rmtree(temp)
        raise
    finally:
        cap.release()
