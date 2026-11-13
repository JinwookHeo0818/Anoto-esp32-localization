"""Anoto PC/MCU runner. All support code lives in src/; results have two files."""
from __future__ import annotations
import argparse
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src'))
from mode import MODE


def choose_video():
    path=ROOT/'input.mp4'
    if path.is_file():return path
    try:
        import tkinter as tk
        from tkinter.filedialog import askopenfilename
        root=tk.Tk();root.withdraw()
        try:
            name=askopenfilename(title='실험할 동영상 선택',filetypes=[('Video','*.mp4 *.mov *.avi *.mkv *.MP4 *.MOV'),('All','*.*')])
        finally:root.destroy()
    except Exception as exc:
        raise RuntimeError('영상 선택창을 열 수 없습니다. --video "동영상 경로"로 지정하세요.') from exc
    if not name:raise RuntimeError('영상 선택 취소')
    return Path(name)


def choose_port():
    from serial.tools import list_ports
    ports=list(list_ports.comports())
    matches=[p for p in ports if p.vid==0x303A]
    if len(matches)==1:
        print('보드 포트:',matches[0].device);return matches[0].device
    if not ports:raise RuntimeError('COM 포트가 없습니다. 보드 연결과 데이터 USB 케이블을 확인하세요.')
    print('Arduino 도구 > 포트에서 확인한 보드를 선택하세요.')
    for i,p in enumerate(ports,1):print(f'{i}: {p.device} | {p.description}')
    try:
        index=int(input('번호: '))-1
        if not 0<=index<len(ports):raise ValueError
        return ports[index].device
    except (ValueError,EOFError) as exc:
        raise RuntimeError('--port COM5처럼 실제 포트를 지정하세요.') from exc


def main(argv=None):
    parser=argparse.ArgumentParser(description=f'Anoto {MODE}: video -> X/Y/Heading, clean two-file output.')
    g=parser.add_mutually_exclusive_group()
    g.add_argument('--video',type=Path)
    g.add_argument('--prepared',type=Path,help='Advanced: use an existing .cache/input directory without modifying it')
    parser.add_argument('--out',default='result',help='Output directory; never overwrites previous results')
    parser.add_argument('--no-show',action='store_true',help='Do not display the live window')
    parser.add_argument('--no-video',action='store_true',help='Write only results.csv, no annotated video')
    parser.add_argument('--debug',action='store_true',help='Additionally save ONE diagnostics.json')
    parser.add_argument('--reprepare',action='store_true',help='Rebuild the cached input / reselect ROI')
    parser.add_argument('--prepare-only',action='store_true',help='Prepare and inspect input without decoding or opening a COM port')
    g=parser.add_mutually_exclusive_group()
    g.add_argument('--roi',action='store_true',help='Select a fixed ROI; MCU resizes the whole selected ROI to fit')
    g.add_argument('--crop',type=int,nargs=4,metavar=('X','Y','W','H'))
    g.add_argument('--auto',action='store_true',help='Automatically select a dot-pattern region (default on MCU)')
    g.add_argument('--full-frame',action='store_true',help='Use the full frame (default on PC); MCU still resizes to 240x180 max')
    parser.add_argument('--max-side',type=int,default=0,help='Explicit optional downsize; 0 = backend defaults')
    parser.add_argument('--target-cells',type=float,default=12.)
    parser.add_argument('--select-time',type=float,default=0.,help='Selection frame time only; replay still uses the entire video')
    parser.add_argument('--source-fps',type=float,default=0.,help='Only override missing/incorrect capture timestamps, not compute speed')
    parser.add_argument('--pitch-mm',type=float,default=1.5)
    parser.add_argument('--offset-mm',type=float,default=.25)
    parser.add_argument('--section',type=int,nargs=2,default=[10,2])
    parser.add_argument('--blocks',type=int,choices=(1,2,3),default=3)
    parser.add_argument('--forward-image-deg',type=float,default=0.)
    parser.add_argument('--max-age-ms',type=float,default=250.,help='FRESH/STALE label only; valid coordinates are always saved/shown')
    parser.add_argument('--grid-mode',choices=('adaptive','fixed'),default='adaptive',help='Adaptive verified reuse (default), or strict fixed-optics comparison')
    parser.add_argument('--finish-wait-ms',type=float,default=2000.,help='After replay stops, collect at most ONE in-flight result for this long; sends no more images')
    parser.add_argument('--preview-fps',type=float,default=10.,help='Display refresh only, does not limit decoding')
    parser.add_argument('--prediction',choices=('none','image','nonholonomic'),default='image')
    parser.add_argument('--timeout',type=float,default=30.,help='Fault timeout, clipped to replay end')
    parser.add_argument('--floor-width-mm',type=float,default=10000.)
    parser.add_argument('--floor-height-mm',type=float,default=10000.)
    if MODE=='MCU':parser.add_argument('--port',help='Actual board COM port; otherwise auto/list selection')
    parser.set_defaults(mcu_limits=False)
    args=parser.parse_args(argv)
    if not math.isfinite(args.pitch_mm) or not math.isfinite(args.offset_mm) or args.pitch_mm<=0 or not .05<args.offset_mm/args.pitch_mm<.25:
        parser.error('Check pitch-mm and offset-mm; require 0.05 < offset/pitch < 0.25.')
    for name in ('max_age_ms','timeout','floor_width_mm','floor_height_mm'):
        if not math.isfinite(getattr(args,name)) or getattr(args,name)<=0:parser.error(name+' must be positive and finite')
    for name in ('select_time','source_fps','finish_wait_ms'):
        if not math.isfinite(getattr(args,name)) or getattr(args,name)<0:parser.error(name+' must be nonnegative and finite')
    if not math.isfinite(args.preview_fps) or not 1<=args.preview_fps<=60:parser.error('preview-fps must be 1..60')
    if args.source_fps and not .1<=args.source_fps<=1000:parser.error('source-fps must be 0 or 0.1..1000')
    if not math.isfinite(args.forward_image_deg) or abs(args.forward_image_deg)>3600:parser.error('Invalid forward-image-deg')
    if not math.isfinite(args.target_cells) or not 9<=args.target_cells<=18:parser.error('target-cells must be 9..18')
    if args.max_side and args.max_side<32:parser.error('max-side must be 0 or >=32')
    if any(not 0<=x<=62 for x in args.section):parser.error('section values must be 0..62')
    if min(args.floor_width_mm,args.floor_height_mm)<8*args.pitch_mm:parser.error('Floor dimensions must cover an 8x8 block')
    import cv2
    import numpy
    cv2.setNumThreads(2)
    from preparation import prepare_input
    args.mode=MODE;args.show=not args.no_show
    prepared=args.prepared.expanduser().resolve() if args.prepared else prepare_input(args.video or choose_video(),args,ROOT,MODE)
    if args.prepare_only:
        print('입력 준비만 완료:',prepared/'input_check.png');return 0
    args.prepared=str(prepared)
    from playback import run
    if MODE=='PC':
        from build import ensure_worker
        from pc_transport import PCStream
        exe=ensure_worker(ROOT)
        return run(args,stream_factory=lambda:PCStream(exe,ROOT/'.cache'/'worker_error.log'))
    args.port=args.port or choose_port()
    print('Arduino 시리얼 모니터/플로터를 닫으세요. 좌표 계산은 MCU에서 수행합니다.')
    return run(args)


if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:print('취소');sys.exit(130)
    except ModuleNotFoundError as exc:
        print(f'필요한 패키지가 없습니다: {exc}. INSTALL.bat를 한 번 실행하세요.',file=sys.stderr);sys.exit(2)
    except Exception as exc:
        print('ERROR:',exc,file=sys.stderr);sys.exit(2)
