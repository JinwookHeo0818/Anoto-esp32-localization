"""Latest-frame replay. Default output: results.csv and annotated.mp4 only.

PC: local C++ process. MCU: native USB serial -> same core on ESP32.
No pose interpolation. Capture/source times and reply times remain distinct.
"""
from __future__ import annotations
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time
import zlib
from common import (Client, CONFIG, FRAME, HELLO, HELLO_OK, START, START_OK,
                    RESULT_OK, ProtocolError, load_manifest, new_output,
                    parse_result, write_json)


def distribution(values):
    v=sorted(float(x) for x in values)
    return None if not v else dict(count=len(v),mean_ms=statistics.fmean(v),
        p95_ms=v[max(0,math.ceil(.95*len(v))-1)],max_ms=v[-1])


def latest_due(times_us,elapsed_s,last_sent):
    index=bisect_right(times_us,math.floor(max(0.,elapsed_s)*1e6))-1
    return index if index>last_sent else None


def open_serial(port,baud):
    import serial
    s=serial.Serial(port=None,baudrate=baud,timeout=.01,write_timeout=.1,
                    xonxoff=False,rtscts=False,dsrdtr=False)
    s.rts=False;s.dtr=True;s.port=port
    try:
        s.open();time.sleep(1.5);s.reset_input_buffer()
    except BaseException:
        s.close();raise
    return s


def config(args,m):
    return CONFIG.pack(m['width'],m['height'],args.pitch_mm,args.offset_mm,*args.section,
        args.blocks,{'none':0,'image':1,'nonholonomic':2}[args.prediction],int(args.grid_mode=='fixed'),int(args.mcu_limits),
        args.floor_width_mm,args.floor_height_mm,args.forward_image_deg)


def save_results(out,m,records):
    """Six columns, one row per original frame; never invent a pose for a skipped one."""
    by_frame={r['frame']:r for r in records}
    with (out/'results.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['관측시각(s)','X(mm)','Y(mm)','Heading(deg)','계산(ms)','도착지연(ms)'])
        for entry in m['frames']:
            r=by_frame.get(entry['index'])
            pose=[f"{r[k]:.3f}" for k in ('x_mm','y_mm','heading_deg')] if r and r['valid'] else ['','','']
            times=[f"{r['compute_ms']:.3f}",f"{r['sample_age_ms']:.3f}"] if r else ['','']
            writer.writerow([f"{entry['time_us']/1e6:.6f}",*pose,*times])


def run(args,stream_factory=None):
    prepared=Path(args.prepared).resolve();m=load_manifest(prepared)
    duration=float(m['source_duration_s']);times=[int(f['time_us']) for f in m['frames']]
    mode=args.mode
    records=[];attempted={};hello={};stream=None;preview=None;out=None
    reason=None;completed=False;run_start=None;active_wall=0.;pending=None;partial=False
    command_start=time.perf_counter();post_wait_s=0.
    setup=dict(mode='realtime_latest',backend=mode,max_age_ms=args.max_age_ms,
        width=m['width'],height=m['height'],source_file=m.get('source_file'),
        coordinate_policy='Keep every code-verified pose; STALE is a label, not a pose filter.',
        settings=dict(pitch_mm=args.pitch_mm,offset_mm=args.offset_mm,section=args.section,
            forward_image_deg=args.forward_image_deg,blocks=args.blocks,prediction=args.prediction,grid_mode=args.grid_mode,mcu_limits=args.mcu_limits),
        reference='Prepared image centre. Not a calibrated robot-centre pose.',
        timing='Source time -> reply arrival age. Compute time is decoder call only, not camera latency.')
    try:
        stream=stream_factory() if stream_factory else open_serial(args.port,115200)
        client=Client(stream,args.timeout)
        body,_=client.transact(HELLO,0,b'',HELLO_OK);hello=json.loads(body)
        if not hello.get('supports_skipped_frames') or int(hello.get('protocol',0))!=2:
            raise ProtocolError('이 묶음의 Arduino 펌웨어를 업로드하세요. 프로토콜 2와 프레임 건너뛰기 지원이 필요합니다.')
        if m['width']>hello['max_width'] or m['height']>hello['max_height']:
            raise ValueError('입력 크기가 수신 프로그램의 한도를 넘습니다. 입력 준비를 다시 실행하세요.')
        if not hello.get('supports_adaptive_grid'):
            raise ProtocolError('새 v7 코드를 사용하려면 firmware/AnotoC6VideoStream/AnotoC6VideoStream.ino를 다시 업로드하세요.')
        print('수신 프로그램:',hello.get('firmware','UNKNOWN'))
        client.transact(START,1,config(args,m),START_OK)
        out=new_output(Path(args.out))
        setup['device']=hello
        simulation='HOST' in hello.get('hardware','')
        if simulation:print('주의: PC 모의 수신기입니다. MCU 실측값이 아닙니다.')
        print(f"{mode} 실험 시작 | {duration:.3f}s | {m['width']}x{m['height']} | 최신 프레임 처리")
        if args.blocks==1:print('주의: 1블록 설정은 서로 다른 8x8 사이의 교차 검증을 생략합니다.')
        with (prepared/'frames.gray').open('rb') as raw:
            if args.show:
                from display import LivePreview
                preview=LivePreview(prepared,m,setup,hello)
            run_start=time.perf_counter();end=run_start+duration
            last_sent=-1;last_print=run_start;last_tick=0.
            def tick():
                nonlocal last_tick
                now=time.perf_counter()
                if preview is not None and now-last_tick>=1/args.preview_fps and now<end:
                    last_tick=now
                    if not preview.show(now-run_start,records):raise KeyboardInterrupt
            while True:
                now=time.perf_counter()
                if now>=end:completed=True;break
                tick()
                io_margin=max(.015,client.last_write_ms*.0015)
                if end-time.perf_counter()<io_margin:
                    time.sleep(max(0.,min(.005,end-time.perf_counter())));continue
                idx=latest_due(times,time.perf_counter()-run_start,last_sent)
                if idx is None:
                    future=times[last_sent+1]/1e6 if last_sent+1<len(times) else duration
                    time.sleep(max(0.,min(.005,run_start+future-time.perf_counter(),end-time.perf_counter())))
                    continue
                raw.seek(idx*m['bytes_per_frame']);data=raw.read(m['bytes_per_frame']);entry=m['frames'][idx]
                if len(data)!=m['bytes_per_frame'] or zlib.crc32(data)!=int(entry['crc32']):
                    raise ValueError(f'프레임 {idx} 캐시가 손상되었습니다. --reprepare로 다시 준비하세요.')
                now=time.perf_counter()
                if now>=end:completed=True;break
                if latest_due(times,now-run_start,last_sent)!=idx:continue
                selected=time.perf_counter()-run_start
                if selected>=duration:completed=True;break
                attempted[idx]=selected;last_sent=idx
                payload=struct.pack('<IQ',idx,entry['time_us'])+data
                transaction_start=time.perf_counter()
                after_playback=False
                try:
                    body,rtt=client.transact(FRAME,idx+2,payload,RESULT_OK,deadline=end,tick=tick)
                except TimeoutError:
                    if time.perf_counter()<end:raise
                    pending=idx;partial=not client.last_write_complete;completed=True
                    active_wall=time.perf_counter()-run_start
                    if partial or args.finish_wait_ms<=0:break
                    # Real-time replay is over: no backlog processing. Collect only
                    # this already-issued result and keep it with its original time.
                    print('재생 종료 | 이미 전송한 마지막 1장 응답만 수신 중 (추가 프레임 전송 없음)')
                    tw=time.perf_counter()
                    try:body=client.finish_pending(idx+2,RESULT_OK,tw+args.finish_wait_ms/1000.)
                    except TimeoutError:
                        post_wait_s=time.perf_counter()-tw;break
                    post_wait_s=time.perf_counter()-tw
                    rtt=(time.perf_counter()-transaction_start)*1000.
                    after_playback=True
                available=time.perf_counter()-run_start
                after_playback=after_playback or available>=duration
                r=parse_result(body)
                if r['frame']!=idx or r['source_us']!=entry['time_us']:
                    raise ProtocolError('결과 프레임 번호 또는 관측시각이 요청과 다릅니다.')
                r.update(source_time_s=r['source_us']/1e6,selected_wall_s=selected,
                    available_wall_s=available,roundtrip_ms=rtt,source_index=entry['source_index'])
                r['sample_age_ms']=(available-r['source_time_s'])*1000
                r['after_playback']=after_playback
                r['fresh']=0<=r['sample_age_ms']<=args.max_age_ms
                r['accepted_live']=bool(r['valid'] and r['fresh'] and not after_playback)
                records.append(r)
                if after_playback:pending=None
                if after_playback or len(records)==1 or time.perf_counter()-last_print>=1:
                    state=('LATE_END' if after_playback else 'FRESH' if r['fresh'] else 'STALE') if r['valid'] else 'LOST'
                    pose=f"X={r['x_mm']:.2f} Y={r['y_mm']:.2f} H={r['heading_deg']:.2f}" if r['valid'] else 'X=-- Y=-- H=--'
                    print(f"t={r['source_time_s']:.3f}s | {pose} | {mode}={r['compute_ms']:.2f}ms age={r['sample_age_ms']:.1f}ms [{state}]",flush=True)
                    if args.debug:print(f"  {r['status']} dots={r['dots']} blocks={r['blocks']} reason={r['reason'] or '-'}",flush=True)
                    last_print=time.perf_counter()
                if after_playback:
                    if not active_wall:active_wall=duration
                    completed=True;break
            if not active_wall:active_wall=time.perf_counter()-run_start
    except KeyboardInterrupt:
        reason='USER_STOP';print('중단: 지금까지 받은 결과만 저장합니다.')
    except Exception as exc:
        reason=str(exc);print('ERROR:',reason,file=sys.stderr)
    finally:
        if run_start is not None and not active_wall:active_wall=time.perf_counter()-run_start
        if stream is not None:
            try:stream.close()
            except Exception as exc:print('연결 종료 경고:',exc,file=sys.stderr)
        if preview is not None:preview.close()
    if out is None:return 2
    save_results(out,m,records)
    good=[r for r in records if r['accepted_live']]
    decoded=[r for r in records if r['valid']]
    summary=dict(backend=mode,completed=completed,error=reason,device=hello,
        source_duration_s=duration,source_frames=m['frame_count'],frames_sent=len(attempted),
        frames_returned=len(records),skipped_frames=m['frame_count']-len(attempted),
        unanswered_requests=len(attempted)-len(records),inflight_at_end=pending,
        decoded_valid=len(decoded),displayed_valid=len(decoded),accepted_live=len(good),
        received_after_playback=sum(r.get('after_playback',False) for r in records),
        post_replay_wait_s=post_wait_s,
        effective_decoded_hz=sum(r['valid'] and not r.get('after_playback',False) for r in records)/active_wall if active_wall else 0.,
        effective_accepted_hz=len(good)/active_wall if active_wall else 0.,
        playback_wall_s=active_wall,playback_overrun_s=max(0.,active_wall-duration),
        partial_write_at_stop=partial,max_age_ms=args.max_age_ms,
        compute=distribution([r['compute_ms'] for r in records]),
        age_at_arrival=distribution([r['sample_age_ms'] for r in records]),
        roundtrip=distribution([r['roundtrip_ms'] for r in records]))
    print(f"완료 | 재생 {active_wall:.3f}/{duration:.3f}s | 좌표 성공 {len(decoded)}/{len(records)}회 | 지연기준 내 {len(good)}회 | 건너뜀 {summary['skipped_frames']}")
    if summary['compute']:
        d=summary['compute'];print(f"{mode} 계산 평균 {d['mean_ms']:.2f}ms | P95 {d['p95_ms']:.2f}ms | 최대 {d['max_ms']:.2f}ms")
    if not decoded:print('코드 검증 성공 좌표 없음. --debug 또는 .cache/input/input_check.png를 확인하세요.')
    elif not good:print('성공 좌표는 모두 저장했습니다. 지연이 커서 FRESH가 없는 것이며, STALE 좌표를 현재 위치로 사용하지 마세요.')
    if partial:print('마지막 패킷 전송이 중단되었습니다. 다음 실험 전에 보드 USB를 다시 연결하세요.')
    export_failed=False
    if not args.no_video:
        print('재생 종료. 결과 영상 저장 중... (재생시간과 별도)')
        started=time.perf_counter()
        try:
            from display import render_video
            dest=render_video(prepared,out,m,setup,hello,records)
            print('영상:',dest.name)
        except Exception as exc:
            export_failed=True;summary['export_error']=str(exc)
            print('결과 영상 저장 실패; results.csv는 보존됨:',exc,file=sys.stderr)
        summary['video_export_wall_s']=time.perf_counter()-started
    summary['command_total_wall_s']=time.perf_counter()-command_start
    if args.debug:
        # A single optional diagnostics file, not a directory full of separate logs.
        states=[];by_frame={r['frame']:r for r in records}
        for entry in m['frames']:
            idx=entry['index'];r=by_frame.get(idx)
            state=('FRESH' if r['accepted_live'] else ('LOST' if not r['valid'] else 'LATE_END' if r.get('after_playback') else 'STALE')) if r else ('NO_RESULT_BY_STOP' if idx in attempted else 'SKIPPED')
            states.append(dict(frame=idx,state=state))
        write_json(out/'diagnostics.json',dict(setup=setup,input=m,summary=summary,frames=records,frame_states=states))
    print('결과 폴더:',out)
    return 0 if completed and not reason and not export_failed else 2
