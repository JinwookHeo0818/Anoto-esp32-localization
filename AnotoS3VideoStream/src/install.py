"""Install missing Python modules only, using exactly this interpreter."""
from importlib.util import find_spec
from pathlib import Path
import subprocess
import sys
from mode import MODE


def main():
    if sys.version_info<(3,10):raise RuntimeError('Python 3.10 이상이 필요합니다.')
    packages={'numpy':'numpy>=1.24,<3','cv2':'opencv-python>=4.8,<5'}
    if MODE=='MCU':packages['serial']='pyserial>=3.5,<4'
    missing=[p for m,p in packages.items() if find_spec(m) is None]
    if missing:subprocess.run([sys.executable,'-m','pip','install',*missing],check=True)
    import cv2,numpy
    if MODE=='MCU':
        import serial
        if not hasattr(serial,'Serial'):raise RuntimeError('pyserial이 아닌 serial 패키지가 설치되어 있습니다. 충돌을 해결해야 합니다.')
    print('Python:',sys.executable)
    print('패키지 확인 완료 | OpenCV',cv2.__version__,'| NumPy',numpy.__version__)
    if 'GUI:                           NONE' in cv2.getBuildInformation():
        print('주의: 현재 OpenCV는 headless입니다. 화면/ROI 선택이 필요하면 같은 환경에서 GUI 빌드 하나만 사용하세요.')
    if MODE=='PC':
        from build import ensure_worker
        ensure_worker(Path(__file__).resolve().parents[1])
    print('설치/준비 완료. 다음부터 START.bat만 실행하세요.')
    return 0

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:print('ERROR:',exc,file=sys.stderr);sys.exit(2)
