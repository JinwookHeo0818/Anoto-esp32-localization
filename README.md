# Anoto-Based Localization on ESP32-S3

Anoto 패턴을 이용하여 위치 $(x,y)$와 방향 $\theta$를 추정하는 C++ 프로그램입니다.

ESP32-C6 및 ESP32-S3에서 실행을 확인했으며, 현재 버전은 **PC에서 녹화 영상의 프레임을 USB로 전송하고 MCU에서 위치를 계산하는 방식**입니다.

**카메라 직접 연결은 아직 구현하지 않았습니다.**

## 프로젝트 구성

- `firmware/`: Arduino IDE에서 ESP32-S3에 업로드할 C++ 코드
- `src/`: PC에서 영상 준비, USB 통신, 결과 표시를 수행하는 코드
- `START.bat`: PC 실험 실행
- `INSTALL.bat`: 필요한 Python 패키지 설치
- `input.mp4`, `input_noise.mp4`: 테스트 영상
- `README_Anoto_S3_KO`: 자세한 설치 및 실행 설명

## 실행 방법

**1. Arduino IDE 설정**

보드: ESP32-S3-DevKitC-1-N8R8

- Board: ESP32S3 Dev Module
- CPU Frequency: 240MHz
- Flash Size: 8MB
- Flash Mode: QIO 80MHz
- PSRAM: OPI PSRAM
- USB Mode: Hardware CDC and JTAG
- USB CDC On Boot: Enabled

<img width="500" alt="Arduino_setting" src="https://github.com/user-attachments/assets/eb0f96a8-7459-4caa-a8f1-5fbdbb434f9f" />

`firmware/`의 S3용 `.ino` 파일을 열고 ESP32-S3에 업로드합니다.

**2. PC 환경 준비**

Python 3.10 이상을 설치한 뒤 PowerShell에서 다음을 실행합니다.

`.\INSTALL.bat`

**3. 영상 실행**

S3를 USB로 연결하고 Arduino 시리얼 모니터를 닫은 뒤 실행합니다.

`.\START.bat --video "input.mp4" --reprepare --debug --out result_s3`

**4. 실행 결과**

실행이 완료되면 결과 폴더에 다음 파일이 저장됩니다.

- `annotated.mp4`: 위치 추정 결과 동영상
- `results.csv`: 위치, 방향 및 계산시간
- `diagnostics.json`: 상세 진단 기록

## 참고

현재 추정 좌표는 입력 영상의 중앙 위치를 기준으로 하며, 로봇 중심 위치로 보정한 값은 아닙니다. 실제 카메라 연결 및 이동 로봇 실험은 후속 구현이 필요합니다.
