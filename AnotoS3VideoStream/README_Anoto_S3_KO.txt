Anoto v7.2 | ESP32-S3 처음 실행하기 (Windows)
대상 보드: ESP32-S3-DevKitC-1-N8R8

무엇을 하는 실험인가요?
PC에서 바닥 점무늬(Anoto 패턴)를 촬영한 영상을 엽니다. PC가 영상에서 흑백 이미지를 보내면,
ESP32-S3가 위치(X, Y)와 방향(H)을 계산합니다. 결과는 PC 화면과 결과 동영상에서 확인합니다.
이번 실험에는 카메라를 보드에 연결할 필요가 없습니다. 실행하는 동안 USB는 연결해 둡니다.

1. 시작하기 전에: 코드 폴더 두 개와 원본 영상 한 개를 준비하세요
--------------------------------------------------------------------
① 보드용 폴더: AnotoS3VideoStream.ino, build_opt.h, src 폴더가 함께 들어 있습니다.
   Arduino IDE로 열어서 S3에 올릴 코드입니다. .ino 파일만 따로 꺼내지 마세요.
② PC용 폴더: START.bat, INSTALL.bat, run.py, src 폴더가 함께 들어 있습니다.
   C6 실험에서 영상 전송에 사용했던 Anoto_MCU 폴더를 그대로 사용합니다.
   처음 받는 사람에게는 이 폴더 전체도 전달해야 합니다. S3 펌웨어 ZIP에는 START.bat가 없습니다.
③ 원본 영상: 처음에는 Anoto 패턴이 잘 보이는 실험용 영상을 input.mp4라는 이름으로
   ②의 START.bat 바로 옆에 놓으세요. 확장자가 이미 .mp4이면 두 번 붙이지 마세요.

준비물: Windows PC, ESP32-S3 보드, 데이터 전송이 가능한 USB 케이블.
ZIP 파일은 먼저 압축을 풀고, 두 폴더 안의 파일은 서로 섞지 않습니다.

2. Arduino IDE와 ESP32 보드 패키지를 설치하세요 (처음 한 번)
--------------------------------------------------------------------
① https://www.arduino.cc/en/software/ 에서 Windows용 Arduino IDE 2를 설치하고 엽니다.
② 파일(File) → 환경설정(Preferences) → 추가 보드 매니저 URL에 아래 주소를 추가합니다.
   기존에 다른 주소가 있다면 지우지 말고 새 항목으로 추가하세요.

   https://espressif.github.io/arduino-esp32/package_esp32_index.json

③ 왼쪽의 보드 매니저(Boards Manager)에서 esp32를 검색합니다.
   esp32 by Espressif Systems를 설치합니다. 이미 설치되어 ESP32S3 Dev Module이 보이면 넘어갑니다.

3. 보드를 연결하고, 보드용 코드를 올리세요 (같은 코드라면 한 번만)
--------------------------------------------------------------------
① S3 보드의 USB라고 적힌 단자에 케이블을 꽂고 PC와 연결합니다.
   단자가 두 개입니다. 이 실험은 UART라고 적힌 단자 대신 USB 단자를 사용합니다.
② Arduino IDE → 파일(File) → 열기(Open)에서 보드용 폴더의 AnotoS3VideoStream.ino를 엽니다.
   PC용 폴더에 예전 C6 스케치가 들어 있어도 이번에 여는 파일은 S3 스케치입니다.
③ 도구(Tools) → 보드(Board)에서 ESP32S3 Dev Module을 먼저 선택합니다.
   그다음 도구(Tools) 메뉴를 다시 열어 아래와 같이 맞춥니다.

   CPU Frequency       : 240MHz
   Flash Size          : 8MB (64Mb)
   Flash Mode          : QIO 80MHz
   PSRAM               : OPI PSRAM
   USB Mode            : Hardware CDC and JTAG
   USB CDC On Boot     : Enabled
   Upload Mode         : UART0 / Hardware CDC (항목이 보일 때)

④ 도구 → 포트(Port)에서 S3의 COM 번호를 선택합니다. COM은 Windows가 붙인 연결 번호입니다.
   어떤 포트인지 모르면 케이블을 뺐을 때 사라지고 다시 꽂았을 때 나타나는 번호를 확인하세요.
⑤ 왼쪽 위의 → 업로드(Upload)를 누릅니다. ✓ 검증(Verify)만 누르면 보드에 올라가지 않습니다.
   아래에 업로드 완료(Done uploading)가 뜰 때까지 기다립니다.
⑥ 첫 업로드가 끝나면 보드의 RESET(또는 EN)을 한 번 눌렀다 뗍니다.
   도구 → 포트에서 현재 번호를 다시 확인하고, 시리얼 모니터와 시리얼 플로터는 닫습니다.
   이제 PC 프로그램이 같은 USB 연결을 사용합니다.

   처음부터 포트가 안 보이거나, Connecting... 뒤에 No serial data received가 뜨면:
   BOOT를 누른 채 RESET(EN)을 짧게 눌렀다 떼고 → 마지막으로 BOOT를 뗍니다.
   도구 → 포트를 다시 열어 새로 나타난 번호를 선택한 뒤 → 업로드를 다시 누릅니다.
   여전히 포트가 없으면 데이터 케이블인지 확인하고 PC의 USB 단자에 직접 연결해 보세요.

4. PC 프로그램을 준비하세요 (새 PC에서 처음 한 번)
--------------------------------------------------------------------
① PC에는 Python 3.10 이상이 필요합니다. 없으면 Windows용 Python을 설치합니다.
   다운로드: https://www.python.org/downloads/windows/
   사용할 수 있는 64비트 버전 예: Python 3.12. 설치 창에 Add python.exe to PATH가 보이면 체크합니다.
② 파일 탐색기에서 START.bat와 INSTALL.bat가 보이는 PC용 폴더를 엽니다.
   위쪽 주소 표시줄을 클릭해 powershell 이라고 입력하고 Enter를 누릅니다.
   이 폴더에서 명령을 입력할 수 있는 창이 열립니다. VS Code는 따로 설치하지 않아도 됩니다.
③ 아래 한 줄을 붙여 넣고 Enter를 누릅니다. 첫 설치에는 인터넷 연결이 필요합니다.

   .\INSTALL.bat

   ‘설치/준비 완료’가 뜨면 다음 단계로 갑니다. 필요한 영상 처리·USB 통신 패키지는
   이 명령이 준비하므로 하나씩 설치할 필요가 없습니다. 같은 환경이면 다음번에는 생략합니다.

5. 원본 영상으로 실험하세요 (영상을 실행할 때마다)
--------------------------------------------------------------------
① S3가 USB에 연결되어 있고, Arduino 시리얼 모니터·플로터가 닫혀 있는지 확인합니다.
② input.mp4가 START.bat와 같은 폴더에 있는지 확인합니다.
③ 위에서 연 PowerShell 창에 아래 한 줄을 붙여 넣고 Enter를 누릅니다.

   .\START.bat --video "input.mp4" --reprepare --debug --out result_s3

   실제 파일 이름이 다르면 "input.mp4"만 바꿉니다. 이 명령은 원본 영상에서 입력을
   새로 준비하고 결과 동영상과 진단 기록을 저장합니다. 캐시 폴더를 찾을 필요가 없습니다.
④ 보드를 자동으로 찾으면 바로 진행합니다. 포트 목록과 ‘번호:’가 나오면 Arduino에서
   확인한 COM 포트가 적힌 줄의 목록 번호를 입력합니다. 예: ‘2: COM10’이면 2를 입력합니다.
⑤ ‘MCU 실험 시작’ 뒤에 위치·방향 결과가 나오면 실행 중입니다. 영상 재생이 끝난 후에도
   결과 동영상을 저장하므로, 마지막 ‘결과 폴더:’가 나올 때까지 창을 닫지 마세요.

6. 결과를 확인하세요
--------------------------------------------------------------------
PowerShell 마지막 줄의 ‘결과 폴더:’ 경로를 파일 탐색기로 엽니다.

   annotated.mp4      영상 위에 위치·방향 결과를 표시한 동영상 (이 파일부터 열어 보세요)
   results.csv        시간별 X, Y, H와 계산시간. Excel에서 열 수 있습니다.
   diagnostics.json   --debug로 저장한 상세 실행 기록. 첫 실행에서는 열지 않아도 됩니다.

같은 이름의 결과 폴더가 이미 있으면 result_s3_2처럼 새 폴더가 생깁니다.
다른 영상으로 다시 실행할 때는 보드 코드를 다시 올리지 말고 5단계만 반복하세요.

막혔을 때 먼저 확인할 것
--------------------------------------------------------------------
• ‘START.bat를 찾을 수 없음’: 보드용 폴더에서 실행했을 수 있습니다. 탐색기에서
  START.bat가 실제로 보이는 PC용 폴더를 열고 거기서 PowerShell을 다시 여세요.
• ‘동영상이 없습니다’: --video 뒤의 파일명(확장자 포함)과 실제 파일명이 같은지 확인하세요.
• ‘포트를 열 수 없음’ 또는 ‘Access is denied’: Arduino 시리얼 모니터·플로터와
  다른 실행 중인 영상 전송 프로그램을 닫으세요. S3의 현재 COM 번호를 다시 확인하세요.
  포트를 직접 지정하려면 5단계 명령 끝에 --port COM10처럼 실제 번호를 붙이세요.
• 영상이 안 저장됨: 명령에 --no-video가 없어야 합니다. 동영상 저장이 끝난 뒤에
  표시된 결과 폴더를 확인하세요. 코덱에 따라 MP4 대신 AVI로 저장될 수 있습니다.

공식 참고: ESP32 보드 패키지 설치 / S3 보드 단자와 버튼 / 첫 USB 업로드
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html
https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html
