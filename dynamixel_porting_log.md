# Pixhawk 6C ↔ Dynamixel 직접 통신 포팅 개발 로그

> 기존 방식(Pixhawk → 중간 컴퓨터(라떼판다) → 다이나믹셀)을 벗어나,
> **TELEM3 포트로 다이나믹셀을 직접 연결·제어**하는 PX4 드라이버를 만드는 과정 전체 기록.
>
> - PX4 포크(작업 저장소): `HJS1019/Dynamixel_Porting` (브랜치: `master`)
> - 원본 랩실 저장소: `kamgaa/PX4-Modifying_fromSEO`
> - 드라이버 위치: `src/drivers/dynamixel/`

---

## 0. 목표와 최종 도달 지점

### 처음 세운 목표
1. PX4에서 다이나믹셀에 서보 위치를 **명령**한다.
2. 다이나믹셀의 **바뀐 위치를 다시 PX4에서 받아온다.**
3. 다이나믹셀은 반이중(half-duplex)이므로 그 부분도 구현한다. (처음엔 나중으로 미뤘다가 중반에 진행)

### 현재 도달 지점 (요약)
| 항목 | 상태 |
|---|---|
| 프로토콜 2.0 패킷/CRC 정확성 | ✅ 검증 완료 (ROBOTIS 공식 예제와 일치) |
| SITL + 에뮬레이터로 읽기/쓰기 왕복 | ✅ 검증 완료 (write 3000 → read 3000) |
| 실기체(fmu-v6c) 빌드 | ✅ 성공 |
| 실기체에서 드라이버 로드·포트 열기 | ✅ 성공 (`opened /dev/ttyS1`) |
| single-wire 반이중 모드 활성화 | ✅ 성공 (실기체 로그로 확인) |
| 실물 서보와의 실제 통신 | ❌ **미해결 (진행 중)** |

---

## 1. 코드 검증 (첫 단계)

AI로 초안 작성한 `dynamixel.cpp` / `dynamixel.hpp`를 프로토콜 단위로 검증했다.

### 정확했던 부분
- **패킷 조립**: Write는 `param_len = 2(주소) + len(데이터)`, `inst_len = 1(명령) + param_len + 2(CRC)`로 LEN 필드를 정확히 계산. 데이터는 리틀엔디언. Goal Position 4바이트 쓰기 시 전체 16바이트로 `packet[16]` 배열에 정확히 맞음.
- **CRC**: `updateCRC` 테이블과 로직이 ROBOTIS 규격대로. `updateCRC(0, packet, total_len - 2)`로 CRC 제외 전체를 계산하는 것도 정확.
- **컨트롤 테이블 주소**: Torque Enable=64, Goal Position=116, Present Position=132 (X-시리즈 규격 일치).
- **Status 패킷 파싱**: 길이 계산, CRC 재검증, 명령 0x55 확인, 에러 바이트 위치, 파라미터 시작 위치 모두 규격대로.

### 프로토콜 2.0 패킷 구조 (참고)
```
[0]FF [1]FF [2]FD [3]00 [4]ID [5]LEN_L [6]LEN_H [7]INST [8...]PARAM [n-1]CRC_L [n]CRC_H
        헤더 고정              └─ LEN = INST + PARAM + CRC 바이트 수 ─┘
        CRC 계산 범위 = 헤더부터 PARAM까지 전체 (CRC 2바이트 제외)
```
- offset 7(INST) = 코드의 `PKT_INST`

### 보완이 필요했던 부분 (모두 반영함)
1. **반이중 에코 문제** — 데이터선이 한 가닥이라 내가 보낸 바이트가 내 RX로 되돌아옴(에코). 기존 `rxStatusPacket`은 수신 앞 7바이트를 무조건 status로 간주 → 에코를 status로 오해. → **헤더(FF FF FD) 재동기화 + status(0x55)가 아니면 건너뛰기** 로직으로 수정.
2. **`tcdrain` 누락** — `O_NONBLOCK`이라 `write`가 큐잉만 하고 리턴. 송신이 다 나가기 전에 읽기 시작하는 것을 방지하기 위해 `txPacket`에 `tcdrain` 추가.
3. **PING 없음** — 초기화 시 서보 응답 확인이 없어 배선/ID 오류를 조용히 넘어감. → `ping()` 추가하고 `init()`에서 각 ID에 "찾음/무응답" 로그.
4. **바이트 스터핑 미구현** — 파라미터에 `FF FF FD` 패턴이 나오면 `FD`를 끼워야 하는 규칙. 현재 용도(위치값 0~4095)에선 이 패턴이 나올 수 없어 안전. 멀티턴/확장 위치 쓰게 되면 필요. (주석으로 표시)
5. **각도 매핑** — 초기엔 −90°~+90°를 0~4095에 매핑했으나, 서보의 0~4095는 물리적으로 0~360°. → **0~2π(rad) → 0~4095**로 수정 (헤더 상수 2개만 변경, 변환 함수는 그대로).
6. **루프 타이밍** — 한 루프에 쓰기 4 + 읽기 4 = 최대 8회 왕복(각 20ms 타임아웃)인데 `px4_usleep(20000)`으로 50Hz 목표. 실제로는 더 느려질 수 있음.

---

## 2. 코드 전체 구조

이 드라이버는 **PX4 내부 uORB 메시지**와 **UART 위의 다이나믹셀 패킷**을 이어주는 번역기다.

### 데이터 흐름
- **명령 방향 (PX4 → 서보)**: 다른 모듈이 `servo_command` 토픽에 각도(rad) 발행 → `run()` 루프가 받아 `angleToPosition()`으로 0~4095 변환 → `writeRegister(id, 116, position, 4)`로 Goal Position에 기록.
- **피드백 방향 (서보 → PX4)**: 같은 루프에서 `readRegister(id, 132, 4, ...)`로 Present Position 읽기 → `positionToAngle()`로 각도 변환 → `servo_angle` 토픽으로 발행.

### 계층 구조 (5단)
```
writeRegister / readRegister / ping   (무엇을 보낼지 결정)
        ↓
txPacket                              (헤더 + CRC 붙여 전송, tcdrain)
        ↓
serialWrite                           (UART 바이트 출력)
        ↓ ... 서보 응답 ...
serialRead                            (바이트 수신)
        ↓
rxStatusPacket                        (헤더 동기화, CRC 검증, 에러 확인)
```

### `_uart_fd` 변수 추적 (시리얼 포트 파일 디스크립터)
규칙: **-1이면 "안 열림/무효", 0 이상이면 "정상 열림"**

| 위치 | 역할 |
|---|---|
| hpp 선언 `int _uart_fd {-1}` | -1로 시작 |
| `openSerial` 76행 `::open(...)` | 실제 핸들 획득 |
| 78행 `if (_uart_fd < 0)` | open 실패 감지 |
| 85행 `tcgetattr` / 120행 `tcsetattr` / 126행 `tcflush` | 포트 설정 |
| `closeSerial` 134~136행 | 닫고 다시 -1로 |
| `serialWrite` 142행 검사 / 146행 `::write` | 바이트 전송 |
| `serialRead` 151행 검사 / 159행 `::read` | 바이트 수신 |
| `txPacket` 257~258행 `tcdrain` | 송신 완료 대기 |
| `writeRegister` 357 / `readRegister` 388 / `ping` 415행 | 사전 유효성 검사 |

**"안전하게 실패한다"의 의미**: 포트가 안 열렸는데 `write(-1, ...)` 같은 위험한 동작을 하면 프로그램이 죽거나 쓰레기 값을 읽을 수 있다. `if (_uart_fd < 0) return -1;`은 위험한 동작을 아예 안 하고 "못 했다"고 정직하게 알려주는 것.

**성능 우려에 대한 답**: 이 검사는 정수 비교 1회(~2나노초)로, 바이트 1개 전송(57600 기준 ~174마이크로초, 8만 배)이나 레지스터 왕복(수 밀리초, 100만 배)에 비해 무시할 수준. 진짜 병목은 ① 보드레이트 ② 왕복 횟수(Sync Write/Bulk Read로 8회→2회 축소 가능) ③ 폴링 간격.

---

## 3. 반이중(Half-duplex) 구현

### 핵심 개념
데이터 선이 한 가닥이라 **한 번에 한 명만 말할 수 있다**(무전기와 동일). 풀듀플렉스 UART(TX/RX 별도)를 이 한 가닥에 붙이려면 "지금은 송신 / 지금은 수신" 전환이 필요.

### 검토한 방법들

**하드웨어 방식**
1. RS-485 트랜시버 (MAX3485 등) + DE/RE를 DIR로 제어
2. TTL 3-state 버퍼 (74HC126 + 74HC04) — ROBOTIS 공식 회로
3. 완성형 어댑터 보드

**소프트웨어 방식**
1. GPIO + `tcdrain` 수동 전환
2. RTS 자동 방향 제어 — **Pixhawk 6C TELEM3는 RTS 핀이 커넥터에 안 나와서 불가**
3. **single-wire (STM32 HDSEL) — 최종 채택**

### TELEM3 핀아웃 (확인 결과)
| 핀 | 신호 |
|---|---|
| 1 | VCC +5V |
| 2 | **USART2_TX (3.3V)** ← 데이터선 연결 |
| 3 | USART2_RX (3.3V) — single-wire에선 미사용 |
| 4, 5 | NOT CONNECTED (일부 SN은 I2C4) |
| 6 | GND |

→ RTS/CTS 핀 없음. TELEM1(UART7)/TELEM2(UART5)에는 있음.

### 채택한 방법: single-wire (STM32 HDSEL)
STM32 USART의 `HDSEL` 비트로 켜는 단선 반이중 모드. TX와 RX가 내부 연결되고, RX 핀은 미사용, **TX 핀은 데이터를 보내지 않을 때 자동으로 해제(release)** 되어 수신 입력으로 동작.

"마지막 비트가 나가면 선을 놓는" 동작을 **하드웨어가 비트 단위로 정확히** 수행 → 소프트웨어 타이밍 계산 불필요.

**코드 (openSerial 내부)**
```cpp
#include <sys/ioctl.h>
#ifdef __PX4_NUTTX
# include <nuttx/serial/tioctl.h>
#endif

// tcsetattr, tcflush 이후
#ifdef TIOCSSINGLEWIRE
  #ifdef SER_SINGLEWIRE_PULLUP
    const unsigned long sw_arg = SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PULLUP;
  #else
    const unsigned long sw_arg = SER_SINGLEWIRE_ENABLED;
  #endif
  if (ioctl(_uart_fd, TIOCSSINGLEWIRE, sw_arg) < 0) {
      PX4_WARN("dynamixel: single-wire 설정 실패 - NuttX defconfig 확인");
  } else {
      PX4_INFO("dynamixel: single-wire half-duplex 모드 활성화됨");
  }
#else
  PX4_INFO("dynamixel: single-wire 미지원 플랫폼 - full-duplex");
#endif
```

**용어 설명**
- **ioctl**: 장치에 read/write로 표현 안 되는 "특별한 부탁"을 하는 통로. `ioctl(포트, 무슨_부탁, 값)` 형태.
- **NuttX**: Pixhawk 안에서 돌아가는 작은 운영체제. PX4는 그 위에서 앱처럼 동작. `open/read/write/ioctl`은 모두 NuttX 시리얼 드라이버로 내려감.
- **#ifdef / #else**: 해당 기능이 있는 환경이면 이 코드를, 없으면 저 코드를 빌드하라는 조건부 스위치. SITL(리눅스)에는 `TIOCSSINGLEWIRE`가 없어 빌드가 깨지지 않도록 분기.

**NuttX single-wire 옵션 켜는 법**
```bash
cd ~/Desktop/Dynamixel_Porting
make px4_fmu-v6c_default nuttxmenuconfig
# 메뉴에서 '/' 키 → SINGLEWIRE 검색 → 스페이스로 켜기 → Esc로 저장
```
설정 파일 위치: `boards/px4/fmu-v6c/nuttx-config/nsh/defconfig`
(실기체 없이도 설정·빌드 가능. 업로드·테스트만 보드 필요.)

### 반이중 3요소가 맞물리는 방식
1. **송신**: `writeRegister` → `txPacket` 전송
2. **송신 완료 대기**: `tcdrain`이 마지막 비트까지 나갔음을 보장
3. **자동 전환**: single-wire가 선을 놓아 수신 상태로 (하드웨어가 자동)
4. **에코 건너뛰기**: `rxStatusPacket`이 자기 에코(0x55 아님)를 건너뛰고 진짜 응답 탐색
5. **수신 완료**: status 패킷 CRC 검증 후 결과 반환

---

## 4. 하드웨어 (배선)

### GPIO 개념
General-Purpose Input/Output. 소프트웨어로 HIGH(3.3V)/LOW(0V)를 내보내거나 읽는 범용 핀. 방향 제어(DIR) 신호에 쓰려 했으나, single-wire 채택으로 불필요해짐.

- **DIR**: Direction(방향)의 약자. 반이중 변환기에 "송신/수신 전환"을 알리는 HIGH/LOW 신호 한 가닥. (외부 버퍼 회로를 쓸 때만 필요)
- Pixhawk 6C에서 GPIO로 쓸 수 있는 핀: **FMU PWM OUT (AUX)** — 메인 프로세서 직결이라 GPIO 재설정 가능. I/O PWM OUT(MAIN)은 별도 I/O 칩 경유라 부적합.

### 서보 모델별 차이

| 항목 | XL430-W250-T | XL330-M288-T |
|---|---|---|
| 데이터선 로직 | **5V** | **3.3V** (X-시리즈 중 유일한 예외) |
| 레벨 시프터 | 필요 (BSS138 등) | **불필요** (Pixhawk 3.3V와 일치) |
| 구동 전원 | 11.1V (3S) | **5V** (3.3~6V, 11.1V 넣으면 파손) |
| 분해능 | 4096 / 360° | 4096 / 360° |
| 컨트롤 테이블 주소 | 64/116/132 | 동일 |
| 최대 보드레이트 | 4.5 Mbps | 4 Mbps급 |

> ROBOTIS 매뉴얼: "XL330의 통신 버스 전압은 다른 다이나믹셀과 달리 3.3V이지만, 5V 통신 버스에 호환됩니다."
> → **레벨(전압) 호환** 얘기이며, **풀업 필요 여부와는 별개**. 5V로 풀업해도 XL330이 견딘다는 근거로 활용 가능.

### 보드레이트 표 (레지스터 주소 8)
| 값 | 보드레이트 |
|---|---|
| 0 | 9,600 |
| 1 | 57,600 (공장 기본) |
| 2 | 115,200 |
| 3 | 1,000,000 |
| 4 | 2,000,000 |
| 5 | 3,000,000 |
| 6 | 4,000,000 |
| 7 | 4,500,000 |

→ 초기 코드의 19200, 230400은 이 표에 없어 서보와 통신 불가. switch 문을 표에 맞게 수정함.
(4.5M은 표준 termios 매크로 `B4500000`이 없어 커스텀 설정 필요.)

### 최종 배선 (XL330 단독, 현재 구성)
```
Pixhawk TELEM3 핀2 (TX, 3.3V) ──┬── XL330 DATA
                                │
                          풀업(내부 or 외부 저항 1kΩ)
                                │
                              3.3V / 5V
TELEM3 핀1 (+5V) ── XL330 VDD
TELEM3 핀6 (GND) ── XL330 GND  (공통 GND 필수)
```
- **RX(핀3)는 사용하지 않음** — single-wire는 TX 한 가닥으로 송수신
- 전원 경로: 배터리 → PM06 → Pixhawk → TELEM3 핀1(5V) → 서보
  (TELEM3 5V는 전류 용량이 작아, 실제 구동 시엔 별도 5V 전원 권장)

### 풀업이 필요한 이유
single-wire는 **오픈드레인** 방식 — 장치는 선을 **GND로 끌어내려 0을 만들 뿐**, 1은 "손을 놓는(release)" 것으로 표현. 아무도 안 잡은 선은 **붕 뜬 상태(floating)** 가 되어 0인지 1인지 판단 불가.

풀업 저항이 선을 3.3V로 약하게 당겨줘서 **"놓으면 1, 끌어내리면 0"** 규칙을 성립시킨다. (스프링으로 위로 당겨진 스위치와 동일한 원리)

---

## 5. 개발 환경 / 빌드

### 빌드 명령
```bash
# 실기체(Pixhawk 6C)용
make px4_fmu-v6c_default
# → 결과물: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4

# SITL(시뮬레이션)용
make px4_sitl_default
```

### 시리얼 포트 매핑 (fmu-v6c 빌드 로그에서 확인)
```
SERIAL_GPS1  /dev/ttyS0
SERIAL_GPS2  /dev/ttyS6
SERIAL_TEL1  /dev/ttyS5
SERIAL_TEL2  /dev/ttyS3
SERIAL_TEL3  /dev/ttyS1   ← 우리가 쓰는 TELEM3
```

### 드라이버를 빌드에 포함시키기
```bash
grep -i dynamixel boards/px4/sitl/default.px4board      # SITL
grep -i dynamixel boards/px4/fmu-v6c/default.px4board   # 실기체
# 없으면: echo "CONFIG_DRIVERS_DYNAMIXEL=y" >> <해당 파일>
```

### 겪은 빌드 문제와 해결

**① microcdr `/usr/local` 권한 에러**
```
file cannot create directory: /usr/local/microcdr-2.0.1/lib. Maybe need administrative privileges.
```
- 원인: `uxrce_dds_client`(ROS2 브리지) 하위 빌드가 설치 경로를 시스템 경로로 잡음. 예전 빌드 캐시가 뒤엉킨 것이 유발.
- 해결: `make distclean` → `git submodule update --init --recursive` → 재빌드. **절대 sudo로 빌드하지 말 것.**

**② ARM 크로스 컴파일러 없음**
```
arm-none-eabi-g++ is not a full path and was not found in the PATH.
```
- 원인: SITL은 PC용(x86) 컴파일러로 되지만, 실기체는 ARM Cortex-M7용 크로스 컴파일러 필요.
- 해결: `bash ./Tools/setup/ubuntu.sh --no-sim-tools` 후 터미널 재시작.

**③ 시계 꼬임으로 CMake 무한 반복**
```
[0/1] Re-running CMake...   (계속 반복)
```
- 원인: 시각이 틀어진 상태에서 파일을 수정해 **미래 시각으로 찍힌 파일**이 생김 → CMake가 매번 "설정이 방금 바뀌었다"고 오해.
- 진단: `find src boards Tools -newermt "now"` → 미래 시각 파일 목록 (여기선 `boards/px4/fmu-v6c/default.px4board` 등)
- 해결:
  ```bash
  find src boards Tools -newermt "now" -exec touch {} +   # 시각을 현재로
  rm -rf build
  make px4_fmu-v6c_default
  ```
- 참고: `timedatectl set-ntp true`로 자동 동기화. `synchronized: no`여도 시각만 맞으면 빌드는 정상.

**④ `%u` 포맷 타입 에러 (실기체 빌드에서만 발생)**
```
error: format '%u' expects argument of type 'unsigned int',
       but argument 6 has type 'uint32_t' {aka 'long unsigned int'}
```
- 원인: ARM에선 `uint32_t`가 `long unsigned int`. SITL(x86)에선 `unsigned int`라 통과했음. PX4는 `-Werror`라 경고도 에러.
- 해결: 로그 인자를 `(unsigned)`로 캐스팅.
  ```cpp
  PX4_INFO("write id=%u addr=%u value=%u len=%u -> %s",
           (unsigned)id, (unsigned)addr, (unsigned)value, (unsigned)len, ok ? "OK" : "FAIL");
  ```

**⑤ `'ok' was not declared in this scope`**
- 원인: 위 수정 중 중괄호 위치가 어긋나 `return ok ? 0 : 1;`이 `if("read")` 블록 **밖**으로 밀려남.
- 해결: `else`의 닫는 `}` → `return ok ? 0 : 1;` → `}`(if 블록 닫기) 순서로 정정.

### Git 충돌 (윈도우 ↔ 우분투)
```
error: Your local changes to the following files would be overwritten by merge
```
- 우분투에서 작업·push, 윈도우에서 pull하려는데 윈도우 로컬에 예전 수정이 남은 경우:
  ```bash
  git checkout -- src/drivers/dynamixel/dynamixel.cpp   # 로컬 변경 버리기
  git pull
  # 여러 파일이면: git reset --hard && git pull
  ```
- 습관: **떠나기 전 push, 시작 전 pull**

---

## 6. 검증 (하드웨어 없이)

### 6-1. 프로토콜/CRC 검증 (성공)
독립적인 방식으로 CRC 테이블을 교차 검증하고, ROBOTIS 공식 예제와 대조:

```
=== (A) 테이블 CRC vs 독립 비트연산 CRC 교차검증 ===
  OK  table=0x0066 bitwise=0x0066
  OK  table=0x4E19 bitwise=0x4E19
  OK  table=0xA413 bitwise=0xA413
  OK  table=0x8007 bitwise=0x8007
  => 테이블 CRC 구현이 정확함

=== (B) ROBOTIS 공식 PING 예제와 대조 ===
  우리 CRC(PING) = 0x4E19  /  ROBOTIS 문서값 = 0x4E19  -> 일치
  완성 PING 패킷 = FF FF FD 00 01 03 00 01 19 4E

=== (C) Write Goal Position (id=1, addr=116, value=1000, len=4) ===
  전체 패킷 = FF FF FD 00 01 09 00 03 74 00 E8 03 00 00 F0 A9
  총 길이   = 16 bytes
  LEN 필드  = 9  (inst1 + addr2 + data4 + crc2)
  데이터부  = E8 03 00 00  (1000=0x03E8 리틀엔디언)
```

### 6-2. SITL + 가짜 서보 에뮬레이터 (성공)
`socat`으로 가상 시리얼 포트 2개를 연결하고, 파이썬 에뮬레이터가 진짜 서보처럼 응답하게 함.

**실행 (터미널 3개)**
```bash
# 터미널 A — 가상 시리얼 포트
socat -d -d PTY,link=/tmp/ttyDXL_a,raw,echo=0 PTY,link=/tmp/ttyDXL_b,raw,echo=0

# 터미널 B — 가짜 서보
python3 dxl_emulator.py /tmp/ttyDXL_b

# 터미널 C — SITL (pxh> 프롬프트까지)
../bin/px4 -s /tmp/empty_rcS      # (rcS가 SIH를 찾아 실패할 때 우회법)
```

**pxh> 에서 실행한 명령과 결과**
```
dynamixel start -d /tmp/ttyDXL_a -b 57600   → single-wire 미지원 플랫폼(정상), ping OK
dynamixel read 1 132 4                       → value=2048  (에뮬레이터 초기값)
dynamixel write 1 116 3000 4                 → OK
dynamixel read 1 132 4                       → value=3000  ✅ 왕복 검증 성공
```

**검증된 것**: 패킷 바이트 정확성, CRC(양방향), Write→status 처리, Read→위치 파싱, 응답 CRC 검증, 드라이버 내 왕복 완결.
**검증 못 한 것**: socat은 TX/RX 분리라 에코가 없어 **에코 스킵 로직과 반이중 전기 타이밍은 미검증**.

### 6-3. 실기체 확인 (부분 성공)
QGC → Analyze Tools → MAVLink Console 에서:

```
nsh> dynamixel start -d /dev/ttyS1 -b 57600
INFO  [dynamixel] dynamixel: single-wire half-duplex 모드 활성화됨   ← 실기체에서만 확인 가능
INFO  [dynamixel] opened /dev/ttyS1 @ 57600 baud
```

**확인된 것**: 드라이버 로드·실행, TELEM3 포트 열림, **single-wire 모드가 실기체에서 실제로 활성화됨**, ping 로직이 "서보 없음"을 정확히 감지.

---

## 7. 현재 미해결 문제 (진행 중)

### 증상
배선 완료 후에도 `ping FAIL`. 아래는 모두 확인된 정상 항목:

| 확인 항목 | 상태 |
|---|---|
| 소프트웨어 (single-wire 활성화, 포트 열림) | ✅ |
| 포트 = TELEM3 = `/dev/ttyS1` | ✅ |
| 서보 정상 (Dynamixel Wizard로 ID=1, 57600 확인) | ✅ |
| 서보 LED 깜빡임 (전원 정상) | ✅ |
| 전원 5V (TELEM3 핀1), GND 공통 | ✅ |
| 데이터선 = TX(핀2) 납땜 | ✅ |
| 보드레이트 여러 값 시도 (57600, 115200, 1M) | ✅ 모두 FAIL |

### 진단 과정

**1) DATA선 idle 전압 측정 → 1.4V**
붕 뜬(floating) 값. 풀업 부족으로 판정 → `SER_SINGLEWIRE_PULLUP` 플래그 추가.

**2) 플래그 추가 후 재측정 → 3.3V**
내부 풀업이 실제로 활성화됨. 신호선은 정상적으로 잡힘. 그러나 여전히 ping FAIL.

**3) `ping()`에 수신 바이트 덤프 디버그 삽입**
```cpp
txPacket(packet);
uint8_t dbg[64] {};
int n = serialRead(dbg, sizeof(dbg), 50);
PX4_INFO("PING id=%u: rx %d bytes", (unsigned)id, n);
for (int i = 0; i < n; i++) { PX4_INFO("  [%d] 0x%02X", i, (unsigned)dbg[i]); }
return (n > 0);
```

**결과 (지연 없이)**
```
PING id=1: rx 1 bytes   [0] 0x4E
PING id=2: rx 1 bytes   [0] 0x01
PING id=3: rx 0 bytes
PING id=4: rx 1 bytes   [0] 0x0A
```
→ 우리가 보낸 ping 패킷(`FF FF FD 00 01 03 00 01 19 4E`, 10바이트) 중 **꼬리 1바이트만** 에코로 돌아옴. (`0x4E`는 id=1 패킷의 마지막 바이트) 나머지 9바이트와 서보 응답은 소실.
※ 이때 뜬 `ping OK`는 디버그 코드의 `return (n > 0)` 때문인 **가짜 성공**.

**결과 (`txPacket` 후 `px4_usleep(2000)` 추가)**
```
PING id=1~4: rx 0 bytes  (전부)
```
→ 지연을 넣으니 그 1바이트 조각마저 사라짐. 서보의 진짜 응답은 애초에 없었고, 그 1바이트는 버퍼 잔재였음.

### 현재 판단
**에코조차 온전히 돌아오지 않음** — 정상 single-wire라면 보낸 10바이트가 통째로 에코로 돌아와야 함. 0~1바이트만 온다는 것은:
- 송신 패킷이 회선으로 제대로 안 나가고 있거나
- single-wire 수신 경로가 거의 열리지 않고 있음

서보/배선 문제라기보다 **single-wire 모드가 실기체에서 기대대로 동작하지 않는 쪽**으로 무게가 기울어 있음.

### 다음 실험 (예정)
`openSerial`의 single-wire ioctl 블록을 **주석 처리**하여 일반 full-duplex로 열고, `px4_usleep(2000)`은 제거한 뒤 `ping 1` 실행:

- **rx가 10바이트 근처로 잡히고 값이 우리 ping 패킷이면** → single-wire 모드가 범인. 설정 방식(플래그 조합, NuttX 쪽 처리) 재검토 필요.
- **여전히 0바이트면** → single-wire와 무관한 송신/배선 방향 문제. TX 핀 납땜, 포트 방향 재의심.

---

## 8. 코드 리뷰 — 실기체 관점 남은 개선 항목

빌드는 되지만 실제 운용 시 문제가 될 수 있는 항목들 (우선순위 순).

### ① UART 동시 접근 (미해결, 실기체 1순위 위험)
`run()` 스레드가 50Hz로 `read/writeRegister`를 호출하는 중에, 콘솔에서 `ping`/`read`를 치면 **다른 스레드(nsh)** 가 같은 `_uart_fd`에 동시 접근 → 바이트 뒤섞임 → CRC 실패, 값 튐.

**해결책: 뮤텍스(mutex)**
"한 번에 한 명만 쓸 수 있게 하는 자물쇠"(Mutual Exclusion). 화장실 한 칸에 자물쇠를 다는 것과 같음. `writeRegister`/`readRegister`/`ping` 각각의 본문(송신+수신 한 세트)을 lock/unlock으로 감싸면 됨. PX4/NuttX에선 `pthread_mutex_t` 사용.

**검토했으나 부적합한 대안들**
- *단순 `bool` 플래그*: `if(busy)` 확인과 `busy=true` 표시 사이에 다른 스레드가 끼어들 수 있음(race condition). 막은 것처럼 보이지만 안 막힘.
- *`std::atomic` + compare_exchange*: 틈은 막지만, 겹치면 요청을 **버림**(`return false`) → run()이 읽는 중에 친 ping이 그냥 씹혀서 `ping FAIL`로 보임. 우리 용도엔 부적합.
- *`_uart_fd`를 변수 2개로 분리*: 핸들을 나눠도 **가리키는 실제 포트·데이터선은 하나** — 문 두 개 단 화장실 한 칸. 여전히 섞임.
- → **뮤텍스만이 "겹치면 기다렸다 순서대로 다 처리"** 라는 원하는 동작을 제공.

### ② 없는 서보 타임아웃 (미해결)
`_servo_ids = {1,2,3,4}` 고정인데 실제 서보는 1개. `run()`이 매 사이클 없는 3개마다 20ms씩 대기 → 루프가 50Hz가 아닌 ~15Hz로 저하. init의 ping도 없는 서보마다 50ms.
→ `_servo_ids`를 실제 개수(`{1}`)로 축소, 루프 카운트도 조정.

### ③ 태스크 스택 (미해결)
`px4_task_spawn_cmd(..., 1536, ...)` → 실기체에선 스택 오버플로우가 **하드폴트(즉시 리부팅)** 로 나타남. **2048 권장.**

### ④ 문서 문자열 갱신 (미해결)
- `print_usage`에 아직 "half-duplex ... 아직 구현되지 않았고" (이제 구현됨)
- 예제가 `-d /dev/ttyS3` (실제 TELEM3는 `/dev/ttyS1`)
- `PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, ...)` 상한이 3M (switch엔 4M까지 있음)

### ⑤ 기타 주의점
- **`servo_command` 단위**: `angleToPosition`은 0~2π 벗어난 값을 **조용히 clamp**. 상위 모듈 붙일 때 rad(0~2π) 규약 확인 필수.
- **Operating Mode**: Goal Position이 동작하려면 서보가 Position 모드(3)여야 함.
- **읽기 실패 시 NAN**: `servo_angle[i] = NAN`으로 발행되므로 구독자가 걸러야 함.
- **`openSerial`의 `default:` 블록 들여쓰기** 가 어긋나 있음(동작엔 무해, 편집 시 혼동 주의).

---

## 9. 부수적으로 정리한 내용

### servo_allocation 모듈 (참고용, 저장소에는 미포함)
라떼판다의 `listen_and_speak_ros.cpp`에 있는 서보 틸트 얼로케이션을 PX4로 옮기는 방안을 검토하고 뼈대 코드를 작성했으나, **현재 `Dynamixel_Porting` 저장소에는 커밋하지 않음.**

- 원본 로직: 4×1 렌치 벡터(fx, fy, tz_trim, 0)를 만들고, 모터 추력 f1~f4와 CoM에 따라 매 주기 바뀌는 4×4 행렬 `SA`를 구성 → `SA·sinθ = wrench` 풀이 → asin → ±0.7rad 포화 → 2차 버터워스 LPF → 서보 각도.
- PX4 이식 시 필요한 변경: Eigen `colPivHouseholderQr().solve()` → `matrix::SquareMatrix<float,4>::I()`.
- 배치 위치 제안: `src/modules/servo_allocation/` 별도 모듈. (PX4 기본 `control_allocator`는 고정 선형 행렬 전제라 부적합 — 이 얼로케이션은 행렬이 매 주기 변하고 asin 비선형)
- 확인된 메시지 정의 (저장소에 이미 존재):
  - `ThrustCommand.msg` → `float32[4] thrust_command`
  - `CenterOfMass.msg` → `float32[3] com_update`
  - `VehicleThrustSetpoint.msg` → `float32[3] xyz`
  - `VehicleTorqueSetpoint.msg` → `float32 yaw_trim`
  - `ServoCommand.msg` → `float32[4] servo_command` (슬롯 4개 — payload용 5번째 서보는 별도 처리 필요)

### 다른 다이나믹셀 모델 호환성
이 드라이버가 의존하는 것은 두 층:
- **프로토콜 2.0 계층** (패킷, CRC, Read/Write/Ping) → 모델 무관 공통
- **모델별 값** (컨트롤 테이블 주소, 분해능) → 모델마다 다를 수 있음

| 모델군 | 호환성 |
|---|---|
| X-시리즈 TTL (XL330, XC330, XL430, XC430, XM430, XM540) | 거의 무수정 (주소 64/116/132 공통) |
| 프로토콜 2.0이지만 테이블 다름 (MX 2.0, Pro) | `ADDR_*` 상수와 분해능만 수정 |
| 프로토콜 1.0 (AX, RX, 구형 MX-TTL) | **불가** — 패킷 구조/CRC가 완전히 다름 |

→ 진짜 기준은 "TTL이냐"가 아니라 **"프로토콜 2.0이냐"**. `dynamixel ping <id>`가 OK면 프로토콜·주소·보드레이트가 다 맞다는 뜻.

### 성능 개선 여지 (추후)
1. **보드레이트 상향** — 57600 → 1M/2M (바이트 전송 시간 약 17배 단축)
2. **Sync Write(0x83) / Bulk Read(0x82)** — 서보 4개를 패킷 한 번에 처리 → 왕복 8회를 2회로 축소 (가장 큰 구조적 개선)
3. **폴링 간격** — `serialRead`의 `px4_usleep(500)`, `run()`의 20ms 주기 조정

---

## 10. 자주 쓴 명령 모음

```bash
# 빌드
make px4_fmu-v6c_default          # 실기체
make px4_sitl_default             # SITL
make distclean                    # 빌드 캐시 정리
git submodule update --init --recursive

# NuttX OS 설정 메뉴
make px4_fmu-v6c_default nuttxmenuconfig

# 드라이버 포함 여부 확인
grep -i dynamixel boards/px4/fmu-v6c/default.px4board

# QGC MAVLink Console (실기체)
dynamixel start -d /dev/ttyS1 -b 57600
dynamixel ping 1
dynamixel read 1 132 4            # Present Position
dynamixel write 1 116 2048 4      # Goal Position → 중앙
dynamixel status
dynamixel stop
listener servo_command
listener servo_angle
```

---

## 부록: 가짜 서보 에뮬레이터 (`dxl_emulator.py`)

SITL 검증에 사용. `socat` 가상 포트에 붙어 프로토콜 2.0 status 패킷으로 응답하며, Goal Position(116)에 쓰면 Present Position(132)에 즉시 반영한다.

```python
#!/usr/bin/env python3
import serial, sys

def crc16(data):  # Protocol 2.0 CRC-16 (poly 0x8005, init 0)
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

port = sys.argv[1] if len(sys.argv) > 1 else '/tmp/ttyDXL_b'
ser = serial.Serial(port, 57600, timeout=0.02)
regs = {132: [0x00, 0x08, 0x00, 0x00]}   # Present Position 초기값 2048

def send_status(id_, err, params):
    length = 1 + 1 + len(params) + 2
    p = [0xFF,0xFF,0xFD,0x00, id_, length & 0xFF, (length>>8)&0xFF, 0x55, err] + list(params)
    crc = crc16(bytes(p))
    ser.write(bytes(p + [crc & 0xFF, (crc>>8)&0xFF]))

# (전체 코드는 별도 파일 참조 — READ/WRITE/PING 처리 및 헤더 재동기화 포함)
```
