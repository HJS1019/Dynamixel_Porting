#!/usr/bin/env python3
# 가짜 다이나믹셀 서보 에뮬레이터 (Protocol 2.0)
# 사용법:
#   1) pip install pyserial
#   2) 터미널1:  socat -d -d PTY,link=/tmp/ttyDXL_a,raw,echo=0 PTY,link=/tmp/ttyDXL_b,raw,echo=0
#   3) 터미널2:  python3 dxl_emulator.py /tmp/ttyDXL_b
#   4) 터미널3(SITL pxh>):  dynamixel start -d /tmp/ttyDXL_a -b 57600
import serial, sys

def crc16(data):  # Dynamixel Protocol 2.0 CRC-16 (poly 0x8005, init 0)
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

port = sys.argv[1] if len(sys.argv) > 1 else '/tmp/ttyDXL_b'
ser = serial.Serial(port, 57600, timeout=0.02)
print(f"[emulator] listening on {port}")

# 아주 단순한 레지스터 맵: 주소 -> 바이트 리스트
regs = {132: [0x00, 0x08, 0x00, 0x00]}  # Present Position(132) 초기값 = 2048

def send_status(id_, err, params):
    length = 1 + 1 + len(params) + 2  # inst(0x55) + err + params + crc
    p = [0xFF, 0xFF, 0xFD, 0x00, id_, length & 0xFF, (length >> 8) & 0xFF, 0x55, err] + list(params)
    crc = crc16(bytes(p))
    p += [crc & 0xFF, (crc >> 8) & 0xFF]
    ser.write(bytes(p))

def handle(pkt):
    if crc16(pkt[:-2]) != (pkt[-2] | (pkt[-1] << 8)):
        print("  ! CRC mismatch -> 무시")
        return
    id_, inst, params = pkt[4], pkt[7], list(pkt[8:-2])

    if inst == 0x01:  # PING
        send_status(id_, 0, [0x24, 0x04, 0x2E])  # model 1060(0x0424) + fw 0x2E
        print(f"  <- PING id={id_}  => status(model=1060, fw=46)")

    elif inst == 0x02:  # READ
        addr = params[0] | (params[1] << 8)
        rlen = params[2] | (params[3] << 8)
        val = (regs.get(addr, [0] * rlen) + [0] * rlen)[:rlen]
        send_status(id_, 0, val)
        print(f"  <- READ id={id_} addr={addr} len={rlen}  => {val}")

    elif inst == 0x03:  # WRITE
        addr = params[0] | (params[1] << 8)
        data = params[2:]
        regs[addr] = data
        if addr == 116:            # Goal Position 쓰면 Present Position에 즉시 반영
            regs[132] = data
        send_status(id_, 0, [])
        print(f"  -> WRITE id={id_} addr={addr} data={data}")

buf = bytearray()
while True:
    data = ser.read(64)
    if data:
        buf += data
    while True:
        idx = buf.find(b'\xFF\xFF\xFD')
        if idx < 0:
            if len(buf) > 2:
                del buf[:-2]       # 헤더가 걸칠 수 있으니 마지막 2바이트만 남김
            break
        del buf[:idx]              # 헤더 앞 잡음 제거
        if len(buf) < 7:
            break                  # 길이 필드까지 아직 안 옴
        length = buf[5] | (buf[6] << 8)
        total = 7 + length
        if len(buf) < total:
            break                  # 패킷 나머지 아직 안 옴
        handle(bytes(buf[:total]))
        del buf[:total]
