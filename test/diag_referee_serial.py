#!/usr/bin/env python3
"""
雷达 0x0305 诊断脚本
- 发送 0x0305 包并同时解析所有 RX 帧
- 重点监控: 0x0001(比赛状态), 0x0003(HP), 0x020C(雷达标记进度), 0x020E(雷达信息)
- 打印发送包的完整 hex dump 便于人工核对
"""
import serial, struct, time, sys

PORT = '/dev/ttyUSB0'
BAUD = 115200

CRC8_TAB = [
    0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
    0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
    0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
    0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
    0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
    0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
    0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
    0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
    0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
    0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
    0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
    0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
    0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
    0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
    0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
    0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
]

def crc8(d):
    c = 0xFF
    for b in d:
        c = CRC8_TAB[c ^ b]
    return c

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c & 0xFFFF

def build_0305(seq):
    """构建 0x0305 包。己方=蓝方, 对方=红方"""
    payload = bytearray(48)
    # 对方(红方)哨兵: (7.0m, 7.5m) = (700cm, 750cm) -- slot 5 (offset 20)
    struct.pack_into('<HH', payload, 20, 700, 750)
    # 己方(蓝方)步兵3: (21.5m, 7.5m) = (2150cm, 750cm) -- slot 2 (offset 32)
    struct.pack_into('<HH', payload, 32, 2150, 750)

    hdr = bytearray(5)
    hdr[0] = 0xA5
    struct.pack_into('<H', hdr, 1, 48)
    hdr[3] = seq & 0xFF
    hdr[4] = crc8(bytes(hdr[:4]))

    pkt = bytes(hdr) + struct.pack('<H', 0x0305) + bytes(payload)
    pkt += struct.pack('<H', crc16(pkt))
    return pkt

GP_NAMES = {0: '未开始', 1: '准备', 2: '自检', 3: '倒计时', 4: '比赛中', 5: '结算'}

def parse_0001(data):
    """解析比赛状态"""
    if len(data) < 4:
        return f"  0x0001 数据太短({len(data)}B)"
    gp = (data[0] >> 4) & 0xF
    remain = struct.unpack_from('<H', data, 1)[0]
    return f"  比赛: {GP_NAMES.get(gp, f'?({gp})')} 剩余{remain}s"

def parse_0003(data):
    """解析HP数据"""
    if len(data) < 4:
        return f"  0x0003 数据太短({len(data)}B)"
    s = "  HP:"
    names = ['红英雄','红工程','红步3','红步4','红步5','红哨兵','红前哨','红基地',
             '蓝英雄','蓝工程','蓝步3','蓝步4','蓝步5','蓝哨兵','蓝前哨','蓝基地']
    n = min(len(data)//2, len(names))
    for i in range(n):
        hp = struct.unpack_from('<H', data, i*2)[0]
        if hp > 0:
            s += f" {names[i]}={hp}"
    return s

def parse_020c(data):
    """解析雷达标记进度"""
    return f"  0x020C 雷达标记: {data.hex()}"

def parse_020e(data):
    """解析雷达信息"""
    if len(data) >= 1:
        b = data[0]
        chance = b & 0x03
        active = (b >> 2) & 1
        return f"  0x020E 雷达信息: 双倍易伤机会={chance} 正在触发={active} raw=0x{b:02x}"
    return f"  0x020E 数据太短"

def parse_0201(data):
    """解析机器人状态"""
    if len(data) < 2:
        return None
    rid = data[0]
    return f"  robot_id={rid}"

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()

    # 先打印一个发送包的完整 hex
    test_pkt = build_0305(0)
    print(f"=== 发送包 hex ({len(test_pkt)}B) ===")
    print(' '.join(f'{b:02x}' for b in test_pkt))
    print(f"  SOF={test_pkt[0]:02x} len={struct.unpack_from('<H',test_pkt,1)[0]}")
    print(f"  seq={test_pkt[3]} crc8={test_pkt[4]:02x}")
    print(f"  cmd_id=0x{struct.unpack_from('<H',test_pkt,5)[0]:04x}")
    # 解析 payload
    pl = test_pkt[7:55]
    slot_names = ['对方英雄','对方工程','对方步3','对方步4','对方空中/步5','对方哨兵',
                  '己方英雄','己方工程','己方步3','己方步4','己方空中/步5','己方哨兵']
    for i in range(12):
        x = struct.unpack_from('<H', pl, i*4)[0]
        y = struct.unpack_from('<H', pl, i*4+2)[0]
        if x > 0 or y > 0:
            print(f"  {slot_names[i]}: ({x}cm, {y}cm) = ({x/100:.1f}m, {y/100:.1f}m)")
    print(f"  crc16=0x{struct.unpack_from('<H',test_pkt,55)[0]:04x}")
    print()

    buf = bytearray()
    seq = 0
    tx_count = 0
    t0 = time.time()
    last_tx = 0
    last_020c = None
    seen_cmds = set()

    duration = 45
    print(f"开始发送+监听 {duration}秒 (5Hz)...")
    print("请确保比赛正在进行中 (game_progress=4)")
    print("=" * 60)

    try:
        while time.time() - t0 < duration:
            now = time.time()
            # 5Hz 发送
            if now - last_tx >= 0.2:
                pkt = build_0305(seq)
                ser.write(pkt)
                ser.flush()
                seq = (seq + 1) & 0xFF
                tx_count += 1
                last_tx = now

            # 读取 RX
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            # 解析帧
            while len(buf) >= 7:
                idx = buf.find(0xA5)
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    buf[:idx] = b''
                if len(buf) < 5:
                    break
                if crc8(bytes(buf[:4])) != buf[4]:
                    buf.pop(0)
                    continue
                dl = struct.unpack_from('<H', buf, 1)[0]
                total = 5 + 2 + dl + 2
                if len(buf) < total:
                    break
                if crc16(bytes(buf[:total-2])) != struct.unpack_from('<H', buf, total-2)[0]:
                    buf.pop(0)
                    continue

                cmd = struct.unpack_from('<H', buf, 5)[0]
                data = bytes(buf[7:7+dl])
                elapsed = time.time() - t0

                if cmd not in seen_cmds:
                    seen_cmds.add(cmd)
                    print(f"[{elapsed:5.1f}s] 首次收到 cmd=0x{cmd:04X} len={dl}", flush=True)

                if cmd == 0x0001:
                    print(f"[{elapsed:5.1f}s] {parse_0001(data)} TX={tx_count}包", flush=True)
                elif cmd == 0x0003:
                    print(f"[{elapsed:5.1f}s] {parse_0003(data)}", flush=True)
                elif cmd == 0x020C:
                    hex_data = data.hex()
                    if hex_data != last_020c:
                        print(f"[{elapsed:5.1f}s] {parse_020c(data)} *** 变化了! ***", flush=True)
                        last_020c = hex_data
                    else:
                        # 每5秒打印一次即使没变
                        pass
                elif cmd == 0x020E:
                    print(f"[{elapsed:5.1f}s] {parse_020e(data)}", flush=True)
                elif cmd == 0x0201:
                    info = parse_0201(data)
                    if info:
                        print(f"[{elapsed:5.1f}s] {info}", flush=True)
                elif cmd == 0x0305:
                    # 如果收到0x0305回显，说明有问题
                    print(f"[{elapsed:5.1f}s] ⚠️  收到0x0305回显! 数据可能被回环了!", flush=True)

                buf[:total] = b''

    except KeyboardInterrupt:
        pass

    ser.close()
    elapsed = time.time() - t0
    print()
    print(f"=== 完成 ===")
    print(f"运行 {elapsed:.1f}s, 共发送 {tx_count} 个 0x0305 包")
    print(f"收到的 cmd 类型: {[hex(c) for c in sorted(seen_cmds)]}")
    if last_020c is not None:
        print(f"最后一次 0x020C 值: {last_020c}")
    else:
        print("未收到 0x020C 帧")

if __name__ == '__main__':
    main()
