# -*- coding: utf-8 -*-

import time
from serial import Serial
from crcmod.predefined import mkCrcFun
from typing import List

"""
DDSM210のテスト用スクリプト

doc: https://www.waveshare.com/wiki/DDSM210
crc8 checktool: https://crccalc.com/

タイヤ直径: d72.6mm

Note:
  10バイトが1回の命令単位。
  基本的なコマンド体系は以下の通り。
  ID, CMD, .., CRC

  Baud: 115200 8N1
  Rate: 500Hz
  OpenLoopMode: 解放状態。breakコマンドでも保持しない。
  VelocityMode: -2100~2100(-210rpm~210rpm) [signed 16bit]
  PositionMode: 0-32767(0~360deg) [unsigned 16bit]

  電源が落ちたときはIDのみ保持していてブレーキ状態になる。

  data[1] 0x64時のモーター制御モード
  data[2]: speed/position high
  data[3]: speed/position low
  data[4]: 0
  data[5]: 0
  data[6]: Acceleration time(加速度) 1 = 0.1ms default=1
  data[7]: Break 0xFFの時はブレーキ動作
  data[8]: 0
  data[9]: CRC
"""


SERIAL_PORT = 'COM3'  # Change as per your COM port
BAUD_RATE = 115200
ID = 0x01  # default

CMD_SETID = [0x55, 0x53]
CMD_DRIVE = [0x64, ]
CMD_FEEDBACK = [0x74, ]
CMD_OBTAIN = [0x75, ]
CMD_MODE_SWITCH = [0xA0,]

ID_SETID = 0xAA  # ID設定時に指定するID

MODE_OPENLOOP = 0x00
MODE_VELOCITY = 0x02
MODE_POSITION = 0x03

crc8 = mkCrcFun('crc-8-maxim')


def to_hex(b):
    """汎用的にhex文字列に変換します。
    """
    if not isinstance(b, list):
        b = [b,]
    return ' '.join(f"{x:02X}" for x in b)


def parse_rotate_motor_result(data):
    if len(data) != 10:
        raise ValueError("Invalid data length. Expected 10 bytes.")

    id = data[0]
    mode = data[1]
    vel_high, vel_low = data[2], data[3]
    cur_high, cur_low = data[4], data[5]
    acc_time = data[6] / 10.0
    temp = data[7]
    err_code = data[8]
    crc_value = data[9]

    velocity = (vel_high << 8) | vel_low
    current = (cur_high << 8) | cur_low

    print(f"ID: {id:02X}")
    print(f"Fixed value (should be 0x64): {mode:02X}")
    print(f"Velocity: {velocity}")
    print(f"Current: {current}")
    print(f"Acceleration time: {acc_time:.1f} ms")
    print(f"Temperature: {temp} ℃")
    print(f"Error code: {err_code:02X}")
    print(f"CRC8: {crc_value:02X} check={crc8(bytearray(data[:9])) == crc_value}")
    if err_code != 0:
        print("Detected errors:", parse_error_code(err_code))


def parse_mileage_motor_result(data):
    if len(data) != 10:
        raise ValueError("Invalid data length. Expected 10 bytes.")

    id = data[0]
    mode = data[1]
    mil_laps_high, mil_laps_sec_high, mil_laps_sec_low, mil_laps_low = data[2], data[3], data[4], data[5]
    pos_high, pos_low = data[6], data[7]
    err_code = data[8]
    crc_value = data[9]

    mileage_laps = (mil_laps_high << 24) | (mil_laps_sec_high << 16) | (mil_laps_sec_low << 8) | mil_laps_low
    position = (pos_high << 8) | pos_low

    print(f"ID: {id:02X}")
    print(f"Fixed value (should be 0x74): {mode:02X}")
    print(f"Mileage laps: {mileage_laps}")
    print(f"Position: {position} (corresponds to {position * 360 / 65535:.1f}°)")
    print(f"Error code: {err_code:02X}")
    print(f"CRC8: {crc_value:02X} check={crc8(bytearray(data[:9])) == crc_value}")
    if err_code != 0:
        print("Detected errors:", parse_error_code(err_code))


def parse_error_code(error_code):
    error_messages = [
        "Save",
        "Overcurrent error",
        "Save",
        "Save",
        "Overtemperature error",
        "Save",
        "Save",
        "Save"
    ]

    detected_errors = []
    for i in range(8):
        if error_code & (1 << i):
            detected_errors.append(error_messages[i])

    return detected_errors


def parse_mode_value(mode_value):
    mode_dict = {
        0x00: "set to open loop",
        0x02: "set to velocity loop",
        0x03: "set to position loop"
    }
    return mode_dict.get(mode_value, "Unknown mode")


def parse_mode_motor_result(data):
    if len(data) != 10:
        raise ValueError("Invalid data length. Expected 10 bytes.")

    id = data[0]
    fixed = data[1]
    mode_value = data[2]
    zeroes = data[3:9]
    crc_value = data[9]

    if any(zero != 0 for zero in zeroes):
        raise ValueError("Invalid data: expected zeros in positions 3 to 8")

    print(f"ID: {id:02X}")
    print(f"Fixed value (should be 0xA0): {fixed:02X}")
    print(f"Mode value: {mode_value:02X} ({parse_mode_value(mode_value)})")
    print(f"CRC8: {crc_value:02X} check={crc8(bytearray(data[:9])) == crc_value}")


def send_command(ser: Serial, id: int, cmd: List[int], data: List[int] = None):
    """シリアル送信

    10バイト送信して戻り値のCRCチェックをしてから返します。

    :param Serial ser: シリアルポート
    :param int id: モーターID (default 0x01)
    :param [int] command: コマンドリスト (8バイト)

    :return: レスポンス (10バイト)
    """

    if data is None:
        data = [0] * (8 - len(cmd))

    # print("Command:", to_hex(cmd))
    # print("Data:", to_hex(data))

    # check
    query = [id,] + cmd + data

    crc = crc8(bytearray(query))
    # print("CRC:", to_hex(crc))

    query.append(crc)

    print("Query   :", to_hex(query))

    if len(query) != 10:
        raise ValueError("Invalid command length")

    ser.write(bytearray(query))
    time.sleep(0.2)

    ret = ser.read(10)
    print("Response:", ' '.join(f"{x:02X}" for x in ret))

    if ret == b'':
        print("No response")
    elif cmd == CMD_DRIVE:
        parse_rotate_motor_result(ret)
    elif cmd == CMD_FEEDBACK:
        parse_mileage_motor_result(ret)
    elif cmd == CMD_OBTAIN:
        parse_mode_motor_result(ret)

    print()

    return ret


def rotate_motor(ser, id, velocity=0, acceleration_ms=0, break_mode=False):
    """モーター制御

    :param Serial ser: シリアルポート
    :param int id: モーターID
    :param int velocity: 速度
    :param int acceleration: 加速度
    :param bool break_mode: ブレーキモード

    :return: レスポンス
    """
    data = [
        (velocity >> 8) & 0xFF,  # data[2]
        velocity & 0xFF,
        0,
        0,
        int(acceleration_ms / 0.1) & 0xFF,
        0 if break_mode is False else 0xFF,
        0,
    ]
    return send_command(ser, id, CMD_DRIVE, data)


def change_id(ser: Serial, new_id: int) -> bool:
    """ID変更

    モーターの電源投入後に1度しか変更を受け付けない。
    2回目以降はレスポンスがNULLとなる。
    5回IDを送信することで変更が適用される。

    :param Serial ser: シリアルポート
    :param int new_id: 新しいID

    :return: 成功したかどうか
    """

    for i in range(5):
        print(f"Set ID to {new_id:02X}, step {i + 1}")
        ret = send_command(ser, ID_SETID, CMD_SETID + [new_id,])
        if ret[9] != crc8(bytearray(ret[:9])):
            print("CRC8 error")
            return False

    print("ID changed to", new_id)
    return True


def test1_velocity_mode(ser):
    # velocity loop mode
    print("# Switching to velocity loop")
    send_command(ser, ID, CMD_MODE_SWITCH + [MODE_VELOCITY,])
    time.sleep(1)

    print("# Querying motor mode")
    send_command(ser, ID, CMD_OBTAIN)
    time.sleep(1)

    print("# rotating motor at 0 rpm")
    rotate_motor(ser, ID, 0)
    time.sleep(3)

    print("# Rotating motor at -100 rpm")
    rotate_motor(ser, ID, -1000)
    time.sleep(3)

    print("# Rotating motor at 100 rpm")
    rotate_motor(ser, ID, 1000)
    time.sleep(3)

    print("# Rotating motor at -2100 rpm")
    rotate_motor(ser, ID, -2100)
    time.sleep(3)

    print("# Rotating motor at 2100 rpm")
    rotate_motor(ser, ID, 2100)
    time.sleep(3)

    print("# Rotating motor acceleration at -2100 rpm, 25.5ms")
    rotate_motor(ser, ID, 2100, 10.0)  # 10ms**10e-3 * 2100rpm = 21s
    time.sleep(25)

    print("# Break command")
    rotate_motor(ser, ID, break_mode=True)


def test2_position_mode(ser):
    # position loop mode
    print("# Switching to position loop")
    send_command(ser, ID, CMD_MODE_SWITCH + [MODE_POSITION,])

    print("# Rotating motor at 0 deg")
    rotate_motor(ser, ID, 0)
    time.sleep(5)

    print("# Rotating motor at 90 deg")
    rotate_motor(ser, ID, int(32767 / 4))
    time.sleep(5)

    print("# Rotating motor at 180 deg")
    rotate_motor(ser, ID, int(32767 / 2))
    time.sleep(5)

    print("# Rotating motor at 360 deg")
    rotate_motor(ser, ID, 32767)
    time.sleep(5)

    print("# Break command")
    rotate_motor(ser, ID, break_mode=True)


def test3_openloop_mode(ser):
    print("# Switching to open loop")
    send_command(ser, ID, CMD_MODE_SWITCH + [MODE_OPENLOOP,])


def test4_change_id(ser):
    # change id
    new_id = 0x02
    change_id(ser, new_id)

    # feedback test
    print("# Feedback")
    send_command(ser, new_id, CMD_FEEDBACK)


def test5_reset_id(ser):
    # reset
    change_id(ser, ID)

    # feedback test
    print("# Feedback")
    send_command(ser, ID, CMD_FEEDBACK)


def test6_feedback(ser):
    # feedback test
    print("# Feedback")
    send_command(ser, ID, CMD_FEEDBACK)


def test7_obtain_mode(ser):
    # obtain mode feedback
    print("# Obtain mode feedback")
    send_command(ser, ID, CMD_OBTAIN)


def test():
    ser = Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    try:
        test1_velocity_mode(ser)
        test2_position_mode(ser)
        test3_openloop_mode(ser)

        # ID変更テストは一度電源を入れ直してから実行してください。
        # test4_change_id(ser)

        # IDを0x01に戻したいときに使います。一度電源を入れ直してから実行してください。
        # test5_reset_id(ser)

        test6_feedback(ser)
        test7_obtain_mode(ser)

        # last stop
        print("# Break command")
        rotate_motor(ser, ID, break_mode=True)

    finally:
        ser.close()


if __name__ == "__main__":
    test()
