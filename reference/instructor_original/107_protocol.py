# --- Python Code: Raspberry Pi Simulator ---
import serial
import time
import struct
import random

# --- 설정 (환경에 맞게 변경하세요) ---
# Arduino가 Serial.begin(115200)을 사용하므로, 이 포트와 통신합니다.
PORT = 'COM7'   # Windows 사용자: COMx, Linux/Mac 사용자: /dev/ttyUSBx 또는 /dev/ttyACMx
BAUDRATE = 115200
SENSOR_PACKET_LENGTH = 29
COMMAND_PACKET_LENGTH = 5

# 명령 패킷 정의 (Run Mode: 0xF5, 0xF5, 0x51, Direction, Speed)
CMD_FORWARD = [0xF5, 0xF5, 0x51, 0x01, 0x32] # Forward (0x01), Speed 50 (0x32)
CMD_STOP = [0xF5, 0xF5, 0x51, 0x05, 0x00]    # Stop (0x05), Speed 0 (0x00)
CMD_RESET = [0xF5, 0xF5, 0x51, 0x06, 0x00]    # reset esp32 (0x06)



def init_serial():
    """시리얼 포트를 초기화하고 연결합니다."""
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print(f"Connected to Arduino on {PORT} at {BAUDRATE} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def send_command_packet(ser, cmd_bytes):
    """지정된 명령 패킷을 바이너리 형태로 전송합니다."""
    # Arduino는 Serial2로 명령을 수신합니다. Python 코드는 Serial로 전송합니다.
    # 하지만 시뮬레이션을 단순화하기 위해, Arduino의 Serial(ROS) 포트로 명령을 전송합니다.
    # 실제 라즈베리파이 통신 환경에 맞게 Serial 포트 설정을 변경하세요. 
    ser.write(bytes(cmd_bytes))
    print(f"Sent command: {[hex(b) for b in cmd_bytes]}")
    

def decode_sensor_packet(packet):
    """29바이트 센서 패킷을 해석하여 콘솔에 출력합니다."""
    if len(packet) != SENSOR_PACKET_LENGTH:
        return

    # 헤더 체크 (0xf5f5)
    if packet[0] != 0xF5 or packet[1] != 0xF5:
        print("Error: Invalid packet header.")
        return

    # 데이터 추출 및 해석 (가속도, 자이로, 마그네틱은 1000을 곱한 int16)
    # 엔코더는 raw int16
    
    # 2-27바이트 (26바이트 데이터)를 Signed 16-bit Little Endian (h)으로 13개 해석
    # 29바이트 중 헤더(2) + 데이터(26) + 체크섬(1)
    
    # 구조체 해석: (9개 센서값 x int16) + (4개 엔코더 x int16)
    # 9 * 2 bytes = 18 bytes
    # 4 * 2 bytes = 8 bytes 
    # 총 26 bytes
    data = struct.unpack('>hhhhhhhhhhhhhB', packet[2:SENSOR_PACKET_LENGTH])
    
    ax_l, ay_l, az_l = data[0:3]
    gx_l, gy_l, gz_l = data[3:6]
    mx_l, my_l, mz_l = data[6:9]
    lf_enc, rf_enc, lr_enc, rr_enc = data[9:13]
    checksum = data[13]

    print("\n--- Sensor Data Received ---")
    print(f"Accel [g]: X={ax_l/1000.0:.3f}, Y={ay_l/1000.0:.3f}, Z={az_l/1000.0:.3f}")
    print(f"Gyro [dps]: X={gx_l/1000.0:.3f}, Y={gy_l/1000.0:.3f}, Z={gz_l/1000.0:.3f}")
    print(f"Mag [uT?]: X={mx_l/1000.0:.3f}, Y={my_l/1000.0:.3f}, Z={mz_l/1000.0:.3f}")
    print(f"Encoder: LF={lf_enc}, RF={rf_enc}, LR={lr_enc}, RR={rr_enc}")
    print(f"Checksum: {hex(checksum)}")


def main():
    last_command_time = 0
    is_forward = False
    ser = init_serial()
    if not ser:
        return

    cycle_count = 0
    send_command_packet(ser, CMD_RESET)
    
    # 5초간 대기하며 Arduino가 초기화 및 센서 데이터를 보내도록 합니다.
    time.sleep(1) 

    # --- Python Code: Raspberry Pi Simulator Loop (수정됨) ---
    # 기존 코드의 main() 함수 loop 부분에 사용됩니다.

    # SENSOR_PACKET_LENGTH = 29 (전역 변수로 가정)
    # ser = serial.Serial 객체 (전역 변수로 가정)
    # decode_sensor_packet(raw_packet) 함수 (전역 함수로 가정)

    while True:
        try:
            # 1. Arduino로부터 센서 데이터 수신 및 헤더 기반 패킷 동기화
            
            # ⭐️ 버퍼에 최소 1바이트 이상 있을 때만 처리 시작
            if ser.in_waiting > 0:
                
                # --- 🚀 패킷 동기화 로직 시작 ---
                
                # 첫 번째 헤더 바이트(0xF5)를 찾을 때까지 버퍼를 읽습니다.
                if ser.read(1)[0] != 0xF5:
                    continue # 다음 루프로 이동 (패킷 아님)

                # 첫 번째 헤더를 찾았습니다. 두 번째 헤더 바이트(0xF5)를 확인합니다.
                if ser.in_waiting > 0:
                    if ser.read(1)[0] != 0xF5:
                        continue # 두 번째 헤더 불일치, 다시 처음부터 검색 (패킷 아님)
                else:
                    # 두 번째 바이트를 읽기 전에 버퍼가 비었으면 동기화 실패.
                    continue
                
                # --- 🎉 헤더(0xF5F5) 동기화 완료! ---

                # 2. 나머지 패킷 길이 (29 - 2 = 27 바이트)를 정확히 읽어 들입니다.
                REMAINING_BYTES = SENSOR_PACKET_LENGTH - 2
                
                if ser.in_waiting >= REMAINING_BYTES:
                    # 헤더(2바이트)를 제외한 나머지 27바이트를 읽습니다.
                    data_payload = ser.read(REMAINING_BYTES)
                    
                    # 완전한 29바이트 패킷을 재조립합니다.
                    raw_packet = bytes([0xF5, 0xF5]) + data_payload
                    
                    # 패킷 처리
                    decode_sensor_packet(raw_packet)
                    
                else:
                    # 나머지 데이터가 아직 도착하지 않았으므로, 
                    # 불완전한 헤더(0xF5F5)만 버퍼에 남긴 채 다음 루프에서 마저 읽기를 기다립니다.
                    # (이 경우 버퍼에 0xF5F5가 남아있지만, 위 로직에서 다시 찾도록 유도)
                    continue 
                
                # --- 🚀 패킷 동기화 로직 끝 ---


            # 2. 명령 패킷 전송 시뮬레이션 (주석 처리된 원본 로직 유지)
            current_time = time.time()
            if current_time - last_command_time >= 3.0: # 1초(1.0초) 경과 체크
                if is_forward:
                    send_command_packet(ser, CMD_STOP)
                    is_forward = False
                else:
                    send_command_packet(ser, CMD_FORWARD)
                    is_forward = True

                last_command_time = current_time # 타이머 업데이트

            time.sleep(0.01) # 루프 지연을 0.1초에서 0.01초로 줄여 응답성을 높입니다.
                
        except KeyboardInterrupt:
            print("\nExiting program.")
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            break

    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()