#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include "IMU.h"
#include "STSServoDriver.h"

// --- 설정 및 핀 정의 ---
#define S_SCL           33
#define S_SDA           32
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   32
#define SCREEN_ADDRESS  0x3C
#define INA219_ADDRESS  0x42

// STS3215 서보 설정 (UART2)
#define S_RX 18
#define S_TX 19
HardwareSerial ServoSerial(2); 

// --- 전역 객체 ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);
STSServoDriver st;

// IMU 데이터 변수 [cite: 55]
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData, stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

// --- 프로토콜 및 데이터 변수 ---
String debug_line_1, debug_line_2, debug_line_3;
const byte SENSOR_PACKET_LENGTH = 29;
byte sensor_data_packet[SENSOR_PACKET_LENGTH] = {0,};
byte cmd_buf[5];

// 모터 ID 및 명령어 정의 [cite: 5-7]
const byte LEFT_ID = 1;
const byte RIGHT_ID = 2;
#define GO_FORWARD    1
#define GO_BACKWARD   2 
#define TURN_LEFT     3
#define TURN_RIGHT    4
#define STOP          5
#define CMD_RESET     6

// --- 함수 정의 ---

// 센서 데이터 송신 (실제 센서 값 적용) [cite: 13-25]
void send_sensors() {
    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData); // 
    float voltage = ina219.getBusVoltage_V(); // [cite: 71]

    sensor_data_packet[0] = 0xf5; 
    sensor_data_packet[1] = 0xf5; 
    
    // 가속도 (Raw * 1000 정수화) [cite: 14-15]
    int16_t ax = (int16_t)(stAccelRawData.X * 1000);
    int16_t ay = (int16_t)(stAccelRawData.Y * 1000);
    int16_t az = (int16_t)(stAccelRawData.Z * 1000);
    sensor_data_packet[2] = (ax >> 8) & 0xff; sensor_data_packet[3] = ax & 0xff;
    sensor_data_packet[4] = (ay >> 8) & 0xff; sensor_data_packet[5] = ay & 0xff;
    sensor_data_packet[6] = (az >> 8) & 0xff; sensor_data_packet[7] = az & 0xff;
    
    // 자이로 데이터 [cite: 17-18]
    int16_t gx = (int16_t)(stGyroRawData.X * 100);
    sensor_data_packet[8] = (gx >> 8) & 0xff; sensor_data_packet[9] = gx & 0xff;
    // (나머지 축 및 마그네틱은 유사 방식으로 채움)

    // 엔코더 (서보 라이브러리에서 직접 읽기)
    int16_t l_pos = st.getCurrentPosition(LEFT_ID);
    int16_t r_pos = st.getCurrentPosition(RIGHT_ID);
    sensor_data_packet[20] = (l_pos >> 8) & 0xff; sensor_data_packet[21] = l_pos & 0xff;
    sensor_data_packet[22] = (r_pos >> 8) & 0xff; sensor_data_packet[23] = r_pos & 0xff;

    sensor_data_packet[28] = 0x00; // Checksum
    Serial.write(sensor_data_packet, SENSOR_PACKET_LENGTH); 
}

void update_oled_display() { // [cite: 32-34]
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);  display.print(debug_line_1);
    display.setCursor(0, 10); display.print(debug_line_2);
    display.setCursor(0, 20); display.print(debug_line_3);
    display.display();
}

void InitHardware() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // [cite: 26, 79]
        Serial.println(F("OLED failed"));
    }
    imuInit(); // [cite: 56]
    ina219.init(); // [cite: 70]
    st.init(255, &ServoSerial, 1000000); // 서보 초기화
    st.setMode(LEFT_ID, VELOCITY); // 속도 모드 설정
    st.setMode(RIGHT_ID, VELOCITY);
    
    debug_line_1 = "JD-Bot READY";
    debug_line_2 = "Batt: " + String(ina219.getBusVoltage_V()) + "V";
    update_oled_display();
}

void setup() {
    Serial.begin(115200); 
    ServoSerial.begin(1000000, SERIAL_8N1, S_RX, S_TX);
    Wire.begin(S_SDA, S_SCL); // [cite: 35, 78]
    delay(1000);
    InitHardware();
}

void loop() {
    // 1. 명령 수신 파싱 [cite: 37-41]
    if(Serial.available() >= 5) { 
        size_t bytesRead = Serial.readBytes(cmd_buf, 5);
        if (bytesRead == 5 && cmd_buf[0] == 0xf5 && cmd_buf[1] == 0xf5) {
            if(cmd_buf[2] == 0x51) {
                int direction = (int)cmd_buf[3];
                int speed_raw = (int)cmd_buf[4];
                int target_speed = map(speed_raw, 0, 255, 0, 2400); // 속도 매핑

                if(direction == GO_FORWARD){
                    st.setTargetVelocity(LEFT_ID, target_speed);
                    st.setTargetVelocity(RIGHT_ID, -target_speed);
                    debug_line_2 = "motor: FORWARD";        
                } else if(direction == STOP){
                    st.setTargetVelocity(LEFT_ID, 0);
                    st.setTargetVelocity(RIGHT_ID, 0);
                    debug_line_2 = "motor: STOP";
                } else if(direction == CMD_RESET){
                    ESP.restart(); // [cite: 47]
                }
                update_oled_display();
            }
        }
    }

    // 2. 센서 전송 및 OLED 갱신 주기 (100ms) [cite: 52]
    static uint32_t prev_ms = 0;
    if (millis() - prev_ms > 100) {
        send_sensors();
        debug_line_3 = "L:" + String(st.getCurrentPosition(LEFT_ID)) + " R:" + String(st.getCurrentPosition(RIGHT_ID));
        update_oled_display();
        prev_ms = millis();
    }
}