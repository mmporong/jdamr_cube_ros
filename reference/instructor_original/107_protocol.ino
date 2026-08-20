// --- Arduino Code: Sensor Data Sender & Command Receiver ---
#include <Wire.h>
#include <Adafruit_SSD1306.h> // OLED 라이브러리 추가

// --- I2C 및 OLED 설정 (ESP32 기준) ---
#define S_SCL           33
#define S_SDA           32
#define SCREEN_WIDTH    128 // OLED display width, in pixels
#define SCREEN_HEIGHT   32  // OLED display height, in pixels
#define OLED_RESET      -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C ///< 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED 출력 버퍼 (디버그 메시지를 저장하고 화면에 표시하기 위함)
String debug_line_1 = "        ";
String debug_line_2 = "        ";
String debug_line_3 = "        ";
// ----------------------------------------------------
// 1. 센서 데이터 송신 패킷 정의 (29 Bytes)
// 헤더(2) + 가속도(3x2) + 자이로(3x2) + 마그네틱(3x2) + 엔코더(4x2) + 체크섬(1) = 29 bytes
// ----------------------------------------------------
const byte SENSOR_PACKET_LENGTH = 29;
byte sensor_data_packet[SENSOR_PACKET_LENGTH] = {0,};

// ----------------------------------------------------
// 2. 모터 제어 명령 수신 패킷 정의 (5 Bytes)
// 헤더(2) + 모드(1) + 방향/각도/모터번호(1) + 속도(1) = 5 bytes
// ----------------------------------------------------
const byte COMMAND_PACKET_LENGTH = 5;
byte cmd_buf[COMMAND_PACKET_LENGTH];

// Run Mode 명령어 정의 (원본 코드에서 가져옴)
#define GO_FORWARD    1
#define GO_BACKWARD   2 
#define TURN_LEFT     3
#define TURN_RIGHT    4
#define STOP          5
#define CMD_RESET     6

// ----------------------------------------------------
// 3. 가상 센서 및 엔코더 데이터
// ----------------------------------------------------
// (원본 코드에서 1000을 곱했으므로, 가상 값도 그에 맞춰 정수 2바이트로 준비)
int ax_l = 123;   // Accel X (가상값)
int ay_l = 456;   // Accel Y
int az_l = 9800;  // Accel Z (대략 1G * 1000)
int gx_l = 100;   // Gyro X
int gy_l = 200;   // Gyro Y
int gz_l = 300;   // Gyro Z
int mx_l = 1000;  // Magnetic X
int my_l = -500;  // Magnetic Y
int mz_l = 2000;  // Magnetic Z

// 엔코더 (2바이트 정수)
int lf_encoder = 10000;
int rf_encoder = 9900;
int lr_encoder = 10100;
int rr_encoder = 10050;

// ----------------------------------------------------
// 4. 함수 정의
// ----------------------------------------------------

void send_sensors() {
    // 헤더 (0-1)
    sensor_data_packet[0] = 0xf5; 
    sensor_data_packet[1] = 0xf5; 
    
    // 가속도 (2-7)
    sensor_data_packet[2] = (ax_l >> 8) & 0xff;
    sensor_data_packet[3] = ax_l & 0xff; 
    sensor_data_packet[4] = (ay_l >> 8) & 0xff;
    sensor_data_packet[5] = ay_l & 0xff;
    sensor_data_packet[6] = (az_l >> 8) & 0xff;
    sensor_data_packet[7] = az_l & 0xff;
    
    // 자이로 (8-13)
    sensor_data_packet[8] = (gx_l >> 8) & 0xff;
    sensor_data_packet[9] = gx_l & 0xff; 
    sensor_data_packet[10] = (gy_l >> 8) & 0xff;
    sensor_data_packet[11] = gy_l & 0xff;
    sensor_data_packet[12] = (gz_l >> 8) & 0xff;
    sensor_data_packet[13] = gz_l & 0xff;
    
    // 마그네틱 (14-19)
    sensor_data_packet[14] = (mx_l >> 8) & 0xff;
    sensor_data_packet[15] = mx_l & 0xff; 
    sensor_data_packet[16] = (my_l >> 8) & 0xff;
    sensor_data_packet[17] = my_l & 0xff;
    sensor_data_packet[18] = (mz_l >> 8) & 0xff;
    sensor_data_packet[19] = mz_l & 0xff;
    
    // 엔코더 (20-27)
    sensor_data_packet[20] = (lf_encoder >> 8) & 0xff;
    sensor_data_packet[21] = lf_encoder & 0xff;
    sensor_data_packet[22] = (rf_encoder >> 8) & 0xff;
    sensor_data_packet[23] = rf_encoder & 0xff;
    sensor_data_packet[24] = (lr_encoder >> 8) & 0xff;
    sensor_data_packet[25] = lr_encoder & 0xff;
    sensor_data_packet[26] = (rr_encoder >> 8) & 0xff;
    sensor_data_packet[27] = rr_encoder & 0xff;
    
    // 체크섬 (28) (임시로 0x00)
    sensor_data_packet[28] = 0x00; 
    
    // Serial 포트로 바이너리 데이터 전송
    Serial.write(sensor_data_packet, SENSOR_PACKET_LENGTH); 

}

void InitScreen(){
  // I2C 핀 초기화 (setup에서 Wire.begin(S_SDA, S_SCL); 실행)
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // OLED 실패 시 시스템 정지 또는 에러 표시
  }
  display.clearDisplay();
  display.display();
  debug_line_1 = "RESET          ";
  debug_line_2 = "System start...";
  debug_line_3 = "               ";
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(debug_line_1);
  display.setCursor(0, 10);
  display.print(debug_line_2);
  display.setCursor(0, 20);
  display.print(debug_line_3);
  display.display();
  delay(2000);
  debug_line_1 = "RUNNING       ";
  debug_line_2 = "motor: STOP   ";
  debug_line_3 = "Encoder: " + String(lf_encoder);
  update_oled_display();
}

void update_oled_display() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // 첫번째 줄
  display.setCursor(0, 0);
  display.print(debug_line_1);
  // 두번째 줄
  display.setCursor(0, 10);
  display.print(debug_line_2); 
  // 세번째 줄
  display.setCursor(0, 20);
  display.print(debug_line_3);
  display.display();
}


void setup() {
    // Serial: ROS2 통신 (센서 데이터 송신)
    Serial.begin(115200); 
    Wire.begin(S_SDA, S_SCL); // ⭐️ I2C 통신 시작 (OLED 및 IMU용)
    delay(1000);
    InitScreen(); // ⭐️ OLED 초기화
}

void loop() {
    // ⭐️ 1. 시리얼 포트(Serial/ROS)에 5바이트 이상의 데이터가 있는지 확인
    if(Serial.available() >= 5) { 
        // 2. 명령 패킷 5바이트를 수신 버퍼로 읽어 들입니다.
        // Serial.readBytes는 수신 가능한 바이트가 부족하면 타임아웃까지 대기합니다.
        size_t bytesRead = Serial.readBytes(cmd_buf, 5); 
        // 3. 읽은 바이트 수가 5인지 확인하고 패킷을 해석합니다.
        if (bytesRead == 5) {
            
            // 4. 헤더 체크: 0xF5, 0xF5
            if(cmd_buf[0] == 0xf5 && cmd_buf[1] == 0xf5) {
                
                // 5. 모드 체크: Run Mode (0x51)?
                if(cmd_buf[2] == 0x51) {
                    
                    int direction = (int)cmd_buf[3]; // Direction (0x01 ~ 0x05)
                    int speed = (int)cmd_buf[4];     // Speed (0x00 ~ 0xFF)
                    
                    // 6. 방향에 따른 모터 제어 함수 호출
                    if(direction == GO_FORWARD){
                        debug_line_2 = "motor: FOWARD ";        
                    } else if(direction == GO_BACKWARD){
                        debug_line_2 = "motor: BACK   ";
                    } else if(direction == TURN_LEFT){
                        debug_line_2 = "motor: LEFT   ";
                    } else if(direction == TURN_RIGHT){
                        debug_line_2 = "motor: RIGHT  ";
                    } else if(direction == STOP){
                        debug_line_2 = "motor: STOP   ";
                    } else if(direction == CMD_RESET){
                        debug_line_2 = "System reset  ";
                        delay(5000);
                        ESP.restart();
                    } else {
                          debug_line_3 = "Command error  ";
                          update_oled_display();
                    }
                } 
                // ToDo: 여기에 0x52 (Diff Drive), 0x53 (Wheel Drive) 등 다른 모드 해석을 추가합니다.
                
                else {
                    debug_line_3 = "Command error  ";
                    update_oled_display();
                }
            } else {
                // 헤더 불일치 시: 시리얼 버퍼 정크 데이터 처리 (옵션)
                debug_line_3 = "Packet error   ";
                update_oled_display();
            }
        }
    }
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 100) {
        send_sensors();
        prev_ms = millis();
        
        // 엔코더 값 가상 업데이트 (움직이는 것처럼 보이게)
        lf_encoder += 10;
        rf_encoder += 10;
        lr_encoder += 10;
        rr_encoder += 10;
        debug_line_3 = "Encoder: " + String(lf_encoder);
        update_oled_display();
    }

}