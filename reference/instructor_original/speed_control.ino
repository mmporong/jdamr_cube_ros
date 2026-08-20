
#include <Arduino.h>
#include "STSServoDriver.h"

STSServoDriver st;
const byte MOTOR_ID = 2; // 테스트할 모터 ID

void setup() {
  Serial.begin(115200);
  
  // 1. 서보 통신 초기화 (UART2 사용 예시: RX=16, TX=17)
  // dirPin이 필요 없는 환경이면 255를 입력합니다.
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);
  st.init(&Serial1, 1000000);

  // 2. 모터를 속도 제어 모드(VELOCITY)로 설정
  // STS3215의 경우 정해진 레지스터(0x21)에 모드 값을 씁니다.
  if (st.setMode(MOTOR_ID, VELOCITY)) {
    Serial.println("모드 설정 완료: VELOCITY Mode");
  }

  // 3. 목표 속도 설정 (단위: counts/s)
  // STS3215의 최대 속도는 약 3400 내외입니다.
  // 양수는 정방향, 음수는 역방향입니다.
  int target_speed = 3000; 
  st.setTargetVelocity(MOTOR_ID, target_speed);
}

void loop() {
  // 현재 속도를 읽어와서 출력 (단위 테스트용)
  int current_speed = st.getCurrentSpeed(MOTOR_ID);
  Serial.print("Current Speed: ");
  Serial.println(current_speed);
  
  delay(500);
}