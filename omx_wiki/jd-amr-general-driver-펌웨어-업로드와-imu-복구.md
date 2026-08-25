---
title: "JD-AMR General Driver 펌웨어 업로드와 IMU 복구"
tags: ["jdamr", "general-driver", "imu", "qmi8658", "uart", "firmware", "safety"]
created: 2026-08-25T01:36:22.939Z
updated: 2026-08-25T01:36:22.939Z
sources: []
links: []
category: debugging
confidence: medium
schemaVersion: 1
---

# JD-AMR General Driver 펌웨어 업로드와 IMU 복구

2026-08-25 General Driver QMI8658C 이상은 센서 고장이 아니라 시작 보정 버그였다. 한 자세에서 중력을 가속도 오프셋으로 제거하고 자기참조 누적하던 코드를 제거했으며, 가속도는 무보정·자이로 바이어스만 최소 40 coherent sample로 보정한다. 최종 QMI readback은 CTRL1=0x40, CTRL2=0x33, CTRL3=0x63, CTRL5=0x11, CTRL7=0x83. LPF는 약 23.9Hz. 최종 598프레임은 CRC/순번 오류 0, 50Hz, accel norm 951.16mg, gyro p95 1.183dps였다. USB 업로드 전 Pi에서 jdamr-base.service를 stop하고 fe215040.serial을 unbind한 뒤 gpio526(GPIO14)을 input으로 둔다. Type-C를 물리적으로 뽑기 전에는 Pi UART를 bind하거나 서비스를 재시작하지 않는다. 현재 Type-C 연결 상태이며 Pi 서비스 inactive, UART unbound, gpio526=in. 메인 배터리/서보 레일은 최종 캡처에서 꺼졌거나 분리되어 INA219 약 0.15V와 flags 0x07이 나왔다. 직전 메인 전원 검증은 약 10.2V로 저전압이므로 3S 충전 전 주행과 장시간 구동 금지. 상세 문서: IMU_DEBUG_HANDOFF_20260825.md. 원본 4MiB 백업: /home/lim/jdamr_controller_flash_backup_pre_imu_fix_20260825.bin, SHA256 ccb02962de19f6ebceb2351f2872710a9bb7c7597e2e58526a766a693499025e.
