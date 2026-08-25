#include "QMI8658.h"
#include "imu_stationary_calibration.h"

#include <Wire.h>
#include <math.h>

#define QMI8658_UINT_MG_DPS
//#define M_PI			(3.14159265358979323846f)
#define ONE_G			(9.807f)


static qmi8658_state g_imu;

static uint8_t sensor_address()
{
	return g_imu.slave ? g_imu.slave : QMI8658_ADDR;
}

bool QMI8658::write_reg(uint8_t reg,uint8_t value)
{
  Wire.beginTransmission(sensor_address());
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
  return last_status == 0;
}

bool QMI8658::read_regs(uint8_t reg, uint8_t *data, size_t len)
{
  if (data == nullptr || len == 0 || len > 255) {
    last_status = 4;
    return false;
  }

  for (int attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(sensor_address());
    Wire.write(reg);
    last_status = Wire.endTransmission(false);
    if (last_status != 0) {
      delayMicroseconds(50);
      continue;
    }

    const size_t received = Wire.requestFrom(
      sensor_address(), static_cast<uint8_t>(len), static_cast<uint8_t>(true));
    if (received == len) {
      for (size_t i = 0; i < len; ++i) {
        data[i] = static_cast<uint8_t>(Wire.read());
      }
      last_status = 0;
      return true;
    }

    while (Wire.available()) {
      Wire.read();
    }
    last_status = 4;
    delayMicroseconds(50);
  }
  return false;
}

uint8_t QMI8658::read_reg(uint8_t reg)
{
	uint8_t value = 0;
	read_regs(reg, &value, 1);
	return value;
}

uint16_t QMI8658::readWord_reg(uint8_t reg)
{
	uint8_t data[2] = {0, 0};
	if (!read_regs(reg, data, sizeof(data))) {
		return 0;
	}
	return static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
}

bool QMI8658::send_ctrl9_command(uint8_t command)
{
	if (!write_reg(Qmi8658Register_Ctrl9, command)) {
		return false;
	}

	const uint32_t start = micros();
	while (static_cast<uint32_t>(micros() - start) < 100000U) {
		const uint8_t status = read_reg(Qmi8658Register_StatusInt);
		if (last_status == 0 && (status & 0x80U)) {
			if (!write_reg(Qmi8658Register_Ctrl9, 0x00)) {
				return false;
			}
			const uint32_t ack_start = micros();
			while (static_cast<uint32_t>(micros() - ack_start) < 10000U) {
				const uint8_t ack_status = read_reg(Qmi8658Register_StatusInt);
				if (last_status == 0 && (ack_status & 0x80U) == 0) {
					return true;
				}
				delayMicroseconds(50);
			}
			return false;
		}
		delayMicroseconds(100);
	}
	write_reg(Qmi8658Register_Ctrl9, 0x00);
	return false;
}

bool QMI8658::enable_locking_mechanism(void)
{
	// QMI8658C datasheet section 13: SyncSample + disabled AHB clock gating.
	// The CTRL9 command must complete in non-SyncSample mode.  SyncSample is
	// enabled afterwards by enableSensors(), which writes CTRL7 = 0x83.
	sync_sample_enabled = false;
	if (!write_reg(Qmi8658Register_Ctrl7, 0x00) ||
	    !write_reg(Qmi8658Register_Cal1_L, 0x01) ||
	    !send_ctrl9_command(0x12)) {
		write_reg(Qmi8658Register_Ctrl7, 0x00);
		return false;
	}
	sync_sample_enabled = true;
	return true;
}

bool QMI8658::read_raw_sample(int16_t acc[3], int16_t gyro[3])
{
	if (sync_sample_enabled) {
		uint8_t status = 0;
		bool available = false;
		// At 896.8 Hz, eight polls cover more than one sample interval while
		// keeping an electrically stuck I2C bus well inside the motor watchdog.
		for (int attempt = 0; attempt < 8; ++attempt) {
			status = read_reg(Qmi8658Register_StatusInt);
			if (last_status == 0 && (status & 0x01U)) {
				available = true;
				break;
			}
			delayMicroseconds(50);
		}
		if (!available) {
			if (status & 0x02U) {
				read_reg(Qmi8658Register_Gz_H);
			}
			return false;
		}
		if ((status & 0x02U) == 0) {
			// Gyroscope ODR code 3 requires a 6 us data-lock delay.
			delayMicroseconds(6);
		}
	}

	uint8_t data[12];
	if (!read_regs(Qmi8658Register_Ax_L, data, sizeof(data))) {
		if (sync_sample_enabled) {
			// Reading GZ_H releases a lock after a failed burst.
			read_reg(Qmi8658Register_Gz_H);
		}
		return false;
	}

	for (int i = 0; i < 3; ++i) {
		acc[i] = static_cast<int16_t>(
			(static_cast<uint16_t>(data[2 * i + 1]) << 8) | data[2 * i]);
		gyro[i] = static_cast<int16_t>(
			(static_cast<uint16_t>(data[2 * i + 7]) << 8) | data[2 * i + 6]);
	}
	return true;
}

bool QMI8658::read_sensor_data(float acc[3], float gyro[3])
{
	int16_t raw_acc_xyz[3];
	int16_t raw_gyro_xyz[3];
	if (!read_raw_sample(raw_acc_xyz, raw_gyro_xyz)) {
		for (int i = 0; i < 3; ++i) {
			acc[i] = g_imu.imu[i];
			gyro[i] = g_imu.imu[i + 3];
		}
		return false;
	}

#if defined(QMI8658_UINT_MG_DPS)
	// mg
  // Serial.println("mg");
	acc[0] = (float)(raw_acc_xyz[0]*1000.0f)/g_imu.ssvt_a - TempAcc.X_Off_Err;
	acc[1] = (float)(raw_acc_xyz[1]*1000.0f)/g_imu.ssvt_a - TempAcc.Y_Off_Err;
	acc[2] = (float)(raw_acc_xyz[2]*1000.0f)/g_imu.ssvt_a - TempAcc.Z_Off_Err;
#else
	// m/s2
  // Serial.println("m/s2");
	acc[0] = (float)(raw_acc_xyz[0]*ONE_G)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1]*ONE_G)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2]*ONE_G)/g_imu.ssvt_a;
#endif

#if defined(QMI8658_UINT_MG_DPS)
	// dps
  // Serial.println("dps");
	gyro[0] = (float)(raw_gyro_xyz[0]*1.0f)/g_imu.ssvt_g - TempGyr.X_Off_Err;
	gyro[1] = (float)(raw_gyro_xyz[1]*1.0f)/g_imu.ssvt_g - TempGyr.Y_Off_Err;
	gyro[2] = (float)(raw_gyro_xyz[2]*1.0f)/g_imu.ssvt_g - TempGyr.Z_Off_Err;
#else
	// rad/s
  // Serial.println("rad/s");
	gyro[0] = (float)(raw_gyro_xyz[0]*M_PI)/(g_imu.ssvt_g*180);		// *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1]*M_PI)/(g_imu.ssvt_g*180);
	gyro[2] = (float)(raw_gyro_xyz[2]*M_PI)/(g_imu.ssvt_g*180);
#endif

	for (int i = 0; i < 3; ++i) {
		g_imu.imu[i] = acc[i];
		g_imu.imu[i + 3] = gyro[i];
	}
	return true;
}

void QMI8658::read_acc(float acc[3])
{
	float gyro[3];
	read_sensor_data(acc, gyro);
}

void QMI8658::read_gyro(float gyro[3])
{
	float acc[3];
	read_sensor_data(acc, gyro);
}



void QMI8658::axis_convert(float data_a[3], float data_g[3], int layout)
{
	float raw[3],raw_g[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];
	//raw[2] = data[2];
	raw_g[0] = data_g[0];
	raw_g[1] = data_g[1];
	//raw_g[2] = data_g[2];

	if(layout >=4 && layout <= 7)
	{
		data_a[2] = -data_a[2];
		data_g[2] = -data_g[2];
	}

	if(layout%2)
	{
		data_a[0] = raw[1];
		data_a[1] = raw[0];

		data_g[0] = raw_g[1];
		data_g[1] = raw_g[0];
	}
	else
	{
		data_a[0] = raw[0];
		data_a[1] = raw[1];

		data_g[0] = raw_g[0];
		data_g[1] = raw_g[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data_a[0] = -data_a[0];
		data_g[0] = -data_g[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data_a[1] = -data_a[1];
		data_g[1] = -data_g[1];
	}
}



void QMI8658::read_xyz(float acc[3], float gyro[3])
{
	unsigned char	status;
	unsigned char data_ready = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
	qmi8658_read_reg(Qmi8658Register_StatusInt, &status, 1);
	if(status&0x01)
	{
		data_ready = 1;
		qmi8658_delay_us(6);	// delay 6us
	}
#else
	status = read_reg(Qmi8658Register_Status0);
	if(status&0x03)
	{
		data_ready = 1;
	}
#endif
	if(data_ready)
	{
		// read_sensor_data(acc, gyro);
		axis_convert(acc, gyro, 0);
#if defined(QMI8658_USE_CALI)
		qmi8658_data_cali(1, acc);
		qmi8658_data_cali(2, gyro);
#endif
		g_imu.imu[0] = acc[0];
		g_imu.imu[1] = acc[1];
		g_imu.imu[2] = acc[2];
		g_imu.imu[3] = gyro[0];
		g_imu.imu[4] = gyro[1];
		g_imu.imu[5] = gyro[2];
	}
	else
	{
		acc[0] = g_imu.imu[0];
		acc[1] = g_imu.imu[1];
		acc[2] = g_imu.imu[2];
		gyro[0] = g_imu.imu[3];
		gyro[1] = g_imu.imu[4];
		gyro[2] = g_imu.imu[5];
		Serial.print("data ready fail!\n");
	}
}



void QMI8658::config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	unsigned char ctl_dada;

	switch(range)
	{
		case Qmi8658AccRange_2g:
			g_imu.ssvt_a = (1<<14);
			break;
		case Qmi8658AccRange_4g:
			g_imu.ssvt_a = (1<<13);
			break;
		case Qmi8658AccRange_8g:
			g_imu.ssvt_a = (1<<12);
			break;
		case Qmi8658AccRange_16g:
			g_imu.ssvt_a = (1<<11);
			break;
		default:
			range = Qmi8658AccRange_8g;
			g_imu.ssvt_a = (1<<12);
	}
	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range|(unsigned char)odr;

	write_reg(Qmi8658Register_Ctrl2, ctl_dada);
// set LPF & HPF
	ctl_dada = read_reg(Qmi8658Register_Ctrl5);
	ctl_dada &= 0xf0;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		// 2.66% of the 896.8 Hz effective ODR = 23.9 Hz, below the
		// 50 Hz host stream Nyquist frequency.
		ctl_dada |= A_LSP_MODE_0;
		ctl_dada |= 0x01;
	}
	else
	{
		ctl_dada &= ~0x01;
	}
	//ctl_dada = 0x00;
	write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void QMI8658::config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada;

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case Qmi8658GyrRange_16dps:
			g_imu.ssvt_g = 2048;
			break;
		case Qmi8658GyrRange_32dps:
			g_imu.ssvt_g = 1024;
			break;
		case Qmi8658GyrRange_64dps:
			g_imu.ssvt_g = 512;
			break;
		case Qmi8658GyrRange_128dps:
			g_imu.ssvt_g = 256;
			break;
		case Qmi8658GyrRange_256dps:
			g_imu.ssvt_g = 128;
			break;
		case Qmi8658GyrRange_512dps:
			g_imu.ssvt_g = 64;
			break;
		case Qmi8658GyrRange_1024dps:
			g_imu.ssvt_g = 32;
			break;
		case Qmi8658GyrRange_2048dps:
			g_imu.ssvt_g = 16;
			break;
//		case Qmi8658GyrRange_4096dps:
//			g_imu.ssvt_g = 8;
//			break;
		default:
			range = Qmi8658GyrRange_512dps;
			g_imu.ssvt_g = 64;
			break;
	}

	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;
	write_reg(Qmi8658Register_Ctrl3, ctl_dada);

// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	ctl_dada = read_reg(Qmi8658Register_Ctrl5);
	ctl_dada &= 0x0f;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= G_LSP_MODE_0;
		ctl_dada |= 0x10;
	}
	else
	{
		ctl_dada &= ~0x10;
	}
	//ctl_dada = 0x00;
	write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void QMI8658::enableSensors(unsigned char enableFlags)
{
	const uint8_t sync_flag = sync_sample_enabled ? 0x80U : 0x00U;
	write_reg(Qmi8658Register_Ctrl7, (enableFlags & 0x03U) | sync_flag);
	g_imu.cfg.enSensors = enableFlags&0x03;

	delay(1);
}

void QMI8658::config_reg(unsigned char low_power)
{
	enableSensors(QMI8658_DISABLE_ALL);
	if(low_power)
	{
		g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_LowPower_21Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_250Hz;
	}
	else
	{
		g_imu.cfg.enSensors = QMI8658_ACCGYR_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_16g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_1000Hz;
		// QMI8658C Rev A defines range code 7 as N/A; use the widest valid range.
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_1000Hz;
	}

	if(g_imu.cfg.enSensors & QMI8658_ACC_ENABLE)
	{
		config_acc(g_imu.cfg.accRange, g_imu.cfg.accOdr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	}
	if(g_imu.cfg.enSensors & QMI8658_GYR_ENABLE)
	{
		config_gyro(g_imu.cfg.gyrRange, g_imu.cfg.gyrOdr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	}
}

unsigned char QMI8658::get_id(void)
{
	unsigned char qmi8658_chip_id = 0x00;
	unsigned char qmi8658_revision_id = 0x00;
	unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
	int retry = 0;
	unsigned char iCount = 0;
	unsigned char firmware_id[3];
	unsigned char uuid[6];
	unsigned int uuid_low, uuid_high;

	while(iCount<2)
	{
		g_imu.slave = qmi8658_slave[iCount];
		retry = 0;
		while((qmi8658_chip_id != 0x05)&&(retry++ < 5))
		{
			qmi8658_chip_id = read_reg(Qmi8658Register_WhoAmI);
			Serial.printf("Qmi8658Register_WhoAmI = 0x%x\n", qmi8658_chip_id);
		}
		if(qmi8658_chip_id == 0x05)
		{
			qmi8658_on_demand_cali();

			g_imu.cfg.ctrl8_value = 0xc0;
			// Address auto-increment on, little-endian output, unused INT pins off.
			// The burst decoder below consumes Ax_L..Gz_H in little-endian order.
			write_reg(Qmi8658Register_Ctrl1, 0x40);
			qmi8658_revision_id = read_reg(Qmi8658Register_Revision);
			// qmi8658_read_reg(Qmi8658Register_firmware_id, firmware_id, 3);
			// qmi8658_read_reg(Qmi8658Register_uuid, uuid, 6);
			write_reg(Qmi8658Register_Ctrl7, 0x00);
			write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
			// uuid_low = (unsigned int)((unsigned int)(uuid[2]<<16)|(unsigned int)(uuid[1]<<8)|(uuid[0]));
			// uuid_high = (unsigned int)((unsigned int)(uuid[5]<<16)|(unsigned int)(uuid[4]<<8)|(uuid[3]));
			// qmi8658_log("qmi8658_init slave=0x%x Revision=0x%x\n", g_imu.slave, qmi8658_revision_id);
			// qmi8658_log("Firmware ID[0x%x 0x%x 0x%x]\n", firmware_id[2], firmware_id[1],firmware_id[0]);
			// qmi8658_log("UUID[0x%x %x]\n", uuid_high ,uuid_low);
			break;
		}
		iCount++;
	}

	return qmi8658_chip_id;
}


void QMI8658::qmi8658_on_demand_cali(void)
{
	Serial.print("qmi8658_on_demand_cali start\n");
	write_reg(Qmi8658Register_Reset, 0xb0);
	delay(10);	// delay
	write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	delay(2200);	// delay 2000ms above
	write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_NOP);
	delay(100);	// delay
	Serial.print("qmi8658_on_demand_cali done\n");
}

unsigned char QMI8658::begin(void)
{
	if(get_id() == 0x05)
	{
#if defined(QMI8658_USE_AMD)
		qmi8658_config_amd();
#endif
#if defined(QMI8658_USE_PEDOMETER)
		qmi8658_config_pedometer(125);
		qmi8658_enable_pedometer(1);
#endif
		config_reg(0);
		if (!enable_locking_mechanism()) {
			Serial.println("QMI8658 SyncSample locking unavailable; using burst-read fallback");
		}
		enableSensors(g_imu.cfg.enSensors);
    if (!dump_reg()) {
      Serial.println("QMI8658 configuration readback failed");
      return 0;
    }
    Serial.println("Keep QMI8658 still - calibrating gyro bias...");
    delay(1000);
    if (!autoOffsets()) {
      return 0;
    }
#if defined(QMI8658_USE_CALI)
		memset(&g_cali, 0, sizeof(g_cali));
#endif
		return 1;
	}
	else
	{
		// Serial.print("qmi8658_init fail\n");
		return 0;
	}
}


bool QMI8658::dump_reg(void)
{
	uint8_t ctrl1 = 0;
	uint8_t ctrl2 = 0;
	uint8_t ctrl3 = 0;
	uint8_t ctrl5 = 0;
	uint8_t ctrl7 = 0;
	const bool read_ok =
		read_regs(Qmi8658Register_Ctrl1, &ctrl1, 1) &&
		read_regs(Qmi8658Register_Ctrl2, &ctrl2, 1) &&
		read_regs(Qmi8658Register_Ctrl3, &ctrl3, 1) &&
		read_regs(Qmi8658Register_Ctrl5, &ctrl5, 1) &&
		read_regs(Qmi8658Register_Ctrl7, &ctrl7, 1);

	Serial.printf(
		"QMI8658 config CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL5=0x%02X CTRL7=0x%02X\n",
		ctrl1, ctrl2, ctrl3, ctrl5, ctrl7);
	if (!read_ok) {
		return false;
	}

	const uint8_t expected_ctrl1 = 0x40;
	const uint8_t expected_ctrl2 =
		static_cast<uint8_t>(g_imu.cfg.accRange) |
		static_cast<uint8_t>(g_imu.cfg.accOdr);
	const uint8_t expected_ctrl3 =
		static_cast<uint8_t>(g_imu.cfg.gyrRange) |
		static_cast<uint8_t>(g_imu.cfg.gyrOdr);
	const uint8_t expected_ctrl7 =
		(g_imu.cfg.enSensors & 0x03U) | (sync_sample_enabled ? 0x80U : 0x00U);
	return ctrl1 == expected_ctrl1 &&
		ctrl2 == expected_ctrl2 &&
		ctrl3 == expected_ctrl3 &&
		ctrl5 == 0x11U &&
		ctrl7 == expected_ctrl7;
}

bool QMI8658::autoOffsets(void){
    float acc[3], gyro[3];
    TempAcc = {0};
    TempGyr = {0};

    jdamr_fw::ImuVector3 accel_sum_mg{0.0f, 0.0f, 0.0f};
    jdamr_fw::ImuVector3 gyro_sum_dps{0.0f, 0.0f, 0.0f};

    int samples = 0;
    for (int attempts = 0; attempts < 200 && samples < 50; ++attempts) {
        if (read_sensor_data(acc, gyro)) {
            accel_sum_mg.x += acc[0];
            accel_sum_mg.y += acc[1];
            accel_sum_mg.z += acc[2];
            gyro_sum_dps.x += gyro[0];
            gyro_sum_dps.y += gyro[1];
            gyro_sum_dps.z += gyro[2];
            ++samples;
        }
        delay(10);
    }

	constexpr int kMinimumCalibrationSamples = 40;
    if (samples < kMinimumCalibrationSamples) {
        Serial.printf(
          "QMI8658 calibration failed: only %d coherent samples (need %d)\n",
          samples, kMinimumCalibrationSamples);
		return false;
    }

	jdamr_fw::ImuStationaryOffsets offsets{};
	if (!jdamr_fw::compute_stationary_offsets(
		gyro_sum_dps, static_cast<size_t>(samples), &offsets)) {
		return false;
	}
	TempAcc = {
		offsets.accel_mg.x,
		offsets.accel_mg.y,
		offsets.accel_mg.z,
	};
	TempGyr = {
		offsets.gyro_dps.x,
		offsets.gyro_dps.y,
		offsets.gyro_dps.z,
	};

	const float divisor = static_cast<float>(samples);
	const float mean_ax = accel_sum_mg.x / divisor;
	const float mean_ay = accel_sum_mg.y / divisor;
	const float mean_az = accel_sum_mg.z / divisor;
	const float accel_norm = sqrtf(
		mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az);
	Serial.printf(
		"QMI8658 raw mean accel=(%.1f,%.1f,%.1f)mg norm=%.1fmg gyro_bias=(%.3f,%.3f,%.3f)dps samples=%d\n",
		mean_ax, mean_ay, mean_az, accel_norm,
		TempGyr.X_Off_Err, TempGyr.Y_Off_Err, TempGyr.Z_Off_Err, samples);
	return true;
}
