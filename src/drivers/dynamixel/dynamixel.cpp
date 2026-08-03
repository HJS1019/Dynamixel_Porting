#include "dynamixel.hpp"

#include <sys/ioctl.h>
#ifdef __PX4_NUTTX
# include <nuttx/serial/tioctl.h>
#endif

#include <cmath>
#include <fcntl.h>
#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdlib>

namespace
{

bool parseUnsignedArgument(const char *text, uint32_t minimum, uint32_t maximum, uint32_t &value, int base = 10)
{
	if (text == nullptr || text[0] == '\0' || text[0] == '-') {
		return false;
	}

	errno = 0;
	char *end = nullptr;
	const unsigned long long parsed = strtoull(text, &end, base);

	if (errno == ERANGE || end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
		return false;
	}

	value = static_cast<uint32_t>(parsed);
	return true;
}

class BusLock
{
public:
	explicit BusLock(pthread_mutex_t &mutex) :
		_mutex(mutex),
		_locked(pthread_mutex_lock(&_mutex) == 0)
	{
	}

	~BusLock()
	{
		if (_locked) {
			pthread_mutex_unlock(&_mutex);
		}
	}

	bool locked() const { return _locked; }

private:
	pthread_mutex_t &_mutex;
	bool _locked;
};

} // namespace

Dynamixel::Dynamixel(const char *port, int baudrate, RunMode run_mode, WireMode wire_mode,
		     uint8_t first_servo_id, uint8_t active_servo_count, unsigned feedback_rate_hz) :
	ModuleParams(nullptr),
	_baudrate(baudrate),
	_run_mode(run_mode),
	_wire_mode(wire_mode),
	_active_servo_count(active_servo_count)
{
	strncpy(_port, port, sizeof(_port) - 1);

	for (unsigned i = 0; i < MAX_SERVOS; ++i) {
		_servo_ids[i] = first_servo_id + i;
	}

	if (feedback_rate_hz > 0) {
		_feedback_interval_us = 1000000 / feedback_rate_hz;
	}

	_mutex_initialized = (pthread_mutex_init(&_bus_mutex, nullptr) == 0);
}

Dynamixel::~Dynamixel()
{
	closeSerial();

	if (_mutex_initialized) {
		pthread_mutex_destroy(&_bus_mutex);
	}
}

bool Dynamixel::init()
{
	if (!_mutex_initialized) {
		PX4_ERR("failed to initialize bus mutex");
		return false;
	}

	updateParams();
	updateCalibration();
	BusLock lock(_bus_mutex);

	if (!lock.locked() || !openSerial()) {
		return false;
	}

	_connected_mask = 0;
	PX4_INFO("bus idle; torque state unchanged (use explicit NSH commands)");
	return true;
}

bool Dynamixel::enableTorque(uint8_t id, bool enable)
{
	return writeRegister(id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, 1);
}


bool Dynamixel::angleToPosition(unsigned servo_index, float angle_rad, uint32_t &position) const
{
	if (servo_index >= MAX_SERVOS || !PX4_ISFINITE(angle_rad)) {
		return false;
	}

	const DynamixelPositionMapping::Calibration calibration {
		_zero_raw[servo_index],
		_direction[servo_index],
		_min_raw[servo_index],
		_max_raw[servo_index]
	};
	return DynamixelPositionMapping::angleToPosition(calibration, angle_rad, position);
}

float Dynamixel::positionToAngle(unsigned servo_index, uint32_t position) const
{
	if (servo_index >= MAX_SERVOS) {
		return NAN;
	}

	const DynamixelPositionMapping::Calibration calibration {
		_zero_raw[servo_index],
		_direction[servo_index],
		_min_raw[servo_index],
		_max_raw[servo_index]
	};
	return DynamixelPositionMapping::positionToAngle(calibration, position);
}

void Dynamixel::updateCalibration()
{
	const int32_t requested_zero[MAX_SERVOS] {
		_param_dxl_s1_zero.get(), _param_dxl_s2_zero.get(), _param_dxl_s3_zero.get(), _param_dxl_s4_zero.get()
	};
	const int32_t requested_direction[MAX_SERVOS] {
		_param_dxl_s1_dir.get(), _param_dxl_s2_dir.get(), _param_dxl_s3_dir.get(), _param_dxl_s4_dir.get()
	};
	const int32_t requested_min[MAX_SERVOS] {
		_param_dxl_s1_min.get(), _param_dxl_s2_min.get(), _param_dxl_s3_min.get(), _param_dxl_s4_min.get()
	};
	const int32_t requested_max[MAX_SERVOS] {
		_param_dxl_s1_max.get(), _param_dxl_s2_max.get(), _param_dxl_s3_max.get(), _param_dxl_s4_max.get()
	};
	BusLock lock(_bus_mutex);

	if (!lock.locked()) {
		PX4_ERR("failed to lock calibration");
		return;
	}

	for (unsigned i = 0; i < MAX_SERVOS; ++i) {
		const DynamixelPositionMapping::Calibration requested {
			requested_zero[i],
			requested_direction[i],
			requested_min[i],
			requested_max[i]
		};

		if (!DynamixelPositionMapping::validCalibration(requested)) {
			PX4_WARN("servo %u calibration rejected: zero=%" PRId32 " dir=%" PRId32
				 " min=%" PRId32 " max=%" PRId32,
				 i + 1, requested.zero, requested.direction,
				 requested.minimum, requested.maximum);
			continue;
		}

		_zero_raw[i] = requested.zero;
		_direction[i] = requested.direction;
		_min_raw[i] = requested.minimum;
		_max_raw[i] = requested.maximum;
	}
}

void Dynamixel::updateParameters()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		updateCalibration();
	}
}
// ============================================================
//  Serial (raw POSIX termios)
// ============================================================

bool Dynamixel::openSerial()
{
	_uart_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open %s (errno=%d)", _port, errno);
		return false;
	}

	struct termios uart_config {};

	if (tcgetattr(_uart_fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed");
		closeSerial();
		return false;
	}

	// raw mode 설정
	uart_config.c_iflag &= ~(INLCR | ICRNL | IGNCR | IXON | IXOFF);
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;

	speed_t speed;

	switch (_baudrate) {
	case 9600:    speed = B9600;    break;

	case 57600:   speed = B57600;   break;

	case 115200:  speed = B115200;  break;

	case 1000000: speed = B1000000; break;

	case 2000000: speed = B2000000; break;

	case 3000000: speed = B3000000; break;

	case 4000000: speed = B4000000; break;

	default:
		PX4_ERR("unsupported baudrate %d", _baudrate);
		closeSerial();
		return false;
	}

	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("tcsetattr failed");
		closeSerial();
		return false;
	}

	if (tcflush(_uart_fd, TCIOFLUSH) != 0) {
		PX4_ERR("initial serial flush failed (errno=%d)", errno);
		closeSerial();
		return false;
	}

#if defined(TIOCSSINGLEWIRE) && defined(SER_SINGLEWIRE_ENABLED)

	if (_wire_mode == WireMode::FullDuplexUart) {
		if (ioctl(_uart_fd, TIOCSSINGLEWIRE, 0) < 0) {
			PX4_ERR("failed to disable single-wire mode (errno=%d)", errno);
			closeSerial();
			return false;
		}

	} else {
		unsigned long sw_arg = SER_SINGLEWIRE_ENABLED;

		if (_wire_mode == WireMode::SingleWirePushPull) {
# ifdef SER_SINGLEWIRE_PUSHPULL
			sw_arg |= SER_SINGLEWIRE_PUSHPULL;
# else
			PX4_ERR("push-pull single-wire is not supported by this platform");
			closeSerial();
			return false;
# endif

		} else {
# ifdef SER_SINGLEWIRE_PULLUP
			sw_arg |= SER_SINGLEWIRE_PULLUP;
# endif
		}

		if (ioctl(_uart_fd, TIOCSSINGLEWIRE, sw_arg) < 0) {
			PX4_ERR("failed to enable requested single-wire mode (errno=%d)", errno);
			closeSerial();
			return false;
		}
	}

#else

	if (_wire_mode != WireMode::FullDuplexUart) {
		PX4_ERR("requested single-wire mode is not supported by this platform");
		closeSerial();
		return false;
	}

#endif

	const char *wire_mode = (_wire_mode == WireMode::FullDuplexUart) ? "uart"
				: (_wire_mode == WireMode::SingleWirePushPull) ? "single-pushpull"
				: "single";
	PX4_INFO("opened %s @ %d baud, wire=%s", _port, _baudrate, wire_mode);
	return true;
}

void Dynamixel::closeSerial()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

int Dynamixel::serialWrite(const uint8_t *buf, int len, hrt_abstime deadline)
{
	if (_uart_fd < 0 || buf == nullptr || len <= 0) {
		return -1;
	}

	int total = 0;

	while (total < len && hrt_absolute_time() < deadline) {
		const int written = ::write(_uart_fd, buf + total, len - total);

		if (written > 0) {
			total += written;
			continue;
		}

		if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
			return -1;
		}

		px4_usleep(100);
	}

	return total;
}

int Dynamixel::serialRead(uint8_t *buf, int len, hrt_abstime deadline)
{
	if (_uart_fd < 0 || buf == nullptr || len <= 0) {
		return -1;
	}

	int total = 0;

	while (total < len && hrt_absolute_time() < deadline) {
		const int n = ::read(_uart_fd, buf + total, len - total);

		if (n > 0) {
			total += n;
			continue;
		}

		if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
			return -1;
		}

		px4_usleep(100);
	}

	return total;
}

bool Dynamixel::flushInput()
{
	return _uart_fd >= 0 && tcflush(_uart_fd, TCIFLUSH) == 0;
}

// ============================================================
//  Dynamixel Protocol 2.0
//  CRC table 출처: ROBOTIS e-Manual "CRC-16 (IBM/ANSI)"
//                  https://emanual.robotis.com/docs/en/dxl/crc/
//  패킷 구조 참고: ArduPilot AP_RobotisServo.cpp
// ============================================================

uint16_t Dynamixel::updateCRC(uint16_t crc_accum, const uint8_t *data, uint16_t data_len)
{
	static const uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for (uint16_t j = 0; j < data_len; j++) {
		const uint16_t i = ((uint16_t)(crc_accum >> 8) ^ data[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

// packet[PKT_LEN_L]/[PKT_LEN_H]에 "Instruction(1)+Params+CRC(2)" 길이가
// 이미 채워져 있다고 가정. 헤더/CRC를 채우고 전송한다.
//
// [주의] 바이트 스터핑(byte stuffing) 미구현:
//   Protocol 2.0은 파라미터 안에 0xFF 0xFF 0xFD 패턴이 나오면 0xFD를 끼워넣어야 한다.
//   현재 용도(위치값 0~4095, 주소/길이 소값)에서는 이 패턴이 나올 수 없어 안전하지만,
//   확장위치/멀티턴 등 큰 값을 다루게 되면 여기서 스터핑을 추가해야 한다.
int Dynamixel::txPacket(uint8_t *packet)
{
	const uint16_t len = packet[PKT_LEN_L] | (packet[PKT_LEN_H] << 8);
	const uint32_t total_len = PKT_INST + static_cast<uint32_t>(len);

	if (total_len > MAX_PACKET_LEN || total_len < 10) {
		return -1;
	}

	packet[PKT_HEADER0]  = DXL_HEADER0;
	packet[PKT_HEADER1]  = DXL_HEADER1;
	packet[PKT_HEADER2]  = DXL_HEADER2;
	packet[PKT_RESERVED] = DXL_RESERVED;

	const uint16_t crc = updateCRC(0, packet, total_len - 2);
	packet[total_len - 2] = crc & 0xFF;
	packet[total_len - 1] = (crc >> 8) & 0xFF;

	const hrt_abstime deadline = hrt_absolute_time() + TRANSACTION_TIMEOUT_MS * 1000;
	const int written = serialWrite(packet, static_cast<int>(total_len), deadline);

	PX4_INFO("TX: written=%d expect=%d errno=%d", written, (int)total_len, errno);   // ★추가

	if (written != static_cast<int>(total_len)) {
		if (written >= 0) {
			++_timeout_count;
		}

		return -1;
	}

	int drain_result = -1;

	do {
		drain_result = tcdrain(_uart_fd);
	} while (drain_result != 0 && errno == EINTR && hrt_absolute_time() < deadline);

	PX4_INFO("TX: drain=%d errno=%d", drain_result, errno);

	if (drain_result != 0) {
		return -1;
	}

	++_tx_packet_count;
	return written;
}

// Status packet(0x55) 수신 및 검증
//
// [변경 요약]
//   기존: 수신 버퍼의 앞 7바이트를 무조건 status 헤더로 간주 → 어긋나면 그대로 실패.
//   변경: (1) 0xFF 0xFF 0xFD 헤더를 바이트 단위로 재동기화하고,
//         (2) 한 패킷을 파싱한 뒤 그것이 우리가 기다리는 Status(0x55)가 아니면
//             (대표적으로 우리가 방금 보낸 패킷의 half-duplex 에코) 버리고 다음 패킷을 찾는다.
//   → 단선(반이중) 결선에서 에코가 섞여 들어와도 진짜 status 패킷을 찾아낸다.
bool Dynamixel::rxStatusPacket(uint8_t expected_id, uint8_t *param_out, uint8_t expected_param_len, int timeout_ms)
{
	if (expected_id > 252 || (expected_param_len > 0 && param_out == nullptr)
	    || expected_param_len > MAX_PACKET_LEN - PKT_INST - 4) {
		return false;
	}

	const hrt_abstime deadline = hrt_absolute_time() + static_cast<hrt_abstime>(timeout_ms) * 1000;

	while (hrt_absolute_time() < deadline) {

		uint8_t buf[MAX_PACKET_LEN] {};

		int matched = 0;

		while (matched < 3) {
			uint8_t b = 0;

			if (serialRead(&b, 1, deadline) != 1) {
				++_timeout_count;
				return false;
			}

			if (matched == 0) {
				matched = (b == DXL_HEADER0) ? 1 : 0;

			} else if (matched == 1) {
				matched = (b == DXL_HEADER1) ? 2 : 0;

			} else {
				// Preserve the overlap in FF FF FF FD so the final
				// FF FF FD sequence is still recognized.
				matched = (b == DXL_HEADER2) ? 3 : (b == DXL_HEADER0) ? 2 : 0;
			}
		}

		buf[PKT_HEADER0] = DXL_HEADER0;
		buf[PKT_HEADER1] = DXL_HEADER1;
		buf[PKT_HEADER2] = DXL_HEADER2;

		if (serialRead(&buf[PKT_RESERVED], 4, deadline) != 4) {
			++_timeout_count;
			return false;
		}

		const uint16_t len = buf[PKT_LEN_L] | (buf[PKT_LEN_H] << 8);

		if (buf[PKT_RESERVED] != DXL_RESERVED || len < 4 || len > MAX_PACKET_LEN - PKT_INST) {
			continue;
		}

		const uint16_t total_len = PKT_INST + len;

		if (serialRead(&buf[PKT_INST], len, deadline) != len) {
			++_timeout_count;
			return false;
		}

		const uint16_t crc_received = buf[total_len - 2] | (buf[total_len - 1] << 8);
		const uint16_t crc_calc = updateCRC(0, buf, total_len - 2);

		if (crc_received != crc_calc) {
			++_rx_crc_error_count;
			continue;
		}

		if (buf[PKT_INST] != INST_STATUS || buf[PKT_ID] != expected_id) {
			continue;
		}

		const uint8_t error = buf[PKT_INST + 1];

		if (error != 0) {
			++_rx_device_error_count;
			PX4_WARN("id=%u returned error=0x%02X",
				 static_cast<unsigned>(expected_id), static_cast<unsigned>(error));
			return false;
		}

		const uint16_t param_len = len - 4;

		if (param_len != expected_param_len) {
			continue;
		}

		if (param_out != nullptr && expected_param_len > 0) {
			memcpy(param_out, &buf[PKT_INST + 2], expected_param_len);
		}

		++_rx_status_count;
		return true;
	}

	++_timeout_count;
	return false;
}

bool Dynamixel::writeRegister(uint8_t id, uint16_t addr, uint32_t value, uint8_t len)
{
	if (!_mutex_initialized) {
		return false;
	}

	BusLock lock(_bus_mutex);

	if (!lock.locked()) {
		return false;
	}

	const bool success = writeRegisterUnlocked(id, addr, value, len);

	if (id != DXL_BROADCAST_ID) {
		setConnected(id, success);
	}

	return success;
}

bool Dynamixel::writeRegisterUnlocked(uint8_t id, uint16_t addr, uint32_t value, uint8_t len)
{
	if (_uart_fd < 0 || (id > 252 && id != DXL_BROADCAST_ID)
	    || (len != 1 && len != 2 && len != 4)) {
		return false;
	}

	uint8_t packet[16] {};
	packet[PKT_ID] = id;

	const uint16_t param_len = 2 + len; // addr(2) + data
	const uint16_t inst_len = 1 + param_len + 2; // instruction + params + crc
	packet[PKT_LEN_L] = inst_len & 0xFF;
	packet[PKT_LEN_H] = (inst_len >> 8) & 0xFF;

	packet[PKT_INST]     = INST_WRITE;
	packet[PKT_INST + 1] = addr & 0xFF;
	packet[PKT_INST + 2] = (addr >> 8) & 0xFF;

	for (uint8_t i = 0; i < len; i++) {
		packet[PKT_INST + 3 + i] = (uint8_t)((value >> (8 * i)) & 0xFF);
	}

	if (!flushInput()) {
		return false;
	}

	if (txPacket(packet) < 0) {
		return false;
	}

	return id == DXL_BROADCAST_ID
	       || rxStatusPacket(id, nullptr, 0, TRANSACTION_TIMEOUT_MS);
}

bool Dynamixel::readRegister(uint8_t id, uint16_t addr, uint8_t len, uint8_t *data_out)
{
	if (!_mutex_initialized) {
		return false;
	}

	BusLock lock(_bus_mutex);

	if (!lock.locked()) {
		return false;
	}

	const bool success = readRegisterUnlocked(id, addr, len, data_out);
	setConnected(id, success);
	return success;
}

bool Dynamixel::readRegisterUnlocked(uint8_t id, uint16_t addr, uint8_t len, uint8_t *data_out)
{
	if (_uart_fd < 0 || id > 252 || len < 1 || len > 4 || data_out == nullptr) {
		return false;
	}

	uint8_t packet[16] {};
	packet[PKT_ID] = id;

	const uint16_t param_len = 4; // addr(2) + read_len(2)
	const uint16_t inst_len = 1 + param_len + 2;
	packet[PKT_LEN_L] = inst_len & 0xFF;
	packet[PKT_LEN_H] = (inst_len >> 8) & 0xFF;

	packet[PKT_INST]     = INST_READ;
	packet[PKT_INST + 1] = addr & 0xFF;
	packet[PKT_INST + 2] = (addr >> 8) & 0xFF;
	packet[PKT_INST + 3] = len & 0xFF;
	packet[PKT_INST + 4] = (len >> 8) & 0xFF;

	if (!flushInput()) {
		return false;
	}

	return txPacket(packet) >= 0
	       && rxStatusPacket(id, data_out, len, TRANSACTION_TIMEOUT_MS);
}

bool Dynamixel::ping(uint8_t id, uint16_t *model_number, uint8_t *firmware_version)
{
	if (!_mutex_initialized) {
		return false;
	}

	BusLock lock(_bus_mutex);

	if (!lock.locked()) {
		return false;
	}

	const bool success = pingUnlocked(id, model_number, firmware_version);
	setConnected(id, success);
	return success;
}

bool Dynamixel::pingUnlocked(uint8_t id, uint16_t *model_number, uint8_t *firmware_version)
{
	if (_uart_fd < 0 || id > 252) {
		return false;
	}

	uint8_t packet[16] {};
	packet[PKT_ID] = id;

	const uint16_t inst_len = 1 + 0 + 2; // instruction + (no params) + crc
	packet[PKT_LEN_L] = inst_len & 0xFF;
	packet[PKT_LEN_H] = (inst_len >> 8) & 0xFF;

	packet[PKT_INST] = INST_PING;

	if (!flushInput()) {
		return false;
	}

	if (txPacket(packet) < 0) {
		return false;
	}

	uint8_t params[3] {};

	if (!rxStatusPacket(id, params, sizeof(params), TRANSACTION_TIMEOUT_MS)) {
		return false;
	}

	if (model_number != nullptr) {
		*model_number = params[0] | (params[1] << 8);
	}

	if (firmware_version != nullptr) {
		*firmware_version = params[2];
	}

	return true;
}

void Dynamixel::setConnected(uint8_t id, bool connected)
{
	for (unsigned i = 0; i < _active_servo_count; ++i) {
		if (_servo_ids[i] == id) {
			if (connected) {
				_connected_mask |= 1u << i;

			} else {
				_connected_mask &= ~(1u << i);
			}

			break;
		}
	}
}

// ============================================================
//  PX4 ModuleBase 보일러플레이트
// ============================================================

int Dynamixel::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("dynamixel",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Dynamixel *Dynamixel::instantiate(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *device_name = nullptr;
	int baudrate = 57600;
	int first_servo_id = 1;
	int active_servo_count = 1;
	int feedback_rate_hz = 20;
	RunMode run_mode = RunMode::Bench;
	WireMode wire_mode = WireMode::SingleWireOpenDrain;

	while ((ch = px4_getopt(argc, argv, "d:b:m:w:i:n:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case 'b': {
				uint32_t parsed = 0;

				if (!parseUnsignedArgument(myoptarg, 1, 4000000, parsed)) {
					print_usage("invalid baudrate");
					return nullptr;
				}

				baudrate = static_cast<int>(parsed);
				break;
			}

		case 'm':
			if (strcmp(myoptarg, "bench") == 0) {
				run_mode = RunMode::Bench;

			} else if (strcmp(myoptarg, "auto") == 0) {
				run_mode = RunMode::Auto;

			} else {
				print_usage("mode must be bench or auto");
				return nullptr;
			}

			break;

		case 'w':
			if (strcmp(myoptarg, "single") == 0) {
				wire_mode = WireMode::SingleWireOpenDrain;

			} else if (strcmp(myoptarg, "single-pushpull") == 0) {
				wire_mode = WireMode::SingleWirePushPull;

			} else if (strcmp(myoptarg, "uart") == 0) {
				wire_mode = WireMode::FullDuplexUart;

			} else {
				print_usage("wire mode must be single, single-pushpull, or uart");
				return nullptr;
			}

			break;

		case 'i': {
				uint32_t parsed = 0;

				if (!parseUnsignedArgument(myoptarg, 0, 252, parsed)) {
					print_usage("first servo ID must be in [0, 252]");
					return nullptr;
				}

				first_servo_id = static_cast<int>(parsed);
				break;
			}

		case 'n': {
				uint32_t parsed = 0;

				if (!parseUnsignedArgument(myoptarg, 1, MAX_SERVOS, parsed)) {
					print_usage("servo count must be in [1, 4]");
					return nullptr;
				}

				active_servo_count = static_cast<int>(parsed);
				break;
			}

		case 'f': {
				uint32_t parsed = 0;

				if (!parseUnsignedArgument(myoptarg, 1, 100, parsed)) {
					print_usage("feedback rate must be in [1, 100] Hz");
					return nullptr;
				}

				feedback_rate_hz = static_cast<int>(parsed);
				break;
			}

		default:
			print_usage("unrecognized flag");
			return nullptr;
		}
	}

	if (device_name == nullptr) {
		print_usage("device (-d) is required");
		return nullptr;
	}

	if (strlen(device_name) >= PORT_NAME_MAX) {
		print_usage("serial device path is too long");
		return nullptr;
	}

	if (first_servo_id + active_servo_count - 1 > 252) {
		print_usage("active servo ID range must end at or below 252");
		return nullptr;
	}

	Dynamixel *instance = new Dynamixel(device_name, baudrate, run_mode, wire_mode,
					    static_cast<uint8_t>(first_servo_id),
					    static_cast<uint8_t>(active_servo_count),
					    static_cast<unsigned>(feedback_rate_hz));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void Dynamixel::run()
{
	if (!init()) {
		PX4_ERR("failed to initialize on %s", _port);
		return;
	}

	if (_run_mode == RunMode::Auto) {
		_next_feedback_time = hrt_absolute_time();
	}

	while (!should_exit()) {
		updateParameters();

		if (_run_mode == RunMode::Bench) {
			px4_usleep(100000);
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();
		servo_command_s cmd{};

		if (_servo_command_sub.update(&cmd)) {
			const hrt_abstime command_now = hrt_absolute_time();
			const bool command_fresh = cmd.timestamp != 0
						   && command_now >= cmd.timestamp
						   && command_now - cmd.timestamp <= COMMAND_TIMEOUT_US;

			if (!command_fresh) {
				PX4_DEBUG("ignored invalid or stale servo_command");
			}

			for (unsigned i = 0; command_fresh && i < _active_servo_count; ++i) {
				const float angle = cmd.servo_command[i];
				uint32_t position = 0;

				if (angleToPosition(i, angle, position)) {
					writeRegister(_servo_ids[i], ADDR_GOAL_POSITION, position, 4);
				}
			}
		}

		if (now >= _next_feedback_time) {
			servo_angle_s angle_msg{};
			angle_msg.valid_mask = 0;

			for (unsigned i = 0; i < MAX_SERVOS; ++i) {
				angle_msg.servo_angle[i] = NAN;
			}

			for (unsigned i = 0; i < _active_servo_count; ++i) {
				uint8_t data[4] {};

				if (readRegister(_servo_ids[i], ADDR_PRESENT_POSITION, 4, data)) {
					const uint32_t raw = data[0]
							     | (static_cast<uint32_t>(data[1]) << 8)
							     | (static_cast<uint32_t>(data[2]) << 16)
							     | (static_cast<uint32_t>(data[3]) << 24);
					const float angle = positionToAngle(i, raw);

					if (PX4_ISFINITE(angle)) {
						angle_msg.servo_angle[i] = angle;
						angle_msg.valid_mask |= 1u << i;
					}
				}
			}

			angle_msg.timestamp = hrt_absolute_time();
			_servo_angle_pub.publish(angle_msg);

			_next_feedback_time = angle_msg.timestamp + _feedback_interval_us;
		}

		px4_usleep(5000);
	}

	{
		BusLock lock(_bus_mutex);

		if (lock.locked()) {
			closeSerial();
		}
	}
}

int Dynamixel::print_status()
{
	BusLock lock(_bus_mutex);

	if (!lock.locked()) {
		PX4_ERR("failed to lock driver state");
		return 1;
	}

	const char *run_mode = (_run_mode == RunMode::Bench) ? "bench" : "auto";
	const char *wire_mode = (_wire_mode == WireMode::FullDuplexUart) ? "uart"
				: (_wire_mode == WireMode::SingleWirePushPull) ? "single-pushpull"
				: "single";
	const unsigned feedback_rate_hz = static_cast<unsigned>(1000000 / _feedback_interval_us);

	PX4_INFO("port=%s baud=%d mode=%s wire=%s servos=%u feedback=%u Hz connected=0x%02x",
		 _port, _baudrate, run_mode, wire_mode,
		 static_cast<unsigned>(_active_servo_count), feedback_rate_hz,
		 static_cast<unsigned>(_connected_mask));
	PX4_INFO("packets: tx=%" PRIu32 " status=%" PRIu32 " timeout=%" PRIu32
		 " crc=%" PRIu32 " device_error=%" PRIu32,
		 _tx_packet_count, _rx_status_count, _timeout_count,
		 _rx_crc_error_count, _rx_device_error_count);

	for (unsigned i = 0; i < _active_servo_count; ++i) {
		PX4_INFO("servo %u id=%u connected=%s zero=%" PRId32 " dir=%" PRId32
			 " min=%" PRId32 " max=%" PRId32,
			 i + 1, static_cast<unsigned>(_servo_ids[i]),
			 (_connected_mask & (1u << i)) ? "yes" : "no",
			 _zero_raw[i], _direction[i], _min_raw[i], _max_raw[i]);
	}

	return 0;
}

int Dynamixel::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("unknown command");
	}

	if (!is_running()) {
		PX4_ERR("dynamixel is not running, start it first");
		return 1;
	}

	Dynamixel *instance = get_instance();

	if (instance == nullptr) {
		PX4_ERR("driver instance is unavailable");
		return 1;
	}

	if (strcmp(argv[0], "write") == 0) {
		if (argc != 5) {
			return print_usage("write requires: <id> <addr> <value> <len>");
		}

		uint32_t id_value = 0;
		uint32_t address_value = 0;
		uint32_t length_value = 0;

		if (!parseUnsignedArgument(argv[1], 0, 252, id_value)
		    || !parseUnsignedArgument(argv[2], 0, 65535, address_value, 0)
		    || !parseUnsignedArgument(argv[4], 1, 4, length_value)
		    || (length_value != 1 && length_value != 2 && length_value != 4)) {
			return print_usage("invalid write ID, address, or length (length must be 1, 2, or 4)");
		}

		const uint32_t maximum_value = length_value == 4 ? UINT32_MAX
					       : (1u << (length_value * 8u)) - 1u;
		uint32_t value = 0;

		if (!parseUnsignedArgument(argv[3], 0, maximum_value, value, 0)) {
			return print_usage("write value does not fit the requested length");
		}

		const uint8_t id = static_cast<uint8_t>(id_value);
		const uint16_t address = static_cast<uint16_t>(address_value);
		const uint8_t length = static_cast<uint8_t>(length_value);
		const bool ok = instance->writeRegister(id, address, value, length);

		PX4_INFO("write id=%u addr=%u value=%" PRIu32 " len=%u -> %s",
			 static_cast<unsigned>(id), static_cast<unsigned>(address), value,
			 static_cast<unsigned>(length), ok ? "OK" : "FAIL");
		return ok ? 0 : 1;
	}

	if (strcmp(argv[0], "read") == 0) {
		if (argc != 4) {
			return print_usage("read requires: <id> <addr> <len>");
		}

		uint32_t id_value = 0;
		uint32_t address_value = 0;
		uint32_t length_value = 0;

		if (!parseUnsignedArgument(argv[1], 0, 252, id_value)
		    || !parseUnsignedArgument(argv[2], 0, 65535, address_value, 0)
		    || !parseUnsignedArgument(argv[3], 1, 4, length_value)) {
			return print_usage("invalid read ID, address, or length (length must be 1..4)");
		}

		const uint8_t id = static_cast<uint8_t>(id_value);
		const uint16_t address = static_cast<uint16_t>(address_value);
		const uint8_t length = static_cast<uint8_t>(length_value);
		uint8_t data[4] {};

		const bool ok = instance->readRegister(id, address, length, data);

		if (ok) {
			uint32_t value = 0;
			memcpy(&value, data, length);
			PX4_INFO("read id=%u addr=%u len=%u -> value=%" PRIu32,
				 static_cast<unsigned>(id), static_cast<unsigned>(address),
				 static_cast<unsigned>(length), value);

		} else {
			PX4_ERR("read id=%u addr=%u failed", static_cast<unsigned>(id), static_cast<unsigned>(address));
		}

		return ok ? 0 : 1;
	}

	if (strcmp(argv[0], "ping") == 0) {
		if (argc != 2) {
			return print_usage("ping requires: <id>");
		}

		uint32_t id_value = 0;

		if (!parseUnsignedArgument(argv[1], 0, 252, id_value)) {
			return print_usage("ping ID must be in [0, 252]");
		}

		const uint8_t id = static_cast<uint8_t>(id_value);
		uint16_t model_number = 0;
		uint8_t firmware_version = 0;
		const bool ok = instance->ping(id, &model_number, &firmware_version);

		if (ok) {
			PX4_INFO("ping id=%u model=%u firmware=%u -> OK",
				 static_cast<unsigned>(id), static_cast<unsigned>(model_number),
				 static_cast<unsigned>(firmware_version));

		} else {
			PX4_ERR("ping id=%u -> FAIL", static_cast<unsigned>(id));
		}

		return ok ? 0 : 1;
	}

	if (strcmp(argv[0], "torque") == 0) {
		if (argc != 3) {
			return print_usage("torque requires: <id> <0|1>");
		}

		uint32_t id_value = 0;
		uint32_t enable_value = 0;

		if (!parseUnsignedArgument(argv[1], 0, 252, id_value)
		    || !parseUnsignedArgument(argv[2], 0, 1, enable_value)) {
			return print_usage("torque requires an ID in [0, 252] and value 0 or 1");
		}

		const uint8_t id = static_cast<uint8_t>(id_value);
		const bool ok = instance->enableTorque(id, enable_value == 1);
		PX4_INFO("torque id=%u value=%u -> %s", static_cast<unsigned>(id),
			 static_cast<unsigned>(enable_value), ok ? "OK" : "FAIL");
		return ok ? 0 : 1;
	}

	if (strcmp(argv[0], "map") == 0) {
		if (argc != 3) {
			return print_usage("map requires: <channel 1..4> <angle_rad>");
		}

		uint32_t channel = 0;

		if (!parseUnsignedArgument(argv[1], 1, MAX_SERVOS, channel)) {
			return print_usage("map channel must be in [1, 4]");
		}

		errno = 0;
		char *end = nullptr;
		const float angle = strtof(argv[2], &end);

		if (errno == ERANGE || end == argv[2] || *end != '\0' || !PX4_ISFINITE(angle)) {
			return print_usage("map angle must be a finite number");
		}

		uint32_t raw = 0;
		float mapped_angle = NAN;
		const unsigned servo_index = static_cast<unsigned>(channel - 1u);
		bool ok = false;
		{
			BusLock lock(instance->_bus_mutex);

			if (lock.locked()) {
				ok = instance->angleToPosition(servo_index, angle, raw);

				if (ok) {
					mapped_angle = instance->positionToAngle(servo_index, raw);
				}
			}
		}

		if (ok) {
			PX4_INFO("channel=%u angle=%.5f rad -> raw=%" PRIu32 " -> %.5f rad",
				 static_cast<unsigned>(channel), (double)angle, raw, (double)mapped_angle);
		}

		return ok ? 0 : 1;
	}

	return print_usage("unknown command");
}

int Dynamixel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dynamixel Protocol 2.0 driver with validated status packets and calibrated
joint-angle conversion. Bench mode is the safe default: it opens the bus and
accepts explicit NSH commands without periodically commanding or polling servos.
Torque is never enabled automatically.

Wire modes:
- single: direct single-wire, open-drain with pull-up when supported
- single-pushpull: direct single-wire push-pull (verify bus contention on a scope)
- uart: normal TX/RX for an external direction-control interface

### Examples
$ dynamixel start -d /dev/ttyS1 -b 57600 -m bench -w single -i 1 -n 1
$ dynamixel ping 1
$ dynamixel read 1 0 2
$ dynamixel read 1 132 4
$ dynamixel map 1 -0.3
$ dynamixel torque 1 0
$ dynamixel stop
$ dynamixel status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 4000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_STRING('m', "bench", "<bench|auto>", "Run mode", true);
	PRINT_MODULE_USAGE_PARAM_STRING('w', "single", "<single|single-pushpull|uart>", "Electrical UART mode", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 0, 252, "First servo ID", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, 4, "Number of consecutive servo IDs", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', 20, 1, 100, "Feedback rate in auto mode", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ping", "ping <id>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write", "write <id> <addr> <value> <len>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "read <id> <addr> <len>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("torque", "torque <id> <0|1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("map", "map <channel 1..4> <angle_rad>");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dynamixel_main(int argc, char *argv[]);

int dynamixel_main(int argc, char *argv[])
{
	return Dynamixel::main(argc, argv);
}
