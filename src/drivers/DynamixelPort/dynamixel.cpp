#include "dynamixel.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdlib>

Dynamixel::Dynamixel(const char *port, int baudrate) :
	_baudrate(baudrate)
{
	strncpy(_port, port, sizeof(_port) - 1);
}

Dynamixel::~Dynamixel()
{
	closeSerial();
}

bool Dynamixel::init()
{
	if (!openSerial()) {
		return false;
	}

	for (uint8_t id : _servo_ids) {
		if (!enableTorque(id, true)) {
			PX4_WARN("dynamixel: failed to enable torque for id=%u", id);
		}
	}

	return true;
}

bool Dynamixel::enableTorque(uint8_t id, bool enable)
{
	return writeRegister(id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, 1);
}

uint32_t Dynamixel::angleToPosition(float angle_rad) const
{
	float clamped = angle_rad;

	if (clamped < SERVO_ANGLE_MIN_RAD) { clamped = SERVO_ANGLE_MIN_RAD; }
	if (clamped > SERVO_ANGLE_MAX_RAD) { clamped = SERVO_ANGLE_MAX_RAD; }

	const float ratio = (clamped - SERVO_ANGLE_MIN_RAD) / (SERVO_ANGLE_MAX_RAD - SERVO_ANGLE_MIN_RAD); // 0~1

	return DXL_POS_MIN + (uint32_t)(ratio * (DXL_POS_MAX - DXL_POS_MIN));
}

float Dynamixel::positionToAngle(uint32_t position) const
{
	const float ratio = float(position - DXL_POS_MIN) / float(DXL_POS_MAX - DXL_POS_MIN); // 0~1

	return SERVO_ANGLE_MIN_RAD + ratio * (SERVO_ANGLE_MAX_RAD - SERVO_ANGLE_MIN_RAD);
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
	case 19200:   speed = B19200;   break;
	case 57600:   speed = B57600;   break;
	case 115200:  speed = B115200;  break;
	case 230400:  speed = B230400;  break;
	case 1000000: speed = B1000000; break;

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

	tcflush(_uart_fd, TCIOFLUSH);

	PX4_INFO("opened %s @ %d baud", _port, _baudrate);
	return true;
}

void Dynamixel::closeSerial()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

int Dynamixel::serialWrite(const uint8_t *buf, int len)
{
	if (_uart_fd < 0) {
		return -1;
	}

	return ::write(_uart_fd, buf, len);
}

int Dynamixel::serialRead(uint8_t *buf, int len, int timeout_ms)
{
	if (_uart_fd < 0) {
		return -1;
	}

	int total = 0;
	const hrt_abstime start = hrt_absolute_time();

	while (total < len) {
		int n = ::read(_uart_fd, buf + total, len - total);

		if (n > 0) {
			total += n;
			continue;
		}

		if (hrt_elapsed_time(&start) > (hrt_abstime)timeout_ms * 1000) {
			break;
		}

		px4_usleep(500);
	}

	return total;
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
int Dynamixel::txPacket(uint8_t *packet)
{
	const uint16_t len = packet[PKT_LEN_L] | (packet[PKT_LEN_H] << 8);
	const uint16_t total_len = PKT_INST + len;

	packet[PKT_HEADER0]  = DXL_HEADER0;
	packet[PKT_HEADER1]  = DXL_HEADER1;
	packet[PKT_HEADER2]  = DXL_HEADER2;
	packet[PKT_RESERVED] = DXL_RESERVED;

	const uint16_t crc = updateCRC(0, packet, total_len - 2);
	packet[total_len - 2] = crc & 0xFF;
	packet[total_len - 1] = (crc >> 8) & 0xFF;

	return serialWrite(packet, total_len);
}

// Status packet(0x55) 수신 및 검증
bool Dynamixel::rxStatusPacket(uint8_t expected_id, uint8_t *param_out, uint8_t expected_param_len, int timeout_ms)
{
	uint8_t buf[MAX_PACKET_LEN] {};

	// 헤더(3)+예약(1)+ID(1)+LEN_L+LEN_H = 7바이트 먼저 확보
	if (serialRead(buf, PKT_INST, timeout_ms) < PKT_INST) {
		return false;
	}

	if (buf[PKT_HEADER0] != DXL_HEADER0 || buf[PKT_HEADER1] != DXL_HEADER1 ||
	    buf[PKT_HEADER2] != DXL_HEADER2) {
		return false;
	}

	const uint16_t len = buf[PKT_LEN_L] | (buf[PKT_LEN_H] << 8); // INST(1)+ERROR(1)+PARAM+CRC(2)
	const uint16_t total_len = PKT_INST + len;

	if (total_len > MAX_PACKET_LEN) {
		return false;
	}

	if (serialRead(&buf[PKT_INST], len, timeout_ms) < len) {
		return false;
	}

	const uint16_t crc_received = buf[total_len - 2] | (buf[total_len - 1] << 8);
	const uint16_t crc_calc = updateCRC(0, buf, total_len - 2);

	if (crc_received != crc_calc) {
		PX4_WARN("dynamixel: CRC mismatch (id=%u)", expected_id);
		return false;
	}

	if (buf[PKT_INST] != INST_STATUS || buf[PKT_ID] != expected_id) {
		return false;
	}

	const uint8_t error = buf[PKT_INST + 1];

	if (error != 0) {
		PX4_WARN("dynamixel: id=%u returned error=0x%02X", expected_id, error);
	}

	const uint8_t param_len = (len >= 4) ? (len - 4) : 0; // len = inst+error+param+crc

	if (param_out != nullptr && expected_param_len > 0) {
		const uint8_t copy_len = (param_len < expected_param_len) ? param_len : expected_param_len;
		memcpy(param_out, &buf[PKT_INST + 2], copy_len);
	}

	return (error == 0);
}

bool Dynamixel::writeRegister(uint8_t id, uint16_t addr, uint32_t value, uint8_t len)
{
	if (_uart_fd < 0 || len > 4) {
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

	txPacket(packet);

	if (id == DXL_BROADCAST_ID) {
		return true; // broadcast에는 status packet이 오지 않음
	}

	return rxStatusPacket(id, nullptr, 0, 20);
}

bool Dynamixel::readRegister(uint8_t id, uint16_t addr, uint8_t len, uint8_t *data_out)
{
	if (_uart_fd < 0 || id == DXL_BROADCAST_ID) {
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

	txPacket(packet);

	return rxStatusPacket(id, data_out, len, 20);
}

// ============================================================
//  PX4 ModuleBase 보일러플레이트
// ============================================================

int Dynamixel::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("dynamixel",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT,
				       1536,
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

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case 'b':
			baudrate = atoi(myoptarg);
			break;

		default:
			print_usage("unrecognized flag");
			return nullptr;
		}
	}

	if (device_name == nullptr) {
		print_usage("device (-d) is required");
		return nullptr;
	}

	Dynamixel *instance = new Dynamixel(device_name, baudrate);

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

	while (!should_exit()) {

		// ---- PX4 -> Dynamixel : servo_command 받아서 Goal Position에 반영 ----
		servo_command_s cmd;

		if (_servo_command_sub.update(&cmd)) {
			for (int i = 0; i < 4; i++) {
				const float angle = cmd.servo_command[i];

				if (!PX4_ISFINITE(angle)) {
					continue; // disarmed/무효값이면 이 채널은 건드리지 않음
				}

				const uint32_t position = angleToPosition(angle);
				writeRegister(_servo_ids[i], ADDR_GOAL_POSITION, position, 4);
			}
		}

		// ---- Dynamixel -> PX4 : Present Position 읽어서 servo_angle publish ----
		servo_angle_s angle_msg{};
		angle_msg.timestamp = hrt_absolute_time();
		bool any_ok = false;

		for (int i = 0; i < 4; i++) {
			uint8_t data[4] {};

			if (readRegister(_servo_ids[i], ADDR_PRESENT_POSITION, 4, data)) {
				uint32_t raw = 0;
				memcpy(&raw, data, 4);
				angle_msg.servo_angle[i] = positionToAngle(raw);
				any_ok = true;

			} else {
				angle_msg.servo_angle[i] = NAN;
			}
		}

		if (any_ok) {
			_servo_angle_pub.publish(angle_msg);
		}

		px4_usleep(20000); // 약 50Hz
	}

	closeSerial();
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

	if (strcmp(argv[0], "write") == 0) {
		if (argc != 5) {
			return print_usage("write requires: <id> <addr> <value> <len>");
		}

		const uint8_t  id    = (uint8_t)atoi(argv[1]);
		const uint16_t addr  = (uint16_t)atoi(argv[2]);
		const uint32_t value = (uint32_t)strtoul(argv[3], nullptr, 0);
		const uint8_t  len   = (uint8_t)atoi(argv[4]);

		const bool ok = get_instance()->writeRegister(id, addr, value, len);
		PX4_INFO("write id=%u addr=%u value=%u len=%u -> %s", id, addr, value, len, ok ? "OK" : "FAIL");
		return ok ? 0 : 1;
	}

	if (strcmp(argv[0], "read") == 0) {
		if (argc != 4) {
			return print_usage("read requires: <id> <addr> <len>");
		}

		const uint8_t  id   = (uint8_t)atoi(argv[1]);
		const uint16_t addr = (uint16_t)atoi(argv[2]);
		const uint8_t  len  = (uint8_t)atoi(argv[3]);
		uint8_t data[4] {};

		const bool ok = get_instance()->readRegister(id, addr, len, data);

		if (ok) {
			uint32_t value = 0;
			memcpy(&value, data, len > 4 ? 4 : len);
			PX4_INFO("read id=%u addr=%u len=%u -> value=%u", id, addr, len, value);

		} else {
			PX4_ERR("read id=%u addr=%u failed", id, addr);
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
Minimal Dynamixel Protocol 2.0 Read/Write driver.
Half-duplex switching, daisy-chain scheduling, actuator/uORB integration
등 통신 방식은 아직 구현되지 않았고, ArduPilot의 AP_RobotisServo와
ROBOTIS DynamixelSDK protocol2_packet_handler를 참고한 raw packet 단위
Read/Write만 지원한다.

### Examples
$ dynamixel start -d /dev/ttyS3 -b 57600
$ dynamixel write 1 116 1024 4
$ dynamixel read 1 132 4
$ dynamixel stop
$ dynamixel status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("write", "write <id> <addr> <value> <len>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "read <id> <addr> <len>");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dynamixel_main(int argc, char *argv[]);

int dynamixel_main(int argc, char *argv[])
{
	return Dynamixel::main(argc, argv);
}
