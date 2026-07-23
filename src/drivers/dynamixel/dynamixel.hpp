#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/servo_command.h>
#include <uORB/topics/servo_angle.h>
#include <cstdint>
#include <cstring>

class Dynamixel : public ModuleBase<Dynamixel>
{
public:
	Dynamixel(const char *port, int baudrate);
	~Dynamixel() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Dynamixel *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	bool init();

	/**
	 * Dynamixel Protocol 2.0 register write (INST_WRITE)
	 * @param id     servo id (1~252), 0xFE = broadcast
	 * @param addr   control table address
	 * @param value  값 (little-endian으로 전송됨)
	 * @param len    바이트 길이 (1, 2, 4)
	 */
	bool writeRegister(uint8_t id, uint16_t addr, uint32_t value, uint8_t len);

	/**
	 * Dynamixel Protocol 2.0 register read (INST_READ)
	 * @param id       servo id
	 * @param addr     control table address
	 * @param len      읽을 바이트 수
	 * @param data_out 결과를 담을 버퍼 (len 바이트 이상)
	 */
	bool readRegister(uint8_t id, uint16_t addr, uint8_t len, uint8_t *data_out);

	/**
	 * Dynamixel Protocol 2.0 ping (INST_PING)
	 * 해당 id 서보가 살아 있는지(응답하는지) 확인한다.
	 * @param id  servo id (broadcast는 지원 안 함)
	 * @return    status packet을 정상 수신하면 true
	 */
	bool ping(uint8_t id);

private:
	// ---------------- Serial (raw POSIX termios) ----------------
	// 통신 방식(half-duplex 전환 등)은 아직 고려하지 않음. 일반 풀듀플렉스 UART로만 open.
	bool openSerial();
	void closeSerial();
	int  serialWrite(const uint8_t *buf, int len);
	int  serialRead(uint8_t *buf, int len, int timeout_ms);

	// ---------------- Dynamixel Protocol 2.0 ----------------
	static constexpr int MAX_PACKET_LEN = 32;

	static constexpr uint8_t DXL_HEADER0 = 0xFF;
	static constexpr uint8_t DXL_HEADER1 = 0xFF;
	static constexpr uint8_t DXL_HEADER2 = 0xFD;
	static constexpr uint8_t DXL_RESERVED = 0x00;

	static constexpr uint8_t DXL_BROADCAST_ID = 0xFE;

	static constexpr uint8_t INST_PING   = 0x01;
	static constexpr uint8_t INST_READ   = 0x02;
	static constexpr uint8_t INST_WRITE  = 0x03;
	static constexpr uint8_t INST_STATUS = 0x55;

	// packet byte offsets (protocol 2.0)
	enum {
		PKT_HEADER0  = 0,
		PKT_HEADER1  = 1,
		PKT_HEADER2  = 2,
		PKT_RESERVED = 3,
		PKT_ID       = 4,
		PKT_LEN_L    = 5,
		PKT_LEN_H    = 6,
		PKT_INST     = 7,
	};

	uint16_t updateCRC(uint16_t crc_accum, const uint8_t *data, uint16_t data_len);
	int  txPacket(uint8_t *packet);
	bool rxStatusPacket(uint8_t expected_id, uint8_t *param_out, uint8_t expected_param_len, int timeout_ms);


	// X-series 공통 control table 주소 (모델 다르면 값 확인 필요)
	static constexpr uint16_t ADDR_TORQUE_ENABLE   = 64;
	static constexpr uint16_t ADDR_GOAL_POSITION   = 116;
	static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

	// 다이나믹셀 서보 ID 1~4로 가정
	// 실제 서보 ID 배선과 다르면 이 배열만 수정하면 됨
	static constexpr uint8_t _servo_ids[4] = {1, 2, 3, 4};

	// servo_command/servo_angle 단위를 라디안 0~360도로 가정.
	// 서보 위치값 0~4095가 물리적으로 0~360도에 대응하므로 이 범위에 그대로 매핑한다.
	// (angleToPosition/positionToAngle는 아래 MIN/MAX 상수만 보고 선형 변환하므로
	//  변환 함수 코드 자체는 수정할 필요가 없다.)
	// 주의: 이제 0 미만/2*pi 초과 각도는 0 또는 2*pi로 clamp된다. 명령을 보내는 쪽에서
	//       0~2*pi(rad) 범위로 각도를 주는지 확인할 것.
	static constexpr float SERVO_ANGLE_MIN_RAD = 0.0f;      //   0deg
	static constexpr float SERVO_ANGLE_MAX_RAD = 6.283185f; // 360deg (2*pi)
	static constexpr uint32_t DXL_POS_MIN = 0;
	static constexpr uint32_t DXL_POS_MAX = 4095;

	bool enableTorque(uint8_t id, bool enable);
	uint32_t angleToPosition(float angle_rad) const;
	float    positionToAngle(uint32_t position) const;

	uORB::Subscription _servo_command_sub{ORB_ID(servo_command)};
	uORB::Publication<servo_angle_s> _servo_angle_pub{ORB_ID(servo_angle)};

	char _port[32] {};
	int  _baudrate {57600};
	int  _uart_fd {-1};
};
