#pragma once

#include "DynamixelPositionMapping.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/servo_command.h>
#include <uORB/topics/servo_angle.h>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <pthread.h>

class Dynamixel : public ModuleBase<Dynamixel>, public ModuleParams
{
public:
	enum class RunMode : uint8_t {
		Bench = 0,
		Auto
	};

	enum class WireMode : uint8_t {
		SingleWireOpenDrain = 0,
		SingleWirePushPull,
		FullDuplexUart
	};

	// ---- 콘솔 명령 위임 ----
	// NuttX는 fd가 태스크별로 관리되어, 콘솔 스레드가 run()의 _uart_fd를 쓰면 EBADF가 난다.
	// 따라서 콘솔은 요청만 남기고 실제 통신은 run() 태스크가 수행한다.
	enum class CmdType : uint8_t {
		None = 0,
		Ping,
		Read,
		Write,
		Torque
	};

	enum class CmdState : uint8_t {
		Idle = 0,
		Requested,
		Done
	};

	struct PendingCommand {
		CmdType  type{CmdType::None};
		CmdState state{CmdState::Idle};
		uint8_t  id{0};
		uint16_t addr{0};
		uint32_t value{0};
		uint8_t  len{0};
		bool     success{false};
		uint32_t result{0};        // read 결과값
		uint16_t model_number{0};  // ping 결과
		uint8_t  firmware{0};      // ping 결과
	};

	Dynamixel(const char *port, int baudrate, RunMode run_mode, WireMode wire_mode,
		  uint8_t first_servo_id, uint8_t active_servo_count, unsigned feedback_rate_hz);
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

	/** @see ModuleBase::print_status() */
	int print_status() override;

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
	bool ping(uint8_t id, uint16_t *model_number = nullptr, uint8_t *firmware_version = nullptr);

private:
	// ---------------- Serial (raw POSIX termios) ----------------
	bool openSerial();
	void closeSerial();
	int serialWrite(const uint8_t *buf, int len, hrt_abstime deadline);
	int serialRead(uint8_t *buf, int len, hrt_abstime deadline);
	bool flushInput();

	// ---------------- Dynamixel Protocol 2.0 ----------------
	static constexpr int MAX_PACKET_LEN = 64;
	static constexpr int TRANSACTION_TIMEOUT_MS = 20;

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
	bool writeRegisterUnlocked(uint8_t id, uint16_t addr, uint32_t value, uint8_t len);
	bool readRegisterUnlocked(uint8_t id, uint16_t addr, uint8_t len, uint8_t *data_out);
	bool pingUnlocked(uint8_t id, uint16_t *model_number, uint8_t *firmware_version);
	void setConnected(uint8_t id, bool connected);

	// X-series 공통 control table 주소 (모델 다르면 값 확인 필요)
	static constexpr uint16_t ADDR_TORQUE_ENABLE   = 64;
	static constexpr uint16_t ADDR_GOAL_POSITION   = 116;
	static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

	static constexpr unsigned MAX_SERVOS = 4;
	static constexpr size_t PORT_NAME_MAX = 32;
	static constexpr uint32_t DXL_POS_MIN = DynamixelPositionMapping::PositionMinimum;
	static constexpr uint32_t DXL_POS_MAX = DynamixelPositionMapping::PositionMaximum;
	static constexpr hrt_abstime COMMAND_TIMEOUT_US = 100000;

	bool enableTorque(uint8_t id, bool enable);
	bool angleToPosition(unsigned servo_index, float angle_rad, uint32_t &position) const;
	float positionToAngle(unsigned servo_index, uint32_t position) const;
	void updateCalibration();
	void updateParameters();

	uORB::Subscription _servo_command_sub{ORB_ID(servo_command)};
	uORB::Publication<servo_angle_s> _servo_angle_pub{ORB_ID(servo_angle)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	pthread_mutex_t _bus_mutex {};
	bool _mutex_initialized{false};

	char _port[PORT_NAME_MAX] {};
	int  _baudrate {57600};
	int  _uart_fd {-1};

	RunMode _run_mode{RunMode::Bench};
	WireMode _wire_mode{WireMode::SingleWireOpenDrain};
	uint8_t _servo_ids[MAX_SERVOS] {1, 2, 3, 4};
	uint8_t _active_servo_count{1};
	hrt_abstime _feedback_interval_us{50000};
	hrt_abstime _next_feedback_time{0};

	int32_t _zero_raw[MAX_SERVOS] {2048, 2048, 2048, 2048};
	int32_t _direction[MAX_SERVOS] {1, 1, 1, 1};
	int32_t _min_raw[MAX_SERVOS] {1592, 1592, 1592, 1592};
	int32_t _max_raw[MAX_SERVOS] {2504, 2504, 2504, 2504};

	uint32_t _tx_packet_count{0};
	uint32_t _rx_status_count{0};
	uint32_t _timeout_count{0};
	uint32_t _rx_crc_error_count{0};
	uint32_t _rx_device_error_count{0};
	uint8_t _connected_mask{0};

	PendingCommand _pending {};
	pthread_mutex_t _cmd_mutex {};
	bool _cmd_mutex_initialized{false};

	void servicePendingCommand();                    // run()이 호출: 요청 처리
	static bool submitCommand(PendingCommand &cmd);  // 콘솔이 호출: 요청 제출 + 결과 대기

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DXL_S1_ZERO>) _param_dxl_s1_zero,
		(ParamInt<px4::params::DXL_S1_DIR>) _param_dxl_s1_dir,
		(ParamInt<px4::params::DXL_S1_MIN>) _param_dxl_s1_min,
		(ParamInt<px4::params::DXL_S1_MAX>) _param_dxl_s1_max,
		(ParamInt<px4::params::DXL_S2_ZERO>) _param_dxl_s2_zero,
		(ParamInt<px4::params::DXL_S2_DIR>) _param_dxl_s2_dir,
		(ParamInt<px4::params::DXL_S2_MIN>) _param_dxl_s2_min,
		(ParamInt<px4::params::DXL_S2_MAX>) _param_dxl_s2_max,
		(ParamInt<px4::params::DXL_S3_ZERO>) _param_dxl_s3_zero,
		(ParamInt<px4::params::DXL_S3_DIR>) _param_dxl_s3_dir,
		(ParamInt<px4::params::DXL_S3_MIN>) _param_dxl_s3_min,
		(ParamInt<px4::params::DXL_S3_MAX>) _param_dxl_s3_max,
		(ParamInt<px4::params::DXL_S4_ZERO>) _param_dxl_s4_zero,
		(ParamInt<px4::params::DXL_S4_DIR>) _param_dxl_s4_dir,
		(ParamInt<px4::params::DXL_S4_MIN>) _param_dxl_s4_min,
		(ParamInt<px4::params::DXL_S4_MAX>) _param_dxl_s4_max
	)
};
