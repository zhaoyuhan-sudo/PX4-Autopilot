/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rtcm_injector_main.cpp
 * RTCM Serial Injector - reads RTCM3 data from serial port and forwards to GPS
 *
 * @author PX4 Development Team
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/Serial.hpp>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/gps_inject_data.h>

#include <cstring>

static constexpr uint8_t RTCM3_PREAMBLE = 0xD3;
static constexpr uint16_t RTCM3_MAX_PAYLOAD_LEN = 1023;
static constexpr uint16_t RTCM3_MAX_FRAME_LEN = 1029; // 1 + 2 + 1023 + 3

/**
 * Calculate RTCM3 CRC-24Q
 * Polynomial: 0x1864CFB
 * Computed over length bytes + payload bytes (not including preamble or CRC)
 */
static uint32_t crc24q(const uint8_t *data, uint16_t len)
{
	constexpr uint32_t poly = 0x1864CFB;
	uint32_t crc = 0;

	for (uint16_t i = 0; i < len; i++) {
		crc ^= ((uint32_t)data[i]) << 16;

		for (int j = 0; j < 8; j++) {
			crc <<= 1;

			if (crc & 0x1000000) {
				crc ^= poly;
			}
		}
	}

	return crc & 0xFFFFFF;
}

/**
 * RTCM3 Frame Parser
 *
 * State machine for parsing RTCM3 frames with CRC-24Q validation.
 * Supports frames up to 1023 bytes payload.
 */
class RTCMParser
{
public:
	enum class ParseState {
		SearchPreamble,
		GetLength,
		GetPayload,
		GetCRC
	};

	struct Statistics {
		uint32_t frames_received{0};
		uint32_t bytes_received{0};
		uint32_t crc_errors{0};
		uint32_t buffer_overflows{0};
	};

	RTCMParser()
	{
		reset();
	}

	/**
	 * Add byte to parser
	 * @param byte Input byte
	 * @return 0=continue parsing, 1=frame complete and valid, -1=CRC error
	 */
	int addByte(uint8_t byte)
	{
		switch (_state) {
		case ParseState::SearchPreamble:
			if (byte == RTCM3_PREAMBLE) {
				_buffer[0] = byte;
				_buffer_pos = 1;
				_state = ParseState::GetLength;
			}

			break;

		case ParseState::GetLength:
			_buffer[_buffer_pos++] = byte;

			if (_buffer_pos == 3) {
				// Extract 10-bit length from bytes 1-2
				// Byte 1: 6 reserved bits + 2 MSB of length
				// Byte 2: 8 LSB of length
				_payload_len = (((uint16_t)_buffer[1] & 0x03) << 8) | _buffer[2];

				if (_payload_len > RTCM3_MAX_PAYLOAD_LEN) {
					// Invalid length, reset and continue searching
					_stats.buffer_overflows++;
					reset();
					return 0;
				}

				if (_payload_len == 0) {
					// Empty payload, go directly to CRC
					_state = ParseState::GetCRC;
					_crc_pos = 0;

				} else {
					_state = ParseState::GetPayload;
				}
			}

			break;

		case ParseState::GetPayload:
			_buffer[_buffer_pos++] = byte;

			if (_buffer_pos == 3 + _payload_len) {
				// Payload complete, now get CRC
				_state = ParseState::GetCRC;
				_crc_pos = 0;
			}

			break;

		case ParseState::GetCRC:
			_buffer[_buffer_pos++] = byte;
			_crc_pos++;

			if (_crc_pos == 3) {
				// Frame complete, verify CRC
				// CRC is computed over length bytes (2) + payload bytes
				uint32_t crc_calc = crc24q(_buffer + 1, 2 + _payload_len);
				uint32_t crc_recv = ((uint32_t)_buffer[_buffer_pos - 3] << 16) |
						    ((uint32_t)_buffer[_buffer_pos - 2] << 8) |
						    _buffer[_buffer_pos - 1];

				if (crc_calc == crc_recv) {
					_stats.frames_received++;
					_stats.bytes_received += _buffer_pos;
					return 1; // Frame complete and valid

				} else {
					_stats.crc_errors++;
					reset();
					return -1; // CRC error
				}
			}

			break;
		}

		return 0; // Continue parsing
	}

	/**
	 * Get pointer to complete frame buffer
	 */
	const uint8_t *getFrame() const
	{
		return _buffer;
	}

	/**
	 * Get length of complete frame
	 */
	uint16_t getFrameLength() const
	{
		return _buffer_pos;
	}

	/**
	 * Get parser statistics
	 */
	const Statistics &getStatistics() const
	{
		return _stats;
	}

	/**
	 * Reset parser to initial state
	 */
	void reset()
	{
		_state = ParseState::SearchPreamble;
		_buffer_pos = 0;
		_payload_len = 0;
		_crc_pos = 0;
	}

private:
	ParseState _state{ParseState::SearchPreamble};
	uint8_t _buffer[RTCM3_MAX_FRAME_LEN];
	uint16_t _buffer_pos{0};
	uint16_t _payload_len{0};
	uint8_t _crc_pos{0};
	Statistics _stats{};
};

/**
 * RTCM Serial Injector Module
 *
 * Reads RTCM3 data from serial port and forwards to GPS via uORB.
 */
class RTCMInjector : public ModuleBase<RTCMInjector>
{
public:
	RTCMInjector();
	~RTCMInjector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	struct Statistics {
		uint32_t uorb_publish_count{0};
		uint64_t start_time{0};
	};

	device::Serial _serial;
	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
	RTCMParser _parser;
	Statistics _stats;

	bool openSerial();
	void publishRTCMFrame(const uint8_t *frame, uint16_t len);
};

RTCMInjector::RTCMInjector()
{
	_stats.start_time = hrt_absolute_time();
}

RTCMInjector::~RTCMInjector()
{
	_serial.close();
}

bool RTCMInjector::openSerial()
{
	// Map parameter index to device path
	const char *port_map[] = {
		"/dev/ttyS1",  // TELEM1
		"/dev/ttyS2",  // TELEM2
		"/dev/ttyS3",  // TELEM3
		"/dev/ttyS4"   // TELEM4
	};

	// Get port index from parameter
	int32_t port_idx = 1;  // Default: TELEM2
	param_t port_param = param_find("RTCM_INJ_PORT");

	if (port_param != PARAM_INVALID) {
		param_get(port_param, &port_idx);
	}

	if (port_idx < 0 || port_idx > 3) {
		PX4_ERR("Invalid port index: %d", (int)port_idx);
		return false;
	}

	// Get baudrate from parameter
	int32_t baudrate = 57600;  // Default
	param_t baud_param = param_find("RTCM_INJ_BAUD");

	if (baud_param != PARAM_INVALID) {
		param_get(baud_param, &baudrate);
	}

	// Configure serial port
	if (!_serial.setPort(port_map[port_idx])) {
		PX4_ERR("Failed to set port: %s", port_map[port_idx]);
		return false;
	}

	if (!_serial.setBaudrate(baudrate)) {
		PX4_ERR("Failed to set baudrate: %d", (int)baudrate);
		return false;
	}

	// Open serial port
	if (!_serial.open()) {
		PX4_ERR("Failed to open serial port: %s", port_map[port_idx]);
		return false;
	}

	PX4_INFO("Opened %s at %d baud", port_map[port_idx], (int)baudrate);
	return true;
}

void RTCMInjector::publishRTCMFrame(const uint8_t *frame, uint16_t len)
{
	gps_inject_data_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.device_id = 0;  // External radio

	size_t offset = 0;

	// Fragment large frames into multiple uORB messages
	while (offset < len) {
		size_t chunk_len = len - offset;

		if (chunk_len > sizeof(msg.data)) {
			chunk_len = sizeof(msg.data);
		}

		msg.len = chunk_len;
		msg.flags = (offset + chunk_len < len) ? 1 : 0;  // Fragmentation flag
		memcpy(msg.data, frame + offset, chunk_len);

		_gps_inject_data_pub.publish(msg);
		_stats.uorb_publish_count++;

		offset += chunk_len;
	}
}

void RTCMInjector::run()
{
	// Check if module is enabled
	int32_t enabled = 0;
	param_t enable_param = param_find("RTCM_INJ_EN");

	if (enable_param != PARAM_INVALID) {
		param_get(enable_param, &enabled);
	}

	if (enabled == 0) {
		PX4_INFO("RTCM Injector disabled (RTCM_INJ_EN=0)");
		return;
	}

	// Open serial port
	if (!openSerial()) {
		PX4_ERR("Failed to open serial port, exiting");
		return;
	}

	PX4_INFO("RTCM Injector started");

	uint8_t buffer[256];
	uint64_t last_stats_time = hrt_absolute_time();

	while (!should_exit()) {
		// Read from serial port with timeout
		int nread = _serial.readAtLeast(buffer, sizeof(buffer), 1, 100000);  // 100ms timeout

		if (nread > 0) {
			// Feed bytes to parser
			for (int i = 0; i < nread; i++) {
				int ret = _parser.addByte(buffer[i]);

				if (ret == 1) {
					// Frame complete and valid
					publishRTCMFrame(_parser.getFrame(), _parser.getFrameLength());
					_parser.reset();

				} else if (ret == -1) {
					// CRC error - parser already incremented error counter
					PX4_WARN("RTCM CRC error");
				}
			}
		}

		// Print statistics every 10 seconds
		uint64_t now = hrt_absolute_time();

		if (now - last_stats_time > 10000000) {  // 10 seconds
			const RTCMParser::Statistics &parser_stats = _parser.getStatistics();
			PX4_INFO("Stats: frames=%u, bytes=%u, crc_errors=%u, uorb_pubs=%u",
				 (unsigned)parser_stats.frames_received,
				 (unsigned)parser_stats.bytes_received,
				 (unsigned)parser_stats.crc_errors,
				 (unsigned)_stats.uorb_publish_count);
			last_stats_time = now;
		}

		// Sleep briefly to avoid busy-waiting
		px4_usleep(10000);  // 10ms
	}

	_serial.close();
	PX4_INFO("RTCM Injector stopped");
}

int RTCMInjector::print_status()
{
	PX4_INFO("RTCM Injector: Running");

	const RTCMParser::Statistics &parser_stats = _parser.getStatistics();
	PX4_INFO("Frames received: %u", (unsigned)parser_stats.frames_received);
	PX4_INFO("Bytes received: %u", (unsigned)parser_stats.bytes_received);
	PX4_INFO("CRC errors: %u", (unsigned)parser_stats.crc_errors);
	PX4_INFO("Buffer overflows: %u", (unsigned)parser_stats.buffer_overflows);
	PX4_INFO("uORB publishes: %u", (unsigned)_stats.uorb_publish_count);

	uint64_t uptime_s = (hrt_absolute_time() - _stats.start_time) / 1000000;
	PX4_INFO("Uptime: %02llu:%02llu:%02llu",
		 (unsigned long long)(uptime_s / 3600),
		 (unsigned long long)((uptime_s % 3600) / 60),
		 (unsigned long long)(uptime_s % 60));

	if (uptime_s > 0) {
		float rate = (float)parser_stats.bytes_received / uptime_s;
		PX4_INFO("Injection rate: %.1f bytes/s", (double)rate);
	}

	return 0;
}

int RTCMInjector::task_spawn(int argc, char *argv[])
{
	RTCMInjector *instance = new RTCMInjector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RTCMInjector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RTCMInjector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RTCM Serial Injector reads RTCM3 differential data from a serial port
(typically TELEM2) and forwards it to the GPS module for RTK positioning.

The module transparently forwards RTCM3 frames without parsing message content.
It performs frame boundary detection and CRC-24Q validation.

### Implementation
- Reads from configurable serial port (TELEM1-4)
- Parses RTCM3 frame structure (preamble, length, payload, CRC24)
- Publishes to gps_inject_data uORB topic with fragmentation support
- Handles frames up to 1023 bytes payload

### Examples
Start the module:
$ rtcm_injector start

Stop the module:
$ rtcm_injector stop

Query status:
$ rtcm_injector status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rtcm_injector", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rtcm_injector_main(int argc, char *argv[])
{
	return RTCMInjector::main(argc, argv);
}
