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
 * @file rtcm_injector_simple.cpp
 * RTCM Serial Passthrough - reads raw data from serial port and forwards to GPS
 * No parsing or validation - GPS device handles frame sync and CRC
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
#include <cerrno>
#include <cstring>

/**
 * RTCM Serial Passthrough Module
 *
 * Reads raw data from serial port and forwards to GPS via uORB.
 * No parsing, validation, or frame synchronization.
 */
class RTCMInjector : public ModuleBase<RTCMInjector>
{
public:
	RTCMInjector();
	~RTCMInjector() override;

	/** @see ModuleBase */
	static RTCMInjector *instantiate(int argc, char *argv[]);

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
		uint32_t bytes_received{0};
		uint32_t packets_sent{0};
		uint64_t start_time{0};
	};

	device::Serial _serial;
	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
	Statistics _stats;
	uint32_t _read_errors{0};
	uint32_t _publish_errors{0};

	bool openSerial();
	bool forwardData(const uint8_t *data, uint16_t len);
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
	// Get parameters
	int32_t port_idx = 1;  // Default TELEM2
	int32_t baudrate = 230400;

	param_t port_param = param_find("RTCM_INJ_PORT");
	param_t baud_param = param_find("RTCM_INJ_BAUD");

	if (port_param != PARAM_INVALID) {
		param_get(port_param, &port_idx);
	}

	if (baud_param != PARAM_INVALID) {
		param_get(baud_param, &baudrate);
	}

	// Map port index to device path (FMU-v6x specific)
	const char *port_map[] = {
		"/dev/ttyS6",  // TELEM1
		"/dev/ttyS4",  // TELEM2
		"/dev/ttyS1",  // TELEM3
		"/dev/ttyS3"   // EXT2
	};

	if (port_idx < 0 || port_idx >= 4) {
		PX4_ERR("Invalid port index: %d", (int)port_idx);
		return false;
	}

	const char *port = port_map[port_idx];

	// Configure serial port
	if (!_serial.setPort(port)) {
		PX4_ERR("Failed to set port: %s", port);
		return false;
	}

	if (!_serial.setBaudrate(baudrate)) {
		PX4_ERR("Failed to set baudrate: %d", (int)baudrate);
		return false;
	}

	// Open serial port
	if (!_serial.open()) {
		PX4_ERR("Failed to open %s", port);
		return false;
	}

	PX4_INFO("Opened %s at %d baud", port, (int)baudrate);
	return true;
}

bool RTCMInjector::forwardData(const uint8_t *data, uint16_t len)
{
	// Split data into chunks if needed (gps_inject_data has limited buffer)
	constexpr size_t chunk_size = sizeof(gps_inject_data_s::data);
	size_t offset = 0;
	bool success = true;

	while (offset < len) {
		gps_inject_data_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.device_id = 0;  // External source

		size_t remaining = len - offset;
		msg.len = (remaining > chunk_size) ? chunk_size : remaining;
		msg.flags = (remaining > chunk_size) ? 1 : 0;  // 1=fragmented

		memcpy(msg.data, data + offset, msg.len);

		if (!_gps_inject_data_pub.publish(msg)) {
			_publish_errors++;
			success = false;
		}

		offset += msg.len;
		_stats.packets_sent++;
	}

	return success;
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

	PX4_INFO("RTCM Passthrough started - forwarding raw data to GPS");

	// Check debug mode
	int32_t debug_mode = 0;
	param_t debug_param = param_find("RTCM_INJ_DEBUG");

	if (debug_param != PARAM_INVALID) {
		param_get(debug_param, &debug_mode);
	}

	uint8_t buffer[1024];
	uint64_t last_stats_time = hrt_absolute_time();

	while (!should_exit()) {
		// Read from serial port with timeout
		int nread = _serial.readAtLeast(buffer, sizeof(buffer), 1, 10000);  // 10ms timeout

		if (nread > 0) {
			// Debug: print raw bytes
			if (debug_mode) {
				PX4_INFO("Received %d bytes, forwarding to GPS", nread);

				// Print in hex format, 16 bytes per line
				for (int i = 0; i < nread && i < 64; i++) {  // Limit to first 64 bytes
					if (i % 16 == 0) {
						printf("\n  %04x: ", i);
					}

					printf("%02x ", buffer[i]);
				}

				if (nread > 64) {
					printf("\n  ... (%d more bytes)", nread - 64);
				}

				printf("\n");
			}

			// Forward raw data to GPS
			if (nread > (int)sizeof(buffer)) {
				// Should never happen, but guard against corruption
				_read_errors++;

			} else {
				forwardData(buffer, (uint16_t)nread);
				_stats.bytes_received += nread;
			}
		}

		// Print statistics every 10 seconds
		if ((hrt_absolute_time() - last_stats_time) > 10000000) {
			uint64_t uptime_s = (hrt_absolute_time() - _stats.start_time) / 1000000;
			float rate = (uptime_s > 0) ? (float)_stats.bytes_received / uptime_s : 0.0f;

			PX4_INFO("Stats: bytes=%u, packets=%u, rate=%.1f B/s, read_err=%u, pub_err=%u",
				 (unsigned)_stats.bytes_received,
				 (unsigned)_stats.packets_sent,
				 (double)rate,
				 (unsigned)_read_errors,
				 (unsigned)_publish_errors);

			last_stats_time = hrt_absolute_time();
		}
	}

	_serial.close();
	PX4_INFO("RTCM Injector stopped");
}

int RTCMInjector::print_status()
{
	PX4_INFO("RTCM Passthrough: Running");
	PX4_INFO("Bytes received: %u", (unsigned)_stats.bytes_received);
	PX4_INFO("Packets sent to GPS: %u", (unsigned)_stats.packets_sent);
	PX4_INFO("Read errors: %u", (unsigned)_read_errors);
	PX4_INFO("Publish errors: %u", (unsigned)_publish_errors);

	uint64_t uptime_s = (hrt_absolute_time() - _stats.start_time) / 1000000;
	PX4_INFO("Uptime: %02llu:%02llu:%02llu",
		 (unsigned long long)(uptime_s / 3600),
		 (unsigned long long)((uptime_s % 3600) / 60),
		 (unsigned long long)(uptime_s % 60));

	if (uptime_s > 0) {
		float rate = (float)_stats.bytes_received / uptime_s;
		PX4_INFO("Data rate: %.1f bytes/s", (double)rate);
	}

	return 0;
}

RTCMInjector *RTCMInjector::instantiate(int argc, char *argv[])
{
	return new RTCMInjector();
}

int RTCMInjector::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rtcm_injector",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_RTCM_INJECTOR,
				      3072,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return PX4_OK;
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
RTCM Serial Passthrough module for external differential radio.

Reads raw RTCM data from a serial port (TELEM1-4) and forwards it to the GPS
device via uORB gps_inject_data topic. No parsing or validation is performed -
the GPS device handles frame synchronization and CRC validation.

### Implementation
The module runs in its own thread and continuously reads from the configured
serial port, forwarding all received data directly to the GPS.

### Examples
Start the module:
$ rtcm_injector start

Stop the module:
$ rtcm_injector stop

Print status:
$ rtcm_injector status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rtcm_injector", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rtcm_injector_main(int argc, char *argv[])
{
	return RTCMInjector::main(argc, argv);
}
