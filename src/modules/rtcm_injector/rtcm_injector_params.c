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
 * RTCM Injector Enable
 *
 * Enable RTCM serial injector module for external differential radio.
 *
 * @boolean
 * @reboot_required true
 * @group RTCM Injector
 */
PARAM_DEFINE_INT32(RTCM_INJ_EN, 0);

/**
 * RTCM Injector Serial Port
 *
 * Serial port for RTCM data input from external differential radio.
 *
 * @min 0
 * @max 3
 * @value 0 TELEM1
 * @value 1 TELEM2
 * @value 2 TELEM3
 * @value 3 TELEM4
 * @reboot_required true
 * @group RTCM Injector
 */
PARAM_DEFINE_INT32(RTCM_INJ_PORT, 1);

/**
 * RTCM Injector Baudrate
 *
 * Baudrate for RTCM serial port.
 *
 * @unit bit/s
 * @min 9600
 * @max 460800
 * @value 9600 9600 baud
 * @value 19200 19200 baud
 * @value 38400 38400 baud
 * @value 57600 57600 baud
 * @value 115200 115200 baud
 * @value 230400 230400 baud
 * @value 460800 460800 baud
 * @reboot_required true
 * @group RTCM Injector
 */
PARAM_DEFINE_INT32(RTCM_INJ_BAUD, 57600);

