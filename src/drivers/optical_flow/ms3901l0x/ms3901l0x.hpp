/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @author Michael Melchior <m.melchior@aviate-it.com>
 *
 * Driver for the Mateksys 3901-L0X optical flow sensor
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#include "ms3901l0x_parser.h"

#define MS3901L0X_DEFAULT_PORT "/dev/ttyS5"

#define SIZE_LINEBUF sizeof(msp_v2_opticalflow_t)

using namespace time_literals;

class MS3901L0X : public px4::ScheduledWorkItem {
public:
	MS3901L0X(const char *port, uint8_t rotation = Rotation::ROTATION_NONE);
	virtual ~MS3901L0X();

	virtual int init();

	void print_info();

private:
	int					_fd									{-1};
	char				_port[20]							{};

	// todo: very hemdsärmeling
	char				_linebuf[SIZE_LINEBUF]	{};
	uint8_t				_linebuf_index 						{0};

	MSP_PARSE_STATE		_parse_state 						{MSP_PARSE_STATE::MSP_PARSE_STATE0_IDLE};
	uint16_t			_msp_cmd							{0};
	uint8_t				_checksum							{0};

	Rotation			_rotation							{Rotation::ROTATION_NONE};

	hrt_abstime			_time_last_read						{0};

	perf_counter_t 		_comms_errors						{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t 		_sample_perf						{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	hrt_abstime			_time_last_counted					{0};
	int					_counter_bytes_max					{0};
	uint16_t			_counter_range						{0};
	uint16_t			_counter_flow						{0};
	uint16_t			_counter_range_max					{0};
	uint16_t			_counter_flow_max					{0};

	// todo: check how, what and why
	static constexpr int kCONVERSIONINTERVAL{9_ms};

	PX4Rangefinder		_px4_rangefinder;

	void				start();
	void				stop();

	void				Run() override;

	int					collect();


	// optical_flow_s		_report;
	// orb_advert_t		_optical_flow_pub;
};
