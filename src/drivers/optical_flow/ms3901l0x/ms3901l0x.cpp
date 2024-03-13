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

#include "ms3901l0x.hpp"

#include <lib/drivers/device/Device.hpp>

MS3901L0X::MS3901L0X(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation) {

	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	// ref. https://www.mateksys.com/?portfolio=3901-l0x#tab-id-2
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L0X);
	_px4_rangefinder.set_min_distance(0.08f);
	_px4_rangefinder.set_max_distance(2.f);
	_px4_rangefinder.set_fov(math::radians(25.f));
}

MS3901L0X::~MS3901L0X()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int MS3901L0X::init() {
	int ret = PX4_OK;

	do {
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return PX4_ERROR;
		}

		// ref https://www.mateksys.com/?portfolio=3901-l0x#tab-id-3
		unsigned speed = B115200;

		struct termios uart_config;
		int termios_state;

		tcgetattr(_fd, &uart_config);
		uart_config.c_oflag &= ~ONLCR;

		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = PX4_ERROR;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;
		uart_config.c_cflag &= ~PARENB;
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~CRTSCTS;

		/* setup for non-canonical mode */
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		/* fetch bytes as they become available */
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: MS3901L0X fd");
			ret = PX4_ERROR;
			break;
		}

	} while (0);

	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int MS3901L0X::collect() {
	perf_begin(_sample_perf);

	int64_t read_elapsed = hrt_elapsed_time(&_time_last_read);

	char readbuf[SIZE_LINEBUF];
	const unsigned readlen = SIZE_LINEBUF - 1;

	int result_read = 0;

	// todo: why int and then unsigned long?!?
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	bool package_complete = false;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		result_read = ::read(_fd, &readbuf[0], readlen);

		if (result_read < 0) {
			PX4_ERR("read err: %d", result_read);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				tcflush(_fd, TCIFLUSH);
				return result_read;

			} else {
				return -EAGAIN;
			}
		}

		if (result_read > _counter_bytes_max) {
			_counter_bytes_max = result_read;
		}

		_time_last_read = hrt_absolute_time();

		for (int i = 0; i < result_read; i++) {
			package_complete |= msp_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &_msp_cmd, &_checksum);
		}

		bytes_available -= result_read;

	} while (bytes_available > 0);

	if (!package_complete) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	if (_msp_cmd == MSP2_CMD_SENSORS_RANGEFINDER) {
		_counter_range++;
		msp_v2_rangefinder_t *tmp_msp_v2_rangefinder = (msp_v2_rangefinder_t*)&_linebuf;
		float distance_m = tmp_msp_v2_rangefinder->distance_mm / 1000.f;
		_px4_rangefinder.update(timestamp_sample, distance_m);
	}

	if (hrt_elapsed_time(&_time_last_counted) > 1_s) {
		_counter_range_max = _counter_range;
		_counter_flow_max = _counter_flow;

		_counter_range = 0;
		_counter_flow = 0;

		_time_last_counted = hrt_absolute_time();
	}

	perf_end(_sample_perf);

	return PX4_OK;
}

void MS3901L0X::start() {
	ScheduleOnInterval(7_ms);
}

void MS3901L0X::stop() {
	ScheduleClear();
}

void MS3901L0X::Run() {
	if (_fd < 0) {
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);
	}

	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(7_ms, 87 * 9);
		return;
	}
}

void MS3901L0X::print_info() {
	PX4_INFO("Using port '%s'", _port);
	PX4_INFO("Max Bytes Read (resetting):'%i'", _counter_bytes_max);
	PX4_INFO("Range Frames per second: '%i'", _counter_range_max);
	PX4_INFO("Flow Frames per second: '%i'", _counter_flow_max);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_counter_bytes_max = 0;
}
