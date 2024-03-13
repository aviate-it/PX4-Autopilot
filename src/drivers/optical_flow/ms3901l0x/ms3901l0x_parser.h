/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#define MSP2_CMD_SENSORS_RANGEFINDER		0x1F01
#define MSP2_CMD_SENSORS_OPTICAL_FLOW		0x1F02

typedef struct {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size_payload;
} __attribute__((packed)) msp_v2_header_t;

// src/main/msp/msp_protocol_v2_sensor_msg.h
typedef struct {
    uint8_t quality;		// [0;255]
    int32_t distance_mm;	// Negative value for out of range
} __attribute__((packed)) msp_v2_rangefinder_t;

// src/main/msp/msp_protocol_v2_sensor_msg.h
typedef struct {
    uint8_t quality;		// [0;255]
    int32_t motion_x;
    int32_t motion_y;
} __attribute__((packed)) msp_v2_opticalflow_t;


enum MSP_PARSE_STATE {
	MSP_PARSE_STATE0_IDLE = 0,
	MSP_PARSE_STATE1_MSPVERSION,
	MSP_PARSE_STATE2_MESSAGE_TYPE,
	MSP_PARSE_STATE3_HEADER,
	MSP_PARSE_STATE4_PAYLOAD,
	MSP_PARSE_STATE5_CHECKSUM
};

bool msp_parse(char c, char *parserbuf, uint8_t *parserbuf_index, enum MSP_PARSE_STATE *state, uint16_t *msp_cmd, uint8_t *checksum);
