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

/**
 * @author Michael Melchior <m.melchior@aviate-it.com>
 *
 * Declarations of parser for the Mateksys 3901-L0X optical flow sensor
 */

#include "ms3901l0x_parser.h"

//#define MS3901L0X_DEBUG

#ifdef MS3901L0X_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_MSP_IDLE",			// waiting for $					MSP_IDLE
	"1_MSP_VERSION",		// waiting for X					MSP_HEADER_START
	"2_MSP_TYPE",			// waiting for <					MSP_HEADER_X
	"3_MSP_HEADER",			// waiting for header (flags, function, payload size)	MSP_HEADER_V2_NATIVE
	"4_MSP_PAYLOAD",		// waiting for payload					MSP_PAYLOAD_V2_NATIVE
	"5_MSP_CHECKSUM"		// waiting for checksum					MSP_CHECKSUM_V2_NATIVE
	"6_MSP_COMMAND_RECEIVED"	// process command					MSP_COMMAND_RECEIVED
};
#endif

uint8_t msp_crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed)
{
    crc ^= a;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ seed;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    return msp_crc8_dvb(crc, a, 0xD5);
}

bool msp_parse(char c, char *parserbuf, uint8_t *parserbuf_index, enum MSP_PARSE_STATE *state, uint16_t *msp_cmd, uint8_t *checksum) {
	bool packet_complete = false;

	switch (*state) {
	case MSP_PARSE_STATE0_IDLE:
		packet_complete = false;

		if (c == '$') {
			*state = MSP_PARSE_STATE1_MSPVERSION;
		}
		break;

	case MSP_PARSE_STATE1_MSPVERSION:
		if (c == 'X') {
			*state = MSP_PARSE_STATE2_MESSAGE_TYPE;
		} else {
			*state = MSP_PARSE_STATE0_IDLE;
		}
		break;

	case MSP_PARSE_STATE2_MESSAGE_TYPE:
		if (c == '<') {
			(*parserbuf_index) = 0;
			*checksum = 0;
			*state = MSP_PARSE_STATE3_HEADER;
		} else {
			*state = MSP_PARSE_STATE0_IDLE;
		}
		break;

	case MSP_PARSE_STATE3_HEADER:
		parserbuf[*parserbuf_index] = c;
		*checksum = msp_crc8_dvb_s2(*checksum, c);
		(*parserbuf_index)++;

		if ((*parserbuf_index) == sizeof(msp_v2_header_t)) {
			msp_v2_header_t *tmp_msp_v2_header = (msp_v2_header_t*)parserbuf;
			if (tmp_msp_v2_header->cmd == MSP2_CMD_SENSORS_RANGEFINDER) {
				if (tmp_msp_v2_header->size_payload == sizeof(msp_v2_rangefinder_t)) {
					(*parserbuf_index) = 0;
					*msp_cmd = MSP2_CMD_SENSORS_RANGEFINDER;

					*state = MSP_PARSE_STATE4_PAYLOAD;
				} else {
					*state = MSP_PARSE_STATE0_IDLE;
				}
			} else if (tmp_msp_v2_header->cmd == MSP2_CMD_SENSORS_OPTICAL_FLOW) {
				if (tmp_msp_v2_header->size_payload == sizeof(msp_v2_opticalflow_t)) {
					(*parserbuf_index) = 0;
					*msp_cmd = MSP2_CMD_SENSORS_OPTICAL_FLOW;

					*state = MSP_PARSE_STATE4_PAYLOAD;
				} else {
					*state = MSP_PARSE_STATE0_IDLE;
				}
			} else {
				*state = MSP_PARSE_STATE0_IDLE;
			}
		}
		break;

	case MSP_PARSE_STATE4_PAYLOAD:
		parserbuf[*parserbuf_index] = c;
		*checksum = msp_crc8_dvb_s2(*checksum, c);
		(*parserbuf_index)++;

		if (*msp_cmd == MSP2_CMD_SENSORS_RANGEFINDER) {
			if (*parserbuf_index == sizeof(msp_v2_rangefinder_t)) {
				*state = MSP_PARSE_STATE5_CHECKSUM;
			}
		} else if (*msp_cmd == MSP2_CMD_SENSORS_OPTICAL_FLOW) {
			if (*parserbuf_index == sizeof(msp_v2_opticalflow_t)) {
				*state = MSP_PARSE_STATE5_CHECKSUM;
			}
		} else {
			*state = MSP_PARSE_STATE0_IDLE;
		}
		break;

	case MSP_PARSE_STATE5_CHECKSUM:
		if (c == *checksum) {
			packet_complete = true;
		}
		*state = MSP_PARSE_STATE0_IDLE;
		break;
	}

#ifdef MS3901L0X_DEBUG
	printf("state: MSP_PARSE_STATE%s, got char: %#02x\n", parser_state[*state], c);
#endif

	return packet_complete;
}
