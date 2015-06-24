/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/board_serial.h>
#include <systemlib/scheduling_priorities.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <uORB/topics/esc_status.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "uavcan_main.hpp"

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * UavcanNode
 */
 
static constexpr unsigned MemPoolSize        = 10752; ///< Refer to the libuavcan manual to learn why
static constexpr unsigned RxQueueLenPerIface = 64;
typedef uavcan::Node<MemPoolSize> Node;
typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{

	if (argc < 2)
	{
		printf("Usage: %s <node-id>\n", argv[0]);
		return 1;
	}
	int self_node_id = atoi(argv[1]);

	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);

	static CanInitHelper can;
	static bool can_initialized = false;

	int32_t bitrate = 0;
	(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);
	if (!can_initialized) {
		const int can_init_res = can.init(bitrate);

		if (can_init_res < 0) {
			warnx("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}

		can_initialized = true;
	}

	Node *node = new Node(can.driver, uavcan_stm32::SystemClock::instance());
	node->setNodeID(self_node_id);
	node->setName("org.uavcan.tutorials");

	while (true)
	{
		const int res = node->start();
		if (res < 0)
		{
			printf("Node start failed: %d, will retry\n", res);
			sleep(1);
		}
		else { break; }
	}

	printf("Hello Sky!\n");
	return OK;
}
