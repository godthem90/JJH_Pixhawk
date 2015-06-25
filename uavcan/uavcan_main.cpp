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
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/helpers/ostream.hpp>

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
static CanInitHelper can;

class MsgController
{
public:
	MsgController(uavcan::INode& node);
	~MsgController();

	int init();
	void msg_broadcast(uavcan::protocol::debug::KeyValue msg);

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void msg_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg);

	typedef uavcan::MethodBinder<MsgController*,
		void (MsgController::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&)>
		MsgCbBinder;

/*	typedef uavcan::MethodBinder<UavcanEscController*, void (UavcanEscController::*)(const uavcan::TimerEvent&)>
		TimerCbBinder;*/

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	uavcan::Subscriber<uavcan::protocol::debug::KeyValue, MsgCbBinder>	_uavcan_sub_msg;
	uavcan::Publisher<uavcan::protocol::debug::KeyValue> _uavcan_pub_msg;
	//uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

};

MsgController::MsgController(uavcan::INode &node) :
	_node(node),
	_uavcan_sub_msg(node),
	_uavcan_pub_msg(node)
{
}

int MsgController::init()
{
    int res = _uavcan_pub_msg.init();
    if (res < 0)
    {
		printf("publisher failed\n");
        exit(1);                   // TODO proper error handling
    }
	_uavcan_pub_msg.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

	// ESC status subscription
	res = _uavcan_sub_msg.start(MsgCbBinder(this, &MsgController::msg_sub_cb));
	if (res < 0)
	{
		warnx("subscriber failed %i\n", res);
		exit(1);
	}

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	//_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::orb_pub_timer_cb));
	//_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

	return res;
}

void MsgController::msg_broadcast(uavcan::protocol::debug::KeyValue msg)
{
	int res = _uavcan_pub_msg.broadcast(msg);
	if (res < 0)
	{
		printf("KV publication failure: %d\n", res);
	}
}

void MsgController::msg_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg)
{
	uavcan::OStream::instance() << msg << uavcan::OStream::endl;
}


extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{

	if (argc < 2)
	{
		printf("Usage: %s <node-id>\n", argv[0]);
		return 1;
	}

	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);

	int32_t bitrate = 0;
	(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);
	int can_init_res = can.init(bitrate);

	if (can_init_res < 0) {
		warnx("CAN driver init failed");
		return can_init_res;	
	}

	Node *node = new Node(can.driver, uavcan_stm32::SystemClock::instance());
	node->setNodeID(atoi(argv[1]));
	node->setName("uavcan");


	while (true)
	{
		const int res = node->start();
		if (res < 0)
		{
			printf("Node start failed: %d, retry\n", res);
			sleep(1);
		}
		else { break; }
	}

	MsgController *_msg_controller = new MsgController(*node);
	_msg_controller->init();

	int i = 0;
    node->setStatusOk();
    while (true)
    {
        int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0)
        {
            printf("Transient failure: %d\n", res);
		}

		uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
		kv_msg.numeric_value.push_back(i);

		kv_msg.key = "random";  // "random"
		kv_msg.key += "_";      // "random_"
		kv_msg.key += "int";  // "random_float"

		_msg_controller->msg_broadcast(kv_msg);
		i++;
	}
}
