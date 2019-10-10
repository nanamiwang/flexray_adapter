#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include <queue.h>
#include <task.h>
#include <event_groups.h>
#include "platform_defs.h"
#include "flexray_registers.h"
#include "flexray_driver.h"
#include "flexray_state_machine.h"
#include "packet.h"
#include "tcp_interface.h"
#include "event.h"

flexray_data g_flexray_data = {
		.state = FLEXRAY_WAITING_CLIENT_CONNECTION,
		.error = FR_ERROR_OK,
		.capture_rx_buf_used = 0U,
		.capture_rx_buf_forwarded = 0U,
		.auto_send_tx_msg_buf_idx = 0xFF,
};

typedef struct {
	packet_header hdr;
	uint8_t payload[cPayloadLengthMax * 2];
}frame_packet;
frame_packet s_frame_packet;

typedef struct {
	packet_header hdr;
	uint16_t u16_data;
}notify_packet;
notify_packet s_notify_packet;

extern uint8_t slot_65_frames[][34];

static flexray_error handle_rx_tx() {
    uint8_t ret = SUCCESS;
    fr_rx_status RxStatus;
    uint8_t u8RxLength = 0U;
    uint8_t i = 0U;
    uint16_t frame_id = 0;
    uint8_t *rx_buf_ptr = NULL;
    uint8_t use_capture_buf = 0U;
    uint32_t j = 0;
    uint8_t rx_idx = 0;
    uint8_t cur_cycle_counter = 0U, next_cycle_counter = 0U;
    uint16_t cur_macro_tick = 0U;
    packet_header *phdr;
	ret = flexray_driver_get_global_time(&cur_cycle_counter, &cur_macro_tick);
	if(SUCCESS != ret)
		return FR_ERROR_SET_ABS_TIMER_GET_GLOBAL_TIME;
    /* Receive on all individual rx msg bufs and FIFOs */
	for(; i < (uint8_t)g_fr_config.individual_rx_msg_buf_count; i++) {
		if((g_flexray_data.capture_rx_buf_used + SIZE_OF_CAPTURE_HEADER + cPayloadLengthMax * 2) < CAPTURE_RX_BUFF_SIZE) {
			rx_buf_ptr = &g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_used + SIZE_OF_CAPTURE_HEADER];
			use_capture_buf = 1;
		} else {
			rx_buf_ptr = &s_frame_packet.payload[0];
			use_capture_buf = 0;
		}
		ret = flexray_driver_read_rx_buffer(i, rx_buf_ptr, &RxStatus, &u8RxLength);
		/* Check the success of the call */
		if(SUCCESS != ret) {
			DBG_PRINT("flexray_driver_read_rx_buffer error %u", ret);
			return FR_ERROR_READ_RX_BUF;
		} else {
			if(FR_RX_STATUS_RECEIVED == RxStatus) {
				if(use_capture_buf) {
					// Save this frame in buffer, send to client later as a batch
					g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_used] = u8RxLength;
					g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_used + sizeof(uint8_t)] = (uint8_t)i;
					g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_used + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(packet_header)] = cur_cycle_counter;
					g_flexray_data.capture_rx_buf_used += (SIZE_OF_CAPTURE_HEADER + u8RxLength);
					// Capture buffer full?
					if((g_flexray_data.capture_rx_buf_used + SIZE_OF_CAPTURE_HEADER + cPayloadLengthMax * 2) >= CAPTURE_RX_BUFF_SIZE) {
						// TODO: Forward captured frames to PC
						for(j = 0; j < g_flexray_data.capture_rx_buf_used;) {
							u8RxLength = *(uint8_t *)&g_flexray_data.capture_rx_buf[j];
							rx_idx = *(uint8_t *)&g_flexray_data.capture_rx_buf[j + sizeof(uint8_t)];
							phdr = (packet_header *)&g_flexray_data.capture_rx_buf[j + sizeof(uint8_t) * 2];
							SET_PACKET_FLAG_FRAME_ID((phdr->flags), g_fr_config.msg_bufs[rx_idx].frame_id);
							tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_FRAME2, phdr, u8RxLength + sizeof(uint8_t));
							j += SIZE_OF_CAPTURE_HEADER +  u8RxLength;
						}
						g_flexray_data.capture_rx_buf_forwarded = g_flexray_data.capture_rx_buf_used;
					}
				} else {
					// Forward this frame to client immediately
					SET_PACKET_FLAG_FRAME_ID(s_frame_packet.hdr.flags, g_fr_config.msg_bufs[i].frame_id);
					tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_FRAME, &s_frame_packet.hdr, u8RxLength);
				}
			}
		}
	}

    if(g_fr_config.flags & FR_CONFIG_FLAG_FIFOA_ENABLED_MASK) {
		while(1) {
			flexray_driver_receive_fifoa(&s_frame_packet.payload[0], &RxStatus, &u8RxLength, &frame_id);
			if(RxStatus != FR_RX_STATUS_RECEIVED && RxStatus != FR_RX_STATUS_RECEIVED_MORE_DATA_AVAILABLE)
				break;
			SET_PACKET_FLAG_FRAME_ID(s_frame_packet.hdr.flags, frame_id);
			tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_FRAME, &s_frame_packet.hdr, u8RxLength);
		}
    }

    if(g_fr_config.flags & FR_CONFIG_FLAG_FIFOB_ENABLED_MASK) {
    	while(1) {
			flexray_driver_receive_fifob(&s_frame_packet.payload[0], &RxStatus, &u8RxLength, &frame_id);
			if(RxStatus != FR_RX_STATUS_RECEIVED && RxStatus != FR_RX_STATUS_RECEIVED_MORE_DATA_AVAILABLE)
				break;
			SET_PACKET_FLAG_FRAME_ID(s_frame_packet.hdr.flags, frame_id);
			tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_FRAME, &s_frame_packet.hdr, u8RxLength);
		}
	}

    /*Handle tx completion*/
	for(i = 0;i < (uint8_t)g_fr_config.individual_tx_msg_buf_count;i++)
		if(g_flexray_data.tx_msg_buf_pending[i] != 0 && flexray_driver_tx_buffer_idle(i) == 1) {
	    	s_notify_packet.u16_data = i;
	    	g_flexray_data.tx_msg_buf_pending[i] = 0;
			tcp_interface_send_packet(PACKET_TYPE_TX_COMPLETED, &s_notify_packet.hdr, sizeof(uint16_t));
		}
	// Replay captured frames
#if 0
	if(g_flexray_data.replay_tx_msg_buf_idx != 0xFFFFU && \
		g_flexray_data.capture_rx_buf_replayed < g_flexray_data.capture_rx_buf_used && \
		flexray_driver_tx_buffer_idle(g_flexray_data.replay_tx_msg_buf_idx) == 1) {
		u8RxLength = *(uint8_t *)&g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_replayed];
		rx_buf_ptr = &g_flexray_data.capture_rx_buf[g_flexray_data.capture_rx_buf_replayed + 1];
		ret = flexray_driver_write_tx_buffer(g_flexray_data.replay_tx_msg_buf_idx, rx_buf_ptr, u8RxLength);
		if(SUCCESS != ret)
			DBG_PRINT("flexray_driver_write_tx_buffer on msg buf %u error %u", g_flexray_data.replay_tx_msg_buf_idx, ret);
		g_flexray_data.capture_rx_buf_replayed += (1 + u8RxLength);
	}
#endif

	if(g_flexray_data.auto_send_tx_msg_buf_idx != 0xFF) {
		if(cur_cycle_counter < cCycleCountMax)
			next_cycle_counter = cur_cycle_counter + 1U;
		else
			next_cycle_counter = 0U;
		ret = flexray_driver_write_tx_buffer(g_flexray_data.auto_send_tx_msg_buf_idx, &slot_65_frames[next_cycle_counter][0], 34);
		if(SUCCESS != ret)
			DBG_PRINT("flexray_driver_write_tx_buffer on msg buf %u error %u", g_flexray_data.auto_send_tx_msg_buf_idx, ret);
	}
    return FR_ERROR_OK;
}

#if 0
/* 40 bytes per task. */
static char task_stat_buf[40 * 10 + 2] = "\r\n";
static void print_task_statistics() {
	vTaskGetRunTimeStats(&task_stat_buf[1]);
	DBG_PRINT(task_stat_buf);
}
#endif

flexray_error set_abs_timer() {
    uint8_t ret, desired_timer_expire_cycle;
    uint8_t cur_cycle_counter = 0U;
    uint16_t cur_macro_tick = 0U;
    int16_t rate_correction, offset_correction;
    double gdMacrotick = ((double)g_fr_config.gdCycle) / ((double)(gMacroPerCycle));
    /* Handle read/write before symbol window */
    uint16_t offset_macroticks = (uint16_t)gMacroPerCycle - (g_fr_config.gdSymbolWindow + g_fr_config.gdNIT);
	/* DEBUG: Track min/max rate/offset corrections */
    if(g_fr_config.flags & FR_CONFIG_FLAG_LOG_STATUS_DATA_MASK) {
		flexray_driver_get_clock_correction(&rate_correction, &offset_correction);
		if(rate_correction > g_flexray_data.max_rate_correction)
			g_flexray_data.max_rate_correction = rate_correction;
		if(rate_correction < g_flexray_data.min_rate_correction)
			g_flexray_data.min_rate_correction = rate_correction;
		if(offset_correction > g_flexray_data.max_offset_correction)
			g_flexray_data.max_offset_correction = offset_correction;
		if(offset_correction < g_flexray_data.max_offset_correction)
			g_flexray_data.min_offset_correction = offset_correction;
    }
    ret = flexray_driver_get_global_time(&cur_cycle_counter, &cur_macro_tick);
    if(SUCCESS != ret)
    	return FR_ERROR_SET_ABS_TIMER_GET_GLOBAL_TIME;

	if(cur_cycle_counter < cCycleCountMax) {
		desired_timer_expire_cycle = cur_cycle_counter + 1U;
	} else {
		desired_timer_expire_cycle = 0U;
	}
	/* Set the timer to expire in the next cycle */
	ret = flexray_driver_set_abs_timer(0U, desired_timer_expire_cycle, offset_macroticks);
	if(SUCCESS != ret) {
		DBG_PRINT("flexray_driver_set_abs_timer error %u", ret);
    	return FR_ERROR_SET_ABS_TIMER;
	}
	/* We sleep for a while, give cpu to other tasks, wait the timer to expire.
	 * FlexRay spec 2.1, B.4.11: max value of gMacroPerCycle is 16000
	*/
	sleep( floor((double)(gMacroPerCycle + offset_macroticks  - cur_macro_tick) * gdMacrotick / 1000.0) );
    return FR_ERROR_OK;
}

void flexray_run()
{
    uint8_t ret;
    uint8_t timer_expired = 0;
    fr_poc_status poc_state;
    EventBits_t event_bits;
    packet_header msg_hdr;
    flexray_error err;
    uint8_t i;

    /* Check in event group, start/stop driver on demand */
    event_bits =  xEventGroupWaitBits(
    		 g_flexray_data.in_event_group,
			 EVENT_GROUP_START_FLEXRAY_STATE_MACHINE_BIT | EVENT_GROUP_STOP_FLEXRAY_STATE_MACHINE_BIT,
    		pdTRUE, pdFALSE, 0 );
	if((event_bits & EVENT_GROUP_START_FLEXRAY_STATE_MACHINE_BIT) != 0) {
		DBG_PRINT("Client request me to start the driver");
		if(g_flexray_data.state != FLEXRAY_WAITING_CLIENT_CONNECTION) {
			DBG_PRINT("Invalid state while starting driver.");
			return;
		}
		g_flexray_data.wait_poc_ready_cycles_counter = MAX_WAIT_POC_STATE_CHANGE_CYCLES;
		g_flexray_data.state = FLEXRAY_INITIALIZED;
		g_flexray_data.max_rate_correction = g_flexray_data.min_rate_correction = g_flexray_data.max_offset_correction = g_flexray_data.min_offset_correction = 0;
		for(i = 0;i < (uint8_t)g_fr_config.individual_tx_msg_buf_count;i++)
			g_flexray_data.tx_msg_buf_pending[i] = 0;
		xEventGroupSetBits(g_flexray_data.out_event_group, EVENT_GROUP_FLEXRAY_STATE_MACHINE_STARTED_BIT);
	}
	else if((event_bits & EVENT_GROUP_STOP_FLEXRAY_STATE_MACHINE_BIT) != 0) {
		DBG_PRINT("Stop the state machine");
#if 0
		print_task_statistics();
#endif
		g_flexray_data.state = FLEXRAY_WAITING_CLIENT_CONNECTION;
		g_flexray_data.capture_rx_buf_used = 0U;
		g_flexray_data.capture_rx_buf_forwarded = 0U;
		g_flexray_data.auto_send_tx_msg_buf_idx = 0xFF;
		xEventGroupSetBits(g_flexray_data.out_event_group, EVENT_GROUP_FLEXRAY_STATE_MACHINE_STOPPED_BIT);
    }

    switch(g_flexray_data.state)
    {
		case FLEXRAY_WAITING_CLIENT_CONNECTION:
			break;
        case FLEXRAY_INITIALIZED:
            /* The FlexRay configuration was set - wait until it reaches the POC:Ready state */
            ret = flexray_driver_get_poc_status(&poc_state);
            /* Check the error status */
            if(SUCCESS != ret) {   /* The call was not successful - go to error state */
            	g_flexray_data.state = FLEXRAY_ERROR;
            	g_flexray_data.error = FR_ERROR_INIT_GET_POC_STATUS;
            	DBG_PRINT("flexray_driver_get_poc_status error %u at FLEXRAY_CONFIGURED", ret);
            	break;
            }
			if(FR_PSR0_PROTSTATE_READY_U16 == poc_state.state) {
				if(g_fr_config.pKeySlotUsedForStartup != 0U) {
					/* Allow the node to be a coldstart node */
					ret = flexray_driver_allow_coldstart();
					/* Check the status */
					if(SUCCESS != ret) {   /* An error has occurred - go to the error state */
						g_flexray_data.state = FLEXRAY_ERROR;
						g_flexray_data.error = FR_ERROR_INIT_ALLOWCOLDSTART;
						DBG_PRINT("flexray_driver_allow_coldstart error %u", ret);
						break;
					}
				}
				 ret = flexray_driver_start_communication();
				 /* Check success of the call (not of the integration to cluster) */
				 if(SUCCESS != ret) {   /* An error has occurred - go to the error state */
					 g_flexray_data.state = FLEXRAY_ERROR;
					 g_flexray_data.error = FR_ERROR_INIT_START_COMM;
					 DBG_PRINT("flexray_driver_start_communication error %u", ret);
				 } else {   /* No error, the controller started joining the cluster - go to the next state */
					 g_flexray_data.state = FLEXRAY_JOINING_CLUSTER;
					 DBG_PRINT("Joining cluster...");
				 }
			} else {   /* The FlexRay controller has not reached the POC:Ready yet */
				if(0 < g_flexray_data.wait_poc_ready_cycles_counter) {
					g_flexray_data.wait_poc_ready_cycles_counter--;
					g_flexray_data.state = FLEXRAY_INITIALIZED;
				} else {   /* Timeout has expired - this is an error */
					g_flexray_data.state = FLEXRAY_ERROR;
					g_flexray_data.error = FR_ERROR_INIT_POC_READY_TIMEOUT;
					DBG_PRINT("Waiting poc:ready timeout");
				}
			}
            break;
        case FLEXRAY_JOINING_CLUSTER:
            ret = flexray_driver_get_poc_status(&poc_state);
            if(SUCCESS != ret) {
            	g_flexray_data.state = FLEXRAY_ERROR;
				g_flexray_data.error = FR_ERROR_JOIN_GET_POC_STATUS;
            	DBG_PRINT("flexray_driver_get_poc_status error %u", ret);
            	break;
            }
			if(FR_PSR0_PROTSTATE_NORMAL_ACTIVE_U16 == poc_state.state) {
				DBG_PRINT("Joining cluster succeeded.");
				tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_JOINED_CLUSTER, &msg_hdr, 0U);
				err = set_abs_timer();
				if(FR_ERROR_OK != err) {
					g_flexray_data.state = FLEXRAY_ERROR;
					g_flexray_data.error = err;
				} else
					g_flexray_data.state = FLEXRAY_CHECK_TIMER_STATUS;
			} else if(FR_PSR0_PROTSTATE_HALT_U16 == poc_state.state) {
				tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_JOIN_CLUSTER_FAILED, &msg_hdr, 0);
				g_flexray_data.state = FLEXRAY_ERROR;
				g_flexray_data.error = FR_ERROR_JOIN_HALT;
			} else
				sleep(0);
            break;
        case FLEXRAY_CHECK_TIMER_STATUS:
            ret = flexray_driver_get_poc_status(&poc_state);
            if(SUCCESS != ret) {
            	g_flexray_data.state = FLEXRAY_ERROR;
				g_flexray_data.error = FR_ERROR_CHECK_TIMER_GET_POC_STATUS;
            	DBG_PRINT("flexray_driver_get_poc_status error %u", ret);
            	break;
            }
			if((FR_PSR0_PROTSTATE_NORMAL_PASSIVE_U16 != poc_state.state) && FR_PSR0_PROTSTATE_NORMAL_ACTIVE_U16 != poc_state.state) {
				DBG_PRINT("Invalid poc state 0x%X before checking abs timer", poc_state.state);
				g_flexray_data.state = FLEXRAY_DISCONNECT_FROM_CLUSTER;
				break;
			}
			ret = flexray_driver_get_timer_irq_status(0U, &timer_expired);
			if(SUCCESS != ret) {
				g_flexray_data.state = FLEXRAY_ERROR;
				g_flexray_data.error = FR_ERROR_CHECK_TIMER_GET_TIMER_IRQ;
				DBG_PRINT("Error in flexray_driver_get_timer_irq_status %u", ret);
			} else {
				if(1 == timer_expired) {
					err = handle_rx_tx();
					if(FR_ERROR_OK != ret) {
						g_flexray_data.state = FLEXRAY_ERROR;
						g_flexray_data.error = err;
						DBG_PRINT("Error in handle_rx");
						break;
					}
					ret = flexray_driver_ack_abs_timer(0U);
					if(SUCCESS != ret) {
						g_flexray_data.state = FLEXRAY_ERROR;
						g_flexray_data.error = FR_ERROR_CHECK_TIMER_ACK_TIMER;
						DBG_PRINT("Error in flexray_driver_ack_abs_timer %u", ret);
						break;
					}
					err = set_abs_timer(); /* Schedule next timer */
					if(FR_ERROR_OK != err) {
						g_flexray_data.state = FLEXRAY_ERROR;
						g_flexray_data.error = err;
					} else
						g_flexray_data.state = FLEXRAY_CHECK_TIMER_STATUS;
					break;
				} else
					sleep(0);
			}
            break;
        case FLEXRAY_DISCONNECT_FROM_CLUSTER:
			tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_DISCONNECTED_FROM_CLUSTER, &msg_hdr, 0);
            g_flexray_data.state = FLEXRAY_ERROR_FINAL;
            break;
        case FLEXRAY_ERROR:
        	/* Fatal error, should not happen. */
        	s_notify_packet.u16_data = g_flexray_data.error;
			tcp_interface_send_packet(PACKET_TYPE_FLEXRAY_FATAL_ERROR, &s_notify_packet.hdr, sizeof(uint16_t));
            g_flexray_data.state = FLEXRAY_ERROR_FINAL;
            break;
        case FLEXRAY_ERROR_FINAL:
        	sleep(1);
            g_flexray_data.state = FLEXRAY_ERROR_FINAL;
            break;
        default:
        	DBG_PRINT("Unknown state");
            break;

    }
    return;
}

uint8_t flexray_write_tx_msg_buf(uint16_t frame_id, uint8_t *payload, uint16_t payload_length) {
	uint8_t ret = SUCCESS;
	uint8_t tx_msg_buf_idx =  (uint8_t)frame_id;
	if(tx_msg_buf_idx >= g_fr_config.individual_tx_msg_buf_count) {
		DBG_PRINT("Invalid transmit msg buf index: %u", tx_msg_buf_idx );
		return FAILED;
	}
	if(g_flexray_data.tx_msg_buf_pending[tx_msg_buf_idx] == 0) {
		g_flexray_data.tx_msg_buf_pending[tx_msg_buf_idx] = 1;
		ret = flexray_driver_write_tx_buffer(tx_msg_buf_idx, payload, payload_length);
		if(SUCCESS != ret) {
			DBG_PRINT("flexray_driver_write_tx_buffer on msg buf %u error %u", tx_msg_buf_idx, ret);
			return FAILED;
		}

	} else {
		/* The slot is busy, notify the client immediately */
    	s_notify_packet.u16_data = frame_id;
		tcp_interface_send_packet(PACKET_TYPE_TX_SLOT_BUSY, &s_notify_packet.hdr, sizeof(uint16_t));
	}
	return(ret);
}
