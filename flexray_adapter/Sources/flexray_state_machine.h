/*
 * flexray_state_machine.h
 *
 */

#ifndef FLEXRAY_STATE_MACHINE_H_
#define FLEXRAY_STATE_MACHINE_H_

#include "packet.h"
#include "flexray_config.h"
#include "flexray_driver.h"

#define CAPTURE_RX_BUFF_SIZE (254 * 1024)
#define SIZE_OF_CAPTURE_HEADER (sizeof(uint8_t) + sizeof(uint8_t) + sizeof(packet_header) + sizeof(uint8_t))

/* FlexRay state machine states*/
typedef enum {
	FLEXRAY_WAITING_CLIENT_CONNECTION = 0x01,
	FLEXRAY_INITIALIZED,
	FLEXRAY_JOINING_CLUSTER,
	FLEXRAY_SET_TIMER,
	FLEXRAY_CHECK_TIMER_STATUS,
	FLEXRAY_DISCONNECT_FROM_CLUSTER,
	FLEXRAY_ERROR,
	FLEXRAY_ERROR_FINAL
} flexray_state;

typedef enum {
	FR_ERROR_OK = 0x0000,
	FR_ERROR_INIT_GET_POC_STATUS,
	FR_ERROR_INIT_ALLOWCOLDSTART,
	FR_ERROR_INIT_START_COMM,
	FR_ERROR_INIT_POC_READY_TIMEOUT,
	FR_ERROR_JOIN_GET_POC_STATUS,
	FR_ERROR_SET_ABS_TIMER_GET_GLOBAL_TIME,
	FR_ERROR_SET_ABS_TIMER,
	FR_ERROR_JOIN_HALT,
	FR_ERROR_CHECK_TIMER_GET_POC_STATUS,
	FR_ERROR_CHECK_TIMER_GET_TIMER_IRQ,
	FR_ERROR_CHECK_TIMER_ACK_TIMER,
	FR_ERROR_READ_RX_BUF,
}flexray_error;

typedef struct
{
	flexray_state state;
    uint32_t wait_poc_ready_cycles_counter;
	void *in_event_group;/* For connect/disconnect events sent from TCP/USB task to FR task */
	void *out_event_group;
	/* tx_msg_buf state, 1 for tx pending, 0 for idle*/
	uint8_t tx_msg_buf_pending[MAX_MSG_BUFS];

	/* Capture mode settings*/
	uint32_t capture_rx_buf_used;
	uint32_t capture_rx_buf_forwarded;
	uint8_t capture_rx_buf[CAPTURE_RX_BUFF_SIZE];

	uint8_t auto_send_tx_msg_buf_idx;

	/* Fields for debugging & statistics. */
	int16_t max_rate_correction;
	int16_t max_offset_correction;
	int16_t min_rate_correction;
	int16_t min_offset_correction;
	flexray_error error;
} flexray_data;

extern flexray_data g_flexray_data;

void flexray_run();
uint8_t flexray_write_tx_msg_buf(uint16_t frame_id, uint8_t *payload, uint16_t payload_length);

#endif /* FLEXRAY_STATE_MACHINE_H_ */
