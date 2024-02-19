#ifndef __CAN_TP_TEST_H__
#define __CAN_TP_TEST_H__

#include "stdint.h"
#include "isotp/isotp.h"

void can_tp_tx_hw(const uint32_t arbitration_id, uint8_t* data, const uint8_t size);
void set_timer(uint16_t time_ms, void (*callback));
int get_data();
void CAN_TP_Receive_interrupt(uint32_t arbitration_id, uint8_t* data, uint16_t dlc, IsoTpShims *shim, IsoTpMessage* recv_mess, IsoTpReceiveHandle* recv_handle);


#endif