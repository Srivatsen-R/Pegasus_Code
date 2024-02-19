#include "can_tp_app.h"
#include "isotp/isotp.h"
#include "isotp/isotp_types.h"



#ifdef PEGASUS_CANTP
#include "fdcan_AL.h"
#include "main.h"
#endif

int data = 0;

void can_tp_tx_hw(const uint32_t arbitration_id, uint8_t* data, const uint8_t size){
    // HAL_Delay(20); 
    Transmit_on_CAN(arbitration_id, S, data, size);

}

void set_timer(uint16_t time_ms, void (*callback)){}

/**
 * The function receives CAN messages and sets a flag when the message is completed.
 * 
 * @param arbitration_id The ID of the CAN message being received.
 * @param data A pointer to the data received over CAN bus.
 * @param dlc The length of the received CAN message data.
 * @param shim A pointer to an IsoTpShims struct, which contains function pointers for printf, CAN
 * transmit, and timer functions used by the ISO-TP protocol.
 * @param recv_mess A pointer to an IsoTpMessage struct that contains information about the received
 * message.
 * @param recv_handle A pointer to an IsoTpReceiveHandle struct, which is used to keep track of the
 * state of the ongoing receive operation. It is updated by the isotp_receive and
 * isotp_continue_receive functions.
 * @param completiton_flag_bit This parameter is a bit position in a flag variable that is used to
 * indicate the completion of the CAN TP receive operation. The bit at this position will be set when
 * the receive operation is completed.
 * @param can_tp_receiver_flag A pointer to a TX_EVENT_FLAGS_GROUP object that is used to signal the
 * completion of the receive operation.
 */

int get_data(){
    return data;
}

void CAN_TP_Receive_interrupt(uint32_t arbitration_id, uint8_t* data, uint16_t dlc, IsoTpShims *shim, IsoTpMessage* recv_mess, IsoTpReceiveHandle* recv_handle)
{
    uint8_t frame_type = data[0] & 0xF0;
    if(frame_type == (uint8_t)0x10 || frame_type == (uint8_t)0x00)
    {
        *shim = isotp_init_shims((void*)printf, (void*)can_tp_tx_hw, (void*)set_timer);
        *recv_handle = isotp_receive(shim, arbitration_id, NULL);
    }
    isotp_continue_receive(shim, recv_mess, recv_handle, arbitration_id, data, dlc);
    if(recv_mess->completed && recv_handle->completed)
    {
        __fdcan_transferMessagesOnID6FA(SUCCESS_MESS, 1);
        set_config_flag(1);
        set_interrupt_flag(1);
    }
}