#include "etc_controller.h"
#include "mbed.h"
#include <cstdio>

ETCController etc{PC_1, PC_2};

CAN can{PB_8, PB_9, 500000};

// Message IDs from CANbus.dbc
static constexpr uint16_t VCU_TPDO_STATUS_ID = 403;
static constexpr uint16_t SME_RPDO_THROTTLE_DEMAND_ID = 390;

// Predefined for use in main()
void send_etc_CAN_messages();
void send_sme_CAN_messages();

int main() {
    printf("Hello World!!\n");

    /*
        Make send_CAN messages loop consistently
        at 20hz (20 messages / second)
    */

    EventQueue queue;
    Thread can_thread;
    can_thread.start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(50ms, send_etc_CAN_messages);
    queue.call_every(50ms, send_sme_CAN_messages);


    // No need to edit this while loop
    while (true) {
        // Must implement in etc/ subfolder
        etc.update_state();
    }

    return 0;
}

/* 
    Must send the VCU_TPDO_STATUS CAN message with
    VCU_ETC_READY_TO_DRIVE & VCU_ETC_MOTOR_ENABLED
    set to 1, and VCU_ETC_IMPLAUS_APPS_OUT_OF_RANGE
    & VCU_ETC_IMPLAUS_APPS_DEVIATION properly added.
*/
void send_etc_CAN_messages() {
    uint16_t data[8] = {0};

    data[0] |= (1 << 0); // VCU_ETC_READY_TO_DRIVE
    data[0] |= (1 << 1); // VCU_ETC_MOTOR_ENABLED

    if (etc.out_of_range_implausibility) {
        data[0] |= (1 << 4); // VCU_ETC_IMPLAUS_APPS_DEVIATION
    }

    if (etc.apps_implausibility) {
        data[0] |= (1 << 6); // VCU_ETC_IMPLAUS_APPS_OUT_OF_RANGE
    }

    CANMessage msg(VCU_TPDO_STATUS_ID, data, 8);
    can.write(msg);
}

/*
    Must send the SME_RPDO_Throttle_Demand CAN
    message with SME_THROTL_TorqueDemand set to the
    torque demand determined by the ETC.
*/
void send_sme_CAN_messages() {
    uint16_t data[8] = {0};

    uint16_t torque_demand = static_cast<uint16_t>(etc.torque_demand);

    data[0] |= (torque_demand & 0xFF); // SME_THROTL_TorqueDemand
    data[1] |= ((torque_demand >> 8) & 0xFF); // SME_THROTL_TorqueDemand

    CANMessage msg(SME_RPDO_THROTTLE_DEMAND_ID, data, 8);
    can.write(msg);
}
