#include "app/can/can.hpp"
#include "app/usb/cdc.hpp"

#include <fdcan.h>

extern "C" {
    
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    auto& can_lazy = hfdcan == &hfdcan1 ? can::can1 : (hfdcan == &hfdcan2 ? can::can2 : can::can3);
    auto field_id  = hfdcan == &hfdcan1 ? usb::field::StatusId::CAN1_ : (hfdcan == &hfdcan2 ? usb::field::StatusId::CAN2_ : usb::field::StatusId::CAN3_);

    if (auto can = can_lazy.try_get()) {
        if (auto cdc = usb::cdc.try_get()) {
            can->read_device_write_buffer(cdc->get_transmit_buffer(), field_id);
        }
    }
}

} // extern "C"
