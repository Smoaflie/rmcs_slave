#include "uart.hpp"

#include "app/usb/cdc.hpp"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* hal_uart_handle, uint16_t size) {
    if (__HAL_UART_GET_FLAG(hal_uart_handle, UART_FLAG_IDLE))
        return;

    uart::Uart::Lazy* uart_lazy = nullptr;
    usb::field::StatusId field_id = usb::field::StatusId::RESERVED_;

    for (auto& uart_info : uart::uart_map){
        if (hal_uart_handle == uart_info.uart->hal_uart_handle_) {
            uart_lazy = &uart_info.uart;
            field_id  = uart_info.field_id;
            break;
        }
    }
    if (uart_lazy == nullptr || field_id == usb::field::StatusId::RESERVED_)   return;

    if (auto uart = uart_lazy->try_get()) {
        if (auto cdc = usb::cdc.try_get()) {
            uart->read_device_write_buffer(cdc->get_transmit_buffer(), field_id, size);
        }
    }
}
