#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>

#include <usart.h>

#include "app/logger/logger.hpp"
#include "app/usb/field.hpp"
#include "app/usb/interrupt_safe_buffer.hpp"
#include "utility/lazy.hpp"

namespace uart {

class Uart {
public:
    using Lazy = utility::Lazy<Uart, UART_HandleTypeDef*, size_t>;

    explicit Uart(UART_HandleTypeDef* hal_uart_handle, uint16_t max_receive_size)
        : hal_uart_handle_(hal_uart_handle)
        , max_receive_size_(max_receive_size) {
        assert(max_receive_size_ <= 64);
        assert(
            HAL_UARTEx_ReceiveToIdle_IT(
                hal_uart_handle_, reinterpret_cast<uint8_t*>(receive_buffer_), max_receive_size_)
            == HAL_OK);
        auto huart_idx = ((hal_uart_handle == &huart1) ? 1 : 
                        (hal_uart_handle == &huart7) ? 2 :
                        (hal_uart_handle == &huart10) ? 3 : 5);
        logger::Logger().printf("[INFO] Init success: huart%d.\n", huart_idx);
    }

    bool read_buffer_write_device(std::byte*& buffer) {
        auto& header = *std::launder(reinterpret_cast<FieldHeader*>(buffer));
        buffer += sizeof(FieldHeader);

        uint8_t size = header.data_size;
        if (!size)
            size = static_cast<uint8_t>(*buffer++);

        auto& transmit_buffer = transmit_buffers_[buffer_writing_.load(std::memory_order::relaxed)];
        uint8_t written_size  = transmit_buffer.written_size.load(std::memory_order::relaxed);

        uint8_t size_allowed = size;
        if (size_allowed > sizeof(transmit_buffer.data) - written_size)
            size_allowed = sizeof(transmit_buffer.data) - written_size;

        transmit_buffer.written_size.store(written_size + size_allowed, std::memory_order::relaxed);
        std::memcpy(&transmit_buffer.data[written_size], buffer, size_allowed);
        buffer += size;

        return size_allowed == size;
    }

    bool try_transmit() {
        auto writing = buffer_writing_.load(std::memory_order::relaxed);
        if (transmit_buffers_[writing].written_size.load(std::memory_order::relaxed) == 0)
            return false;

        if (!device_ready())
            return false;

        transmit_buffers_[!writing].written_size.store(0, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order::release);
        buffer_writing_.store(!writing, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order::release);

        // Note: Must read written_size again here to avoid data loss.
        assert(
            HAL_UART_Transmit_IT(
                hal_uart_handle_, reinterpret_cast<uint8_t*>(transmit_buffers_[writing].data),
                transmit_buffers_[writing].written_size.load(std::memory_order::relaxed))
            == HAL_OK);

        return true;
    }

private:
    friend void ::HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);

    bool device_ready() { return hal_uart_handle_->gState == HAL_UART_STATE_READY; }

    bool read_device_write_buffer(
        usb::InterruptSafeBuffer& buffer_wrapper, usb::field::StatusId field_id, uint16_t size) {
        assert(size);

        std::byte* buffer = buffer_wrapper.allocate(sizeof(FieldHeader) + (size > 15) + size);
        if (buffer) {
            // Write field header
            auto& header = *new (buffer) FieldHeader{};
            buffer += sizeof(FieldHeader);
            header.field_id = static_cast<uint8_t>(field_id);
            if (size <= 15) {
                // Store 4-bit size and field-id together
                header.data_size = size;
            } else {
                // Store size to a separate byte
                header.data_size = 0;
                *(buffer++)      = static_cast<std::byte>(size);
            }

            // Write received data
            std::memcpy(buffer, receive_buffer_, size);
            buffer += size;
        }

        // Because receive_buffer_ does not set to zero, the data is inaccurate
        logger::Logger().printf("[DEBUG] UART%d success fetch date: %s \n", 
                                static_cast<int>(field_id) - 4, 
                                reinterpret_cast<const char*>(receive_buffer_));
        assert(
            HAL_UARTEx_ReceiveToIdle_IT(
                hal_uart_handle_, reinterpret_cast<uint8_t*>(receive_buffer_), max_receive_size_)
            == HAL_OK);

        return static_cast<bool>(buffer);
    }

    UART_HandleTypeDef* hal_uart_handle_;

    struct __attribute__((packed)) FieldHeader {
        uint8_t field_id  : 4;
        uint8_t data_size : 4;
    };

    std::byte receive_buffer_[64];
    uint16_t max_receive_size_;

    struct {
        std::atomic<uint8_t> written_size = 0;
        std::byte data[128];
    } transmit_buffers_[2];
    std::atomic<uint8_t> buffer_writing_ = 0;
};



inline constinit Uart::Lazy uart1{&huart1, 15};
inline constinit Uart::Lazy uart2{&huart7, 15};
inline constinit Uart::Lazy uart3{&huart10, 15};
inline constinit Uart::Lazy uart_dbus{&huart5, 31};

struct UartMap{
    uart::Uart::Lazy& uart;
    usb::field::StatusId field_id;
};
inline constinit UartMap uart_map[] = {
    {uart::uart1, usb::field::StatusId::UART1_},
    {uart::uart2, usb::field::StatusId::UART2_},
    {uart::uart3, usb::field::StatusId::UART3_},
    {uart::uart_dbus, usb::field::StatusId::UART6_},
};

} // namespace uart
