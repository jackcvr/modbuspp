#pragma once

#include <modbus.h>

#if !LIBMODBUS_VERSION_CHECK(3, 1, 6) || LIBMODBUS_VERSION_CHECK(4, 0, 0)
#error "Unsupported libmodbus version: must be >= 3.1.6 and < 4.0.0"
#endif

#include <cerrno>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

#if __cplusplus >= 202302L && __has_include(<expected>)
#include <expected>
#define MODBUSPP_HAS_EXPECTED
#endif

namespace modbus {

inline const char* strerror(int err) noexcept {
    return modbus_strerror(err);
}

class ModbusException : public std::runtime_error {
public:
    explicit ModbusException(const std::string& msg, int code)
        : std::runtime_error(msg + ": " + strerror(code)), code_(code) {}

    auto code() const noexcept {
        return code_;
    }

private:
    int code_;
};

struct ExceptionPolicy {
    static int handle(int rc, const char* msg) {
        if (rc == -1) {
            throw ModbusException(msg, errno);
        }
        return rc;
    }
};

struct ValuePolicy {
    static int handle(int value, const char*) noexcept {
        return value;
    }
};

#ifdef MODBUSPP_HAS_EXPECTED
class ModbusError {
public:
    ModbusError(int code) : code_(code) {}

    auto code() const noexcept {
        return code_;
    }

    auto message() const noexcept {
        return strerror(code_);
    }

private:
    int code_;
};

struct ExpectedPolicy {
    static std::expected<int, ModbusError> handle(int value, const char*) noexcept {
        if (value == -1) {
            return std::unexpected(ModbusError(errno));
        }
        return value;
    }
};
#endif

struct ContextDeleter {
    void operator()(modbus_t* ctx) const noexcept {
        if (ctx) {
            modbus_close(ctx);
            modbus_free(ctx);
        }
    }
};

template <typename ErrorPolicy = ExceptionPolicy>
class Device {
public:
    explicit Device(const std::string& device = "/dev/ttyUSB0", int baud = 115200,
                    char parity = 'N', int data_bit = 8, int stop_bit = 1) {
        modbus_t* ctx = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
        if (!ctx) {
            throw std::system_error(errno, std::generic_category(), "Failed to create RTU context");
        }
        ctx_.reset(ctx);
    }

    explicit Device(const std::string& ip, int port) {
        modbus_t* ctx = modbus_new_tcp(ip.c_str(), port);
        if (!ctx) {
            throw std::system_error(errno, std::generic_category(), "Failed to create TCP context");
        }
        ctx_.reset(ctx);
    }

    explicit Device(const std::string& node, const std::string& service) {
        modbus_t* ctx = modbus_new_tcp_pi(node.c_str(), service.c_str());
        if (!ctx) {
            throw std::system_error(errno, std::generic_category(),
                                    "Failed to create TCP PI context");
        }
        ctx_.reset(ctx);
    }

    modbus_t* get() const noexcept {
        return ctx_.get();
    }

    void close() noexcept {
        modbus_close(ctx_.get());
    }

    auto flush() {
        return ErrorPolicy::handle(modbus_flush(ctx_.get()), "Flush failed");
    }

    auto set_slave(int slave_id) {
        return ErrorPolicy::handle(modbus_set_slave(ctx_.get(), slave_id), "Set slave failed");
    }

    int get_slave() const noexcept {
        return modbus_get_slave(ctx_.get());
    }

    auto tcp_listen(int nb_connection) {
        return ErrorPolicy::handle(modbus_tcp_listen(ctx_.get(), nb_connection),
                                   "TCP listen failed");
    }

    auto tcp_accept(int& socket) {
        return ErrorPolicy::handle(modbus_tcp_accept(ctx_.get(), &socket), "TCP accept failed");
    }

    auto connect() {
        return ErrorPolicy::handle(modbus_connect(ctx_.get()), "Connect failed");
    }

    auto get_serial_mode() {
        return ErrorPolicy::handle(modbus_rtu_get_serial_mode(ctx_.get()),
                                   "Get serial mode failed");
    }

    auto set_serial_mode(int mode) {
        return ErrorPolicy::handle(modbus_rtu_set_serial_mode(ctx_.get(), mode),
                                   "Set serial mode failed");
    }

    auto get_rts() {
        return ErrorPolicy::handle(modbus_rtu_get_rts(ctx_.get()), "Get RTS failed");
    }

    auto set_rts(int mode) {
        return ErrorPolicy::handle(modbus_rtu_set_rts(ctx_.get(), mode), "Set RTS failed");
    }

    auto set_custom_rts(void (*set_rts_cb)(modbus_t* ctx, int on)) {
        return ErrorPolicy::handle(modbus_rtu_set_custom_rts(ctx_.get(), set_rts_cb),
                                   "Set custom RTS failed");
    }

    auto get_rts_delay() {
        return ErrorPolicy::handle(modbus_rtu_get_rts_delay(ctx_.get()), "Get RTS delay failed");
    }

    auto set_rts_delay(int us) {
        return ErrorPolicy::handle(modbus_rtu_set_rts_delay(ctx_.get(), us),
                                   "Set RTS delay failed");
    }

    auto read_bits(int addr, int nb, uint8_t* dest) {
        return ErrorPolicy::handle(modbus_read_bits(ctx_.get(), addr, nb, dest),
                                   "Read bits failed");
    }

    auto read_input_bits(int addr, int nb, uint8_t* dest) {
        return ErrorPolicy::handle(modbus_read_input_bits(ctx_.get(), addr, nb, dest),
                                   "Read input bits failed");
    }

    auto write_bit(int addr, bool status) {
        return ErrorPolicy::handle(modbus_write_bit(ctx_.get(), addr, status ? 1 : 0),
                                   "Write bit failed");
    }

    auto write_bits(int addr, int nb, const uint8_t* data) {
        return ErrorPolicy::handle(modbus_write_bits(ctx_.get(), addr, nb, data),
                                   "Write bits failed");
    }

    auto read_registers(int addr, int nb, uint16_t* dest) {
        return ErrorPolicy::handle(modbus_read_registers(ctx_.get(), addr, nb, dest),
                                   "Read registers failed");
    }

    auto read_input_registers(int addr, int nb, uint16_t* dest) {
        return ErrorPolicy::handle(modbus_read_input_registers(ctx_.get(), addr, nb, dest),
                                   "Read input registers failed");
    }

    auto write_register(int addr, uint16_t value) {
        return ErrorPolicy::handle(modbus_write_register(ctx_.get(), addr, value),
                                   "Write register failed");
    }

    auto write_registers(int addr, int nb, const uint16_t* data) {
        return ErrorPolicy::handle(modbus_write_registers(ctx_.get(), addr, nb, data),
                                   "Write registers failed");
    }

    auto write_and_read_registers(int write_addr, int write_nb, const uint16_t* src, int read_addr,
                                  int read_nb, uint16_t* dest) {
        return ErrorPolicy::handle(modbus_write_and_read_registers(ctx_.get(), write_addr, write_nb,
                                                                   src, read_addr, read_nb, dest),
                                   "Write and read registers failed");
    }

    auto mask_write_register(int addr, uint16_t and_mask, uint16_t or_mask) {
        return ErrorPolicy::handle(modbus_mask_write_register(ctx_.get(), addr, and_mask, or_mask),
                                   "Mask write failed");
    }

    auto set_error_recovery(modbus_error_recovery_mode mode) {
        return ErrorPolicy::handle(modbus_set_error_recovery(ctx_.get(), mode),
                                   "Set error recovery failed");
    }

    auto set_socket(int s) {
        return ErrorPolicy::handle(modbus_set_socket(ctx_.get(), s), "Set socket failed");
    }

    int get_socket() const noexcept {
        return modbus_get_socket(ctx_.get());
    }

    auto set_response_timeout(uint32_t sec, uint32_t usec) {
        return ErrorPolicy::handle(modbus_set_response_timeout(ctx_.get(), sec, usec),
                                   "Set response timeout failed");
    }

    auto get_response_timeout(uint32_t* sec, uint32_t* usec) const {
        return ErrorPolicy::handle(modbus_get_response_timeout(ctx_.get(), sec, usec),
                                   "Get response timeout failed");
    }

    auto set_byte_timeout(uint32_t sec, uint32_t usec) {
        return ErrorPolicy::handle(modbus_set_byte_timeout(ctx_.get(), sec, usec),
                                   "Set byte timeout failed");
    }

    auto get_byte_timeout(uint32_t* sec, uint32_t* usec) const {
        return ErrorPolicy::handle(modbus_get_byte_timeout(ctx_.get(), sec, usec),
                                   "Get byte timeout failed");
    }

    auto set_indication_timeout(uint32_t sec, uint32_t usec) {
        return ErrorPolicy::handle(modbus_set_indication_timeout(ctx_.get(), sec, usec),
                                   "Set indication timeout failed");
    }

    auto get_indication_timeout(uint32_t* sec, uint32_t* usec) const {
        return ErrorPolicy::handle(modbus_get_indication_timeout(ctx_.get(), sec, usec),
                                   "Get indication timeout failed");
    }

    int get_header_length() const noexcept {
        return modbus_get_header_length(ctx_.get());
    }

    void set_debug(bool enable) noexcept {
        modbus_set_debug(ctx_.get(), enable ? 1 : 0);
    }

    auto receive(uint8_t* req) {
        return ErrorPolicy::handle(modbus_receive(ctx_.get(), req), "Receive failed");
    }

    auto receive_confirmation(uint8_t* rsp) {
        return ErrorPolicy::handle(modbus_receive_confirmation(ctx_.get(), rsp),
                                   "Receive confirmation failed");
    }

    auto reply(const uint8_t* req, int req_length, modbus_mapping_t* mb_mapping) {
        return ErrorPolicy::handle(modbus_reply(ctx_.get(), req, req_length, mb_mapping),
                                   "Reply failed");
    }

    auto reply_exception(const uint8_t* req, unsigned int exception_code) {
        return ErrorPolicy::handle(modbus_reply_exception(ctx_.get(), req, exception_code),
                                   "Reply exception failed");
    }

    auto report_slave_id(int max_dest, uint8_t* dest) {
        return ErrorPolicy::handle(modbus_report_slave_id(ctx_.get(), max_dest, dest),
                                   "Report slave ID failed");
    }

    auto send_raw_request(const uint8_t* raw_req, int raw_req_length) {
        return ErrorPolicy::handle(modbus_send_raw_request(ctx_.get(), raw_req, raw_req_length),
                                   "Send raw request failed");
    }

private:
    std::unique_ptr<modbus_t, ContextDeleter> ctx_;
};

struct MappingDeleter {
    void operator()(modbus_mapping_t* m) const noexcept {
        if (m) {
            modbus_mapping_free(m);
        }
    }
};

class Mapping {
public:
    Mapping(int nb_bits, int nb_input_bits, int nb_registers, int nb_input_registers) {
        modbus_mapping_t* map =
            modbus_mapping_new(nb_bits, nb_input_bits, nb_registers, nb_input_registers);
        if (!map) {
            throw std::system_error(errno, std::generic_category(), "Failed to allocate mapping");
        }
        map_.reset(map);
    }

    Mapping(unsigned int start_bits, unsigned int nb_bits, unsigned int start_input_bits,
            unsigned int nb_input_bits, unsigned int start_registers, unsigned int nb_registers,
            unsigned int start_input_registers, unsigned int nb_input_registers) {
        modbus_mapping_t* map = modbus_mapping_new_start_address(
            start_bits, nb_bits, start_input_bits, nb_input_bits, start_registers, nb_registers,
            start_input_registers, nb_input_registers);
        if (!map) {
            throw std::system_error(errno, std::generic_category(),
                                    "Failed to allocate mapping with start addresses");
        }
        map_.reset(map);
    }

    modbus_mapping_t* get() const noexcept {
        return map_.get();
    }

    uint8_t* tab_bits() noexcept {
        return map_->tab_bits;
    }
    uint8_t* tab_input_bits() noexcept {
        return map_->tab_input_bits;
    }
    uint16_t* tab_registers() noexcept {
        return map_->tab_registers;
    }
    uint16_t* tab_input_registers() noexcept {
        return map_->tab_input_registers;
    }

private:
    std::unique_ptr<modbus_mapping_t, MappingDeleter> map_;
};

}  // namespace modbus
