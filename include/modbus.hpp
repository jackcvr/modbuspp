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

struct ContextDeleter {
    void operator()(modbus_t* ctx) const noexcept {
        if (ctx) {
            modbus_close(ctx);
            modbus_free(ctx);
        }
    }
};

class BaseDevice {
public:
    modbus_t* get() const noexcept {
        return ctx_.get();
    }

    void close() noexcept {
        modbus_close(ctx_.get());
    }

    int flush() {
        return modbus_flush(ctx_.get());
    }

    int set_slave(int slave_id) {
        return modbus_set_slave(ctx_.get(), slave_id);
    }

    int get_slave() const noexcept {
        return modbus_get_slave(ctx_.get());
    }

    int connect() {
        return modbus_connect(ctx_.get());
    }

    int read_bits(int addr, int nb, uint8_t* dest) {
        return modbus_read_bits(ctx_.get(), addr, nb, dest);
    }

    int read_input_bits(int addr, int nb, uint8_t* dest) {
        return modbus_read_input_bits(ctx_.get(), addr, nb, dest);
    }

    int write_bit(int addr, bool status) {
        return modbus_write_bit(ctx_.get(), addr, status ? 1 : 0);
    }

    int write_bits(int addr, int nb, const uint8_t* data) {
        return modbus_write_bits(ctx_.get(), addr, nb, data);
    }

    int read_registers(int addr, int nb, uint16_t* dest) {
        return modbus_read_registers(ctx_.get(), addr, nb, dest);
    }

    int read_input_registers(int addr, int nb, uint16_t* dest) {
        return modbus_read_input_registers(ctx_.get(), addr, nb, dest);
    }

    int write_register(int addr, uint16_t value) {
        return modbus_write_register(ctx_.get(), addr, value);
    }

    int write_registers(int addr, int nb, const uint16_t* data) {
        return modbus_write_registers(ctx_.get(), addr, nb, data);
    }

    int write_and_read_registers(int write_addr, int write_nb, const uint16_t* src, int read_addr,
                                 int read_nb, uint16_t* dest) {
        return modbus_write_and_read_registers(ctx_.get(), write_addr, write_nb, src, read_addr,
                                               read_nb, dest);
    }

    int mask_write_register(int addr, uint16_t and_mask, uint16_t or_mask) {
        return modbus_mask_write_register(ctx_.get(), addr, and_mask, or_mask);
    }

    int set_error_recovery(modbus_error_recovery_mode mode) {
        return modbus_set_error_recovery(ctx_.get(), mode);
    }

    int set_socket(int s) {
        return modbus_set_socket(ctx_.get(), s);
    }

    int get_socket() const noexcept {
        return modbus_get_socket(ctx_.get());
    }

    int set_response_timeout(uint32_t sec, uint32_t usec) {
        return modbus_set_response_timeout(ctx_.get(), sec, usec);
    }

    int get_response_timeout(uint32_t* sec, uint32_t* usec) const {
        return modbus_get_response_timeout(ctx_.get(), sec, usec);
    }

    int set_byte_timeout(uint32_t sec, uint32_t usec) {
        return modbus_set_byte_timeout(ctx_.get(), sec, usec);
    }

    int get_byte_timeout(uint32_t* sec, uint32_t* usec) const {
        return modbus_get_byte_timeout(ctx_.get(), sec, usec);
    }

    int set_indication_timeout(uint32_t sec, uint32_t usec) {
        return modbus_set_indication_timeout(ctx_.get(), sec, usec);
    }

    int get_indication_timeout(uint32_t* sec, uint32_t* usec) const {
        return modbus_get_indication_timeout(ctx_.get(), sec, usec);
    }

    int get_header_length() const noexcept {
        return modbus_get_header_length(ctx_.get());
    }

    void set_debug(bool enable) noexcept {
        modbus_set_debug(ctx_.get(), enable ? 1 : 0);
    }

    int receive(uint8_t* req) {
        return modbus_receive(ctx_.get(), req);
    }

    int receive_confirmation(uint8_t* rsp) {
        return modbus_receive_confirmation(ctx_.get(), rsp);
    }

    int reply(const uint8_t* req, int req_length, modbus_mapping_t* mb_mapping) {
        return modbus_reply(ctx_.get(), req, req_length, mb_mapping);
    }

    int reply_exception(const uint8_t* req, unsigned int exception_code) {
        return modbus_reply_exception(ctx_.get(), req, exception_code);
    }

    int report_slave_id(int max_dest, uint8_t* dest) {
        return modbus_report_slave_id(ctx_.get(), max_dest, dest);
    }

    int send_raw_request(const uint8_t* raw_req, int raw_req_length) {
        return modbus_send_raw_request(ctx_.get(), raw_req, raw_req_length);
    }

protected:
    explicit BaseDevice(modbus_t* ctx) : ctx_(ctx) {
        if (!ctx_) {
            throw std::system_error(errno, std::generic_category(), "Failed to create context");
        }
    }

    std::unique_ptr<modbus_t, ContextDeleter> ctx_;
};

class Device : public BaseDevice {
protected:
    static int check(int rc, const char* msg) {
        if (rc == -1) {
            throw ModbusException(msg, errno);
        }
        return rc;
    }

public:
    using BaseDevice::BaseDevice;

    int flush() {
        return check(BaseDevice::flush(), "Flush failed");
    }

    int set_slave(int slave_id) {
        return check(BaseDevice::set_slave(slave_id), "Set slave failed");
    }

    int connect() {
        return check(BaseDevice::connect(), "Connect failed");
    }

    int read_bits(int addr, int nb, uint8_t* dest) {
        return check(BaseDevice::read_bits(addr, nb, dest), "Read bits failed");
    }

    int read_input_bits(int addr, int nb, uint8_t* dest) {
        return check(BaseDevice::read_input_bits(addr, nb, dest), "Read input bits failed");
    }

    int write_bit(int addr, bool status) {
        return check(BaseDevice::write_bit(addr, status), "Write bit failed");
    }

    int write_bits(int addr, int nb, const uint8_t* data) {
        return check(BaseDevice::write_bits(addr, nb, data), "Write bits failed");
    }

    int read_registers(int addr, int nb, uint16_t* dest) {
        return check(BaseDevice::read_registers(addr, nb, dest), "Read registers failed");
    }

    int read_input_registers(int addr, int nb, uint16_t* dest) {
        return check(BaseDevice::read_input_registers(addr, nb, dest),
                     "Read input registers failed");
    }

    int write_register(int addr, uint16_t value) {
        return check(BaseDevice::write_register(addr, value), "Write register failed");
    }

    int write_registers(int addr, int nb, const uint16_t* data) {
        return check(BaseDevice::write_registers(addr, nb, data), "Write registers failed");
    }

    int write_and_read_registers(int write_addr, int write_nb, const uint16_t* src, int read_addr,
                                 int read_nb, uint16_t* dest) {
        return check(BaseDevice::write_and_read_registers(write_addr, write_nb, src, read_addr,
                                                          read_nb, dest),
                     "Write and read registers failed");
    }

    int mask_write_register(int addr, uint16_t and_mask, uint16_t or_mask) {
        return check(BaseDevice::mask_write_register(addr, and_mask, or_mask), "Mask write failed");
    }

    int set_error_recovery(modbus_error_recovery_mode mode) {
        return check(BaseDevice::set_error_recovery(mode), "Set error recovery failed");
    }

    int set_socket(int s) {
        return check(BaseDevice::set_socket(s), "Set socket failed");
    }

    int set_response_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_response_timeout(sec, usec), "Set response timeout failed");
    }

    int get_response_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_response_timeout(sec, usec), "Get response timeout failed");
    }

    int set_byte_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_byte_timeout(sec, usec), "Set byte timeout failed");
    }

    int get_byte_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_byte_timeout(sec, usec), "Get byte timeout failed");
    }

    int set_indication_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_indication_timeout(sec, usec),
                     "Set indication timeout failed");
    }

    int get_indication_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_indication_timeout(sec, usec),
                     "Get indication timeout failed");
    }

    int receive(uint8_t* req) {
        return check(BaseDevice::receive(req), "Receive failed");
    }

    int receive_confirmation(uint8_t* rsp) {
        return check(BaseDevice::receive_confirmation(rsp), "Receive confirmation failed");
    }

    int reply(const uint8_t* req, int req_length, modbus_mapping_t* mb_mapping) {
        return check(BaseDevice::reply(req, req_length, mb_mapping), "Reply failed");
    }

    int reply_exception(const uint8_t* req, unsigned int exception_code) {
        return check(BaseDevice::reply_exception(req, exception_code), "Reply exception failed");
    }

    int report_slave_id(int max_dest, uint8_t* dest) {
        return check(BaseDevice::report_slave_id(max_dest, dest), "Report slave ID failed");
    }

    int send_raw_request(const uint8_t* raw_req, int raw_req_length) {
        return check(BaseDevice::send_raw_request(raw_req, raw_req_length),
                     "Send raw request failed");
    }
};

class RTUDevice : public Device {
public:
    explicit RTUDevice(const std::string& device = "/dev/ttyUSB0", int baud = 115200,
                       char parity = 'N', int data_bit = 8, int stop_bit = 1)
        : Device(modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit)) {}

    int get_serial_mode() {
        return check(modbus_rtu_get_serial_mode(ctx_.get()), "Get serial mode failed");
    }

    int set_serial_mode(int mode) {
        return check(modbus_rtu_set_serial_mode(ctx_.get(), mode), "Set serial mode failed");
    }

    int get_rts() {
        return check(modbus_rtu_get_rts(ctx_.get()), "Get RTS failed");
    }

    int set_rts(int mode) {
        return check(modbus_rtu_set_rts(ctx_.get(), mode), "Set RTS failed");
    }

    int set_custom_rts(void (*set_rts_cb)(modbus_t* ctx, int on)) {
        return check(modbus_rtu_set_custom_rts(ctx_.get(), set_rts_cb), "Set custom RTS failed");
    }

    int get_rts_delay() {
        return check(modbus_rtu_get_rts_delay(ctx_.get()), "Get RTS delay failed");
    }

    int set_rts_delay(int us) {
        return check(modbus_rtu_set_rts_delay(ctx_.get(), us), "Set RTS delay failed");
    }
};

class TCPDevice : public Device {
public:
    explicit TCPDevice(const std::string& ip, int port)
        : Device(modbus_new_tcp(ip.c_str(), port)) {}

    explicit TCPDevice(const std::string& node, const std::string& service)
        : Device(modbus_new_tcp_pi(node.c_str(), service.c_str())) {}

    int tcp_listen(int nb_connection) {
        return check(modbus_tcp_listen(ctx_.get(), nb_connection), "TCP listen failed");
    }

    int tcp_accept(int& socket) {
        return check(modbus_tcp_accept(ctx_.get(), &socket), "TCP accept failed");
    }
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

#ifdef MODBUSPP_HAS_EXPECTED
namespace exp {

class ModbusError {
public:
    ModbusError(int code) : code_(code) {}

    auto code() const noexcept {
        return code_;
    }

    auto message() const noexcept {
        return modbus::strerror(code_);
    }

private:
    int code_;
};

using Result = std::expected<int, ModbusError>;

class Device : public BaseDevice {
protected:
    static Result check(int rc) {
        if (rc == -1) {
            return std::unexpected(ModbusError(errno));
        }
        return rc;
    }

public:
    using BaseDevice::BaseDevice;

    Result flush() {
        return check(BaseDevice::flush());
    }

    Result set_slave(int slave_id) {
        return check(BaseDevice::set_slave(slave_id));
    }

    Result connect() {
        return check(BaseDevice::connect());
    }

    Result read_bits(int addr, int nb, uint8_t* dest) {
        return check(BaseDevice::read_bits(addr, nb, dest));
    }

    Result read_input_bits(int addr, int nb, uint8_t* dest) {
        return check(BaseDevice::read_input_bits(addr, nb, dest));
    }

    Result write_bit(int addr, bool status) {
        return check(BaseDevice::write_bit(addr, status));
    }

    Result write_bits(int addr, int nb, const uint8_t* data) {
        return check(BaseDevice::write_bits(addr, nb, data));
    }

    Result read_registers(int addr, int nb, uint16_t* dest) {
        return check(BaseDevice::read_registers(addr, nb, dest));
    }

    Result read_input_registers(int addr, int nb, uint16_t* dest) {
        return check(BaseDevice::read_input_registers(addr, nb, dest));
    }

    Result write_register(int addr, uint16_t value) {
        return check(BaseDevice::write_register(addr, value));
    }

    Result write_registers(int addr, int nb, const uint16_t* data) {
        return check(BaseDevice::write_registers(addr, nb, data));
    }

    Result write_and_read_registers(int write_addr, int write_nb, const uint16_t* src,
                                    int read_addr, int read_nb, uint16_t* dest) {
        return check(BaseDevice::write_and_read_registers(write_addr, write_nb, src, read_addr,
                                                          read_nb, dest));
    }

    Result mask_write_register(int addr, uint16_t and_mask, uint16_t or_mask) {
        return check(BaseDevice::mask_write_register(addr, and_mask, or_mask));
    }

    Result set_error_recovery(modbus_error_recovery_mode mode) {
        return check(BaseDevice::set_error_recovery(mode));
    }

    Result set_socket(int s) {
        return check(BaseDevice::set_socket(s));
    }

    Result set_response_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_response_timeout(sec, usec));
    }

    Result get_response_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_response_timeout(sec, usec));
    }

    Result set_byte_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_byte_timeout(sec, usec));
    }

    Result get_byte_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_byte_timeout(sec, usec));
    }

    Result set_indication_timeout(uint32_t sec, uint32_t usec) {
        return check(BaseDevice::set_indication_timeout(sec, usec));
    }

    Result get_indication_timeout(uint32_t* sec, uint32_t* usec) const {
        return check(BaseDevice::get_indication_timeout(sec, usec));
    }

    Result receive(uint8_t* req) {
        return check(BaseDevice::receive(req));
    }

    Result receive_confirmation(uint8_t* rsp) {
        return check(BaseDevice::receive_confirmation(rsp));
    }

    Result reply(const uint8_t* req, int req_length, modbus_mapping_t* mb_mapping) {
        return check(BaseDevice::reply(req, req_length, mb_mapping));
    }

    Result reply_exception(const uint8_t* req, unsigned int exception_code) {
        return check(BaseDevice::reply_exception(req, exception_code));
    }

    Result report_slave_id(int max_dest, uint8_t* dest) {
        return check(BaseDevice::report_slave_id(max_dest, dest));
    }

    Result send_raw_request(const uint8_t* raw_req, int raw_req_length) {
        return check(BaseDevice::send_raw_request(raw_req, raw_req_length));
    }
};

class RTUDevice : public Device {
public:
    explicit RTUDevice(const std::string& device = "/dev/ttyUSB0", int baud = 115200,
                       char parity = 'N', int data_bit = 8, int stop_bit = 1)
        : Device(modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit)) {}

    Result get_serial_mode() {
        return check(modbus_rtu_get_serial_mode(ctx_.get()));
    }

    Result set_serial_mode(int mode) {
        return check(modbus_rtu_set_serial_mode(ctx_.get(), mode));
    }

    Result get_rts() {
        return check(modbus_rtu_get_rts(ctx_.get()));
    }

    Result set_rts(int mode) {
        return check(modbus_rtu_set_rts(ctx_.get(), mode));
    }

    Result set_custom_rts(void (*set_rts_cb)(modbus_t* ctx, int on)) {
        return check(modbus_rtu_set_custom_rts(ctx_.get(), set_rts_cb));
    }

    Result get_rts_delay() {
        return check(modbus_rtu_get_rts_delay(ctx_.get()));
    }

    Result set_rts_delay(int us) {
        return check(modbus_rtu_set_rts_delay(ctx_.get(), us));
    }
};

class TCPDevice : public Device {
public:
    explicit TCPDevice(const std::string& ip, int port)
        : Device(modbus_new_tcp(ip.c_str(), port)) {}

    explicit TCPDevice(const std::string& node, const std::string& service)
        : Device(modbus_new_tcp_pi(node.c_str(), service.c_str())) {}

    Result tcp_listen(int nb_connection) {
        return check(modbus_tcp_listen(ctx_.get(), nb_connection));
    }

    Result tcp_accept(int& socket) {
        return check(modbus_tcp_accept(ctx_.get(), &socket));
    }
};

}  // namespace exp
#endif

}  // namespace modbus
