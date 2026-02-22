#include <exception>
#include <print>

#include "modbus.hpp"

int main() {
    try {
        modbus::RTUDevice slave("/dev/ttyUSB0", 115200, 'N', 8, 1);

        slave.set_slave(1);
        slave.connect();

        modbus::Mapping mapping(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);

        std::println("RTU Slave started on /dev/ttyUSB0 (ID: 1)...");

        for (;;) {
            uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
            int rc = slave.receive(query);

            if (rc > 0) {
                slave.reply(query, rc, mapping.get());
            } else {
                break;
            }
        }

    } catch (const modbus::ModbusException& e) {
        std::println(stderr, "Modbus Error: {} (Code: {})", e.what(), e.code());
        return -1;
    } catch (const std::exception& e) {
        std::println(stderr, "Standard Error: {}", e.what());
        return -1;
    }

    return 0;
}
