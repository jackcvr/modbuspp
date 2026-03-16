#include <cstdlib>
#include <exception>
#include <print>

#include "modbus.hpp"

int main() {
    try {
        modbus::RTUDevice<modbus::ExpectedPolicy> slave("/dev/ttyUSB0", 115200, 'N', 8, 1);

        if (auto res = slave.set_slave(1); !res) {
            std::println(stderr, "Modbus Error: Set slave failed: {} (Code: {})",
                         res.error().message(), res.error().code());
            return EXIT_FAILURE;
        }

        if (auto res = slave.connect(); !res) {
            std::println(stderr, "Modbus Error: Connect failed: {} (Code: {})",
                         res.error().message(), res.error().code());
            return EXIT_FAILURE;
        }

        // Mapping constructor still throws std::system_error on allocation failure
        modbus::Mapping mapping(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);

        std::println("RTU Slave started on /dev/ttyUSB0 (ID: 1)...");

        for (;;) {
            uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
            auto res = slave.receive(query);

            if (!res) {
                std::println(stderr, "Modbus Error: Receive failed: {} (Code: {})",
                             res.error().message(), res.error().code());
                break;
            }

            int rc = *res;
            if (rc > 0) {
                if (auto rep_res = slave.reply(query, rc, mapping.get()); !rep_res) {
                    std::println(stderr, "Modbus Error: Reply failed: {} (Code: {})",
                                 rep_res.error().message(), rep_res.error().code());
                    break;
                }
            } else {
                break;
            }
        }

    } catch (const std::exception& e) {
        std::println(stderr, "Standard Error: {}", e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
