#include <chrono>
#include <cstdint>
#include <cstdio>
#include <print>

#include "modbus.hpp"

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);

    const int n_loop = 100;
    const uint32_t slave_id = 1;

    try {
        modbus::Device client("/dev/ttyUSB1", 115200, 'N', 8, 1);
        client.set_slave(slave_id);
        client.connect();

        modbus::Mapping mapping(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);

        int nb_points;
        uint32_t bytes;

        // --- READ BITS TEST ---
        std::println("READ BITS\n");
        nb_points = MODBUS_MAX_READ_BITS;

        auto start_bits = std::chrono::steady_clock::now();
        for (int i = 0; i < n_loop; i++) {
            client.read_bits(0, nb_points, mapping.tab_bits());
        }
        auto end_bits = std::chrono::steady_clock::now();

        auto elapsed_bits =
            std::chrono::duration_cast<std::chrono::microseconds>(end_bits - start_bits);
        double ms_bits = elapsed_bits.count() / 1000.0;

        double rate_bits = (n_loop * nb_points) * 1000.0 / ms_bits;
        std::println("Transfer rate in points/seconds:");
        std::println("* {:.2f} points/s\n", rate_bits);

        bytes = n_loop * (nb_points / 8) + ((nb_points % 8) ? 1 : 0);
        double kib_s_bits = (bytes / 1024.0) * 1000.0 / ms_bits;
        std::println("Values:");
        std::println("* {} x {} values", n_loop, nb_points);
        std::println("* {:.3f} ms for {} bytes", ms_bits, bytes);
        std::println("* {:.2f} KiB/s\n", kib_s_bits);

        // --- READ REGISTERS TEST ---
        std::println("READ REGISTERS\n");
        nb_points = MODBUS_MAX_READ_REGISTERS;

        auto start_regs = std::chrono::steady_clock::now();
        for (int i = 0; i < n_loop; i++) {
            client.read_registers(0, nb_points, mapping.tab_registers());
        }
        auto end_regs = std::chrono::steady_clock::now();

        auto elapsed_regs =
            std::chrono::duration_cast<std::chrono::microseconds>(end_regs - start_regs);
        double ms_regs = elapsed_regs.count() / 1000.0;

        double rate_regs = (n_loop * nb_points) * 1000.0 / ms_regs;
        std::println("Transfer rate in points/seconds:");
        std::println("* {:.2f} registers/s\n", rate_regs);

        bytes = n_loop * nb_points * sizeof(uint16_t);
        double kib_s_regs = (bytes / 1024.0) * 1000.0 / ms_regs;
        std::println("Values:");
        std::println("* {} x {} values", n_loop, nb_points);
        std::println("* {:.3f} ms for {} bytes", ms_regs, bytes);
        std::println("* {:.2f} KiB/s\n", kib_s_regs);

    } catch (const std::exception& e) {
        std::println(stderr, "Error: {}", e.what());
        return -1;
    }

    return 0;
}
