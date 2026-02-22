# Modbuspp

A modern header-only C++17/23 wrapper for [libmodbus](https://github.com/stephane/libmodbus).

## Usage Examples

RTU Master (Default: Exceptions)

Uses the default ExceptionPolicy. Throws modbus::ModbusException on failure.

```C++
#include <modbus.hpp>
#include <vector>
#include <iostream>

int main() {
    try {
        modbus::RTUDevice device("/dev/ttyUSB0", 19200, 'N', 8, 1);

        device.set_slave(1);
        device.connect();

        std::vector<uint16_t> buffer(10);

        // Read 10 registers starting at address 0
        device.read_registers(0, 10, buffer.data());

        for (auto val : buffer) {
            std::cout << "Val: " << val << std::endl;
        }
    } catch (const modbus::ModbusException& e) {
        std::cerr << "Modbus Error: " << e.what() << " (Code: " << e.code() << ")" << std::endl;
        return 1;
    }
    return 0;
}
```

TCP Master (C++23 std::expected)

Uses ExpectedPolicy to return std::expected<int, ModbusError> instead of throwing.

```C++
#include <modbus.hpp>
#include <vector>
#include <print>

// Use the ExpectedPolicy
using TCPDevice = modbus::TCPDevice<modbus::ExpectedPolicy>;

int main() {
    TCPDevice device("127.0.0.1", 502);

    if (auto res = device.connect(); !res) {
        std::println(stderr, "Connection failed: {}", res.error().message());
        return 1;
    }

    std::vector<uint16_t> buffer(10);

    auto result = device.read_registers(0, 10, buffer.data());
    if (result) {
        std::println("Read {} registers successfully.", *result);
    } else {
        std::println(stderr, "Read failed: {}", result.error().message());
    }

    return 0;
}
```

RTU Slave / Server

Shows how to use the Mapping class to handle requests.

```C++
#include <modbus.hpp>
#include <iostream>
#include <vector>

int main() {
    try {
        modbus::Device device("/dev/ttyUSB0", 19200, 'N', 8, 1);
        device.set_slave(1);
        device.connect();

        // Allocate mapping: 10 bits, 10 input bits, 10 regs, 10 input regs
        modbus::Mapping map(10, 10, 10, 10);

        // Access raw pointers to set values
        map.tab_registers()[0] = 0xABCD;

        std::cout << "Waiting for requests..." << std::endl;

        std::vector<uint8_t> query(MODBUS_RTU_MAX_ADU_LENGTH);

        // Blocking receive
        int rc = device.receive(query.data());

        if (rc > 0) {
            // Reply using the mapping
            device.reply(query.data(), rc, map.get());
            std::cout << "Replied to request." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```

## License

[MIT](https://spdx.org/licenses/MIT.html)
