

**Goals:**
- Build a complete drivers from scratch using:
- Custom register-level drivers (GPIO, UART, ADC, SPI, I2C, SPI)
- No ST HAL libraries (pure CMSIS + bare-metal)
- Modular API design for reusability
- 

---

**-Feature1_addition**

- **System Clock:** HSI 48MHz configuration via RCC registers
- **SysTick Timer:** 1ms timebase with overflow-safe delays
- **GPIO Driver:** Pin mode, alternate function, read/write
- **SPI Driver:** Possible APIs of read write, Interrupts and status,flags handling
-  **UART Driver:** 115200 baud with printf redirection
- **ADC Driver:** Internal temperature sensor with factory calibration
