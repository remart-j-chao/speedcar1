# STM32 smart-car firmware

## Build and flash

- Primary project: `MDK-ARM/micro_smartcar.uvprojx`, targeting `STM32F103C8`.
- Open it in Keil uVision5 and use **F7 / Build** and **F8 / Download**. Configure ST-Link or DAP-Link for SWD in uVision.
- A Windows command-line build is also possible when `uv4.exe` is installed on `PATH`:

  ```powershell
  uv4 -b MDK-ARM/micro_smartcar.uvprojx
  ```

  If `uv4.exe` is not on `PATH`, invoke `C:\Keil_v5\UV4\UV4.exe` explicitly. The resulting image is written under `MDK-ARM/micro_smartcar/`.
- Clean generated Keil files with `MDK-ARM/清除缓存文件.bat`.
- USART3 at 115200 8N1 is the debug output.
- There is no unit-test framework, host-side test suite, or configured lint command in this repository.

## Memory and safety

- The default build is close to the MDK-Lite 32 KB Flash limit. Keep `GYRO_DEBUG` in mind: floating-point `printf` support can add enough code to cause linker error L6047U. Test linker size after sensor or library changes.
- Negative-pressure fan startup is hardware-relevant and potentially hazardous. Preserve the two-stage ESC startup sequence and leave `IF_start_Brushless_motor` disabled while developing unless the motor hardware is intentionally being tested.
- `printf` in `fputc()` is blocking UART transmission.

## Architecture and data flow

- The application starts in `Core/Src/main.c`. Generated `MX_*_Init()` functions configure HAL peripherals; application startup and the main polling loop are kept inside CubeMX `USER CODE` sections.
- CubeMX configuration is authoritative for the generated peripheral setup:
  - `micro_smartcar.ioc` records the STM32CubeMX project and pin/peripheral configuration.
  - `Core/Src/stm32f1xx_hal_msp.c` contains HAL pin, DMA, and clock setup.
  - `Core/Src/stm32f1xx_it.c` contains vector dispatch and IRQ handlers.
- Run CubeMX regeneration with care because generated files contain custom application code. Keep user additions inside `USER CODE BEGIN/END` markers and retain the custom ESC block in `main.c`; regenerate only after preserving changes for manual merge.
- BMI270 has three layers:
  - `Core/code/bmi2.c`, `bmi270.c`, and their headers are the Bosch sensor API; do not hand-edit them for board integration.
  - `Core/code/bmi270_port.c` adapts the API to SPI1 + DMA, software CS on PA4, HAL callbacks, and DWT microsecond delay.
  - `Core/code/dodo_BMI270.c` is the application-facing API. Its globals and conversion macros expose raw IMU samples and physical units.
- The multiplexer path is `Core/code/multiplexer.c`. `MUX_get_value()` scans channels 0..11 and packs channel state into a `uint16_t`; channel extraction is intentionally reversed in the bit field by `MUX_GET_CHANNEL()`.
- ESC output uses TIM1 update DMA rather than another PWM channel. TIM1 provides 10 kHz update events; DMA1_Channel5 writes PA11's `BSRR` value from a 200-slot table, producing a 50 Hz frame. Frame changes are synchronized through the DMA completion callback in `main.c`.

## Peripherals and pin conventions

- TIM1:
  - Channel 1/2 drive the two motor PWM outputs on PA8/PA9.
  - DMA-generated update waveform drives the brushless ESC on PA11.
- BMI270 SPI1 is on PA4 CS, PA5 SCK, PA6 MISO, and PA7 MOSI. Use DMA1_Channel2 for RX and DMA1_Channel3 for TX.
- The 12-channel multiplexer uses PA12 as the read input and PA15/PB3/PB8/PB9 as address outputs.
- TIM3 encoder inputs PB4/PB5 are the left wheel; TIM4 encoder inputs PB6/PB7 are the right wheel.
- TIM2 is configured as a 1 kHz base timer, but its periodic application callback is not yet implemented.
- USART3 PB10/PB11 is the only debug UART.
- `GPIO_RUN` is the logical name for PB14.

## Code conventions specific to this project

- Keep CubeMX-generated HAL configuration in the generated initialization functions. Put application logic in CubeMX `USER CODE` regions and synchronize non-generated pin names with `micro_smartcar.ioc`.
- Do not bypass the BOSCH driver interface. Board-level SPI reads/writes belong in `bmi270_port.c`, while initialization and sample access belong in `dodo_BMI270.c`.
- BMI270 physical-unit conversions must stay synchronized with sensor configuration. The current conversion factors are acceleration `4096 LSB/g` and gyroscope `16.4 LSB/(deg/s)` for the configured ±8 g / ±2000 dps ranges; changing range requires changing both configuration and `BMI270_transition_factor`.
- After any sensor-range change, update the corresponding conversion macro use before exposing converted values.
- MUX channel numbering is left-to-right 0..11 and `MUX_GET_CHANNEL()` maps channel 0 to bit 11. Preserve this packing and extraction convention at call sites.
- SPI and multiplexer timing-sensitive delays are based on `DWT->CYCCNT`; updates must initialize and use the DWT counter consistently.
- HAL DMA completion must be matched to the correct peripheral and channel before setting completion state. SPI callbacks in `bmi270_port.c` must distinguish SPI1 transactions shared by DMA RX and TX.
- The DMA table for PA11 must be edited carefully: an ESC frame is 200 slots of 100 us, and width changes must not overwrite slots currently being consumed by DMA.
