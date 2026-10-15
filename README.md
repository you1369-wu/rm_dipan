# rm_imu

RoboMaster C board firmware project for IMU, remote control, CAN motor control,
PID training, LED, buzzer, and serial plotting tasks.

The project targets STM32F407 and is generated from STM32CubeMX, then extended
with C/C++ application code and the `sp_middleware` submodule.

## Features

- Read BMI088 IMU data over SPI.
- Print IMU and debug data through USART1.
- Receive DBUS remote-control data.
- Control RoboMaster motors over CAN.
- Run PID training and plotting tasks.
- Drive board LED and buzzer feedback.

## Project Layout

```text
applications/          User application tasks and C++ control logic
Core/                  STM32CubeMX generated core files
Drivers/               STM32 HAL and CMSIS files
Middlewares/           FreeRTOS and STM32 middleware files
cmake/                 CMake toolchain and CubeMX CMake files
sp_middleware/         Shared middleware submodule
CMakeLists.txt         Top-level build target and source list
CMakePresets.json      Debug and Release CMake presets
rm_imu.ioc             STM32CubeMX project file
```

## Environment

Recommended tools:

- STM32CubeMX / STM32CubeCLT or STM32Cube VS Code extension
- CMake 3.22 or newer
- Ninja
- GNU Arm Embedded Toolchain, usually installed by STM32Cube
- Git with submodule support

## First Setup

Clone the repository with submodules:

```powershell
git clone --recursive https://github.com/you1369-wu/rm.git
cd rm
```

If the repository has already been cloned, initialize the middleware submodule:

```powershell
git submodule update --init --recursive
```

This step is required because files such as
`sp_middleware/io/bmi088/bmi088.cpp` are not stored directly in this repository.

## Build

Configure and build with the Debug preset:

```powershell
cmake --preset Debug
cmake --build build/Debug
```

Or build from VS Code with the STM32/CMake extension using the `Debug` preset.

Release build:

```powershell
cmake --preset Release
cmake --build build/Release
```

Do not commit `build/`; regenerate build files locally when needed.

## CubeMX Resources

- SPI1: BMI088 communication bus.
- PA4: accelerometer chip select, GPIO output, default high.
- PB0: gyroscope chip select, GPIO output, default high.
- USART1: serial output, 115200-8-N-1.
- PB6: USART1_TX.
- PB7: USART1_RX.
- TIM10_CH1 / PF6: optional BMI088 heater PWM.

## Development Notes

- BMI088 chip-select pins are active low.
- Keep the SPI baud rate below 10 MHz. With PCLK2 at 84 MHz, SPI prescaler 16
  gives 5.25 MHz.
- When adding new `.c` or `.cpp` files, add them to `target_sources()` in
  `CMakeLists.txt`.
- Keep application headers under `applications/` or another included path.
- Keep shared drivers and reusable code in `sp_middleware` when they are meant
  to be reused across projects.

## Troubleshooting

### CMake cannot find `sp_middleware` source files

Run:

```powershell
git submodule update --init --recursive
```

If the submodule URL was changed, also run:

```powershell
git submodule sync --recursive
git submodule update --init --recursive
```

### Build fails with missing `cc1.exe`

If the compiler prints an error like:

```text
fatal error: cannot execute .../cc1.exe: CreateProcess: No such file or directory
```

the STM32Cube GNU toolchain installation is incomplete or damaged. Reinstall the
GNU Tools for STM32 package from STM32Cube, or delete the damaged toolchain
folder under:

```text
C:\Users\<user>\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\
```

Then reopen VS Code or STM32Cube and let the toolchain download again.
