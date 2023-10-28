# Apple 32: A STM32F103VET6 implemented Apple I emulator with ChatGPT and WiFi support

# Software Mode
- Apple I emulator
- Apple I emulator with ChatGPT support
  - Directly use the onboard esp8266 Wifi to connect to ChatGPT API

# STM32 Onboard Key Functions
- K1: EMULATOR ROOT TERMINAL
- K2: TURBO MODE

# Video Output Methods
- VGA 
- Serial (Both to GPU and UART output)

# Hardware
- STM32F103VET6 (Emulating 6502 CPU)
  - 64KB SDRAM
  - Onboard esp8266 Wifi for ChatGPT support
- NodeMCU-12E board (Emulating 6551 ACIA, aka the GPU)
  - 128KB SDRAM
  - Retro style, e.g. Scanlines, CRT effect
- PS/2 keyboard
- VGA monitor
- SD Card (For loading ROMs)
- Onboard Buzzer
- 3D printed case

# Hardware Connection
- STM32F103VET6 <- UART & GPIO (Enable) -> NodeMCU-12E < - VGA -> VGA Monitor
- STM32F103VET6 <- PS/2 -> PS/2 Keyboard
- STM32F103VET6 <- SDIO -> SD Card
- STM32F103VET6 <- UART -> Onboard esp8266 Wifi (ChatGPT Server)
- STM32F103VET6 <- UART -> Serial Monitor (PC / ChatGPT Proxy)
- STM32F103VET6 <- GPIO -> Buzzer

# Memory Map (0x0000 - 0xFFFF) (64KB)
- 0x0000 - 0x8000: RAM
- 0xD010 - 0xD012: PIA
- 0xE000 - 0xEFFF: BASIC ROM
- 0xFF00 - 0xFFFF: Woz Monitor ROM

# Progress
- [ ] STM32 6502 Emulation (Debug with UART, Testing with loading Apple 1 ROMs) 
- [ ] Order VGA wire, VGA supported monitor and a PS/2 keyboard.
- [ ] PS/2 Input test and implement to STM32 as input device.
- [ ] NodeMCU GPU Emulation (Have some library, easier than 6502 Emulation)
- [ ] VGA testing with whole system
- [ ] Implement ChatGPT API via onboard esp8266
- [ ] Make boot screen looks pro
- [ ] SDIO for custom ROMs loading (maybe, depend on time)