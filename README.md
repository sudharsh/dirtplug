# dirtplug

dirtplug interprets analog-out from a capacitative moisture sensor and a uvb sensor on an esp32 RISC-V MCU.

## Build and Flash

- Prerequistes from [ESP IDF Template](https://github.com/esp-rs/esp-idf-template)

- ```sh
    cargo b && espflash /dev/ttyUSB0 target/riscv32imc-esp-espidf/debug/dirtplug
  ```