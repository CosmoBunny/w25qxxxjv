# w25qxxxjv

Rust driver for Winbond W25QxxJV SPI flash devices (W25Q32JV / W25Q64JV / W25Q128JV), built for no-std targets with `embedded-hal`.

---

## Features

- Standard, Fast, Dual, Quad-Output SPI read modes  
- 24-bit addressing on ≤128 Mbit devices  
- Page-aligned writes (256 B), sector/block/chip erase  
- Status-register protection (BP/SEC/TB/CMP) and individual locks (WPS=1)  
- Busy-polling of WIP bit after write/erase  
- Address bounds checking by model (Q32/Q64/Q128)  

---

## Usage

```rust
use w25qxxxjv::{
    W25QXXXJV, SpiSpeed, Model, Portion, Part
};
# // `spi`, `cs_pin`, and `timer` must implement embedded-hal traits
let mut w25q = W25QXXXJV::new(
    spi,
    pins.gpio13.into_push_pull_output(),
    SpiSpeed::Single,
    Model::Q128,
    &mut timer,
).unwrap();

// 1. Disable all protection (BP2:0 = 000, SEC = 0, TB = 0, CMP = 0)
w25q.protect_portion(Portion::Upper, Part::Zero, false).unwrap();

// 2. Write “Hello World” at address 0x0000
w25q.write_program(0x0000, b"Hello World").unwrap();

// 3. Read it back
let mut buf = [0u8; 11];
w25q.read_data(0x0000, &mut buf).unwrap();
assert_eq!(&buf, b"Hello World");
