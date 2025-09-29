# AXS5106L Touch Controller Driver

A `no_std` Rust driver for the AXS5106L capacitive touch controller, designed 
for embedded systems.

This driver was ported from the original C code found in the AXS5106L Arduino 
library as provided by the example on Waveshare's website for the 
[ESP32-C6 1.47inch Touch Display Development Board](https://www.waveshare.com/esp32-c6-touch-lcd-1.47.htm).

## Features

- ✅ Support for up to 5 simultaneous touch points
- ✅ Screen rotation handling (0°, 90°, 180°, 270°) 
- ✅ I2C communication using `embedded-hal` traits
- ✅ Proper error handling with detailed error types
- ✅ `no_std` compatible for embedded systems
- ✅ Safe Rust with minimal `unsafe` code

## Hardware Requirements

- AXS5106L touch controller IC
- I2C interface
- GPIO pin for reset control
- Optional GPIO pin for interrupt detection

## Usage

### Basic Example

**Default Pin Mapping in the ESP32-C6 1.47inch Touch Display Development Board:**

| AXS5106L Pin | ESP32-C6 Pin | Description          |
|--------------|--------------|----------------------|
| SDA          | GPIO 18      | I2C Data             |
| SCL          | GPIO 19      | I2C Clock            |
| RST          | GPIO 20      | Reset                |
| INT          | GPIO 21      | Interrupt (optional) |


```rust ignore
use esp_idf_svc::hal::{delay::Delay, gpio::PinDriver, i2c, peripherals};
use axs5106l::{Axs5106l, Rotation};

let mut delay = Delay::new_default(); 

let peripherals = peripherals::Peripherals::take()?;
let _i2c = peripherals.i2c0;
let sda = peripherals.pins.gpio18;
let scl = peripherals.pins.gpio19;
let reset_pin = PinDriver::output(peripherals.pins.gpio20)?;

let config = i2c::config::Config::default();
let i2c = i2c::I2cDriver::new(_i2c, sda, scl, &config)?;

// Create the driver instance
let mut touch_driver = Axs5106l::new(
    i2c,               // Your I2C interface
    reset_pin,         // Your reset GPIO pin
    172,               // Display width
    320,               // Display height
    Rotation::Rotate0, // Screen rotation
);

// Initialize the touch controller
touch_driver.init(&mut delay)?;

// Poll for touch data
loop {
  if let Some(touches) = touch_driver.read_touch_data()? {
      for touch in touches.points.iter() {
          println!("Touch X: {}, Y: {}", touch.x, touch.y);
      }
  }
}
```

### With Interrupt Handling

For interrupt-driven applications, you can use the driver with your platform's interrupt system:

#### Interrupt handling considerations

The original C code uses Arduino's `attachInterrupt()` to handle touch events
via a falling edge interrupt. In embedded Rust, interrupt handling is more
platform-specific and typically requires:

1. **Platform-specific interrupt setup**: Each HAL (Hardware Abstraction Layer)
   has its own way to configure interrupts (e.g., `stm32f4xx-hal`, `esp-idf-hal`)

2. **Shared state management**: Use `Mutex<RefCell<T>>` or atomic types for
   sharing data between interrupt handlers and main code

3. **RTIC or embassy for async**: Consider using RTIC (Real-Time Interrupt-driven
   Concurrency) or embassy for more sophisticated interrupt-driven applications

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

_Vibe-coded by Claude in 20 minutes, there may be bugs. If you find any, please report them._
