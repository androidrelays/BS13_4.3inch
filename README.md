# BS14 ESP32-S3 PlatformIO Project

This is a PlatformIO project for ESP32-S3 that replicates the functionality of the Arduino Giga BS14_final_settings sketch.

## Key Changes from Arduino Giga Version

1. **BLE Library**: Converted from ArduinoBLE to ESP32 BLE (BLEDevice, BLEServer, BLECharacteristic)
2. **Display**: Uses Arduino_GFX with ST7796 driver instead of Arduino_H7_Video
3. **Touch**: Uses TouchDrvFT6X36 instead of Arduino_GigaDisplayTouch
4. **Display Resolution**: 320x480 (3.5" LCD) instead of Arduino Giga's display

## Pin Configuration

**IMPORTANT**: You need to adjust the following pin definitions in `src/main.cpp` to match your ESP32-S3 board:

### LED Pins (active-low RGB)
```cpp
const int redled = 4;   // Change to your RGB LED pins
const int greenled = 5; // Change to your RGB LED pins
const int blueled = 6;  // Change to your RGB LED pins
```

### Breaker Control Pins
```cpp
const int sense = 10;   // Sense pin A (default)
const int senseB = 11;  // Sense pin B (alternative)
const int pin39 = 12;
const int pin41 = 13;
const int openInput = 14;
const int closeInput = 15;
```

### Display Pins (already configured for 3.5" LCD)
```cpp
#define GFX_BL 6        // Backlight
#define SPI_MISO 2
#define SPI_MOSI 1
#define SPI_SCLK 5
#define LCD_DC 3
#define I2C_SDA 8
#define I2C_SCL 7
```

## Building and Uploading

1. Install PlatformIO
2. Open this project in PlatformIO
3. Adjust pin definitions in `src/main.cpp` to match your hardware
4. Build: `pio run`
5. Upload: `pio run --target upload`
6. Monitor: `pio device monitor`

## BLE Compatibility

The BLE implementation uses ESP32 BLE APIs which are compatible with the Flutter app. The UUIDs match exactly:
- Service: `12345678-1234-1234-1234-123456789abc`
- Command Characteristic: `87654321-4321-4321-4321-cba987654321`
- Status Characteristic: `11011111-2222-3333-4444-555555555555`
- Lock Characteristic: `22222222-3333-4444-5555-666666666666`
- Sense Characteristic: `33333333-4444-5555-6666-777777777777`

## Notes

- LVGL version: 9.3.0
- The display uses a 320x480 resolution (portrait mode by default, rotated 270 degrees)
- All UI functionality from the original Arduino Giga sketch is preserved

