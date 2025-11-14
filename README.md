# BS14 ESP32-S3 LCD Project - 4.3" Display

This is an optimized PlatformIO project for ESP32-S3 with 4.3" LCD display that features custom rotation system and touch mapping.

## Key Features

1. **BLE Library**: ESP32 BLE (BLEDevice, BLEServer, BLECharacteristic)
2. **Display**: Uses Arduino_GFX with RGB Parallel Interface (Arduino_RGB_Display)
3. **Touch**: Uses TouchDrvGT911 for 4.3" display
4. **Display Resolution**: 800x480 (4.3" LCD)

## Pin Configuration

**IMPORTANT**: The following pin definitions are configured for the ESP32-S3 4.3" Touch LCD display:

### Display Pins (RGB Parallel Interface)
- **RGB Panel**: 
  - DE=40, VSYNC=41, HSYNC=39, PCLK=42
  - R0~R4: 45, 48, 47, 21, 14
  - G0~G5: 5, 6, 7, 15, 16, 4
  - B0~B4: 8, 3, 46, 9, 1
- **Backlight**: GPIO 2

### Touch Panel (GT911)
- **SDA**: GPIO 8
- **SCL**: GPIO 9
- **IRQ**: GPIO 4

### LED Pins (active-low RGB)
```cpp
const int redled = 19;   // Avoids RGB panel conflicts
const int greenled = 20; // Avoids RGB panel conflicts
const int blueled = 18;  // OK - not used by RGB panel
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

## Building and Uploading

1. Install PlatformIO
2. Open this project in PlatformIO
3. Build: `pio run -e esp32-s3-devkitm-1`
4. Upload: `pio run -e esp32-s3-devkitm-1 --target upload`
5. Monitor: `pio device monitor -e esp32-s3-devkitm-1`

## BLE Compatibility

The BLE implementation uses ESP32 BLE APIs which are compatible with the Flutter app. The UUIDs match exactly:
- Service: `12345678-1234-1234-1234-123456789abc`
- Command Characteristic: `87654321-4321-4321-4321-cba987654321`
- Status Characteristic: `11011111-2222-3333-4444-555555555555`
- Lock Characteristic: `22222222-3333-4444-5555-666666666666`
- Sense Characteristic: `33333333-4444-5555-6666-777777777777`

## Notes

- LVGL version: 8.4.0
- Display resolution: 800x480 (4.3" LCD)
- Uses RGB parallel interface instead of SPI
- PSRAM enabled for larger display buffers
- All UI functionality from the original Arduino Giga sketch is preserved

## Hardware Requirements

- ESP32-S3 development board with PSRAM
- ESP32-S3 Touch LCD 4.3" display module
- Compatible with Waveshare-style 4.3" displays
