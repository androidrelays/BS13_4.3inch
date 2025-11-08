# ESP32-S3 LCD Display Rotations and Coordinate System Guide

## Display Physical Properties
- **Display Size**: 3.5 inch LCD
- **Physical Resolution**: 320x480 pixels
- **Default Orientation**: Portrait (320 wide × 480 tall)

## Coordinate System Basics

### Physical Coordinates (Touch Sensor)
The touch sensor always reports coordinates in the same physical orientation:
- **X-axis**: 0 to 319 (left to right, 320 pixels wide)
- **Y-axis**: 0 to 479 (top to bottom, 480 pixels tall)
- **Origin (0,0)**: Always at the physical top-left corner of the display
- **Max point (319,479)**: Always at the physical bottom-right corner

### Logical Coordinates (LVGL/Software)
LVGL uses logical coordinates that change based on the current rotation:
- These are the coordinates that UI elements use
- The coordinate system rotates with the display
- Origin and maximum values change with rotation

## The Four Rotations Explained

### 🔄 Rotation 0° (Portrait - Default)
```
┌─────────────┐  ← Physical top (Y=0)
│    (0,0)    │
│      ↓      │
│             │
│             │
│             │
│             │
│      ↑      │
│  (319,479)  │
└─────────────┘  ← Physical bottom (Y=479)
↑             ↑
X=0         X=319
```
- **Orientation**: Portrait (tall)
- **Logical Resolution**: 320×480
- **Coordinate Mapping**: No transformation needed
  - Touch (x,y) → Screen (x,y)
- **Use Case**: Default mobile-like interface

### 🔄 Rotation 90° (Landscape - Clockwise)
```
Physical Left    Physical Right
     ↓                ↓
┌─────────────────────┐  ← Physical top (now left side)
│(0,0)           (479,0)│
│                     │
│                     │
│(0,319)       (479,319)│
└─────────────────────┘  ← Physical bottom (now right side)
```
- **Orientation**: Landscape (wide)
- **Logical Resolution**: 480×320 (width×height swapped)
- **Coordinate Mapping**: 
  - Touch (x,y) → Screen (479-y, 319-x)
- **Visual Effect**: Display rotated 90° clockwise
- **Use Case**: Landscape interfaces, games

### 🔄 Rotation 180° (Portrait - Inverted)
```
┌─────────────┐  ← Physical top (now bottom of UI)
│  (319,479)  │
│      ↑      │
│             │
│             │
│             │
│             │
│      ↓      │
│    (0,0)    │
└─────────────┘  ← Physical bottom (now top of UI)
```
- **Orientation**: Portrait (tall) - upside down
- **Logical Resolution**: 320×480
- **Coordinate Mapping**: 
  - Touch (x,y) → Screen (319-x, y)
- **Visual Effect**: Display flipped 180°
- **Use Case**: Inverted mounting scenarios

### 🔄 Rotation 270° (Landscape - Counter-clockwise)
```
Physical Left    Physical Right
     ↓                ↓
┌─────────────────────┐  ← Physical top (now right side of UI)
│(479,319)       (0,319)│
│                     │
│                     │
│(479,0)           (0,0)│
└─────────────────────┘  ← Physical bottom (now left side of UI)
```
- **Orientation**: Landscape (wide)
- **Logical Resolution**: 480×320
- **Coordinate Mapping**: 
  - Touch (x,y) → Screen (y, x)
- **Visual Effect**: Display rotated 90° counter-clockwise
- **Use Case**: Alternative landscape orientation

## Touch Coordinate Transformations

### Why Transformation is Needed
The touch sensor always reports physical coordinates, but LVGL expects logical coordinates that match the current rotation. We need to transform between these coordinate systems.

### Current Implementation in Code
```cpp
switch (currentRotation) {
  case 0:   // Portrait
    logical_x = physical_x;
    logical_y = physical_y;
    break;
    
  case 90:  // Landscape (90° clockwise)
    logical_x = LCD_VER_RES - 1 - physical_y;  // 479 - y
    logical_y = LCD_HOR_RES - 1 - physical_x;  // 319 - x
    break;
    
  case 180: // Portrait inverted
    logical_x = LCD_HOR_RES - 1 - physical_x;  // 319 - x
    logical_y = physical_y;
    break;
    
  case 270: // Landscape (90° counter-clockwise)
    logical_x = physical_y;
    logical_y = physical_x;
    break;
}
```

## Quick Reference Table

| Rotation | Orientation | LVGL Resolution | Touch Transform | Visual Description |
|----------|-------------|-----------------|-----------------|-------------------|
| 0°       | Portrait    | 320×480        | (x,y) → (x,y)   | Normal phone orientation |
| 90°      | Landscape   | 480×320        | (x,y) → (479-y, 319-x) | Rotated clockwise |
| 180°     | Portrait    | 320×480        | (x,y) → (319-x, y) | Upside down |
| 270°     | Landscape   | 480×320        | (x,y) → (y, x)  | Rotated counter-clockwise |

## Common Issues and Debugging

### If Touch is Not Working in a Rotation:
1. Check if X coordinates are inverted (left/right swapped)
2. Check if Y coordinates are inverted (up/down swapped)
3. Check if X and Y axes are swapped
4. Enable debug output to see coordinate transformations

### Debug Output Format:
```
Touch - Rotation: 90°, Physical: (150,200), Logical: (279,169), LVGL res: 480x320
```
- **Physical**: What the touch sensor reports
- **Logical**: What gets sent to LVGL
- **LVGL res**: Current logical screen resolution

This should help you understand how the coordinate transformations work and troubleshoot any remaining touch issues!