# Touch Rotation Fix

## Problem Description
The touch coordinates were not working correctly for certain display rotations:
- 0° (Portrait): Working fine
- 90° (Landscape): Settings button doesn't work
- 180° (Portrait inverted): Working fine  
- 270° (Landscape): Completely wrong touch points

## Root Cause
The touch coordinate transformation logic in the `my_touchpad_read()` function was incorrectly mapping physical touch coordinates to logical screen coordinates for the rotated orientations.

## Fix Applied
Updated the coordinate transformation logic in `src/main.cpp`, lines ~353-380:

### Changes Made:

1. **90° Rotation (Landscape - clockwise)**:
   ```cpp
   // OLD (incorrect):
   logical_x = (int16_t)(lvgl_hor_res - 1 - ((physical_y * lvgl_hor_res) / LCD_VER_RES));
   logical_y = (int16_t)((physical_x * lvgl_ver_res) / LCD_HOR_RES);
   
   // NEW (correct):
   logical_x = LCD_VER_RES - 1 - physical_y;
   logical_y = physical_x;
   ```

2. **180° Rotation (Portrait inverted)**:
   ```cpp
   // OLD (incorrect):
   logical_x = physical_x; // No X inversion
   logical_y = physical_y; // No Y inversion
   
   // NEW (correct):
   logical_x = LCD_HOR_RES - 1 - physical_x;
   logical_y = LCD_VER_RES - 1 - physical_y;
   ```

3. **270° Rotation (Landscape - counter-clockwise)**:
   ```cpp
   // OLD (incorrect):
   logical_x = (int16_t)((physical_y * lvgl_hor_res) / LCD_VER_RES);
   logical_y = (int16_t)(lvgl_ver_res - 1 - ((physical_x * lvgl_ver_res) / LCD_HOR_RES));
   
   // NEW (correct):
   logical_x = physical_y;
   logical_y = LCD_HOR_RES - 1 - physical_x;
   ```

## How It Works
- **Physical coordinates**: Always range from (0,0) to (320,480) regardless of rotation
- **Logical coordinates**: Must be transformed to match the current LVGL coordinate system
- **Key insight**: Use direct coordinate mapping instead of scaling calculations

### Coordinate Mapping:
- **0°**: No transformation needed
- **90°**: Physical (x,y) → Logical (479-y, x)
- **180°**: Physical (x,y) → Logical (319-x, 479-y)  
- **270°**: Physical (x,y) → Logical (y, 319-x)

## Testing Instructions
1. Build and upload the firmware
2. Test touch responsiveness in each rotation:
   - Use the rotate button to cycle through orientations
   - Try pressing the settings button in each orientation
   - Test other UI elements (open/close buttons, switches)
   - Verify touch coordinates are accurate in all rotations

## Debug Mode
If issues persist, uncomment the debug output in the touch function (lines ~394-408) to see actual coordinate transformations in the serial monitor.

## Constants Used
- `LCD_HOR_RES = 320` (display width in portrait)
- `LCD_VER_RES = 480` (display height in portrait)

These values represent the physical display dimensions and are used as the basis for coordinate transformations.