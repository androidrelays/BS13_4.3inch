//lvgl 8.4 - Adapted for ESP32-S3
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>
#include <stdint.h>
#include "TCA9554.h"
#include "TouchDrvFT6X36.hpp"
#include "REG/FT6X36Constants.h"

// Display configuration - ESP32-S3 3.5" LCD
#define GFX_BL 6
#define SPI_MISO 2
#define SPI_MOSI 1
#define SPI_SCLK 5
#define LCD_CS -1
#define LCD_DC 3
#define LCD_RST -1
#define LCD_HOR_RES 320
#define LCD_VER_RES 480
#define I2C_SDA 8
#define I2C_SCL 7

// LED pins (active-low RGB) - UPDATED TO AVOID CONFLICTS
const int redled = 16;   // Changed from 4 to avoid conflicts
const int greenled = 17; // Changed from 5 (was conflicting with SPI_SCLK)
const int blueled = 18;  // Changed from 6 (was conflicting with GFX_BL)

// Breaker control pins - ADJUST THESE FOR YOUR ESP32-S3 BOARD
const int sense = 10;   // Sense pin A (default)
const int senseB = 11;  // Sense pin B (alternative)
const int pin39 = 12;
const int pin41 = 13;
const int openInput = 14;
const int closeInput = 15;

// BLE UUIDs matching Flutter app expectations
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define COMMAND_CHAR_UUID "87654321-4321-4321-4321-cba987654321"
#define STATUS_CHAR_UUID "11011111-2222-3333-4444-555555555555"
#define LOCK_CHAR_UUID "22222222-3333-4444-5555-666666666666"
#define SENSE_CHAR_UUID "33333333-4444-5555-6666-777777777777"

// BLE objects
BLEServer *pServer = NULL;
BLECharacteristic *pCommandChar = NULL;
BLECharacteristic *pStatusChar = NULL;
BLECharacteristic *pLockChar = NULL;
BLECharacteristic *pSenseChar = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Display and touch objects
TCA9554 TCA(0x20);
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, SPI_SCLK, SPI_MOSI, SPI_MISO);
// Start with rotation 0, LVGL will handle rotation
Arduino_GFX *gfx = new Arduino_ST7796(bus, LCD_RST, 0, true, LCD_HOR_RES, LCD_VER_RES);
TouchDrvFT6X36 touch;

// LVGL display buffers
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;
lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;
lv_disp_t *disp = NULL;

// Sense selection state
bool settingsLoaded = false;
bool senseA_selected = true;

// UI objects (same as original)
lv_obj_t *switch_69;
lv_obj_t *ui_container;
lv_obj_t *tight_container;
lv_obj_t *switch_container;
lv_obj_t *vertical_switch_container = NULL; // Added for dynamic resize access
lv_obj_t *btn_open;
#ifndef LV_SYMBOL_LOCK
#define LV_SYMBOL_LOCK "\xef\x80\xa3"
#endif
lv_obj_t *btn_close;
lv_obj_t *close_btn_overlay;
lv_obj_t *open_btn_overlay;
lv_obj_t *lock_icon_btn = NULL;
lv_obj_t *lock_icon_label = NULL;
lv_obj_t *lock_container = NULL;
lv_obj_t *btn_settings = NULL;
lv_obj_t *settings_modal = NULL;
lv_obj_t *btn_sense_a = NULL;
lv_obj_t *btn_sense_b = NULL;
lv_obj_t *top_bar = NULL;
lv_obj_t *top_bar_bt_status = NULL;
lv_obj_t *btn_rotate = NULL;

// State variables
bool breakerstate = true;
bool locked = false;
bool switchToggled = true;
unsigned long lock_press_start = 0;
unsigned long last_switch_toggle = 0;
unsigned long lock_button_restore_time = 0;
bool bluetoothConnected = false;
unsigned long lastStatusSent = 0;
int currentRotation = 0;

// Function declarations
static void set_leds();
static void set_breaker_state(bool open);
static void open_btn_cb(lv_event_t *e);
static void close_btn_cb(lv_event_t *e);
static void switch_toggled_cb(lv_event_t *e);
static void update_button_styles();
static void rotate_screen_cb(lv_event_t *e);
static void send_status_to_flutter();
static void lock_icon_event_cb(lv_event_t *e);
static void update_lock_icon();
static void settings_btn_cb(lv_event_t *e);
static void sense_a_btn_cb(lv_event_t *e);
static void sense_b_btn_cb(lv_event_t *e);
static void close_settings_cb(lv_event_t *e);
static void disconnect_bluetooth_cb(lv_event_t *e);
static void create_ui();
static void create_top_bar();
static void update_top_bar_bt_status();

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      bluetoothConnected = true;
      update_button_styles();
      update_lock_icon();
      set_leds();
      update_top_bar_bt_status();
      
      delay(800);
      
      uint8_t currentStatus[3] = {
        (uint8_t)(breakerstate ? 1 : 0),
        (uint8_t)(switchToggled ? 1 : 0),
        (uint8_t)(locked ? 1 : 0)
      };
      pStatusChar->setValue(currentStatus, 3);
      pStatusChar->notify();
      
      uint8_t currentLock = locked ? 1 : 0;
      pLockChar->setValue(&currentLock, 1);
      pLockChar->notify();
      
      uint8_t currentSense = senseA_selected ? 0 : 1;
      pSenseChar->setValue(&currentSense, 1);
      pSenseChar->notify();
      
      send_status_to_flutter();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      bluetoothConnected = false;
      update_button_styles();
      update_lock_icon();
      set_leds();
      update_top_bar_bt_status();
      BLEDevice::startAdvertising();
    }
};

// BLE Characteristic callbacks
class CommandCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() >= 3) {
        bool newBreakerState = (uint8_t)value[0] == 1;
        bool newSwitchState  = (uint8_t)value[1] == 1;
        bool newLockState    = (uint8_t)value[2] == 1;

        bool stateChanged = false;

        if (locked != newLockState) {
          locked = newLockState;
          uint8_t val = locked ? 1 : 0;
          pLockChar->setValue(&val, 1);
          pLockChar->notify();
          update_lock_icon();
          update_button_styles();
          stateChanged = true;
        }

        if (switchToggled != newSwitchState) {
          switchToggled = newSwitchState;
          if (switch_69) {
            if (switchToggled) {
              // UP position - adjust based on orientation and switch size
              if (currentRotation == 90 || currentRotation == 270) {
                lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px more in landscape: Y changed from -3 to -2
              } else {
                lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px: Y changed from -3 to -2
              }
              lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0);
            } else {
              // DOWN position - adjust based on orientation and switch size
              if (currentRotation == 90 || currentRotation == 270) {
                lv_obj_set_pos(switch_69, -2, 42); // Moved down 1px more in landscape: Y changed from 41 to 42
              } else {
                lv_obj_set_pos(switch_69, -2, 38); // Moved down 1px: Y changed from 37 to 38
              }
              lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0);
            }
          }
          update_button_styles();
          stateChanged = true;
        }

        if (!locked && newBreakerState != breakerstate) {
          set_breaker_state(newBreakerState);
          update_button_styles();
          stateChanged = true;
        }

        if (stateChanged) {
          send_status_to_flutter();
          update_button_styles();
          set_leds();
        }
      }
    }
};

class LockCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() > 0) {
        uint8_t newVal = (uint8_t)value[0];
        bool newLockState = (newVal == 1);
        
        static unsigned long lastLockWrite = 0;
        unsigned long currentTime = millis();
        
        if (locked != newLockState && (currentTime - lastLockWrite > 300)) {
          locked = newLockState;
          update_lock_icon();
          update_button_styles();
          send_status_to_flutter();
          lastLockWrite = currentTime;
        }
      }
    }
};

class SenseCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() > 0) {
        uint8_t newVal = (uint8_t)value[0];
        bool newSenseA = (newVal == 0);
        
        if (senseA_selected != newSenseA) {
          senseA_selected = newSenseA;
          
          if (btn_sense_a && btn_sense_b) {
            lv_obj_set_style_bg_color(btn_sense_a, senseA_selected ? lv_color_hex(0x00AA00) : lv_color_hex(0xCCCCCC), 0);
            lv_obj_set_style_bg_color(btn_sense_b, senseA_selected ? lv_color_hex(0xCCCCCC) : lv_color_hex(0x00AA00), 0);
          }
          
          set_breaker_state(breakerstate);
        }
      }
    }
};

// Display flushing
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp_drv);
}

// Touch read
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  int16_t x[1], y[1];
  uint8_t touched = touch.getPoint(x, y, 1);

  if (touched) {
    data->state = LV_INDEV_STATE_PR;
    
    // Manual coordinate transformation for custom rotation system
    // Physical coordinates from touch controller (always 320x480 orientation)
    int16_t physical_x = x[0];
    int16_t physical_y = y[0];
    int16_t screen_x, screen_y;
    
    // Transform coordinates to match the current screen orientation
    switch (currentRotation) {
      case 0: // Portrait (0°) - direct mapping
        screen_x = physical_x;
        screen_y = physical_y;
        break;
        
      case 90: // Landscape (90° clockwise)
        // Physical (320x480) -> Screen (480x320)
        screen_x = physical_y;
        screen_y = LCD_HOR_RES - 1 - physical_x;
        break;
        
      case 180: // Portrait inverted (180°)
        // Physical (320x480) -> Screen (320x480) but flipped
        screen_x = LCD_HOR_RES - 1 - physical_x;
        screen_y = LCD_VER_RES - 1 - physical_y;
        break;
        
      case 270: // Landscape (270° clockwise)
        // Physical (320x480) -> Screen (480x320)
        screen_x = LCD_VER_RES - 1 - physical_y;
        screen_y = physical_x;
        break;
        
      default:
        screen_x = physical_x;
        screen_y = physical_y;
        break;
    }
    
    // Set the screen coordinates directly
    data->point.x = screen_x;
    data->point.y = screen_y;
    
    // Touch debug output removed for performance
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void lcd_reset(void) {
  TCA.write1(1, 1);
  delay(10);
  TCA.write1(1, 0);
  delay(10);
  TCA.write1(1, 1);
  delay(200);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32-S3 BS14 Starting...");

  senseA_selected = true;
  settingsLoaded = true;

  // Initialize I2C and TCA9554
  Wire.begin(I2C_SDA, I2C_SCL);
  TCA.begin();
  TCA.pinMode1(1, OUTPUT);
  lcd_reset();

  // Initialize touch
  if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS)) {
    Serial.println("Failed to find FT6X36 - check your wiring!");
    while (1) delay(1000);
  }

  // Initialize display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(RGB565_BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  // Initialize LVGL
  lv_init();
  
  // Display is 320x480 - use these dimensions for LVGL
  screenWidth = LCD_HOR_RES;  // 320
  screenHeight = LCD_VER_RES; // 480
  
  bufSize = screenWidth * 120;

  disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
  disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, bufSize);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;  // 320
  disp_drv.ver_res = screenHeight; // 480
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp = lv_disp_drv_register(&disp_drv);
  
  Serial.print("LVGL display registered: ");
  Serial.print(disp_drv.hor_res);
  Serial.print(" x ");
  Serial.println(disp_drv.ver_res);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // Initialize BLE
  BLEDevice::init("BS14");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandChar->setCallbacks(new CommandCharCallbacks());

  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());

  pLockChar = pService->createCharacteristic(
    LOCK_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pLockChar->addDescriptor(new BLE2902());
  pLockChar->setCallbacks(new LockCharCallbacks());

  pSenseChar = pService->createCharacteristic(
    SENSE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pSenseChar->addDescriptor(new BLE2902());
  pSenseChar->setCallbacks(new SenseCharCallbacks());

  // Set initial values
  uint8_t initialStatus[3] = {(uint8_t)(breakerstate ? 1 : 0), (uint8_t)(switchToggled ? 1 : 0), (uint8_t)(locked ? 1 : 0)};
  pStatusChar->setValue(initialStatus, 3);
  uint8_t initialLock = locked ? 1 : 0;
  pLockChar->setValue(&initialLock, 1);
  uint8_t initialSense = senseA_selected ? 0 : 1;
  pSenseChar->setValue(&initialSense, 1);

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");

  // Initialize pins
  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(sense, OUTPUT);
  pinMode(senseB, OUTPUT);
  pinMode(pin39, OUTPUT);
  pinMode(pin41, OUTPUT);
  pinMode(openInput, INPUT);
  pinMode(closeInput, INPUT);

  digitalWrite(redled, HIGH);
  digitalWrite(greenled, HIGH);
  digitalWrite(blueled, HIGH);
  digitalWrite(pin39, LOW);
  digitalWrite(pin41, LOW);

  breakerstate = true;
  if (senseA_selected) {
    digitalWrite(sense, LOW);
    digitalWrite(senseB, HIGH);
  } else {
    digitalWrite(sense, HIGH);
    digitalWrite(senseB, LOW);
  }

  set_leds();

  currentRotation = 0;
  
  // GFX driver in default orientation (rotation 0)
  gfx->setRotation(0);
  
  disp = lv_disp_get_default();
  // Keep LVGL in default orientation
  lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

  create_ui();
  create_top_bar();
  update_lock_icon();
  
  // Default portrait layout positioning
  lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15);
  lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 115);
  if (lock_icon_btn) {
    lv_obj_align(lock_icon_btn, LV_ALIGN_TOP_MID, 0, 58);
  }
}

void loop() {
  // LVGL v8.4 doesn't have lv_tick_inc - tick is handled automatically
  lv_timer_handler();

  // Simple debug to confirm Serial Monitor is working
  static unsigned long lastHeartbeat = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat > 5000) { // Every 5 seconds
    Serial.println("System alive - 5 second heartbeat");
    lastHeartbeat = currentTime;
  }

  // Handle BLE connection state
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Handle hardware inputs (DISABLED - these pins might be floating and causing issues)
  // If you need hardware inputs, make sure the pins are properly pulled up/down
  // For now, commenting out to prevent interference with UI controls
  /*
  static unsigned long lastInputCheck = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastInputCheck > 100) { // Only check every 100ms to avoid spam
    bool openInputActive = digitalRead(openInput);
    bool closeInputActive = digitalRead(closeInput);
    
    // Debug hardware inputs occasionally
    static int debugCounter = 0;
    if (++debugCounter > 50) { // Every 5 seconds
      Serial.print("Hardware inputs: openInput=");
      Serial.print(openInputActive);
      Serial.print(", closeInput=");
      Serial.print(closeInputActive);
      Serial.print(", locked=");
      Serial.print(locked);
      Serial.print(", breakerstate=");
      Serial.println(breakerstate);
      debugCounter = 0;
    }

    if (!locked) {
      if (openInputActive && !breakerstate) {
        Serial.println("HARDWARE INPUT: Opening breaker via openInput pin");
        set_breaker_state(true);
        send_status_to_flutter();
      } else if (closeInputActive && !openInputActive && switchToggled && breakerstate) {
        Serial.println("HARDWARE INPUT: Closing breaker via closeInput pin");
        set_breaker_state(false);
        send_status_to_flutter();
      }
    }
    lastInputCheck = currentTime;
  }
  */

  delay(1);
}

// Helper functions (keeping all the original UI functions)
static void send_status_to_flutter() {
  if (deviceConnected) {
    uint8_t status[3] = {(uint8_t)(breakerstate ? 1 : 0), (uint8_t)(switchToggled ? 1 : 0), (uint8_t)(locked ? 1 : 0)};
    
    static uint8_t lastStatus[3] = {255, 255, 255};
    bool statusChanged = (status[0] != lastStatus[0] || status[1] != lastStatus[1] || status[2] != lastStatus[2]);
    
    if (statusChanged) {
      Serial.print("STATUS UPDATE: B=");
      Serial.print(breakerstate ? "OPEN" : "CLOSED");
      Serial.print(", S=");
      Serial.print(switchToggled ? "UP" : "DOWN");
      Serial.print(", L=");
      Serial.println(locked ? "LOCKED" : "UNLOCKED");
      
      lastStatus[0] = status[0];
      lastStatus[1] = status[1];
      lastStatus[2] = status[2];
    }
    
    pStatusChar->setValue(status, 3);
    pStatusChar->notify();
    
    uint8_t lockVal = locked ? 1 : 0;
    pLockChar->setValue(&lockVal, 1);
    pLockChar->notify();
    
    uint8_t senseVal = senseA_selected ? 0 : 1;
    pSenseChar->setValue(&senseVal, 1);
    pSenseChar->notify();
  }
}

static void set_leds() {
  if (breakerstate) {
    digitalWrite(greenled, LOW);
    digitalWrite(redled, HIGH);
    digitalWrite(blueled, HIGH);
  } else {
    digitalWrite(redled, LOW);
    digitalWrite(greenled, HIGH);
    digitalWrite(blueled, HIGH);
  }
}

static void set_breaker_state(bool open) {
  Serial.println("========================================");
  Serial.print("set_breaker_state called: open=");
  Serial.print(open);
  Serial.print(", locked=");
  Serial.print(locked);
  Serial.print(", current breakerstate=");
  Serial.println(breakerstate);
  
  if (locked) {
    Serial.println("set_breaker_state: BREAKER IS LOCKED - returning");
    Serial.println("========================================");
    return;
  }
  
  bool previousState = breakerstate;
  breakerstate = open;  // Set state FIRST before any other operations
  
  Serial.print("Previous state: ");
  Serial.print(previousState ? "OPEN" : "CLOSED");
  Serial.print(" -> New state: ");
  Serial.println(breakerstate ? "OPEN" : "CLOSED");
  
  // Always update pins when state changes, or if we're forcing a state
  if (previousState != breakerstate || true) {  // Always update pins to ensure they're correct
    int activeSensePin = senseA_selected ? sense : senseB;
    
    Serial.print("Updating pins. Active sense pin: ");
    Serial.println(activeSensePin);
    
    if (breakerstate) {
      Serial.println("Setting breaker to OPEN state");
      digitalWrite(pin39, LOW);
      delay(5);
      digitalWrite(pin41, LOW);
      digitalWrite(activeSensePin, LOW);
      digitalWrite(senseA_selected ? senseB : sense, HIGH);
      Serial.print("OPEN pins set: pin39=LOW, pin41=LOW, pin");
      Serial.print(activeSensePin);
      Serial.println("=LOW");
    } else {
      Serial.println("Setting breaker to CLOSED state");
      digitalWrite(pin41, HIGH);
      digitalWrite(pin39, HIGH);
      digitalWrite(activeSensePin, HIGH);
      digitalWrite(senseA_selected ? senseB : sense, HIGH);
      Serial.print("CLOSED pins set: pin39=HIGH, pin41=HIGH, pin");
      Serial.print(activeSensePin);
      Serial.println("=HIGH");
    }
  }
  
  Serial.print("About to call update_button_styles() - breakerstate=");
  Serial.println(breakerstate ? "OPEN" : "CLOSED");
  update_button_styles();
  set_leds();
  Serial.print("After update_button_styles() - breakerstate=");
  Serial.println(breakerstate ? "OPEN" : "CLOSED");
  Serial.println("========================================");
}

static void switch_toggled_cb(lv_event_t *e) {
  // Debouncing - ignore rapid consecutive presses
  unsigned long currentTime = millis();
  if (currentTime - last_switch_toggle < 300) { // 300ms debounce
    Serial.println("Switch toggle ignored - too soon after last toggle");
    return;
  }
  last_switch_toggle = currentTime;
  
  Serial.println("=== CUSTOM VERTICAL SWITCH TOGGLED ON ARDUINO ===");
  
  // Toggle the state
  switchToggled = !switchToggled;
  
  Serial.print("Switch is now: ");
  Serial.println(switchToggled ? "UP" : "DOWN");
  
  // Update the visual position and color of the switch knob
  if (switchToggled) {
    // UP position (switch ON) - adjust position based on orientation and switch size
    if (currentRotation == 90 || currentRotation == 270) {
      // Landscape: larger switch (58x58) in larger container (69x115)
      lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px more in landscape: Y changed from -3 to -2
    } else {
      // Portrait: original switch (50x50) in original container (60x100)
      lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px: Y changed from -3 to -2
    }
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for ON
  } else {
    // DOWN position (switch OFF) - adjust position based on orientation and switch size
    if (currentRotation == 90 || currentRotation == 270) {
      // Landscape: larger switch (58x58) in larger container (69x115)
      lv_obj_set_pos(switch_69, -2, 42); // Moved down 1px more in landscape: Y changed from 41 to 42
    } else {
      // Portrait: original switch (50x50) in original container (60x100)
      lv_obj_set_pos(switch_69, -2, 38); // Moved down 1px: Y changed from 37 to 38
    }
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for OFF (position shows state)
  }
  
  if (!switchToggled && !breakerstate) {
    Serial.println("Switch down and breaker closed - opening breaker");
    set_breaker_state(true);
  } else {
    Serial.println("Updating button styles only");
    update_button_styles();
  }
  // ALWAYS send status update when switch is toggled, regardless of BLE connection
  send_status_to_flutter();
  Serial.println("=== SWITCH TOGGLE COMPLETE ===");
}

static void open_btn_cb(lv_event_t *e) {
  Serial.println("=== OPEN BUTTON PRESSED ON ARDUINO ===");
  if (locked) {
    Serial.println("Button press ignored - device is locked");
    return;
  }
  set_breaker_state(true);
  // ALWAYS send status update when button is pressed, regardless of BLE connection
  send_status_to_flutter();
  Serial.println("=== OPEN BUTTON COMPLETE ===");
}

static void close_btn_cb(lv_event_t *e) {
  Serial.println("=== CLOSE BUTTON PRESSED ON ARDUINO ===");
  if (locked) {
    Serial.println("Button press ignored - device is locked");
    return;
  }
  if (switchToggled && breakerstate) {
    Serial.println("Closing breaker (switch is up and breaker is open)");
    set_breaker_state(false);
    // ALWAYS send status update when button is pressed, regardless of BLE connection
    send_status_to_flutter();
  } else {
    Serial.print("Close button ignored - switchToggled=");
    Serial.print(switchToggled);
    Serial.print(", breakerstate=");
    Serial.println(breakerstate);
  }
  Serial.println("=== CLOSE BUTTON COMPLETE ===");
}

static void rotate_screen_cb(lv_event_t *e) {
  Serial.println("========================================");
  Serial.println("=== ROTATE BUTTON PRESSED ===");
  Serial.println("========================================");
  
  static unsigned long lastRotationTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastRotationTime < 500) {
    Serial.println("Rotate ignored - too soon after last rotation");
    return;
  }
  lastRotationTime = currentTime;

  Serial.print("Current rotation: ");
  Serial.print(currentRotation);
  currentRotation = (currentRotation + 90) % 360;
  Serial.print(" -> New rotation: ");
  Serial.println(currentRotation);
  lv_disp_t *disp = lv_disp_get_default();

  switch (currentRotation) {
    case 0:   
      gfx->setRotation(0);
      // Update LVGL display driver resolution for portrait
      disp_drv.hor_res = LCD_HOR_RES;  // 320
      disp_drv.ver_res = LCD_VER_RES;  // 480
      break;
    case 90:  
      gfx->setRotation(1);
      // Update LVGL display driver resolution for landscape
      disp_drv.hor_res = LCD_VER_RES;  // 480
      disp_drv.ver_res = LCD_HOR_RES;  // 320
      break;
    case 180: 
      gfx->setRotation(2);
      // Update LVGL display driver resolution for portrait
      disp_drv.hor_res = LCD_HOR_RES;  // 320
      disp_drv.ver_res = LCD_VER_RES;  // 480
      break;
    case 270: 
      gfx->setRotation(3);
      // Update LVGL display driver resolution for landscape
      disp_drv.hor_res = LCD_VER_RES;  // 480
      disp_drv.ver_res = LCD_HOR_RES;  // 320
      break;
  }
  
  // Notify LVGL of the resolution change
  lv_disp_drv_update(disp, &disp_drv);
  
  // Reposition UI elements based on orientation
  if (currentRotation == 90 || currentRotation == 270) {
    // Landscape orientations - center switch on left, center controls on right, moved down
    lv_obj_set_size(switch_container, 200, 220); // Made 15px taller in landscape: changed from 205 to 220
    lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 20, 27); // Moved down 7px more: changed from (20, 20) to (20, 27)
    
    // Make switch components 15% larger in landscape orientation
    if (vertical_switch_container) {
      lv_obj_set_size(vertical_switch_container, 69, 115); // 15% larger: 60x100 -> 69x115
      lv_obj_set_style_radius(vertical_switch_container, 35, 0); // Adjusted radius: 30 -> 35
      lv_obj_set_pos(vertical_switch_container, 56, 0); // Moved left 2px and down 2px in landscape: changed from (58, -2) to (56, 0)
    }
    
    // Make middle container larger in landscape to accommodate larger white switch
    lv_obj_t *middle_container = lv_obj_get_child(switch_container, 1); // Get middle container (second child after UP label)
    if (middle_container) {
      lv_obj_set_size(middle_container, 170, 120); // Increased height from 100 to 120 to fit larger switch
    }
    if (switch_69) {
      lv_obj_set_size(switch_69, 58, 58); // 15% larger: 50x50 -> 58x58
      lv_obj_set_style_radius(switch_69, 29, 0); // Adjusted radius: 25 -> 29
      // Adjust position to account for size change while maintaining relative positioning
      if (switchToggled) {
        lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px more in landscape: Y changed from -3 to -2
      } else {
        lv_obj_set_pos(switch_69, -2, 42); // Moved down 1px more in landscape: Y changed from 41 to 42
      }
    }
    
    lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -20, 60); // Moved down 5px more: changed from (55) to (60)
    if (lock_icon_btn) {
      lv_obj_set_size(lock_icon_btn, 180, 60); // Smaller for landscape
      lv_obj_align(lock_icon_btn, LV_ALIGN_RIGHT_MID, -35, -50); // Moved left 10px more: changed from (-25, -50) to (-35, -50)
    }
    
    // Make open and close buttons taller in landscape orientation
    if (btn_open) {
      lv_obj_set_size(btn_open, 200, 67); // 7px taller: 60 -> 67
    }
    if (btn_close) {
      lv_obj_set_size(btn_close, 200, 67); // 7px taller: 60 -> 67
    }
    
    // Update button overlays to match new button sizes in landscape
    if (open_btn_overlay) {
      lv_obj_set_size(open_btn_overlay, 200, 67); // Match button size: 67px tall
    }
    if (close_btn_overlay) {
      lv_obj_set_size(close_btn_overlay, 200, 67); // Match button size: 67px tall
    }
  } else {
    // Portrait orientations - vertical layout with adjusted positioning
    lv_obj_set_size(switch_container, 215, 185); // 15px wider in portrait: changed from 200 to 215
    
    // Restore original switch component sizes in portrait orientation
    if (vertical_switch_container) {
      lv_obj_set_size(vertical_switch_container, 60, 100); // Original size
      lv_obj_set_style_radius(vertical_switch_container, 30, 0); // Original radius
      lv_obj_set_pos(vertical_switch_container, 58, 0); // Restore original position in portrait
    }
    
    // Restore middle container size in portrait orientation
    lv_obj_t *middle_container = lv_obj_get_child(switch_container, 1); // Get middle container (second child after UP label)
    if (middle_container) {
      lv_obj_set_size(middle_container, 170, 100); // Restore original size: 170x100
    }
    
    if (switch_69) {
      lv_obj_set_size(switch_69, 50, 50); // Original size
      lv_obj_set_style_radius(switch_69, 25, 0); // Original radius
      // Restore positions moved down 1px
      if (switchToggled) {
        lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px: Y changed from -3 to -2
      } else {
        lv_obj_set_pos(switch_69, -2, 38); // Moved down 1px: Y changed from 37 to 38
      }
    }
    
    lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15); // Moved down 15px: changed from 0 to 15
    lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 115); // Moved up 20px in portrait: changed from (0, 135) to (0, 115)
    if (lock_icon_btn) {
      lv_obj_set_size(lock_icon_btn, 215, 50); // Made shorter in portrait: changed from 70 to 50
      lv_obj_align(lock_icon_btn, LV_ALIGN_TOP_MID, 0, 58); // Moved down 3px: changed from (0, 55) to (0, 58)
    }
    
    // Make buttons wider and taller in portrait orientation
    if (btn_open) {
      lv_obj_set_size(btn_open, 215, 70); // Made taller in portrait: changed from 60 to 70
    }
    if (btn_close) {
      lv_obj_set_size(btn_close, 215, 70); // Made taller in portrait: changed from 60 to 70
    }
    
    // Update overlay sizes to match new button sizes in portrait orientation
    if (open_btn_overlay) {
      lv_obj_set_size(open_btn_overlay, 215, 70); // Match new button size: 215x70
    }
    if (close_btn_overlay) {
      lv_obj_set_size(close_btn_overlay, 215, 70); // Match new button size: 215x70
    }
  }
  
  // Recreate top bar after rotation to ensure correct positioning
  create_top_bar();
  // Update Bluetooth status font size based on new orientation
  update_top_bar_bt_status();

  // Ensure screen and main container remain non-scrollable after rotation
  lv_obj_t *rot_scr = lv_scr_act();
  lv_obj_clear_flag(rot_scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(rot_scr, LV_DIR_NONE);
  lv_obj_set_scrollbar_mode(rot_scr, LV_SCROLLBAR_MODE_OFF);
  if (ui_container) {
    lv_obj_clear_flag(ui_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(ui_container, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(ui_container, LV_SCROLLBAR_MODE_OFF);
  }

  // Force a full screen refresh
  lv_refr_now(disp);
  lv_obj_invalidate(lv_scr_act());
}

static void lock_icon_event_cb(lv_event_t *e) {
  uint32_t code = lv_event_get_code(e);
  
  if (code == LV_EVENT_PRESSED) {
    lock_press_start = millis();
    Serial.println("Lock button pressed - hold for 400ms");
    
    // Simple visual feedback - just change the text
    if (locked) {
      lv_label_set_text(lock_icon_label, "HOLD TO UNLOCK");
    } else {
      lv_label_set_text(lock_icon_label, "HOLD TO LOCK");
    }
    
  } else if (code == LV_EVENT_PRESSING) {
    if (lock_press_start && (millis() - lock_press_start > 400)) {
      lock_press_start = 0;
      locked = !locked;
      Serial.print("Lock toggled - locked: ");
      Serial.println(locked);

      // sync with BLE - track when we write to avoid echo processing
      uint8_t val = locked ? 1 : 0;
      pLockChar->setValue(&val, 1);
      pLockChar->notify();
      
      // Update timing to prevent echo processing
      static unsigned long lastLockWrite = 0;
      lastLockWrite = millis();

      update_lock_icon();
      update_button_styles();
      // ALWAYS send status update when lock is toggled, regardless of BLE connection
      send_status_to_flutter();
    }
    
  } else if (code == LV_EVENT_RELEASED) {
    // Always restore to normal state when released
    update_lock_icon();
    lock_press_start = 0;
  }
}

static void update_lock_icon() {
  if (!lock_icon_label || !lock_icon_btn) return;
  if (locked) {
    lv_label_set_text(lock_icon_label, "HOLD TO UNLOCK");
    lv_obj_set_style_bg_color(lock_icon_btn, lv_color_hex(0xFF9800), 0);
  } else {
    lv_label_set_text(lock_icon_label, "HOLD TO LOCK");
    lv_obj_set_style_bg_color(lock_icon_btn, lv_color_hex(0x1976D2), 0);
  }
}

static void settings_btn_cb(lv_event_t *e) {
  Serial.println("========================================");
  Serial.println("=== SETTINGS BUTTON PRESSED ===");
  Serial.println("========================================");
  
  // Delete existing modal if it exists to recreate it with proper orientation
  if (settings_modal) {
    lv_obj_del(settings_modal);
    settings_modal = NULL;
    btn_sense_a = NULL;
    btn_sense_b = NULL;
  }
  
  // Create modal dialog if it doesn't exist
  if (!settings_modal) {
    settings_modal = lv_obj_create(lv_scr_act());
    
    // Get current screen dimensions to adapt modal size
    lv_obj_t *scr = lv_scr_act();
    int32_t screen_width = lv_obj_get_width(scr);
    int32_t screen_height = lv_obj_get_height(scr);
    
    // Adapt modal size based on orientation
    int32_t modal_width, modal_height;
    if (currentRotation == 90 || currentRotation == 270) {
      // Landscape orientation (480x320)
      modal_width = 400;  // Fits within 480px width
      modal_height = 280; // Fits within 320px height
    } else {
      // Portrait orientation (320x480)  
      modal_width = 300;  // Fits within 320px width
      modal_height = 400; // Fits within 480px height
    }
    
    lv_obj_set_size(settings_modal, modal_width, modal_height);
    lv_obj_align(settings_modal, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(settings_modal, lv_color_hex(0xDDDDDD), 0);
    lv_obj_set_style_border_width(settings_modal, 3, 0);
    lv_obj_set_style_border_color(settings_modal, lv_color_hex(0x000000), 0);
    // Reduce padding in landscape to prevent overlapping
    lv_obj_set_style_pad_all(settings_modal, (currentRotation == 90 || currentRotation == 270) ? 8 : 15, 0);
    // Make sure modal is on top and clickable
    lv_obj_clear_flag(settings_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(settings_modal);
    
    // Title
    lv_obj_t *title = lv_label_create(settings_modal);
    lv_label_set_text(title, "SENSE SELECTION");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Adapt button layout based on orientation
    int32_t btn_width = (currentRotation == 90 || currentRotation == 270) ? 180 : 260;
    int32_t btn_height = (currentRotation == 90 || currentRotation == 270) ? 50 : 60;
    int32_t btn_spacing = (currentRotation == 90 || currentRotation == 270) ? 30 : 70; // Reduced from 40 to 30 for landscape
    int32_t disconnect_btn_width = (currentRotation == 90 || currentRotation == 270) ? 250 : 260; // Increased from 220 to 250 for landscape (30px wider)
    
    // Sense A button
    btn_sense_a = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_sense_a, btn_width, btn_height);
    lv_obj_align(btn_sense_a, LV_ALIGN_TOP_MID, 0, btn_spacing);
    lv_obj_set_style_border_width(btn_sense_a, 4, 0);
    lv_obj_set_style_border_color(btn_sense_a, lv_color_hex(0x000000), 0);
    // Set initial color based on current selection
    lv_obj_set_style_bg_color(btn_sense_a, senseA_selected ? lv_color_hex(0x00AA00) : lv_color_hex(0xCCCCCC), 0);
    lv_obj_add_event_cb(btn_sense_a, sense_a_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_a = lv_label_create(btn_sense_a);
    lv_label_set_text(label_a, "SENSE A");
    lv_obj_set_style_text_font(label_a, &lv_font_montserrat_16, 0);
    lv_obj_center(label_a);
    
    // Sense B button
    btn_sense_b = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_sense_b, btn_width, btn_height);
    // Reduced spacing between A and B buttons in landscape: 20px -> 10px for landscape
    int32_t btn_b_spacing = (currentRotation == 90 || currentRotation == 270) ? 10 : 20;
    lv_obj_align(btn_sense_b, LV_ALIGN_TOP_MID, 0, btn_spacing + btn_height + btn_b_spacing);
    lv_obj_set_style_border_width(btn_sense_b, 4, 0);
    lv_obj_set_style_border_color(btn_sense_b, lv_color_hex(0x000000), 0);
    // Set initial color based on current selection
    lv_obj_set_style_bg_color(btn_sense_b, senseA_selected ? lv_color_hex(0xCCCCCC) : lv_color_hex(0x00AA00), 0);
    lv_obj_add_event_cb(btn_sense_b, sense_b_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_b = lv_label_create(btn_sense_b);
    lv_label_set_text(label_b, "SENSE B");
    lv_obj_set_style_text_font(label_b, &lv_font_montserrat_16, 0);
    lv_obj_center(label_b);
    
    // Disconnect Bluetooth button (only show if connected)
    lv_obj_t *btn_disconnect_bt = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_disconnect_bt, disconnect_btn_width, 40); // Using separate width for disconnect button
    // Adjusted positioning to account for reduced spacing between A and B buttons
    int32_t disconnect_y_pos = btn_spacing + (btn_height + btn_b_spacing) + btn_height + 10; // Use calculated spacing
    lv_obj_align(btn_disconnect_bt, LV_ALIGN_TOP_MID, 0, disconnect_y_pos);
    lv_obj_set_style_bg_color(btn_disconnect_bt, lv_color_hex(0xFF4444), 0); // Red for disconnect
    lv_obj_set_style_border_width(btn_disconnect_bt, 3, 0);
    lv_obj_set_style_border_color(btn_disconnect_bt, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(btn_disconnect_bt, disconnect_bluetooth_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_disconnect = lv_label_create(btn_disconnect_bt);
    lv_label_set_text(label_disconnect, "DISCONNECT BLUETOOTH");
    lv_obj_set_style_text_font(label_disconnect, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label_disconnect, lv_color_hex(0xFFFFFF), 0); // White text
    lv_obj_center(label_disconnect);
    
    // Close button
    lv_obj_t *btn_close_settings = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_close_settings, (currentRotation == 90 || currentRotation == 270) ? 120 : 200, 50);
    lv_obj_align(btn_close_settings, LV_ALIGN_BOTTOM_MID, 0, (currentRotation == 90 || currentRotation == 270) ? -5 : -15); // Reduced bottom offset for landscape
    lv_obj_set_style_border_width(btn_close_settings, 3, 0);
    lv_obj_set_style_border_color(btn_close_settings, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(btn_close_settings, close_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_close = lv_label_create(btn_close_settings);
    lv_label_set_text(label_close, "CLOSE");
    lv_obj_set_style_text_font(label_close, &lv_font_montserrat_16, 0);
    lv_obj_center(label_close);
  }
  
  // Update button colors based on current selection
  if (btn_sense_a && btn_sense_b) {
    if (senseA_selected) {
      lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0x00AA00), 0); // Green
      lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0xCCCCCC), 0); // Gray
    } else {
      lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0xCCCCCC), 0); // Gray
      lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0x00AA00), 0); // Green
    }
  }
  
  // Show modal and bring to front
  lv_obj_clear_flag(settings_modal, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(settings_modal);
  // Force a screen refresh
  lv_refr_now(lv_disp_get_default());
  Serial.println("Settings modal opened and brought to front");
}

static void sense_a_btn_cb(lv_event_t *e) {
  Serial.println("Sense A selected");
  senseA_selected = true;
  
  // Update visuals
  lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0x00AA00), 0); // Green
  lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0xCCCCCC), 0); // Gray
  
  // Send update to app via BLE
  if (deviceConnected) {
    uint8_t value = 0; // Sense A
    pSenseChar->setValue(&value, 1);
    pSenseChar->notify();
  }
  
  // Apply the current breaker state with the new sense pin
  set_breaker_state(breakerstate);
}

static void sense_b_btn_cb(lv_event_t *e) {
  Serial.println("Sense B selected");
  senseA_selected = false;
  
  // Update visuals
  lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0xCCCCCC), 0); // Gray
  lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0x00AA00), 0); // Green
  
  // Send update to app via BLE
  if (deviceConnected) {
    uint8_t value = 1; // Sense B
    pSenseChar->setValue(&value, 1);
    pSenseChar->notify();
  }
  
  // Apply the current breaker state with the new sense pin
  set_breaker_state(breakerstate);
}

static void disconnect_bluetooth_cb(lv_event_t *e) {
  Serial.println("Disconnect Bluetooth button pressed");
  if (deviceConnected && pServer) {
    Serial.println("Disconnecting from central device...");
    pServer->disconnect(pServer->getConnId());
    deviceConnected = false;
    bluetoothConnected = false;
    update_top_bar_bt_status();
    update_button_styles();
    update_lock_icon();
    set_leds();
    Serial.println("Disconnected successfully");
  } else {
    Serial.println("No connected device to disconnect");
  }
}

static void close_settings_cb(lv_event_t *e) {
  Serial.println("=== CLOSE SETTINGS BUTTON PRESSED ===");
  if (settings_modal) {
    Serial.println("Hiding settings modal");
    lv_obj_add_flag(settings_modal, LV_OBJ_FLAG_HIDDEN);
    // Force a screen refresh
    lv_refr_now(lv_disp_get_default());
    Serial.println("Settings modal hidden");
  } else {
    Serial.println("ERROR: settings_modal is NULL!");
  }
}

static void update_button_styles() {
  if (!btn_open || !btn_close || !ui_container || !switch_container || !tight_container || !switch_69) {
    Serial.println("update_button_styles: Missing UI elements, returning");
    return;
  }
  
  Serial.print("UPDATE STYLES: locked=");
  Serial.print(locked);
  Serial.print(", switchToggled=");
  Serial.print(switchToggled);
  Serial.print(", breakerstate=");
  Serial.println(breakerstate ? "OPEN" : "CLOSED");
  
  // Update background color based on breaker state - THIS IS THE KEY VISUAL INDICATOR
  if (breakerstate) {
    Serial.println("Setting background to GREEN (OPEN)");
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0x00AA00), 0); // Green for open
  } else {
    Serial.println("Setting background to RED (CLOSED)");
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0xAA0000), 0); // Red for closed
  }
  
  // Always start by clearing everything
  lv_obj_clear_state(btn_open, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
  lv_obj_add_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN);

  if (locked) {
    Serial.println("LOCKED STATE - disabling all buttons and showing lock overlays");
    // When LOCKED: disable everything and show lock overlays
    lv_obj_add_state(btn_open, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x333300), 0);
    lv_obj_add_state(btn_close, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x330000), 0);
    lv_obj_add_state(switch_69, LV_STATE_DISABLED);
    
    // Show LOCK overlays (not prohibition overlays)
    lv_obj_clear_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN);
    
  } else {
    Serial.println("UNLOCKED STATE - normal operation");
    // When UNLOCKED: normal operation
    lv_obj_clear_state(switch_69, LV_STATE_DISABLED);
    
    if (breakerstate) {
      // Breaker is OPEN
      lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x00AA00), 0);
      
      if (!switchToggled) {
        Serial.println("Switch is DOWN - disabling close button with prohibition overlay");
        // Switch DOWN - disable close button with prohibition overlay (safety rule)
        lv_obj_add_state(btn_close, LV_STATE_DISABLED);
        // Set disabled state background color specifically to override LVGL's automatic lightening
        lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x110000), LV_STATE_DISABLED);
        lv_obj_clear_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN); // Show prohibition
        // Keep open button normal (no overlay)
        lv_obj_add_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
      } else {
        Serial.println("Switch is UP - all buttons enabled, no overlays");
        // Switch UP - both buttons enabled, no overlays
        lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x880000), 0); // Medium-dark red, darker than background but still visible
        lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
        lv_obj_add_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
      }
    } else {
      Serial.println("Breaker is CLOSED - all buttons enabled, no overlays");
      // Breaker is CLOSED - both buttons enabled, no overlays
      lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x005500), 0);
      lv_obj_set_style_bg_color(btn_close, lv_color_hex(0xAA0000), 0);
      lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
      lv_obj_add_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

// Create UI function - called once at startup
static void create_ui() {
  // Main container
  lv_obj_t *screen = lv_scr_act();
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(screen, LV_DIR_NONE);
  lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

  ui_container = lv_obj_create(screen);
  lv_obj_set_size(ui_container, LV_PCT(100), LV_PCT(100));
  // Set initial background color based on breaker state
  if (breakerstate) {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0x00AA00), 0); // Green for open
  } else {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0xAA0000), 0); // Red for closed
  }
  lv_obj_clear_flag(ui_container, LV_OBJ_FLAG_SCROLLABLE);

  // Switch container - portrait orientation size
  switch_container = lv_obj_create(ui_container);
  lv_obj_set_size(switch_container, 215, 185); // Portrait size: 215x185
  lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 75); // Portrait position: top center
  lv_obj_set_style_bg_color(switch_container, lv_color_hex(0xFFFF00), 0); // Yellow background
  lv_obj_set_style_border_width(switch_container, 5, 0);
  lv_obj_set_style_border_color(switch_container, lv_color_hex(0x000000), 0);
  lv_obj_set_flex_flow(switch_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(switch_container, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Space around for better spacing
  lv_obj_set_style_pad_all(switch_container, 8, 0); // Reduced padding from 15 to 8 for tighter spacing around UP/DOWN labels
  lv_obj_clear_flag(switch_container, LV_OBJ_FLAG_SCROLLABLE); // Make non-scrollable

  // UP label
  lv_obj_t *label_up = lv_label_create(switch_container);
  lv_label_set_text(label_up, "UP");
  lv_obj_set_style_text_color(label_up, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_up, &lv_font_montserrat_16, 0);

// Middle container for switch and 69 label (NO FLEX - manual positioning)
  lv_obj_t *middle_container = lv_obj_create(switch_container);
  lv_obj_set_size(middle_container, 170, 100); // Portrait size: 170x100
  lv_obj_align(middle_container, LV_ALIGN_LEFT_MID, 0, -20); // Adjusted Y position
  // REMOVE FLEX - this was causing transform conflicts
  // lv_obj_set_flex_flow(middle_container, LV_FLEX_FLOW_ROW);
  // lv_obj_set_flex_align(middle_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(middle_container, 0, 0);
  lv_obj_set_style_border_width(middle_container, 0, 0);
  lv_obj_set_style_bg_opa(middle_container, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(middle_container, LV_OBJ_FLAG_SCROLLABLE); // Make non-scrollable

  // 69 label - manually positioned on the left
  lv_obj_t *label_69 = lv_label_create(middle_container);
  lv_label_set_text(label_69, "69");
  lv_obj_set_style_text_color(label_69, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_69, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(label_69, 15, 35); // Adjusted for smaller container

  // Custom vertical switch using two buttons arranged vertically
  // Create a container for our custom vertical switch - portrait sizing
  vertical_switch_container = lv_obj_create(middle_container);
  lv_obj_set_size(vertical_switch_container, 60, 100); // Portrait size: 60x100
  lv_obj_set_pos(vertical_switch_container, 58, 0); // Portrait position
  lv_obj_set_style_bg_color(vertical_switch_container, lv_color_hex(0xFFFFFF), 0); // White background for switch track
  lv_obj_set_style_border_width(vertical_switch_container, 2, 0);
  lv_obj_set_style_border_color(vertical_switch_container, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(vertical_switch_container, 30, 0); // Portrait radius: 30
  lv_obj_set_style_pad_all(vertical_switch_container, 5, 0); // Slightly more padding
  lv_obj_clear_flag(vertical_switch_container, LV_OBJ_FLAG_SCROLLABLE); // Make non-scrollable
  
  // Create the switch "knob" - this will move up/down - portrait sizing
  switch_69 = lv_btn_create(vertical_switch_container);
  lv_obj_set_size(switch_69, 50, 50); // Portrait size: 50x50
  lv_obj_set_style_radius(switch_69, 25, 0); // Portrait radius: 25
  lv_obj_set_style_border_width(switch_69, 1, 0);
  lv_obj_set_style_border_color(switch_69, lv_color_hex(0x000000), 0);
  lv_obj_clear_flag(switch_69, LV_OBJ_FLAG_SCROLLABLE); // Make non-scrollable
  
  // Position the knob based on switch state
  if (switchToggled) {
    // UP position (switch ON) - moved down 1px: Y changed from -3 to -2
    lv_obj_set_pos(switch_69, -2, -2); // Moved down 1px: Y changed from -3 to -2
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for ON
  } else {
    // DOWN position (switch OFF) - moved down 1px: Y changed from 37 to 38
    lv_obj_set_pos(switch_69, -2, 38); // Moved down 1px: Y changed from 37 to 38
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for OFF (will distinguish by position)
  }
  
  // Add click event to toggle the switch
  lv_obj_add_event_cb(switch_69, switch_toggled_cb, LV_EVENT_CLICKED, NULL);
  
  // Disable switch if locked
  if (locked) lv_obj_add_state(switch_69, LV_STATE_DISABLED);

  // DOWN label
  lv_obj_t *label_down = lv_label_create(switch_container);
  lv_label_set_text(label_down, "DOWN");
  lv_obj_set_style_text_color(label_down, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_down, &lv_font_montserrat_16, 0);

  // Button container for open/close - positioned to fit on screen
  tight_container = lv_obj_create(ui_container);
  lv_obj_set_size(tight_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(tight_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tight_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(tight_container, 5, 0);
  lv_obj_set_style_pad_row(tight_container, 10, 0);
  lv_obj_set_style_border_width(tight_container, 0, 0);
  lv_obj_set_style_bg_opa(tight_container, LV_OPA_TRANSP, 0);
  lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15); // Portrait position: bottom center

  // Open button - portrait sizing
  btn_open = lv_btn_create(tight_container);
  lv_obj_set_size(btn_open, 215, 70); // Portrait size: 215x70
  lv_obj_set_style_border_width(btn_open, 3, 0);
  lv_obj_set_style_border_color(btn_open, lv_color_hex(0x000000), 0);
  lv_obj_add_event_cb(btn_open, open_btn_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_open = lv_label_create(btn_open);
  lv_label_set_text(label_open, "OPEN");
  lv_obj_set_style_text_color(label_open, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_open, &lv_font_montserrat_20, 0); // Increased from 16 to 20 for larger text
  lv_obj_center(label_open);

  // Close button - portrait sizing
  btn_close = lv_btn_create(tight_container);
  lv_obj_set_size(btn_close, 215, 70); // Portrait size: 215x70
  lv_obj_set_style_border_width(btn_close, 3, 0);
  lv_obj_set_style_border_color(btn_close, lv_color_hex(0x000000), 0);
  lv_obj_add_event_cb(btn_close, close_btn_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label_close = lv_label_create(btn_close);
  lv_label_set_text(label_close, "CLOSE");
  lv_obj_set_style_text_color(label_close, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_close, &lv_font_montserrat_20, 0); // Increased from 16 to 20 for larger text
  lv_obj_center(label_close);

  // Create overlay for disabled state (circle slash symbol)
  close_btn_overlay = lv_obj_create(btn_close);
  lv_obj_set_size(close_btn_overlay, 215, 70); // Updated to match new button size: 215x70
  lv_obj_align(close_btn_overlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(close_btn_overlay, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_40, 0);
  lv_obj_set_style_radius(close_btn_overlay, 0, 0);
  lv_obj_add_flag(close_btn_overlay, LV_OBJ_FLAG_HIDDEN);
  
  // Create circle (O)
  lv_obj_t *circle_label = lv_label_create(close_btn_overlay);
  lv_label_set_text(circle_label, "O");
  lv_obj_set_style_text_color(circle_label, lv_color_hex(0xFF0000), 0);
  lv_obj_set_style_text_font(circle_label, &lv_font_montserrat_16, 0);
  lv_obj_center(circle_label);

  lv_obj_t *slash_label = lv_label_create(close_btn_overlay);
  lv_label_set_text(slash_label, "/");
  lv_obj_set_style_text_color(slash_label, lv_color_hex(0xFF0000), 0);
  lv_obj_set_style_text_font(slash_label, &lv_font_montserrat_16, 0);
  lv_obj_center(slash_label);

  // Create overlay for disabled state (lock symbol) for open button
  open_btn_overlay = lv_obj_create(btn_open);
  lv_obj_set_size(open_btn_overlay, 215, 70); // Updated to match new button size: 215x70
  lv_obj_align(open_btn_overlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(open_btn_overlay, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_40, 0);
  lv_obj_set_style_radius(open_btn_overlay, 0, 0);
  lv_obj_add_flag(open_btn_overlay, LV_OBJ_FLAG_HIDDEN);
  
  lv_obj_t *lock_label_open = lv_label_create(open_btn_overlay);
  lv_label_set_text(lock_label_open, LV_SYMBOL_LOCK);
  lv_obj_set_style_text_color(lock_label_open, lv_color_hex(0xAA0000), 0);
  lv_obj_set_style_text_font(lock_label_open, &lv_font_montserrat_16, 0);
  lv_obj_center(lock_label_open);

  // Lock icon button - portrait sizing
  lock_icon_btn = lv_btn_create(ui_container);
  lv_obj_set_size(lock_icon_btn, 215, 50); // Portrait size: 215x50
  lv_obj_set_style_radius(lock_icon_btn, 8, 0); // Slightly more rounded for better feel
  lv_obj_set_style_border_width(lock_icon_btn, 4, 0); // Thicker border for better visual feedback
  lv_obj_set_style_border_color(lock_icon_btn, lv_color_hex(0x000000), 0);
  
  // Register for all touch events
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSING, NULL);
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_RELEASED, NULL);
  
  // Create label
  lock_icon_label = lv_label_create(lock_icon_btn);
  lv_obj_set_style_text_font(lock_icon_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(lock_icon_label, lv_color_hex(0x000000), 0);
  lv_obj_center(lock_icon_label);

  // Note: Settings and Rotate buttons are now created in create_top_bar()

  // Initialize button styles
  update_button_styles();
}

// Create top bar with settings, rotate, and Bluetooth status
static void create_top_bar() {
  // Delete existing top bar if it exists
  if (top_bar) {
    lv_obj_del(top_bar);
    top_bar = NULL;
  }
  
  // Get screen dimensions
  lv_obj_t *scr = lv_scr_act();
  int32_t screen_width = lv_obj_get_width(scr);
  int32_t screen_height = lv_obj_get_height(scr);
  
  // Create top bar on screen (will be moved to foreground)
  top_bar = lv_obj_create(scr);
  lv_obj_set_size(top_bar, screen_width, 70); // Increased from 50 to 70 for thicker bar
  lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x000000), 0); // Black background
  lv_obj_set_style_bg_opa(top_bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(top_bar, 0, 0);
  lv_obj_set_style_radius(top_bar, 0, 0); // Squared edges - no rounded corners
  lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);
  // Make top bar click-through except for buttons
  lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_CLICKABLE);
  // Move to foreground to ensure it's always on top
  lv_obj_move_foreground(top_bar);
  
  // Settings button on the left
  btn_settings = lv_btn_create(top_bar);
  lv_obj_set_size(btn_settings, 60, 50); // Smaller size for symbol
  lv_obj_align(btn_settings, LV_ALIGN_LEFT_MID, 5, 0);
  lv_obj_set_style_bg_color(btn_settings, lv_color_hex(0xFF9800), 0);
  lv_obj_set_style_border_width(btn_settings, 2, 0);
  lv_obj_set_style_border_color(btn_settings, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(btn_settings, 5, 0);
  // Ensure button is clickable and on top
  lv_obj_clear_flag(btn_settings, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(btn_settings, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(btn_settings);
  lv_obj_t *settings_label = lv_label_create(btn_settings);
  lv_label_set_text(settings_label, LV_SYMBOL_SETTINGS); // Use settings symbol instead of text
  lv_obj_set_style_text_font(settings_label, &lv_font_montserrat_20, 0); // Use larger font for better symbol visibility
  lv_obj_set_style_text_color(settings_label, lv_color_hex(0x000000), 0);
  lv_obj_center(settings_label);
  lv_obj_add_event_cb(btn_settings, settings_btn_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(btn_settings, settings_btn_cb, LV_EVENT_PRESSED, NULL); // Also on press
  Serial.println("Settings button created and callback attached");
  
  // Bluetooth connection status label (between settings and rotate)
  top_bar_bt_status = lv_label_create(top_bar);
  lv_obj_set_style_text_font(top_bar_bt_status, &lv_font_montserrat_16, 0); // Larger font to match thicker bar
  // Move Bluetooth status left 10px in portrait orientation
  int32_t bt_x_offset = (currentRotation == 0 || currentRotation == 180) ? -10 : 0;
  lv_obj_align(top_bar_bt_status, LV_ALIGN_CENTER, bt_x_offset, 0);
  update_top_bar_bt_status();
  
  // Rotate button on the right
  btn_rotate = lv_btn_create(top_bar);
  lv_obj_set_size(btn_rotate, 90, 50); // Smaller size to fit text better: changed from 140 to 90
  lv_obj_align(btn_rotate, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_set_style_bg_color(btn_rotate, lv_color_hex(0x2196F3), 0);
  lv_obj_set_style_border_width(btn_rotate, 2, 0);
  lv_obj_set_style_border_color(btn_rotate, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(btn_rotate, 5, 0);
  // Ensure button is clickable and on top
  lv_obj_clear_flag(btn_rotate, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(btn_rotate, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(btn_rotate);
  lv_obj_add_event_cb(btn_rotate, rotate_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(btn_rotate, rotate_screen_cb, LV_EVENT_PRESSED, NULL); // Also on press
  Serial.println("Rotate button created and callback attached");
  lv_obj_t *rotate_label = lv_label_create(btn_rotate);
  lv_label_set_text(rotate_label, "ROTATE");
  lv_obj_set_style_text_font(rotate_label, &lv_font_montserrat_14, 0); // Smaller font to fit better: changed from 16 to 14
  lv_obj_set_style_text_color(rotate_label, lv_color_hex(0x000000), 0);
  lv_obj_center(rotate_label);
}

// Update Bluetooth status in top bar
static void update_top_bar_bt_status() {
  if (!top_bar_bt_status) {
    return; // UI element not created yet
  }
  
  // Show Bluetooth status in all orientations now that we have more space
  lv_obj_clear_flag(top_bar_bt_status, LV_OBJ_FLAG_HIDDEN);
  
  char status_text[50];
  #ifndef LV_SYMBOL_BLUETOOTH
  #define LV_SYMBOL_BLUETOOTH "\xef\x8a\x93"
  #endif
  
  // Use smaller font in portrait orientation for better fit, standard font for landscape
  bool is_portrait = (currentRotation == 0 || currentRotation == 180);
  const lv_font_t *status_font = is_portrait ? &lv_font_montserrat_12 : &lv_font_montserrat_16;
  lv_obj_set_style_text_font(top_bar_bt_status, status_font, 0);
  
  if (bluetoothConnected) {
    snprintf(status_text, sizeof(status_text), "%s Connected", LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0x00FF00), 0); // Green
  } else {
    snprintf(status_text, sizeof(status_text), "%s Disconnected", LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0xFF0000), 0); // Red
  }
  lv_label_set_text(top_bar_bt_status, status_text);
}

// Test function to check red pixels - COMMENTED OUT due to API issues
// The Arduino_H7_Video class doesn't have drawPixel method as used here
// If you need to test display colors, you can create LVGL color objects instead
/*
void test_red_pixels() {
  Serial.println("Testing red pixels...");
  
  // This approach doesn't work with Arduino_H7_Video
  // Would need to use LVGL color objects and screen filling instead
  
  Serial.println("Color test function disabled - use LVGL UI for color testing");
}
*/