#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

/**
 * LCD Driver Foundation for ESP32-S3-Touch-LCD-2 (Waveshare)
 *
 * This file provides the foundation for LCD and touch control using LVGL.
 * Based on ESP32S3 reference code from references/ESP32-S3_MCU_Example_including_lcd_driver/
 *
 * Hardware:
 * - Display: ST7789 240x320 TFT LCD
 * - Touch: CST816D capacitive touch controller (I2C)
 * - MCU: ESP32-S3-WROOM-1
 *
 * Pin Configuration (ESP32-S3-Touch-LCD-2):
 * - LCD SPI:
 *   - SCLK: GPIO39
 *   - MOSI: GPIO38
 *   - MISO: GPIO40
 *   - DC:   GPIO42
 *   - CS:   GPIO45
 *   - BL:   GPIO1 (backlight)
 * - Touch I2C:
 *   - SDA:  GPIO48
 *   - SCL:  GPIO47
 *
 * Libraries Required:
 * - LVGL (v8.x or v9.x)
 * - Arduino_GFX_Library (for ST7789 display)
 *
 * TODO: Implement full UI using EEZ Studio
 */

#include <Arduino.h>

//=============================================================================
// LCD HARDWARE CONFIGURATION
//=============================================================================

// LCD SPI pins
#define LCD_PIN_SCLK    39
#define LCD_PIN_MOSI    38
#define LCD_PIN_MISO    40
#define LCD_PIN_DC      42
#define LCD_PIN_RST     -1  // Not used (tied to EN)
#define LCD_PIN_CS      45
#define LCD_PIN_BL      1   // Backlight

// Touch I2C pins (shared with encoder on this board)
#define TOUCH_PIN_SDA   48
#define TOUCH_PIN_SCL   47
#define TOUCH_I2C_ADDR  0x15  // CST816D touch controller

// LCD display settings
#define LCD_ROTATION    0     // Portrait mode (0, 1, 2, 3)
#define LCD_WIDTH       240
#define LCD_HEIGHT      320

// Backlight PWM settings
#define LCD_BL_FREQ     5000
#define LCD_BL_BITS     10
#define LCD_BL_DUTY     80    // 80% brightness (0-100)

//=============================================================================
// LCD DRIVER CLASS
//=============================================================================

class LCDDriver {
public:
    /**
     * Initialize the LCD display
     * @return true if successful
     */
    bool begin();

    /**
     * Set backlight brightness
     * @param brightness Brightness level 0-100%
     */
    void setBacklight(uint8_t brightness);

    /**
     * Get current backlight brightness
     * @return Brightness level 0-100%
     */
    uint8_t getBacklight();

    /**
     * Turn backlight on
     */
    void backlightOn();

    /**
     * Turn backlight off
     */
    void backlightOff();

    /**
     * Clear the display
     */
    void clear();

    /**
     * Update display (call periodically in loop)
     */
    void update();

private:
    uint8_t _backlight_level;
    bool _initialized;
};

//=============================================================================
// TOUCH DRIVER CLASS (CST816D)
//=============================================================================

class TouchDriver {
public:
    /**
     * Initialize the touch controller
     * @return true if successful
     */
    bool begin();

    /**
     * Read touch state
     * @return true if screen is touched
     */
    bool read();

    /**
     * Get X coordinate of touch point
     * @return X coordinate (0-239)
     */
    uint16_t getX();

    /**
     * Get Y coordinate of touch point
     * @return Y coordinate (0-319)
     */
    uint16_t getY();

    /**
     * Check if screen is currently touched
     * @return true if touched
     */
    bool isTouched();

private:
    uint16_t _touch_x;
    uint16_t _touch_y;
    bool _is_touched;
    bool _initialized;
};

//=============================================================================
// LVGL INTEGRATION NOTES
//=============================================================================

/*
 * To integrate LVGL:
 *
 * 1. Install libraries via Arduino Library Manager:
 *    - LVGL (https://github.com/lvgl/lvgl)
 *    - Arduino_GFX_Library (https://github.com/moononournation/Arduino_GFX)
 *
 * 2. Configure LVGL (lv_conf.h):
 *    - Copy lv_conf_template.h to lv_conf.h
 *    - Set LV_COLOR_DEPTH 16
 *    - Enable LV_USE_LOG for debugging
 *    - Configure buffer size based on available RAM
 *
 * 3. Initialize LVGL in setup():
 *    - Create display buffer (recommend 1/10 of screen size for ESP32-S3)
 *    - Register display driver with flush callback
 *    - Register input device (touch) with read callback
 *    - Call lv_timer_handler() in loop()
 *
 * 4. Create UI:
 *    - Use EEZ Studio to design UI
 *    - Export to LVGL format
 *    - Include generated UI files
 *
 * Example initialization code structure:
 *
 *   // In setup():
 *   lcdDriver.begin();
 *   touchDriver.begin();
 *   lv_init();
 *
 *   // Create display buffer (24KB for 240x320 at 1/10 screen)
 *   static lv_disp_draw_buf_t draw_buf;
 *   static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];
 *   lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);
 *
 *   // Register display
 *   static lv_disp_drv_t disp_drv;
 *   lv_disp_drv_init(&disp_drv);
 *   disp_drv.hor_res = LCD_WIDTH;
 *   disp_drv.ver_res = LCD_HEIGHT;
 *   disp_drv.flush_cb = my_disp_flush;  // Your flush function
 *   disp_drv.draw_buf = &draw_buf;
 *   lv_disp_drv_register(&disp_drv);
 *
 *   // Register touch input
 *   static lv_indev_drv_t indev_drv;
 *   lv_indev_drv_init(&indev_drv);
 *   indev_drv.type = LV_INDEV_TYPE_POINTER;
 *   indev_drv.read_cb = my_touchpad_read;  // Your touch read function
 *   lv_indev_drv_register(&indev_drv);
 *
 *   // In loop():
 *   lv_timer_handler();  // Let LVGL do its work
 *   delay(5);
 *
 * Reference Files:
 * - references/ESP32-S3_MCU_Example_including_lcd_driver/08_lvgl_example.ino
 * - references/ESP32-S3_MCU_Example_including_lcd_driver/lv_conf.h
 * - references/esp-32_motor_control/LVGL_Driver.h
 * - references/esp-32_motor_control/LVGL_Driver.cpp
 */

#endif // LCD_DRIVER_H
