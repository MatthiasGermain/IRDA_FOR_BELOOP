#include "oled.h"
#include <Wire.h>
#include "SSD1306.h"
#include "SSD1306Wire.h" 

SSD1306 display(0x3c, 5, 4);

void oled_init() {
    display.init();
    display.flipScreenVertically();
    display.clear();
}

void oled_display_text(const char* text, int x, int y) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(x, y, text);
    display.display();
}

void oled_clear() {
    display.clear();
    display.display();
}