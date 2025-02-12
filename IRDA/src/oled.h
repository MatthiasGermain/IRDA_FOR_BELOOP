#ifndef OLED_H
#define OLED_H
  
void oled_init();
void oled_display_text(const char* text, int x, int y);
void oled_clear();
#endif // OLED_H
