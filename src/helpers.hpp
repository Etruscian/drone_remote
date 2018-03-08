#include <mbed.h>

typedef union _floatUintUnion {
    float f;
    char c[4];
} _floatUint;

void writeText(Adafruit_SSD1306_I2c * display, char * text, uint16_t length, uint8_t x, uint8_t y){
    (*display).setTextCursor(x,y);
    for (int i = 0; i<length; i++){
        (*display).writeChar(*text++);
    }
};