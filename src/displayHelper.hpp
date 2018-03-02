#include <mbed.h>

void writeText(Adafruit_SSD1306_I2c * display, char * text, uint16_t length, uint8_t x, uint8_t y){
    (*display).setTextCursor(x,y);
    for (int i = 0; i<length; i++){
        (*display).writeChar(*text++);
    }
};

void writeFloat(Adafruit_SSD1306_I2c * display, float number, uint16_t length, uint8_t x, uint8_t y){
    (*display).setTextCursor(x,y);
    uint16_t tempNumber = (uint16_t)number*10;
    (*display).writeChar((char)((tempNumber/100) & 9 )+48);
    (*display).writeChar((char)((tempNumber/10) & 9 )+48);
    (*display).writeChar('.');
    (*display).writeChar((char)((tempNumber) & 9 )+48);
}