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

void writeFloat(Adafruit_SSD1306_I2c * display, float number, uint16_t length, uint8_t x, uint8_t y){
    char string[10];
    sprintf(string, "%u.%02u", (uint16_t)((number)), (uint16_t)((number-(uint16_t)number)*100));
    writeText(display, string, length, x, y);
};

uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
    //Subtract the oldest number from the prev sum, add the new number
    *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
    //Assign the nextNum to the position in the array
    ptrArrNumbers[pos] = nextNum;
    //return the average
    return *ptrSum / len;
};