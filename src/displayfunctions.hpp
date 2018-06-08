#include <mbed.h>
#include "Adafruit_SSD1306.h"
#include "Watchdog.h"

typedef union _floatUintUnion {
    float f;
    char c[4];
} _floatUint;

extern Adafruit_SSD1306_I2c display;
extern Watchdog watchdog;
extern volatile bool switch1, switch2, switch3, switch4, packetReceived;
uint8_t counterTX, counterRX, packetNotReceivedCounter = 0;
extern volatile uint8_t signalStrength;
extern _floatUint rxBuffer;

AnalogIn battery(A6);
float txBatteryLevel;

void writeText(char * text, uint16_t length, uint8_t x, uint8_t y){
    display.setTextCursor(x,y);
    for (int i = 0; i<length; i++){
        display.writeChar(*text++);
    }
};

void writeFloat(float number, uint16_t length, uint8_t x, uint8_t y){
    char string[10];
    sprintf(string, "%u.%02u", (uint16_t)((number)), (uint16_t)((number-(uint16_t)number)*100));
    writeText(string, length, x, y);
};

void screenLoop(void)
{
    watchdog.Service();
    if (switch1)
    {
        writeText(const_cast<char *>("ARMED"), 5, 98, 0);
    }
    else
    {
        writeText(const_cast<char *>("SAFE "), 5, 98, 0);
    }

    if (switch2)
    {
        writeText(const_cast<char *>("STABLE"), 6, 0, 0);
    }
    else
    {
        writeText(const_cast<char *>("ACRO  "), 6, 0, 0);
    }

    if (switch3)
    {
        writeText(const_cast<char *>("TEST"), 4, 40, 0);
    }
    else
    {
        writeText(const_cast<char *>("TEST"), 4, 40, 0);
    }

    if (switch4)
    {
        writeText(const_cast<char *>("TEST"), 4, 70, 0);
    }
    else
    {
        writeText(const_cast<char *>("TEST"), 4, 70, 0);
    }

    txBatteryLevel = battery.read() * 3.3 / (220.0 / (220.0 + 473.0));
    if (txBatteryLevel <= 6.8)
    {
        counterTX++;
        if (counterTX == 5)
        {
            display.setTextColor(BLACK, WHITE);
            writeFloat(txBatteryLevel, 4, 15, 14);
            display.setTextColor(WHITE, BLACK);
        }
        else if (counterTX == 10)
        {
            display.setTextColor(WHITE, BLACK);
            writeFloat(txBatteryLevel, 4, 15, 14);
            counterTX = 0;
        }
    } else {
        display.setTextColor(WHITE, BLACK);
        writeFloat(txBatteryLevel, 4, 15, 14);
        counterTX = 0;
    }

    if (packetReceived)
    {
        if (rxBuffer.f < 13.6)
        {
            counterRX++;
            if (counterRX == 5)
            {
                display.setTextColor(BLACK, WHITE);
                writeFloat(rxBuffer.f, 4, 15, 24);
                display.setTextColor(WHITE, BLACK);
            }
            else if (counterRX == 10)
            {
                display.setTextColor(WHITE, BLACK);
                writeFloat(rxBuffer.f, 4, 15, 24);
                counterRX = 0;
            }
        } else {
            display.setTextColor(WHITE, BLACK);
            writeFloat(rxBuffer.f, 4, 15, 24);
            counterRX = 0;
        }


        packetReceived = false;
        packetNotReceivedCounter = 0;
    }
    else if (!packetNotReceivedCounter)
    {
        writeText(const_cast<char *>("----"), 4, 15, 24);
        packetNotReceivedCounter++;
    }

    if (signalStrength <= 5)
    {
        writeText(const_cast<char *>("---"), 3, 110, 14);
    }
    else if ((signalStrength > 5) && (signalStrength <= 20))
    {
        writeText(const_cast<char *>("LOW"), 3, 110, 14);
    }
    else
    {
        char text[10];
        sprintf(text, " %u", signalStrength);
        if (signalStrength < 100)
            writeText(text, 3, 110, 14);
        else
            writeText(&text[1], 3, 110, 14);
    }
    display.display();
};