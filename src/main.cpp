#include <mbed.h>
#include "nRF24L01P.hpp"
#include "Adafruit_SSD1306.h"
#include "helpers.hpp"
#include "stdio.h"

Serial pc(USBTX, USBRX);

InterruptIn radioInterrupt(D8);
Ticker radioTicker, screenTicker;
I2C i2cDevice(PB_7, PB_6);
AnalogIn throttlePin(A1), rollPin(A3), pitchPin(A2), yawPin(A0), battery(A6);
DigitalIn switch1Pin(D4, PullUp), switch2Pin(D5, PullUp);
nRF24L01P radio(D11, D12, D13, D10, D9);
Adafruit_SSD1306_I2c display(i2cDevice, D2);

_floatUint rxBuffer, gimbalValues[4];
char sendBuffer[18];
uint8_t status, pos = 0, signalStrengthArray[100], signalStrengthRaw, packetNotReceivedCounter;
volatile uint8_t signalStrength;
uint16_t sum = 0;
float throttle, roll, pitch, yaw;
volatile bool switch1, switch2, oldSwitch1, oldSwitch2, packetReceived;

uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
    //Subtract the oldest number from the prev sum, add the new number
    *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
    //Assign the nextNum to the position in the array
    ptrArrNumbers[pos] = nextNum;
    //return the average
    return *ptrSum / len;
}

void interruptHandler(void)
{
    status = radio.getStatusRegister();
    if (status == 0)
    { //sendBuffer not ready?
        while (status == 0)
        {
            status = radio.getStatusRegister();
        }
    }

    if (status & 1)
    { // TX FIFO full
        radio.disable();
        radio.flushTX();
    }
    if (status & 16)
    { // max TX retransmits
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07, 16);
        signalStrengthRaw = 0;
    }
    if (status & 32)
    { // TX sent (ACK package available if autoAck is enabled)
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07, 32);
        signalStrengthRaw = 100;
    }
    if (status & 64)
    { // RX received
        radio.read((status & 14) >> 1, (char *)rxBuffer.c, 4);
        radio.setRegister(0x07, 64);
        packetReceived = true;
    }

    signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), signalStrengthRaw);
    pos++;
    if (pos >= 100)
    {
        pos = 0;
    }
}

void radioLoop(void)
{
    switch1 = true; //(bool)switch1Pin;
    switch2 = true; //(bool)switch2Pin;

    gimbalValues[0].f = throttlePin.read();
    gimbalValues[1].f = (rollPin.read() - 0.54) / 45 * 100;
    gimbalValues[2].f = -1 * (pitchPin.read() - 0.5) * 2;
    gimbalValues[3].f = (yawPin.read() - 0.55) / 47 * 100;

    for (int i = 0; i < 16; i++)
    {
        sendBuffer[i] = gimbalValues[i / 4].c[i & 3];
    }
    sendBuffer[16] = ((switch1 & 1) << 1) | (switch2 & 1);

    radio.write(NRF24L01P_PIPE_P0, &sendBuffer[0], 17);
}

void screenLoop(void)
{
    if (switch1 != oldSwitch1)
    {
        if (switch1)
        {
            writeText(&display, "ARMED", 5, 0, 0);
        }
        else
        {
            writeText(&display, "SAFE ", 5, 0, 0);
        }
        oldSwitch1 = switch1;
    }

    if (switch2 != oldSwitch2)
    {
        if (switch2)
        {
            writeText(&display, "STABLE", 6, 0, 10);
        }
        else
        {
            writeText(&display, "ACRO  ", 6, 0, 10);
        }
        oldSwitch2 = switch2;
    }

    char test[10];
    float batteryLevel = battery.read() * 14.20;
    sprintf(test, "%u.%u", (uint16_t)((batteryLevel)), (uint16_t)((batteryLevel * 100)) & 99);
    writeText(&display, test, 4, 85, 0);

    if (packetReceived)
    {
        sprintf(test, "%u.%u", (uint16_t)(rxBuffer.f), (uint16_t)((rxBuffer.f * 100)) & 99);
        writeText(&display, test, 4, 85, 10);
        packetReceived = false;
        packetNotReceivedCounter = 0;
    }
    else if (!packetNotReceivedCounter)
    {
        writeText(&display, "----", 4, 85, 10);
        packetNotReceivedCounter++;
    }

    display.setTextCursor(85, 20);

    if (signalStrength <= 5)
    {
        writeText(&display, "---", 3, 85, 20);
    }
    else if ((signalStrength > 5) && (signalStrength <= 20))
    {
        writeText(&display, "LOW", 3, 85, 20);
    }
    else
    {
        char text[10];
        sprintf(text, "%u", signalStrength);
        writeText(&display, text, 3, 85, 20);
    }

    pc.printf("%u\n", display.transferComplete);
    display.display();
}

int main()
{
    i2cDevice.frequency(400000);
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.setRotation(2);
    display.command(0x00 | 0x0); // low col = 0
    display.command(0x10 | 0x0); // hi col = 0
    display.command(0x40 | 0x0); // line #0

    uint8_t statRegister = radio.getStatusRegister();
    if ((statRegister != 0x08) && (statRegister != 0x0e) && (statRegister != 0x0f))
    {
        writeText(&display, "TX ERROR", 8, 70, 0);
        char text[10];
        sprintf(text, "STATUS %u", statRegister);
        writeText(&display, text, 10, 46, 10);
        display.display();
    }
    else
    {
        radio.powerUp();
        radio.setRfFrequency(2400 + 101);
        radio.setTransferSize(10);
        radio.setCrcWidth(16);
        radio.setTxAddress(0x007DEADBEE);
        radio.setRxAddress(0x007DEADBEE);
        radio.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
        radio.setAirDataRate(NRF24L01P_DATARATE_250_KBPS);
        radio.enableAutoRetransmit(1000, 3);
        radio.setTransmitMode();
        radioInterrupt.fall(&interruptHandler);

        writeText(&display, "TX", 2, 70, 0);
        writeText(&display, "RX", 2, 70, 10);
        writeText(&display, "TXRX", 4, 58, 20);

        display.display();
        radioTicker.attach(&radioLoop, 0.01);
        screenLoop();
        screenTicker.attach(&screenLoop, 0.1);
    }
}