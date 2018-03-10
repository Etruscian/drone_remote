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
DigitalIn switch1Pin(D3, PullDown), switch2Pin(D5, PullDown);
nRF24L01P radio(D11, D12, D13, D10, D9);
Adafruit_SSD1306_I2c display(i2cDevice, D2);

_floatUint rxBuffer, gimbalValues[4];
char sendBuffer[18];
uint8_t status, pos = 0, signalStrengthArray[100], signalStrengthRaw, packetNotReceivedCounter;
volatile uint8_t signalStrength;
uint16_t sum = 0;
float throttle, roll, pitch, yaw;
volatile bool switch1, switch2, oldSwitch1 = false, oldSwitch2, packetReceived;

void interruptHandler(void)
{
    status = radio.getStatusRegister();
    if (status == 0)
    { //status not ready
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

    signalStrength++; // = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), signalStrengthRaw);
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

    if (switch1)
    {
        writeText(&display, const_cast<char *>("ARMED"), 5, 98, 0);
    }
    else
    {
        writeText(&display, const_cast<char *>("SAFE "), 5, 98, 0);
    }

    if (switch2)
    {
        writeText(&display, const_cast<char *>("STABLE"), 6, 0, 0);
    }
    else
    {
        writeText(&display, const_cast<char *>("ACRO  "), 6, 0, 0);
    }

    writeFloat(&display, battery.read() * 14.20, 4, 15, 14);

    if (packetReceived)
    {
        writeFloat(&display, rxBuffer.f, 4, 15, 24);
        packetReceived = false;
        packetNotReceivedCounter = 0;
    }
    else if (!packetNotReceivedCounter)
    {
        writeText(&display, const_cast<char *>("----"), 4, 15, 24);
        packetNotReceivedCounter++;
    }

    if (signalStrength <= 5)
    {
        writeText(&display, const_cast<char *>("---"), 3, 110, 14);
    }
    else if ((signalStrength > 5) && (signalStrength <= 20))
    {
        writeText(&display, const_cast<char *>("LOW"), 3, 110, 14);
    }
    else
    {
        char text[10];
        sprintf(text, " %u", signalStrength);
        if (signalStrength < 100)
            writeText(&display, text, 3, 110, 14);
        else
            writeText(&display, &text[1], 3, 110, 14);
    }
    display.display();
}

int main()
{
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.setRotation(2);
    display.command(0x00 | 0x0); // low col = 0
    display.command(0x10 | 0x0); // hi col = 0
    display.command(0x40 | 0x0); // line #0

    uint8_t statRegister = radio.getStatusRegister();
    if ((statRegister != 0x08) && (statRegister != 0x0e) && (statRegister != 0x0f))
    {
        writeText(&display, const_cast<char *>("TX ERROR"), 8, 70, 0);
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

        writeText(&display, const_cast<char *>("TX"), 2, 0, 14);
        writeText(&display, const_cast<char *>("RX"), 2, 0, 24);
        writeText(&display, const_cast<char *>("TXRX"), 4, 83, 14);
        display.drawLine(0,10,128,10,WHITE);

        display.display();
        radioTicker.attach(&radioLoop, 0.01);
        screenLoop();
        screenTicker.attach(&screenLoop, 0.1);
    }
}