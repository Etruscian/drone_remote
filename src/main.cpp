#include <mbed.h>
#include "Watchdog.h"
#include "nRF24L01P.hpp"
#include "Adafruit_SSD1306.h"
#include "displayfunctions.hpp"
#include "helpers.hpp"
#include "stdio.h"

Serial pc(USBTX, USBRX);

Watchdog watchdog;
InterruptIn radioInterrupt(D8);
Ticker radioTicker, screenTicker, screenBlinkTicker;
I2C i2cDevice(PB_7, PB_6);
AnalogIn throttlePin(A1), rollPin(A3), pitchPin(A2), yawPin(A0);
DigitalIn switch1Pin(D6, PullUp), switch2Pin(D3, PullUp), switch3Pin(D7), switch4Pin(A7);
nRF24L01P radio(D11, D12, D13, D10, D9);
Adafruit_SSD1306_I2c display(i2cDevice, D2);

_floatUint rxBuffer, gimbalValues[4];
char sendBuffer[18];
uint8_t status, pos = 0, signalStrengthArray[100], signalStrengthRaw;
volatile uint8_t signalStrength;
uint16_t sum = 0;
float throttle, roll, pitch, yaw;
volatile bool switch1, switch2, switch3, switch4, packetReceived;

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

    if (status & 0b00000001)
    { // TX FIFO full
        radio.disable();
        radio.flushTX();
    }
    if (status & 0b00010000)
    { // max TX retransmits
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07, 0b00010000);
        signalStrengthRaw = 0;
    }
    if (status & 0b00100000)
    { // TX sent (ACK package available if autoAck is enabled)
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07, 0b00100000);
        signalStrengthRaw = 100;
    }
    if (status & 0b01000000)
    { // RX received
        radio.read((status & 14) >> 1, (char *)rxBuffer.c, 4);
        radio.setRegister(0x07, 0b01000000);
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
    switch1 = switch1Pin.read();
    switch2 = switch2Pin.read();
    switch3 = switch3Pin.read();
    switch4 = switch4Pin.read();

    gimbalValues[0].f = throttlePin.read();
    gimbalValues[1].f = (rollPin.read() - 0.544) / 45 * 100;
    gimbalValues[2].f = (pitchPin.read() - 0.5) * 2;
    gimbalValues[3].f = -1*((yawPin.read() - 0.585) / 47 * 100);

    for (int i = 0; i < 16; i++)
    {
        sendBuffer[i] = gimbalValues[i / 4].c[i & 3];
    }
    sendBuffer[16] = ((switch4 & 1) << 3) | ((switch3 & 1) << 2) | ((switch1 & 1) << 1) | (switch2 & 1);
    radio.write(NRF24L01P_PIPE_P0, &sendBuffer[0], 17);
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
        writeText(const_cast<char *>("TX ERROR"), 8, 70, 0);
        char text[10];
        sprintf(text, "STATUS %u", statRegister);
        writeText(text, 10, 46, 10);
        display.display();
    }
    else
    {
        radio.powerUp();
        radio.setRfFrequency(2400 + 101);
        radio.setTransferSize(17);
        radio.setCrcWidth(16);
        radio.setTxAddress(0x007deadbee);
        radio.setRxAddress(0x007deadbee);
        radio.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
        radio.setAirDataRate(NRF24L01P_DATARATE_250_KBPS);
        radio.enableAutoRetransmit(1000, 3);
        radio.setTransmitMode();
        radioInterrupt.fall(&interruptHandler);

        writeText(const_cast<char *>("TX"), 2, 0, 14);
        writeText(const_cast<char *>("RX"), 2, 0, 24);
        writeText(const_cast<char *>("TXRX"), 4, 83, 14);
        display.drawLine(0, 10, 128, 10, WHITE);
        // display.drawLine(36,0,36,10,WHITE);
        // display.drawLine(70,0,70,10,WHITE);
        // display.drawLine(70,0,70,10,WHITE);

        display.display();
        radioTicker.attach(&radioLoop, 0.01);
        screenLoop();
        screenTicker.attach(&screenLoop, 0.1);
        watchdog.Configure(0.5);
    }
}