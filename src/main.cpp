#include <mbed.h>
#include "nRF24L01P.hpp"
#include "Adafruit_SSD1306.h"
#include "displayHelper.hpp"
#include "stdio.h"

Serial pc(USBTX, USBRX);

InterruptIn radioInterrupt(D8);
Ticker ticker, ticker2;
I2C i2cDevice(PB_7, PB_6);
AnalogIn throttlePin(A0), rollPin(A1), pitchPin(A2), yawPin(A3), battery(A6);
DigitalIn switch1Pin(D4), switch2Pin(D5);
nRF24L01P radio(D11, D12, D13, D10, D9);
Adafruit_SSD1306_I2c display(i2cDevice, D2);

typedef union _floatUintUnion {
    float f;
    char c[4];
} _floatUint;

_floatUint rxBuffer;
char data[10];
uint8_t status, pos = 0, signalStrengthArray[10], signalStrengthRaw;
volatile uint8_t  signalStrength;
uint16_t sum = 0;
float throttle, roll, pitch, yaw;
bool switch1, switch2, packetReceived;

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
    { //data not ready?
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
        radio.read((status & 14) >> 1, &rxBuffer.c[0], 4);
        radio.setRegister(0x07, 64);
        packetReceived = true;
    }
    signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), signalStrengthRaw);
    // pos = (pos+1) & 49;
    pos++;
    if (pos >=10){
        pos = 0;
    }
}

void mainLoop(void)
{
    throttle = throttlePin.read();
    roll = rollPin.read();
    pitch = pitchPin.read();
    yaw = yawPin.read();
    switch1 = switch1Pin.read();
    switch2 = switch2Pin.read();

    // data[0] = (throttle & 0xFF);
    // data[1] = throttle >> 8;
    // data[2] = (roll & 0xFF);
    // data[3] = roll >> 8;
    // data[4] = (pitch & 0xFF);
    // data[5] = pitch >> 8;
    // data[6] = (yaw & 0xFF);
    // data[7] = yaw >> 8;
    data[8] = ((switch1 & 1) << 1) | (switch2 & 1);

    radio.write(NRF24L01P_PIPE_P0, &data[0], 9);
}

void screenLoop(void)
{
    if (switch1){
        writeText(&display, "ARMED", 5, 0,0);
    } else {
        writeText(&display, "SAFE", 4, 0,0);
    }

    if (switch2){
        writeText(&display, "STABLE", 6, 0,10);
    } else {
        writeText(&display, "ACRO", 4, 0,10);
    }
    char test[10];
    float batteryLevel = battery.read() * 14.20;
    sprintf(test, "%u.%u", (uint16_t)((batteryLevel)),(uint16_t)((batteryLevel*100)) & 99);
    writeText(&display, test , 4, 85,0);

    if (packetReceived)
    {
        sprintf(test, "%u.%u", (uint16_t)(rxBuffer.f),(uint16_t)((rxBuffer.f * 100)) & 99);
        writeText(&display, test , 4, 85,10);
        packetReceived = false;
    } else {
        writeText(&display, "----", 4, 85, 10)
    }

    display.setTextCursor(85, 20);

    if (signalStrength <= 5)
    {
        writeText(&display, "---", 3, 85,20);
    }
    else if ((signalStrength > 5) && (signalStrength <= 20))
    {
        writeText(&display, "LOW", 3, 85,20);
    }
    else
    {   char text[10];
        sprintf(text, "%u", signalStrength);
        writeText(&display, text , 3, 85,20);
    }
    display.display();
}

int main()
{
    i2cDevice.frequency(400000);
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.setRotation(2);

    uint8_t statRegister = radio.getStatusRegister();
    if ((statRegister != 0x08) && (statRegister != 0x0e) && (statRegister != 0x0f))
    {
        writeText(&display, "TX ERROR", 8, 70,0);
        char text[10];
        sprintf(text, "STATUS %u" ,statRegister);
        writeText(&display, text , 10, 46,10);
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
        // radio.setRegister(0x1D,6);
        radio.setTransmitMode();
        radioInterrupt.fall(&interruptHandler);

        writeText(&display, "TX", 2, 70,0);
        writeText(&display, "RX", 2, 70,10);
        writeText(&display, "TXRX", 4, 58,20);

        display.display();
        ticker.attach(&mainLoop, 0.05);
        screenLoop();
        ticker2.attach(&screenLoop, 0.5);
        // while(true){
            // wait(1);
        // }
    }
}