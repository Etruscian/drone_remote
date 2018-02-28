#include <mbed.h>
#include "nRF24L01P.hpp"
#include "Adafruit_SSD1306.h"

InterruptIn radioInterrupt(D8);
Ticker ticker;
I2C i2cDevice(PB_7, PB_6);
AnalogIn throttlePin(A0), rollPin(A1), pitchPin(A2), yawPin(A3), battery(A6);
DigitalIn switch1Pin(D4), switch2Pin(D5);

nRF24L01P radio(D11,D12,D13,D10,D9);
Adafruit_SSD1306_I2c display(i2cDevice,D2);

char rxBuffer[10], data[10];
uint8_t status, pos = 0, signalStrengthArray[256], signalStrength, signalStrengthRaw;
uint16_t throttle, roll, pitch, yaw, sum = 0;
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

void interruptHandler(void){
    status = radio.getStatusRegister();
    if (status == 0){ //data not ready?
        while (status == 0 ){
            status = radio.getStatusRegister();
        }
    }

    if (status & 1){ // TX FIFO full
        radio.disable();
        radio.flushTX();
    }
    if (status & 16){ // max TX retransmits
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07,16);
        signalStrengthRaw = 0;
    }
    if (status & 32){ // TX sent (ACK package available if autoAck is enabled)
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07,32);
        signalStrengthRaw = 100;
    }
    if (status & 64){ // RX received
        radio.read((status & 14) >> 1, &rxBuffer[0],2);
        radio.setRegister(0x07,64);
        packetReceived = true;
    }

    signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), signalStrengthRaw);
    pos++;
}

void mainLoop(void){
    throttle = throttlePin.read();
    roll = rollPin.read();
    pitch = pitchPin.read();
    yaw = yawPin.read();
    switch1 = switch1Pin.read();
    switch2 = switch2Pin.read();

    data[0]= (throttle & 0xFF);
    data[1]= throttle >> 8;
    data[2]= (roll & 0xFF);
    data[3]= roll >> 8;
    data[4]= (pitch & 0xFF);
    data[5]= pitch >> 8;
    data[6]= (yaw & 0xFF);
    data[7]= yaw >> 8;
    data[8] = ((switch1 & 1) << 1) | (switch2 & 1);

    radio.write(NRF24L01P_PIPE_P0, &data[0], 10);
}

void screenLoop(void){
    display.setTextCursor(75,0);
    display.writeChar((char)(battery.read()*10.35)+48);
    display.writeChar('.');
    display.writeChar((char)((uint8_t)(battery.read()*103.5) & 9)+48);
    if (packetReceived){
        display.setTextCursor(75,10);
        float rxBatteryLevel = *(reinterpret_cast<float*>(rxBuffer));
        display.writeChar((char)(rxBatteryLevel)+48);
        display.writeChar('.');
        display.writeChar((char)((uint8_t)(rxBatteryLevel) & 9)+48);
        packetReceived = false;
    }
    display.setTextCursor(75,20);
    display.writeChar((char)(signalStrength & 9)+48);
    display.writeChar((char)((signalStrength/10) & 9)+48);
    display.writeChar((char)((signalStrength/100) & 1)+48);
    display.display();
}

int main() {
    radio.powerUp();
    radio.setRfFrequency(2400 + 50);
    radio.setTransferSize(10);
    radio.setCrcWidth(16);
    radio.setTxAddress(0x007FFFFFFF);
    radio.setRxAddress(0x007FFFFFFF);
    radio.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
    radio.setAirDataRate(NRF24L01P_DATARATE_250_KBPS);
    radio.enableAutoRetransmit(500, 3);
    radio.setTransmitMode();
    radioInterrupt.fall(&interruptHandler);

    i2cDevice.frequency(400000);
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.setRotation(2);
    display.setTextCursor(60,0);
    display.writeChar('T');
    display.writeChar('X');

    display.setTextCursor(60,10);
    display.writeChar('R');
    display.writeChar('X');


    display.setTextCursor(48,20);
    display.writeChar('T');
    display.writeChar('X');
    display.writeChar('R');
    display.writeChar('X');

    display.display();
    ticker.attach(&mainLoop, 0.01);
    ticker.attach(&screenLoop, 0.5);
}