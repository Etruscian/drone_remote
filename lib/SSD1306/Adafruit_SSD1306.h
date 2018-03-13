/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*
 *  Modified by Neal Horman 7/14/2012 for use in mbed
 */

#ifndef _ADAFRUIT_SSD1306_H_
#define _ADAFRUIT_SSD1306_H_

#include "mbed.h"
#include "Adafruit_GFX.h"

#include <vector>
#include <algorithm>

// A DigitalOut sub-class that provides a constructed default state
class DigitalOut2 : public DigitalOut
{
public:
	DigitalOut2(PinName pin, bool active = false) : DigitalOut(pin) { write(active); };
	DigitalOut2& operator= (int value) { write(value); return *this; };
	DigitalOut2& operator= (DigitalOut2& rhs) { write(rhs.read()); return *this; };
	operator int() { return read(); };
};

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

/** The pure base class for the SSD1306 display driver.
 *
 * You should derive from this for a new transport interface type,
 * such as the SPI and I2C drivers.
 */
class Adafruit_SSD1306 : public Adafruit_GFX
{
public:
	Adafruit_SSD1306(PinName RST, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
		: Adafruit_GFX(rawWidth,rawHeight)
		, rst(RST,false)
	{
		buffer[0] = 0x0;
		buffer[1] = 0x00|0x0;
		buffer[2] = 0x0;
		buffer[3] = 0x10|0x0;
		buffer[4] = 0x0;
		buffer[5] = 0x40|0x0;
		buffer[6] = 0x40;
	};

	void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC);
	
	// These must be implemented in the derived transport driver
	virtual void command(uint8_t c) = 0;
	virtual void commandAsync(uint8_t c) = 0;
	virtual void data(uint8_t c) = 0;
	virtual void drawPixel(int16_t x, int16_t y, uint16_t color);

	/// Clear the display buffer    
	void clearDisplay(void);
	virtual void invertDisplay(bool i);

	/// Cause the display to be updated with the buffer content.
	void display();
	/// Fill the buffer with the AdaFruit splash screen.
	virtual void splash();
    
protected:
	virtual void sendDisplayBuffer() = 0;
	DigitalOut2 rst;

	// the memory buffer for the LCD
	char buffer[32*128/8 + 1];
};


/** This is the SPI SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_Spi : public Adafruit_SSD1306
{
public:
	/** Create a SSD1306 SPI transport display driver instance with the specified DC, RST, and CS pins, as well as the display dimentions
	 *
	 * Required parameters
	 * @param spi - a reference to an initialized SPI object
	 * @param DC (Data/Command) pin name
	 * @param RST (Reset) pin name
	 * @param CS (Chip Select) pin name
	 *
	 * Optional parameters
	 * @param rawHeight - the vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - the horizonal number of pixels for the display, defaults to 128
	 */
	Adafruit_SSD1306_Spi(SPI &spi, PinName DC, PinName RST, PinName CS, uint8_t rawHieght = 32, uint8_t rawWidth = 128)
	    : Adafruit_SSD1306(RST, rawHieght, rawWidth)
	    , cs(CS,true)
	    , dc(DC,false)
	    , mspi(spi)
	    {
		    begin();
		    splash();
		    display();
	    };

	void command(uint8_t c)
	{
	    cs = 1;
	    dc = 0;
	    cs = 0;
	    mspi.write(c);
	    cs = 1;
	};

	void commandAsync(uint8_t c)
	{
	    cs = 1;
	    dc = 0;
	    cs = 0;
	    mspi.write(c);
	    cs = 1;
	};

	void data(uint8_t c)
	{
	    cs = 1;
	    dc = 1;
	    cs = 0;
	    mspi.write(c);
	    cs = 1;
	};

protected:
	void sendDisplayBuffer()
	{
		cs = 1;
		dc = 1;
		cs = 0;

		for(uint16_t i=0, q=sizeof(buffer); i<q; i++)
			mspi.write(buffer[i]);

		if(height() == 32)
		{
			for(uint16_t i=0, q=sizeof(buffer); i<q; i++)
				mspi.write(0);
		}

		cs = 1;
	};

	DigitalOut2 cs, dc;
	SPI &mspi;
};

/** This is the I2C SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_I2c : public Adafruit_SSD1306
{
public:
	#define SSD_I2C_ADDRESS     0x78
	/** Create a SSD1306 I2C transport display driver instance with the specified RST pin name, the I2C address, as well as the display dimensions
	 *
	 * Required parameters
	 * @param i2c - A reference to an initialized I2C object
	 * @param RST - The Reset pin name
	 *
	 * Optional parameters
	 * @param i2cAddress - The i2c address of the display
	 * @param rawHeight - The vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - The horizonal number of pixels for the display, defaults to 128
	 */
	Adafruit_SSD1306_I2c(I2C &i2c, PinName RST, uint8_t i2cAddress = SSD_I2C_ADDRESS, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
	    : Adafruit_SSD1306(RST, rawHeight, rawWidth)
	    , mi2c(i2c)
	    , mi2cAddress(i2cAddress)
	    {
		    begin();
		    splash();
		    display();
			mi2c.frequency(1000000);
	    };

	void command(uint8_t c)
	{
		char buff[2];
		buff[0] = 0; // Command Mode
		buff[1] = c;
		mi2c.write(mi2cAddress, buff, sizeof(buff));
	};

	void commandAsync(uint8_t c)
	{
		char buff[2];
		buff[0] = 0; // Command Mode
		buff[1] = c;
		transferComplete = false;
		mi2c.transfer(mi2cAddress, buff, 2, buff, 0, callback(this,&Adafruit_SSD1306_I2c::i2cInterruptHandler), I2C_EVENT_ALL);
	};

	void data(uint8_t c)
	{
		char buff[2];
		buff[0] = 0x40; // Data Mode
		buff[1] = c;
		mi2c.write(mi2cAddress, buff, sizeof(buff));
	};

	void i2cInterruptHandler(int callback){
		if (callback == I2C_EVENT_TRANSFER_COMPLETE){
			transferComplete = true;
		}
		if (callback == I2C_EVENT_ERROR){
			transferComplete = true;
		}
		if (callback == I2C_EVENT_ERROR_NO_SLAVE){
			transferComplete = true;
		}
		if (callback == I2C_EVENT_TRANSFER_EARLY_NACK){
			transferComplete = true;
		}
	};

	uint8_t i2cTransferStatus;
	
protected:	
	void sendDisplayBuffer()
	{
		// if (transferComplete){
			// mi2c.abort_transfer();
			transferComplete = false;

			i2cTransferStatus = mi2c.transfer(mi2cAddress, buffer, sizeof(buffer), buffer, 0, callback(this,&Adafruit_SSD1306_I2c::i2cInterruptHandler), I2C_EVENT_ALL);
		// }
	};

	I2C &mi2c;
	uint8_t mi2cAddress;
};

#endif