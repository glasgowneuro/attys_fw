/**
   Attys firmware
   Copyright (C) 2016, Bernd Porr, mail@berndporr.me.uk
   
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
**/

#define FW_VERSION "0.99"

// for debugging
// #define FAKE_ADC_DATA

#include <msp430.h>
#include <stdio.h>
#include "mpu9250.h"
#include "base64.h"

// clock speeds for the ADC
// 16Mhz / 250
#define ADC_CLOCK_SLOW 250
// 16MHz / 2
#define ADC_CLOCK_FAST 2

// uart character available?
unsigned char uart_rx_avail = 0;
// character which has been received
unsigned char uart_rx_char = 0;

// SPI character after an SPI transfer
unsigned char spi_rx_avail = 0;
unsigned char spi_rx_char = 0;

// don't interpret the rx characters as commands
unsigned char ignore_rx = 0;

// send OK
unsigned char doSendOK = 0;

// all the data which is transmitted
// if it's in binary form then it's exactly in this format
// needs to be packed so that it's as small as possible
struct __attribute__((__packed__)) bin_data_t {
        uint32_t adc_ch1 : 24;
	uint32_t adc_ch2 : 24;
	
	uint8_t adc_gpio;
	uint8_t timestamp;

	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;

	uint16_t mag_x;
	uint16_t mag_y;
	uint16_t mag_z;

} alldata;

// for higher data rates we only transmit the ADC
// but not the acc/mag
struct __attribute__((__packed__)) {
	uint32_t adc_ch1 : 24;
	uint32_t adc_ch2 : 24;
	
	uint8_t adc_gpio;
	uint8_t timestamp;
} adc_data_t;

#define WATCHDOGINIT 255
volatile static uint8_t watchdog = WATCHDOGINIT;

char sendBuffer[80];
volatile static uint8_t hasData = 0;

// this is a counter which turns the green LED off
uint16_t powergoodoff = 10;

// if one data is sent via bluetooth
volatile static unsigned char send_data = 0;

// 0=CSV, 1=base64 binary format
unsigned char base64output = 0;

// send all channels or just the ADC
unsigned char send_all_data = 1;

// if one every successful command replies with "OK"
unsigned char verbose = 1;

// self test status: 1=test successful, 0=error
unsigned char adc_stat;
unsigned char acc_stat;
unsigned char mag_stat;

// the whoami registers of the acc/mag
uint8_t mpu9250_AK8963_whoami = 0;
uint8_t mpu9250_whoami = 0;

// the buffer which receives the commands
#define CONFIG_BUFFER_SIZE 8
unsigned char config_buffer[CONFIG_BUFFER_SIZE];
uint16_t config_ptr = 0;

// forward declations
void uart_tx(unsigned char c);
void sendText(const unsigned char *txt);
void trigOK();

// brute force delay
void delay(uint16_t n)
{
	uint16_t i;

	for(i=0;i<n;i++) {
		volatile uint32_t r = i*10;
	}
}


// make the power LED go off for a short while
void flashPowerLED()
{
	// toggles the power LED for about a second
	powergoodoff = 10;
}


// fatal condition makes green LED flash forever
void fatalLED()
{
	// interrupts off
	__dint();
	// flash forever
	while (1) {
		// switch it off and count down
		delay(30000);
		P2OUT &= ~0x80;
		delay(30000);
		P2OUT |= 0x80;
	}
}


// is called from the main interrupt and is counted
// down till zero and then the LED is switched back on
void controlPowerLED()
{
	// flash power LED
	if (powergoodoff) {
		// switch it off and count down
		P2OUT &= ~0x80;
		powergoodoff--;
	} else {
		// switch it on
		P2OUT |= 0x80;
	}
}


// SPI transmission and reception
// this is blocking until the transmission has
// finished
unsigned char spi_txrx(unsigned char tx)
{
	volatile unsigned char rx;

	// in case a transfer is still ongoing (it shouldn't)
	// we wait
	while (!(IFG2 & UCB0TXIFG));
	// trasmit a byte
	UCB0TXBUF = tx;
	// wait for the byte to arrive
	while (!(IFG2 & UCB0RXIFG));
	// and read the received byte
	rx = UCB0RXBUF;
	return rx;
}




// ADS1292: ADC
// init the adc
// this can be called any time
// argument is the divisor for the SPI clock
void adc_init_spi(unsigned char d)
{
	UCB0BR0 = d;
	// inactive state of the SPI is low
	UCB0CTL0 &= ~UCCKPL;
	// Phase=0, data is already ready after /CS
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

}

// send a command to the ADC
// the delay is very, very important
void adc_command(unsigned char value)
{
	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	// send out to SPI
	spi_txrx(value);

	// wait for the CS to go back high again
	// very important
	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


// writing to an ADC register
// delay very important for operation
void adc_write_reg(unsigned char index, unsigned char value)
{
	// register writes are only possible at low
	// clock rates
	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	// bit 6 indicates a write
	spi_txrx(index | 0x40);
	spi_txrx(0x00);
	spi_txrx(value);

	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


/////////////////
// ADS1292

// reading a register
unsigned char adc_read_reg(unsigned char index)
{
	unsigned char ret;

	adc_init_spi(ADC_CLOCK_SLOW);

	// CS to low
	P1OUT &= ~BIT4;

	// disable continous read
	spi_txrx(0x11);

	// wait for the CS to go back high again
	// very important
	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	delay(1000);

        // CS to low
	P1OUT &= ~BIT4;

	// indicates a register read
	spi_txrx(index | 0x20);
	spi_txrx(0x00);
	ret = spi_txrx(0x00);

	delay(1000);

	// CS to high
	P1OUT |= BIT4;

	delay(1000);

	// CS to low
	P1OUT &= ~BIT4;

	// enable continous read
	spi_txrx(0x10);

	// wait for the CS to go back high again
	// very important
	delay(1000);

	// CS to high
	P1OUT |= BIT4;	

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
	
	return ret;
}


// read both channels and GPIO at the same time with
// one SPI read
void adc_read_data()
{
	uint32_t b2,b1,b0;

	// Max possible clock
	adc_init_spi(ADC_CLOCK_FAST);
	
	// CS to low
	P1OUT &= ~BIT4;

	// getting the data straight away
	// 1st byte is the status
	adc_stat = spi_txrx(0x00);
	// 2nd byte the internal GPIO connector
	// and we add the power status
	alldata.adc_gpio = (spi_txrx(0x00) | ( (P2IN & 0x40) << 1 ));
	// padding with zeros
	spi_txrx(0x00);

	// 24bit reading from ADC channel 1
	b2 = spi_txrx(0x00);
	b1 = spi_txrx(0x00);
	b0 = spi_txrx(0x00);
	// merge bits and convert to unsigned integer
	alldata.adc_ch1 = ((b2 << 16) | (b1 << 8) | b0) ^ 0x00800000;
#ifdef FAKE_ADC_DATA
	alldata.adc_ch1 = 0x00855555;
#endif

	// 24bit reading from ADC channel 2
	b2 = spi_txrx(0x00);
	b1 = spi_txrx(0x00);
	b0 = spi_txrx(0x00);
	// merge bits and convert to unsigned integer
	alldata.adc_ch2 = ((b2 << 16) | (b1 << 8) | b0) ^ 0x00800000;
#ifdef FAKE_ADC_DATA
	alldata.adc_ch2 = 0x00855555;
#endif

	// CS to high
	P1OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


// set the sampling rate of the ADC
// this also sets the sampling rate of the whole system
void setSamplingRate(unsigned char v) {
	v = v & 0b00000011;
       	adc_write_reg(0x01,v);
}

// starting the ADC by pulling the START pin high
void startADC() {
	// set start to high
	P1OUT |= BIT3;
}

// stopping the ADC
void stopADC() {
	// set start to low
	P1OUT &= ~BIT3;
	send_data = 0;
	hasData = 0;
}

// init the ADC and start it
void initADC()
{
	setSamplingRate(0);
	// switch the REF on
	adc_write_reg(0x02,0b10100000);
	// RLD signal generated externally
	adc_write_reg(0x0a,0b00000001);
	startADC();
}


//////////////
/// MPU9250


// reads / writes from a register
// xmtype defines if it's the acc or the magnetometer
unsigned char mpu9250_txrx(unsigned char readreg,
                        unsigned char addr, 
			unsigned char c)
{
	unsigned char rx;

	// 1MHz SPI clock
	UCB0BR0 = 16;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS
	P2OUT &= ~BIT4;

	if (readreg) {
		addr |= 0x80;
		c = 0;
	} else {
		addr &= 0x7f;
	}

	rx=spi_txrx(addr);
	rx=spi_txrx(c);

	// CS
	P2OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;

	return rx;
}


// reads from multiple registers
void mpu9250_rx_multi(unsigned char addr, 
		      unsigned char n,
		      unsigned char *rx)
{
	int i;

	// 1MHz SPI clock
	UCB0BR0 = 16;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS
	P2OUT &= ~BIT4;

	addr |= 0x80;
	spi_txrx(addr);

	for(i=0; i<n; i++) {
		*rx=spi_txrx(0);
		rx++;
	}

	// CS
	P2OUT |= BIT4;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}


uint8_t mpu9250_readRegister(const uint8_t register_addr) {
	return mpu9250_txrx(1,register_addr,0);
}


uint8_t mpu9250_writeRegister(const uint8_t register_addr,
			      const uint8_t value) {
	return mpu9250_txrx(0,register_addr,value);
}


int16_t mpu9250_readRegisters(const uint8_t msb_register, 
			      const uint8_t lsb_register) {
    uint8_t msb = mpu9250_readRegister(msb_register);
    uint8_t lsb = mpu9250_readRegister(lsb_register);
    return (((int16_t)msb) << 8) | lsb;
}


int mpu9250_checkwhoIam() {
	int i = 0;
	mpu9250_whoami = 0;
	for(i=0;i<32000;i++) {
		mpu9250_whoami = mpu9250_readRegister(MPU9250_WHO_AM_I);
		if ( mpu9250_whoami == WHOAMI_DEFAULT_VAL ) return 1;
		if ( mpu9250_whoami == WHOAMI_RESET_VAL ) return 1;
	}
	return 0;
}	


// should return 48h
uint8_t mpu9250_AK8963_read_reg(uint8_t reg) {
	uint8_t response;
	
	// talk to slave at address AK8963_I2C_ADDR, READ OP
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);
	// read from register AK8963_WIA
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG, reg);
	// read one byte from slave 0
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL, 0x81);
	delay(10);
	response=mpu9250_readRegister(MPUREG_EXT_SENS_DATA_00);
	return response;
}


int mpu9250_AK8963_checkwhoIam() {
	int i;
	mpu9250_AK8963_whoami = 0;
	for(i=0;i<32000;i++) {
		mpu9250_AK8963_whoami = mpu9250_AK8963_read_reg(AK8963_WIA);
		if ( mpu9250_AK8963_whoami == 0x48 ) return 1;
	}
	return 0;
}	


void mpu9250_setFullScaleAccelRange(const uint8_t range) {
	// normal operation
        mpu9250_writeRegister(MPU9250_ACCEL_CONFIG, range);
}



void initmpu9250() {
	// master reset
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x80);
	delay(0xffff);
	delay(0xffff);
	// wake it up with internal oscillator
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x00);
	delay(0xffff);
	delay(0xffff);
	// switch clock source to PLL
	mpu9250_writeRegister(MPU9250_PWR_MGMT_1,0x01);
	delay(0xffff);
	delay(0xffff);
	// check that we are alive and know who we are
	acc_stat = mpu9250_checkwhoIam();
	delay(0xffff);

	// switch on all sensors
	mpu9250_writeRegister(MPU9250_PWR_MGMT_2,0x07);
	delay(0xffff);

	// sampling rate 1kHz, 90Hz bandwidth
	mpu9250_writeRegister(MPU9250_CONFIG,0x02);
	delay(0xffff);

	// /8 = 125Hz
	mpu9250_writeRegister(MPU9250_SMPLRT_DIV,7);
	delay(0xffff);

	// set the range of the accelerometer
	mpu9250_setFullScaleAccelRange(MPU9250_FULL_SCALE_16G);
	delay(0xffff);

	// Enable the I2C Master I/F module
	mpu9250_writeRegister(MPU9250_USER_CTRL,0x20);
	delay(0xffff);
	// 400kHz clock for I2C
        mpu9250_writeRegister(MPUREG_I2C_MST_CTRL,0x0D);
	delay(0xffff);

	// magnetometer reset
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG,AK8963_CNTL2);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_DO,0x01);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL,0x81);
	delay(0xffff);

	// check if it's back to normal operation and knows...
	mag_stat = mpu9250_AK8963_checkwhoIam();
	delay(0xffff);

	// set the magnetometer to contious mode 1
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG,AK8963_CNTL1);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_DO,0x12);
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL,0x81);
	delay(0xffff);

	// initiate a block read from the magnetometer to the MPU
	mpu9250_writeRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);
	// read the high byte of the X direction
	mpu9250_writeRegister(MPUREG_I2C_SLV0_REG, AK8963_HXL);
	// read 7 bytes (x,y,z,status)
	mpu9250_writeRegister(MPUREG_I2C_SLV0_CTRL, 0x87);
	delay(0xffff);
}


void mpu9250_read_data() {
	uint8_t rx[22];

	mpu9250_rx_multi( MPU9250_ACCEL_XOUT_H, 20, rx );

	alldata.accel_x = (((uint16_t)(rx[0])<<8) | (uint16_t)(rx[1])) ^ 0x8000;
	alldata.accel_y = (((uint16_t)(rx[2])<<8) | (uint16_t)(rx[3])) ^ 0x8000;
	alldata.accel_z = (((uint16_t)(rx[4])<<8) | (uint16_t)(rx[5])) ^ 0x8000;

	alldata.mag_x = (((uint16_t)(rx[14])) | (uint16_t)(rx[15])<<8) ^ 0x8000;
	alldata.mag_y = (((uint16_t)(rx[16])) | (uint16_t)(rx[17])<<8) ^ 0x8000;
	alldata.mag_z = (((uint16_t)(rx[18])) | (uint16_t)(rx[19])<<8) ^ 0x8000;
}


void sendInfo() {
	uint8_t r;
	char tmp[4];

	sendText("ATTYS firmware version ");
	sendText(FW_VERSION);
	sendText("\r\n");

	sendText("MSP430G2553: ");
	sendText("P1IN=");
	sprintf(tmp,"%02x",P1IN);
	sendText(tmp);
	sendText(",P2IN=");
	sprintf(tmp,"%02x",P2IN);
	sendText(tmp);
	sendText("\r\n");

	sendText("Power: ");
	if (P1IN & 0x40) {
		sendText("5V USB power\r\n");
	} else {
		sendText("Battery\r\n");
	}
	
	sendText("Acceleromter: ");
	if (acc_stat) {
		sendText("OK\r\n");
	} else {
		sendText("fault\r\n");
	}
	
	sendText("Magnetometer: ");
	if (mag_stat) {
		sendText("OK\r\n");
	} else {
		sendText("fault\r\n");
	}
	
	sendText("Analog to Digital converter: ");
	// read ID control register: value for ADS1292R
	r = adc_read_reg(0);
	if ((adc_read_reg(0)==0x73) || (r==0x73)) {
		sendText("OK\r\n");
	} else {
		sendText("fault\r\n");
	}

	sendText("MPU9250:\r\n");
	for(r=0;r<0x7f;r++) {
		sprintf(tmp,"%02x,",mpu9250_readRegister(r));
		sendText(tmp);
		if ((r & 0x0f) == 0x0f) sendText("\r\n");
	}

	sendText("\r\nADS1292:\r\n");
	for(r=0;r<0x0c;r++) {
		sprintf(tmp,"%02x,",adc_read_reg(r));
		sendText(tmp);
	}

	sendText("\r\n");
}
	


// receive a character from the serial port and process it (commands)
__attribute__((interrupt(USCIAB0RX_VECTOR)))
void USCI0RX_ISR(void)
{
	unsigned char v;
	
	if (!(IFG2 & UCA0RXIFG)) return;
	flashPowerLED();
	// store the received char
       	uart_rx_char = UCA0RXBUF;
	// character available
	uart_rx_avail = 1;
	// if we send commands to the RN42 just ignore the characters
	if (ignore_rx) return;
	// store it in the buffer
	config_buffer[config_ptr] = uart_rx_char;
	// increment pointer if possible
	if (config_ptr<CONFIG_BUFFER_SIZE)
		config_ptr++;
	// check for carrige return or line feed
	if ((uart_rx_char == 13) || (uart_rx_char == 10)) {
		// add zero to terminate string properly
		config_buffer[config_ptr] = 0;
		// check if we have an '=' sign and
		// at least one character after the '=' sign
		if ( (config_buffer[1] == '=') && 
		     (config_ptr>1) ) {
			switch (config_buffer[0]) {
			case 'a':
			case 'A':
				v = atoi(config_buffer+2);
				v = v & 0b01110111;
				adc_write_reg(4,v);
				trigOK();
				flashPowerLED();
				break;
			case 'b':
			case 'B':
				v = atoi(config_buffer+2);
				v = v & 0b01110111;
				adc_write_reg(5,v);
				trigOK();
				flashPowerLED();
				break;
			case 'c':
			case 'C':
				v = atoi(config_buffer+2);
				v = v & 0b00111111;
				adc_write_reg(7,v);
				trigOK();
				flashPowerLED();
				break;
			case 'i':
			case 'I':
				v = atoi(config_buffer+2);
				v = v << 2;
				v = v & 0b00001100;
				// set current
				adc_write_reg(3,v);
				trigOK();
				flashPowerLED();
				break;
			case 'r':
			case 'R':
				setSamplingRate(atoi(config_buffer+2));
				trigOK();
				flashPowerLED();
				break;
			case 'x':
			case 'X':
				send_data = atoi(config_buffer+2);
				if (send_data) {
					startADC();
				} else {
					stopADC();
					trigOK();
				}
				flashPowerLED();
				break;
			case 'd':
			case 'D':
				base64output = (atoi(config_buffer+2) != 0);
				trigOK();
				flashPowerLED();
				break;
			case 'f':
			case 'F':
				send_all_data = (atoi(config_buffer+2) != 0);
				trigOK();
				flashPowerLED();
				break;
			case 'v':
			case 'V':
				verbose = (atoi(config_buffer+2) != 0);
				trigOK();
				flashPowerLED();
				break;
			case 't':
			case 'T':
				v = atoi(config_buffer+2) & 0x03;
				mpu9250_setFullScaleAccelRange(v << 3);
				trigOK();
				flashPowerLED();
				break;
			case 's':
			case 'S':
				if (!send_data) {
					sendInfo();
					flashPowerLED();
				}
				break;
			case 'm':
			case 'M':
				// cause a reset
				send_data = 0;
				WDTCTL = 0;
				break;
			default:
				sendText("ERR");
				uart_tx(0x0D);
				uart_tx(0x0A);	
			}
		}
		// always reset the buffer after a cr or lf
		config_ptr = 0;
		// set the string to zero (being paranoid)
		config_buffer[0] = 0;
		config_buffer[1] = 0;
		config_buffer[2] = 0;
	}
	__bic_SR_register_on_exit(LPM0_bits);
}


// transmit a character to the bluetooth module
void uart_tx(unsigned char c)
{
	unsigned int timeout = 30000;
        // is RTS high? Then let's wait till it goes low.
        while ( (P2IN & 0x02 ) && (timeout>0) ) {timeout--;};
        // Timeout? Let's discard the data
        if (timeout==0) {
		watchdog--;
		if (!watchdog) WDTCTL = 0;
		return;
	}
	watchdog = WATCHDOGINIT;
	// USCI_A0 TX buffer ready?
	while (!(IFG2&UCA0TXIFG));
	// TX -> RXed character
	UCA0TXBUF = c;    
}

// receive data and wait for it. In case of a timeout we
// return 0xff
unsigned char uart_rx()
{
	int timeout = 3000;
	while ((!uart_rx_avail) && (timeout>0)) {timeout--;};
	uart_rx_avail = 0;
	if (timeout) 
		return uart_rx_char;
	else
		return 0xff;
}


// send a standard string to the serial output
void sendText(const unsigned char *txt)
{
	while ((*txt)!=0)
	{
		uart_tx(*txt);
		txt++;
	}
}


void trigOK() {
	doSendOK = 1;
}


// send OK
void sendOK()
{
	if (!verbose) return;
	char oktxt[]="OK";
	char *txt=oktxt;
	while ((*txt)!=0)
	{
		uart_tx(*txt);
		txt++;
	}
	uart_tx(0x0D);
	uart_tx(0x0A);
}


void setUART230k() {
	// set baud rate to 230,000K
	UCA0BR0 = 69;
	UCA0BR1 = 0;
	// UCBRSx = 5, UCBRFx = 0 for fraction
	UCA0MCTL = UCBRS2 + UCBRS0;
	// **Initialize USCI state machine**
	UCA0CTL1 &= ~UCSWRST;
	uart_rx_char = UCA0RXBUF;
}


#define RN42WAIT 0x8000
void initRN42() {
	ignore_rx = 1;
	// changing the baudrate
	delay(RN42WAIT);
	uart_tx('$');
	uart_tx('$');
	uart_tx('$');
	delay(RN42WAIT);
	sendText("SU,23");
	uart_tx(10);
	delay(RN42WAIT);
	sendText("R,1");
	uart_tx(10);
	delay(RN42WAIT);

	setUART230k();
	// get rid of any rubbish character
	uart_tx(10);
	uart_tx(10);
	uart_tx(10);

	// now at the higher one
	delay(RN42WAIT);
	uart_tx('$');
	uart_tx('$');
	uart_tx('$');
	delay(RN42WAIT);
	sendText("S-,GN-ATTYS1");
	uart_tx(10);
	delay(RN42WAIT);
	sendText("R,1");
	uart_tx(10);
	delay(RN42WAIT);

	ignore_rx = 0;
	
	return;
}


void sendDataAsText(char* tmp) {

	// send it to the host
	// the '-1' makes it easier to detect a broken sample
	sprintf(tmp,"01,%02x,%04x,%04x,%04x,%04x,%04x,%04x,%l06x,%l06x,%02x",
		alldata.timestamp,
		alldata.accel_x,alldata.accel_y,alldata.accel_z,
		alldata.mag_x,alldata.mag_y,alldata.mag_z,
		alldata.adc_ch1,alldata.adc_ch2,alldata.adc_gpio);
}

void sendDataAsBase64(char *coded_dst) {
	if (send_all_data) {
		Base64encode(coded_dst,(char*)(&alldata),sizeof(alldata));
	} else {
		Base64encode(coded_dst,(char*)(&alldata),sizeof(adc_data_t));
	}
}



__attribute__((interrupt(PORT2_VECTOR)))
void port2ISR(void)
{
	controlPowerLED();

	// not data ready which has caused it
	if (!(P2IFG & BIT3)) return;

	// get the data from the ADC converter
	adc_read_data();

	mpu9250_read_data();

	if (send_data) {
		if (!hasData) {
			hasData = 1;
			if (base64output) {
				sendDataAsBase64(sendBuffer);
			} else {
				sendDataAsText(sendBuffer);
			}
		}
		alldata.timestamp++;
	}

	P2IFG &= ~BIT3;

	__bic_SR_register_on_exit(LPM0_bits);
}





// init all the ports of the MSP430
static inline void initMSP430()
{
	int i;

	WDTCTL = WDTPW + WDTHOLD;
	DCOCTL = 0;
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

 	// first, get everything into a nice state
	P1SEL  = 0x00;
	P2SEL  = 0x00;
	
	P1SEL2  = 0x00;
	P2SEL2  = 0x00;
	
	P1REN = 0x00;
	P2REN = 0x00;
  
	P1DIR = 0x00;
	P2DIR = 0x00;
	
	P1OUT = 0x00;
	P2OUT = 0x00;
 
        P1IES  = 0x00;
        P1IE   = 0x00;

        P2IES  = 0x00;
        P2IE   = 0x00;
 
	// Bluetooth RESET, active low
	P2DIR |= 0x01;
	delay(1000);
	P2OUT |= 0x01;
	delay(1000);
	P2OUT &= ~0x01;
	// set it to high
	delay(1000);
	P2OUT |= 0x01;

	// BT CTS
	P2DIR |= 0x04;
	// keep it low so that we can receive data, we are always ready
	P2OUT &= ~0x04;

	// LED, power good, P2.7
	P2DIR |= 0x80;
	P2OUT |= 0x80;

	// power sense, P2.6
	// enable pull/up/down
	P2REN |= 0x40;
	// set to pulldown
	P2OUT &= ~0x40;

	// UART
	P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	P1SEL2 = BIT1 + BIT2;                      
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK, 16MHz
	// division factor for 16MHz @ 115200baud, N = 138
	UCA0BR0 = 138;
	UCA0BR1 = 0;
	// UCBRSx = 7, UCBRFx = 0 for fraction
	UCA0MCTL = UCBRS2 + UCBRS1 + UCBRS0;
	// **Initialize USCI state machine**
	UCA0CTL1 &= ~UCSWRST;
       	IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	uart_rx_char = UCA0RXBUF;

	// SPI
	// CS_G
	P2DIR |= BIT5;
	P2OUT |= BIT5;
	// CS_M
	P2DIR |= BIT4;
	P2OUT |= BIT4;
	// CS_ADC
	P1DIR |= BIT4;
	P1OUT |= BIT4;

	// START_ADC line to zero (not running)
	P1DIR |= BIT3;
	P1OUT &= ~BIT3;
	// RESET ADC
	P1DIR |= BIT0;
	P1OUT |= BIT0;
	delay(1000);
	P1OUT &= ~BIT0;
	delay(1000);
	P1OUT |= BIT0;
	delay(0xffff);

	// config the SPI
	P1SEL |= BIT5 + BIT6 + BIT7;
	P1SEL2 |= BIT5 + BIT6 + BIT7;
	UCB0CTL0 |= UCCKPL + UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2;                     // SMCLK
	UCB0BR0 = 2;                              // 8MHz SPI clock
	UCB0BR1 = 0;                              //
	UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	
	P2OUT &= ~BIT5;                           // Now with SPI signals initialized,
	P2OUT |= BIT5;                            // reset slave
	
	P2OUT &= ~BIT4;                           // Now with SPI signals initialized,
	P2OUT |= BIT4;                            // reset slave

}

void enableInterrupts()
{

        // P2.3 interrupt enabled for serial
        P2IE |= BIT3;
        // P2.3 high/low edge
        P2IES |= BIT3;   
        // P2.3 IFG cleared. We are ready to go
        P2IFG &= ~BIT3;

	// start interrupts
	__eint();

}








// main program
void main(void)
{
	// init all ports and switch off the watchdog
	initMSP430();

	initADC();

	initmpu9250();

	enableInterrupts();

	initRN42();

	send_data = 1;

	for(;;) {
		if (hasData) {
			if (send_data) {
				sendText(sendBuffer);
				uart_tx(0x0D);
				uart_tx(0x0A);
			}
			hasData = 0;
		}
		if ((!hasData) && (!send_data) && (doSendOK)) {
			sendOK();
			doSendOK = 0;
		}
		_BIS_SR(LPM0_bits + GIE);
	}
}
