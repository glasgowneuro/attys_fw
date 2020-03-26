/**
   Attys firmware
   Copyright (C) 2016-2020, Bernd Porr, mail@berndporr.me.uk
   
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

#define FW_VERSION "0.990"

// for debugging
// #define FAKE_ADC_DATA

#include <msp430.h>
#include <stdio.h>
#include "LSM9DS1_Registers.h"
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

volatile static uint8_t whoAmIAccelerometer = 0;
volatile static uint8_t whoAmIMagnetometer = 0;

// self test status: 1=test successful, 0=error
volatile static unsigned char adc_stat;
volatile static unsigned char acc_stat;
volatile static unsigned char mag_stat;

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



// LSM


// lsm9DS1 read/write operation via SPI
// reads / writes from a register
// mag defines if it's the accelerometer/gyro (0) or magnetometer (1)
// READ: bitOR the register address with LSM9DS1_REGISTER_READ and just transmit 0
// WRITE: specify register and value. That's it.
unsigned char lsmWriteRegister(const unsigned char addr, 
			   const unsigned char c,
			   const unsigned char mag)
{
	unsigned char rx;

	// 8MHz SPI clock
	UCB0BR0 = 2;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS low
	if (mag)
		P2OUT &= ~BIT4;
	else
		P2OUT &= ~BIT5;

	rx=spi_txrx(addr);
	rx=spi_txrx(c);

	// CS high again
	if (mag)
		P2OUT |= BIT4;
	else
		P2OUT |= BIT5;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;

	return rx;
}


unsigned char lsmReadRegister(const unsigned char addr,
			      const unsigned char mag) {
	return lsmWriteRegister(addr | 0x80, 
				0,
				mag);
}


// reads from multiple registers
void lsmReadBytes(unsigned char addr,
		  unsigned char *rx,
		  unsigned char n,
		  const unsigned char mag)
{
	unsigned char i;

	// 8MHz SPI clock
	UCB0BR0 = 2;
	// inactive state of the SPI is high
	UCB0CTL0 |= UCCKPL;
	// Phase=0, data is changed on the first edge
	UCB0CTL0 &= ~UCCKPH;
	// **Initialize USCI state machine**
	UCB0CTL1 &= ~UCSWRST;

	// CS low
	if (mag)
		P2OUT &= ~BIT4;
	else
		P2OUT &= ~BIT5;

	addr |= 0x80;
	if (mag) {
		addr |= 0x40;
	}
	spi_txrx(addr);

	for(i=0; i<n; i++) {
		*rx=spi_txrx(0);
		rx++;
	}

	if (mag)
		P2OUT |= BIT4;
	else
		P2OUT |= BIT5;

	// bring USCI back into reset
	UCB0CTL1 |= UCSWRST;
}



void initAccel(unsigned char fullrange) {
	whoAmIAccelerometer = lsmReadRegister(WHO_AM_I_XG, 0);
	if (whoAmIAccelerometer == 104) {
		acc_stat = 1;
	}
	
	//    CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//    [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//    DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//        00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//    Zen_XL - Z-axis output enabled
	//    Yen_XL - Y-axis output enabled
	//    Xen_XL - X-axis output enabled
	uint8_t tempRegValue = (1<<5) | (1<<4) | (1<<3);	
	lsmWriteRegister(CTRL_REG5_XL, tempRegValue, 0);

	// sets sampling rate: 476Hz
	tempRegValue = (5 << 5);
    
	switch (fullrange) {
	case 1:
		tempRegValue |= (0x2 << 3);
		break;
	case 2:
		tempRegValue |= (0x3 << 3);
		break;
	case 3:
		tempRegValue |= (0x1 << 3);
		break;
	}
	
	lsmWriteRegister(CTRL_REG6_XL, tempRegValue, 0);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue =
		(1<<7) |  // Set HR bit
		(2 << 5); // ODR/9
	lsmWriteRegister(CTRL_REG7_XL, tempRegValue,0);
}


void initGyro(unsigned char gyroOn) {

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	uint8_t tempRegValue = 0;
	
	if (gyroOn) {
		tempRegValue =
			(5 << 5) |  // seting sampling rate to 476Hz
			(0x3 << 3); // gyro at 2000dps
		lsmWriteRegister(CTRL_REG1_G, tempRegValue, 0);
	} else {
		// power down
		lsmWriteRegister(CTRL_REG1_G, 0, 0);
		return;
	}
	
	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	lsmWriteRegister(CTRL_REG2_G, 0x00, 0);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	lsmWriteRegister(CTRL_REG3_G, 0x00, 0);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	tempRegValue |= (1<<5);
	tempRegValue |= (1<<4);
	tempRegValue |= (1<<3);
	tempRegValue |= (1<<1);
	lsmWriteRegister(CTRL_REG4, tempRegValue, 0);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	lsmWriteRegister(ORIENT_CFG_G, tempRegValue, 0);
}


void initMag()
{
	whoAmIMagnetometer = lsmReadRegister(WHO_AM_I_M,1);
	if (whoAmIMagnetometer == 61) {
		mag_stat = 1;
	}
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//    00:low-power, 01:medium performance
	//    10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	uint8_t tempRegValue =
		(0x3 << 5) |
		(0x7 << 2);
	lsmWriteRegister(CTRL_REG1_M, tempRegValue, 1);
    
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	// 16 gauss or 1600E-6 Tesla
	tempRegValue = (0x3 << 5);
	lsmWriteRegister(CTRL_REG2_M, tempRegValue, 1);
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//    00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	lsmWriteRegister(CTRL_REG3_M, tempRegValue, 1); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//    00:low-power mode, 01:medium performance
	//    10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = (3 << 2);
	lsmWriteRegister(CTRL_REG4_M, tempRegValue, 1);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//    0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	lsmWriteRegister(CTRL_REG5_M, tempRegValue, 1);
}

void readAccel() {
	uint8_t temp[6] = {0,0,0,0,0,0};
	lsmReadBytes(OUT_X_L_XL, temp, 6, 0);
	alldata.accel_x = (((uint16_t)(temp[1]) << 8) | (uint16_t)(temp[0])) ^ 0x8000;
	alldata.accel_y = (((uint16_t)(temp[3]) << 8) | (uint16_t)(temp[2])) ^ 0x8000;
	alldata.accel_z = (((uint16_t)(temp[5]) << 8) | (uint16_t)(temp[4])) ^ 0x8000;
}

void readMag() {
	uint8_t temp[6] = {0,0,0,0,0,0};
	lsmReadBytes(OUT_X_L_M, temp, 6, 1);
	alldata.mag_x = (((uint16_t)(temp[1]) << 8) | (uint16_t)(temp[0])) ^ 0x8000;
	alldata.mag_y = (((uint16_t)(temp[3]) << 8) | (uint16_t)(temp[2])) ^ 0x8000;
	alldata.mag_z = (((uint16_t)(temp[5]) << 8) | (uint16_t)(temp[4])) ^ 0x8000;
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

	sendText("LSMDS1:\r\n");
	for(r=0;r<0x7f;r++) {
		sprintf(tmp,"%02x,",lsmReadRegister(r,0));
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
				// mpu9250_setFullScaleAccelRange(v << 3); fixme
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

	readAccel();
	readMag();

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

	initAccel(3);
	initMag();

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
