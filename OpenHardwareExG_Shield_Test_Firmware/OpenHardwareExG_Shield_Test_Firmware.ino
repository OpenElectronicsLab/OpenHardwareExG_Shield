/*
  OpenHardwareExG_Shield_Test_Firmware
 */

#include "ads1298.h"
using namespace ADS1298;
#include <stdio.h>
#include <SPI.h>

// some Arduino's use 5.0 ...
#define ANALOG_REFERENCE_VOLTAGE 3.3

// wait this long before attempting to test a short
#define STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS 25

// wait additional time before testing other board features
#define STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS 25

// time to wait between testing state changes
#define DIGITAL_STATE_CHANGE_DELAY_MICROS 2

#ifdef  _VARIANT_ARDUINO_DUE_X_
#define SPI_CLOCK_DIVIDER_VAL 21
#else
// #define SPI_CLOCK_DIVIDER_VAL SPI_CLOCK_DIV4
#define SPI_CLOCK_DIVIDER_VAL SPI_CLOCK_DIV8
#endif

#define IPIN_DRDY 2

byte adc_rreg(int reg)
{
	SPI.transfer(RREG | reg);
	SPI.transfer(0);	// number of registers to be read/written
	return SPI.transfer(0);
}

void adc_wreg(int reg, int val)
{
	SPI.transfer(WREG | reg);
	SPI.transfer(0);	// number of registers to be read/written
	SPI.transfer(val);
}

bool bad_magic(const ADS1298::Data_frame &frame)
{
	return 0xC0 != (frame.data[0] & 0xF0);
}

long channel_value(const ADS1298::Data_frame &frame, unsigned channel)
{
	unsigned num = channel - 1;
	const uint8_t *data = frame.data;
	signed long bits_17_24 =
	    (((signed long)((int8_t) data[3 + (num * 3)])) << 16);
	unsigned long bits_9_16 = (((unsigned long)data[4 + (num * 3)]) << 8);
	unsigned long bits_0_8 = (((unsigned long)data[5 + (num * 3)]));

	signed long val = bits_17_24 | bits_9_16 | bits_0_8;
	return val;
}

void read_data_frame(ADS1298::Data_frame * frame)
{
	SPI.transfer(RDATA);
	for (int i = 0; i < frame->size; ++i) {
		frame->data[i] = SPI.transfer(0);
	}
}

void to_hex(char byte, char *buf)
{
	int i;
	char nibbles[2];

	nibbles[0] = (byte & 0xF0) >> 4;
	nibbles[1] = (byte & 0x0F);

	for (i = 0; i < 2; i++) {
		if (nibbles[i] < 10) {
			buf[i] = '0' + nibbles[i];
		} else {
			buf[i] = 'A' + nibbles[i] - 10;
		}
	}
	buf[2] = '\0';
}

void format_data_frame(const ADS1298::Data_frame &frame, char *byte_buf)
{
	uint8_t in_byte;
	unsigned int pos = 0;

	byte_buf[pos++] = '[';
	byte_buf[pos++] = 'g';
	byte_buf[pos++] = 'o';
	byte_buf[pos++] = ']';

	for (int i = 0; i < frame.size; ++i) {
		if (i > 0 && i % 3 == 0) {
			byte_buf[pos++] = ' ';
		}
		in_byte = frame.data[i];
		to_hex(in_byte, byte_buf + pos);
		pos += 2;
	}

	byte_buf[pos++] = '[';
	byte_buf[pos++] = 'o';
	byte_buf[pos++] = 'n';
	byte_buf[pos++] = ']';
	byte_buf[pos++] = '\n';
	byte_buf[pos++] = 0;
}

struct ShiftOutputs {
	unsigned simulate_board_below:1;
	unsigned master_ics:1;
	unsigned slave_ics:1;
	unsigned signal_a:1;
	unsigned signal_b:1;
	unsigned success_led:1;
	unsigned fault_led:1;
	unsigned enable_shield:1;

	ShiftOutputs()
	{
		simulate_board_below = 0;
		master_ics = 1;
		slave_ics = 1;
		signal_a = 0;
		signal_b = 0;
		success_led = 0;
		fault_led = 0;
		enable_shield = 1;
	}

	static ShiftOutputs power_off()
	{
		ShiftOutputs result;
		result.master_ics = 0;
		result.slave_ics = 0;
		result.enable_shield = 0;

		return result;
	}
};

struct ShiftInputs {
	unsigned slave_and_slave_cs:1;
	unsigned master_and_master_cs:1;
	unsigned i_master:1;
	unsigned master:1;
	unsigned go_button:1;
	unsigned i_cs:1;
	unsigned dout:1;
	unsigned i_drdy:1;

	unsigned mosi_iso:1;
	unsigned SCLKiso:1;
	unsigned i_csiso:1;
	unsigned master_iso:1;
	unsigned clk_iso:1;
	unsigned i_drdy_iso:1;
	unsigned dout_iso:1;
	unsigned unused1:1;

	unsigned gpio1:1;
	unsigned gpio2:1;
	unsigned gpio3:1;
	unsigned gpio4:1;
	unsigned daisyin:1;
	unsigned biasinv:1;
	unsigned unused2:1;
	unsigned unused3:1;
};

enum Pins {
	PIN_SHIFT_IN = A0,
	PIN_DIV_VIN_ISO = A1,
	PIN_DIV_3V3_ISO = A2,
	PIN_DIV_GND_ISO = A3,
	PIN_RESIST_GND_ISO = A4,
	PIN_BIASOUT_FILT = A5,

	IPIN_MASTER_DRDY = 2,
	IPIN_SHIFT_OUT_SRCLR = 3,
	PIN_SHIFT_OUT_SRCLK = 4,
	PIN_SHIFT_OUT_RCLK = 5,
	PIN_SHIFT_OUT = 6,
	IPIN_SHIELD_SHORTED = 7,
	PIN_SHIFT_CLK = 8,
	IPIN_SHIFT_SH_LD = 9,
};

enum Pogos {
	P1_IN8N = 1,		// P1 == 1 for easier debug
	P2_IN7N,
	P3_IN6N,
	P4_IN5N,
	P5_IN4N,
	P6_IN3N,
	P7_IN2N,
	P8_IN1N,
	P9_BIASOUT_FILT,	// A5
	P10,			// not present (removed during design phase)
	P11_IN8P,
	P12_IN7P,
	P13_IN6P,
	P14_IN5P,
	P15_IN4P,
	P16_IN3P,
	P17_IN2P,
	P18_IN1P,
	P19,			// not present (removed during design phase)
	P20,			// not present (removed during design phase)
	P21,			// not present (removed during design phase)
	P22,			// not present (removed during design phase)
	P23_MOSI_ISO,		// U2 input E
	P24_SCLK_ISO,		// U2 input F
	P25_INV_CS_ISO,		// U2 input G
	P26_MASTER_ISO,		// U2 input H
	P27_DOUT_ISO,		// U2 input A
	P28_INV_DRDY_ISO,	// U2 input B
	P29_VIN_ISO,		// A1 via voltage divider
	P30_3V3_ISO,		// A2 via voltage divider
	P31_GND_ISO,		// A3 via voltage divider, A4 via resistor
	P32_GPIO1_ISO,		// U1 input E
	P33_GPIO2_ISO,		// U1 input F
	P34_GPIO3_ISO,		// U1 input G
	P35_GPIO4_ISO,		// U1 input H
	P36_DAISY_INV_ISO,	// U1 input C
	P37_BIAS_INV,		// U1 input B
	P38_CLK_ISO,		// U2 input C
	P39_SLAVE_AND_SLAVE_CS,	// U4 input E
	P40_MASTER_AND_MASTER_CS,	// U4 input F
	P41_INV_MASTER,		// U4 input G
	P42_INV_CS,		// U4 input D
	P43_MASTER,		// U4 input C
	P44_DOUT,		// U4 input B
	P45_INV_DRDY,		// U4 input A
	P46_SHIELD_VCC,		// toggled on U6 by ENABLE_SHIELD (U3 output H)
	P47_SHIELD_3V3,		// toggled on U6 by ENABLE_SHIELD (U3 output H)
	P48_SHIELD_5V,		// toggled on U6 by ENABLE_SHIELD (U3 output H)
	P49_ARDUINO_GND,	// toggled on U6 by ENABLE_SHIELD (U3 output H)
	P50_MASTER_INV_CS,	// U3 input B
	P51_SLAVE_INV_CS,	// U3 input C
	P52_MASTER_INV_DRDY,	// PIN2
	P53_MISO,		// MISO
	P54_MOSI,		// MOSI
	P55_SCLK		// SCLK
};

enum Delays {
	shift_clock_delay = 1
};

void shift_out(bool value)
{
	digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW);	// should be a NOP
	digitalWrite(PIN_SHIFT_OUT, value ? HIGH : LOW);
	delayMicroseconds(shift_clock_delay);
	digitalWrite(PIN_SHIFT_OUT_SRCLK, HIGH);
	delayMicroseconds(shift_clock_delay);
	digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW);
}

void write_shift_out(const struct ShiftOutputs &output)
{
	shift_out(output.enable_shield);
	shift_out(output.fault_led);
	shift_out(output.success_led);
	shift_out(output.signal_b);
	shift_out(output.signal_a);
	shift_out(output.slave_ics);
	shift_out(output.master_ics);
	shift_out(output.simulate_board_below ? 0 : 1);	// IPIN ~SEND_TO_GND~_MASTER

	digitalWrite(PIN_SHIFT_OUT_RCLK, HIGH);
	delayMicroseconds(shift_clock_delay);
	digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);
}

unsigned shift_in()
{
	unsigned d = digitalRead(PIN_SHIFT_IN);
	digitalWrite(PIN_SHIFT_CLK, HIGH);
	delayMicroseconds(shift_clock_delay);
	digitalWrite(PIN_SHIFT_CLK, LOW);
	delayMicroseconds(shift_clock_delay);
	return d;
}

struct ShiftInputs read_shift_in()
{
	struct ShiftInputs input;

	digitalWrite(IPIN_SHIFT_SH_LD, LOW);
	delayMicroseconds(shift_clock_delay);
	digitalWrite(IPIN_SHIFT_SH_LD, HIGH);
	delayMicroseconds(shift_clock_delay);

	input.go_button = shift_in();
	input.i_master = shift_in();
	input.master_and_master_cs = shift_in();
	input.slave_and_slave_cs = shift_in();
	input.i_cs = shift_in();
	input.master = shift_in();
	input.dout = shift_in();
	input.i_drdy = shift_in();

	input.master_iso = shift_in();
	input.i_csiso = shift_in();
	input.SCLKiso = shift_in();
	input.mosi_iso = shift_in();
	input.unused1 = shift_in();
	input.clk_iso = shift_in();
	input.i_drdy_iso = shift_in();
	input.dout_iso = shift_in();

	input.gpio4 = shift_in();
	input.gpio3 = shift_in();
	input.gpio2 = shift_in();
	input.gpio1 = shift_in();
	input.unused2 = shift_in();
	input.daisyin = shift_in();
	input.biasinv = shift_in();
	input.unused3 = shift_in();

	return input;
}

// the setup function runs once when you press reset or power the board
void setup()
{
	// initialize digital pin 13 as an output.
	pinMode(13, OUTPUT);

	pinMode(PIN_SHIFT_IN, INPUT);
	pinMode(PIN_SHIFT_CLK, OUTPUT);
	pinMode(IPIN_SHIFT_SH_LD, OUTPUT);

	pinMode(PIN_DIV_VIN_ISO, INPUT);	// un-needed?
	pinMode(PIN_DIV_3V3_ISO, INPUT);	// un-needed?
	pinMode(PIN_DIV_GND_ISO, INPUT);	// un-needed?
	pinMode(PIN_RESIST_GND_ISO, OUTPUT);
	pinMode(IPIN_SHIELD_SHORTED, INPUT);

	pinMode(PIN_BIASOUT_FILT, INPUT);
	pinMode(IPIN_MASTER_DRDY, INPUT);

	// set up the pins for the output shift registers
	pinMode(PIN_SHIFT_OUT_RCLK, OUTPUT);
	pinMode(PIN_SHIFT_OUT_SRCLK, OUTPUT);
	pinMode(PIN_SHIFT_OUT, OUTPUT);
	pinMode(IPIN_SHIFT_OUT_SRCLR, OUTPUT);

	digitalWrite(IPIN_SHIFT_OUT_SRCLR, HIGH);
	digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);

	// init shift-in pins
	digitalWrite(IPIN_SHIFT_SH_LD, HIGH);
	digitalWrite(PIN_SHIFT_CLK, LOW);

	// zero the shift-out (including enable_shield=0)
	ShiftOutputs output = ShiftOutputs::power_off();
	write_shift_out(output);

	// initialize the USB Serial connection
	Serial.begin(115200);
}

bool check_for_short()
{
	unsigned long timeout_milliseconds = 200;
	unsigned long start;

	start = millis();
	while ((millis() - start) < timeout_milliseconds) {
		unsigned d = digitalRead(IPIN_SHIELD_SHORTED);
		if (!d) {
			// disable power to the board
			ShiftOutputs output = ShiftOutputs::power_off();
			write_shift_out(output);
			return true;
		}
	}
	return false;
}

bool check_div_gnd_high_fault()
{
	int gnd = analogRead(PIN_DIV_GND_ISO);	// 0 - 1023
	char buf[1024];
	sprintf(buf, "PIN_DIV_GND_ISO: %d", gnd);
	Serial.println(buf);

	if (gnd < 65) {
		Serial.println("PIN_DIV_GND_ISO too low");
		return true;
	}
	if (gnd > 100) {
		Serial.println("PIN_DIV_GND_ISO too high");
		return true;
	}

	return false;
}

bool check_div_gnd_low_fault()
{
	int gnd = analogRead(PIN_DIV_GND_ISO);	// 0 - 1023
	char buf[1024];
	sprintf(buf, "PIN_DIV_GND_ISO: %d", gnd);
	Serial.println(buf);

	if (gnd > 10) {
		Serial.println("PIN_DIV_GND_ISO too high, should be zero");
		return true;
	}

	return false;
}

bool check_3v3_bogus_iso_fault()
{
	int val0 = analogRead(PIN_DIV_GND_ISO);	// 0 - 1023

	// bc <<<"scale=3; 11* ((93 * 3.3)/1023) "
	// 3.300

	char buf[1024];
	sprintf(buf, "PIN_DIV_GND_ISO: %d", val0);
	Serial.println(buf);

	int val1 = analogRead(PIN_DIV_3V3_ISO);	// 0 - 1023
	sprintf(buf, "PIN_DIV_3V3_ISO: %d", val1);
	Serial.println(buf);

	/* ANALOG_REFERENCE_VOLTAGE */
	float voltage = 11.0 * (((val1 - val0) * 3.3) / 1023.0);

	sprintf(buf, "3V3_ISO approx: %5.3f\n", voltage);
	Serial.println(buf);

	if (voltage < 2.75) {
		Serial.println("3V3_ISO below 2.75 (too low for u6)");
		return true;
	}

	if (voltage > 3.5) {
		Serial.println("3V3_ISO above 3.5 (too high for ADS)");
		return true;
	}

	return false;
}

bool check_5v_bogus_iso_fault()
{
	int val0 = analogRead(PIN_DIV_GND_ISO);	// 0 - 1023

	char buf[1024];
	sprintf(buf, "PIN_DIV_GND_ISO: %d", val0);
	Serial.println(buf);

	int val1 = analogRead(PIN_DIV_VIN_ISO);	// 0 - 1023
	sprintf(buf, "PIN_DIV_VIN_ISO: %d", val1);
	Serial.println(buf);

	float voltage = 11.0 * (((val1 - val0) * 3.3) / 1023.0);

	sprintf(buf, "VIN_ISO approx: %5.3f\n", voltage);
	Serial.println(buf);

	if (voltage < 4.85) {
		Serial.println("VIN_ISO below 4.85 (too low for ADS)");
		return true;
	}

	if (voltage > 5.15) {
		Serial.println("VIN_ISO above 5.15 (too high for ADS)");
		return true;
	}

	return false;
}

struct error_code {
	unsigned blink_code;
	const char *error_txt;
};

struct error_code ERROR_BLINK_SUCCESS = { 0x00000, "Success (no errors)" };
struct error_code ERROR_BLINK_SHORT = { 0x00001, "Short detected" };
struct error_code ERROR_BLINK_GND_HI = { 0x00002, "problem with GND iso" };
struct error_code ERROR_BLINK_3V3_ISO = { 0x00003, "problem with 3v3 iso" };
struct error_code ERROR_BLINK_VIN_ISO = { 0x00004, "problem with VIN iso" };
struct error_code ERROR_BLINK_GND_LOW = { 0x00005, "problem with GND iso" };
struct error_code ERROR_BLINK_FIRST_SHIFT_IN = { 0x00006, "first read" };
struct error_code ERROR_BLINK_MCS_SHIFT_IN = { 0x00007, "MCS read" };
struct error_code ERROR_BLINK_SCS_SHIFT_IN = { 0x00008, "SCS read" };
struct error_code ERROR_BLINK_BOTH_CS_SHIFT_IN = { 0x00009, "BOTH CS read" };
struct error_code ERROR_BLINK_SLAVE_SHIFT_IN = { 0x0000A, "SLAVE" };
struct error_code ERROR_BLINK_SLAVE_MCS_SHIFT_IN = { 0x0000B, "SLAVE MCS" };
struct error_code ERROR_BLINK_SLAVE_SCS_SHIFT_IN = { 0x0000C, "SLAVE SCS" };
struct error_code ERROR_BLINK_SLAVE_BOTH_CS_SHIFT_IN =
    { 0x0000D, "SLAVE BOTH CS" };
struct error_code ERROR_BLINK_MOSI = { 0x0000E, "MOSI" };
struct error_code ERROR_BLINK_SCLK = { 0x0000F, "SCLK" };
struct error_code ERROR_BLINK_CHIP_ID = { 0x00010, "CHIP ID" };
struct error_code ERROR_BLINK_GPIO = { 0x00011, "GPIO" };
struct error_code ERROR_BLINK_NO_DRDY_1 = { 0x00012, "No DRDY 1" };
struct error_code ERROR_BLINK_BAD_MOJO = { 0x00013, "Bad MAGIC" };
struct error_code ERROR_BLINK_DATA_OOR = { 0x00014, "Data out of range" };

bool delay_or_go_button(unsigned delay_millis)
{
	unsigned long start = millis();

	while (millis() - start < delay_millis) {
		ShiftInputs input = read_shift_in();
		if (input.go_button) {
			return true;
		}
	}
	return false;
}

void blink_error(struct error_code err)
{
	Serial.println(err.error_txt);

	ShiftOutputs errorOutput = ShiftOutputs::power_off();

	for (int i = 0; i < 10 /* or board removed */ ; ++i) {
		for (unsigned j = 0; j < err.blink_code; ++j) {
			errorOutput.fault_led = 0;
			write_shift_out(errorOutput);
			if (delay_or_go_button(200)) {
				goto EXIT_BLINK_ERROR;
			}
			errorOutput.fault_led = 1;
			write_shift_out(errorOutput);
			if (delay_or_go_button(300)) {
				goto EXIT_BLINK_ERROR;
			}
		}
		if (delay_or_go_button(2000)) {
			goto EXIT_BLINK_ERROR;
		}
	}

EXIT_BLINK_ERROR:
	errorOutput.fault_led = 0;
	write_shift_out(errorOutput);
}

/* the ADS1299 datasheet
   (page 38 of SBAS499A - JULY 2012 - REVISED AUGUST 2012)
   shows that dout is undefined at some times, set compare_dout to false
   if comparing during an undefined time */
unsigned long shift_in_mismatch(struct ShiftInputs *expected,
				struct ShiftInputs *actual, bool compare_dout =
				true)
{
	unsigned long i = 0;
	unsigned long errors = 0;
	// bool match;
	char buf[255];

	const char *fmt = "Mismatch comparing %s: expected %u but was %u";
	const char *skip = "Surpressed comparing %s: expected %u but was %u";

	if (expected->slave_and_slave_cs != actual->slave_and_slave_cs) {
		errors |= (1L << i);
		sprintf(buf, fmt, "slave_and_slave_cs",
			expected->slave_and_slave_cs,
			actual->slave_and_slave_cs);
		Serial.println(buf);
	}
	++i;
	if (expected->master_and_master_cs != actual->master_and_master_cs) {
		errors |= (1L << i);
		sprintf(buf, fmt, "master_and_master_cs",
			expected->master_and_master_cs,
			actual->master_and_master_cs);
		Serial.println(buf);
	}
	++i;
	if (expected->i_master != actual->i_master) {
		errors |= (1L << i);
		sprintf(buf, fmt, "i_master",
			expected->i_master, actual->i_master);
		Serial.println(buf);
	}
	++i;
	if (expected->master != actual->master) {
		errors |= (1L << i);
		sprintf(buf, fmt, "master",
			expected->master, actual->master);
		Serial.println(buf);
	}
	++i;
	if (0 && expected->go_button != actual->go_button) {
		errors |= (1L << i);
		sprintf(buf, fmt, "go_button",
			expected->go_button, actual->go_button);
		Serial.println(buf);
	}
	++i;
	if (expected->i_cs != actual->i_cs) {
		errors |= (1L << i);
		sprintf(buf, fmt, "i_cs",
			expected->i_cs, actual->i_cs);
		Serial.println(buf);
	}
	++i;
	if (compare_dout && expected->dout != actual->dout) {
		errors |= (1L << i);
		sprintf(buf, fmt, "dout",
			expected->dout, actual->dout);
		Serial.println(buf);
	}
	++i;
	if (expected->i_drdy != actual->i_drdy) {
		errors |= (1L << i);
		sprintf(buf, fmt, "i_drdy",
			expected->i_drdy, actual->i_drdy);
		Serial.println(buf);
	}
	++i;

	if (expected->mosi_iso != actual->mosi_iso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "mosi_iso",
			expected->mosi_iso, actual->mosi_iso);
		Serial.println(buf);
	}
	++i;
	if (expected->SCLKiso != actual->SCLKiso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "SCLKiso",
			expected->SCLKiso, actual->SCLKiso);
		Serial.println(buf);
	}
	++i;
	if (expected->i_csiso != actual->i_csiso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "i_csios",
			expected->i_csiso, actual->i_csiso);
		Serial.println(buf);
	}
	++i;
	if (expected->master_iso != actual->master_iso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "master_iso",
			expected->master_iso, actual->master_iso);
		Serial.println(buf);
	}
	++i;
	if (expected->clk_iso != actual->clk_iso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "clk_iso",
			expected->clk_iso, actual->clk_iso);
		Serial.println(buf);
	}
	++i;
	if (expected->i_drdy_iso != actual->i_drdy_iso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "i_drdy_iso",
			expected->i_drdy_iso, actual->i_drdy_iso);
		Serial.println(buf);
	}
	++i;
	if (compare_dout && expected->dout_iso != actual->dout_iso) {
		errors |= (1L << i);
		sprintf(buf, fmt, "dout_iso",
			expected->dout_iso, actual->dout_iso);
		Serial.println(buf);
	}
	++i;
	if (0 && expected->unused1 != actual->unused1) {
		errors |= (1L << i);
		sprintf(buf, fmt, "unused1",
			expected->unused1, actual->unused1);
		Serial.println(buf);
	}
	++i;

	if (expected->gpio1 != actual->gpio1) {
		errors |= (1L << i);
		sprintf(buf, fmt, "gpio1",
			expected->gpio1, actual->gpio1);
		Serial.println(buf);
	}
	++i;
	if (expected->gpio2 != actual->gpio2) {
		errors |= (1L << i);
		sprintf(buf, fmt, "gpio2",
			expected->gpio2, actual->gpio2);
		Serial.println(buf);
	}
	++i;
	if (expected->gpio3 != actual->gpio3) {
		errors |= (1L << i);
		sprintf(buf, fmt, "gpio3",
			expected->gpio3, actual->gpio3);
		Serial.println(buf);
	}
	++i;
	if (expected->gpio4 != actual->gpio4) {
		// errors |= (1L<<i); // FIXME: Eric's board is solder-bridged to DRDY
		sprintf(buf, skip, "gpio4",
			expected->gpio4, actual->gpio4);
		Serial.println(buf);
	}
	++i;
	if (expected->daisyin != actual->daisyin) {
		errors |= (1L << i);
		sprintf(buf, fmt, "daisyin",
			expected->daisyin, actual->daisyin);
		Serial.println(buf);
	}
	++i;
	if (expected->biasinv != actual->biasinv) {
		errors |= (1L << i);
		sprintf(buf, fmt, "biasinv",
			expected->biasinv, actual->biasinv);
		Serial.println(buf);
	}
	++i;
	if (0 && expected->unused2 != actual->unused2) {
		errors |= (1L << i);
		sprintf(buf, fmt, "unused2",
			expected->unused2, actual->unused2);
		Serial.println(buf);
	}
	++i;
	if (0 && expected->unused3 != actual->unused3) {
		errors |= (1L << i);
		sprintf(buf, fmt, "unused3",
			expected->unused3, actual->unused3);
		Serial.println(buf);
	}
	++i;

	if (actual->master == actual->i_master) {
		errors |= (1L << i);
		Serial.println("inconsistent master, i_master");
	};
	++i;

	if (actual->master != actual->master_iso) {
		errors |= (1L << i);
		Serial.println("inconsistent master, master_iso");
	};
	++i;

	if (actual->dout != actual->dout_iso) {
		errors |= (1L << i);
		Serial.println("inconsistent dout, dout_iso");
	};
	++i;

	if (actual->i_cs != actual->i_csiso) {
		errors |= (1L << i);
		Serial.println("inconsistent i_cs, i_csiso");
	};
	++i;

	if (actual->i_drdy != actual->i_drdy_iso) {
		errors |= (1L << i);
		Serial.println("inconsistent i_drdy, i_drdy_iso");
	};
	++i;

	if (errors) {
		char buf[80];
		sprintf(buf, "(errors: 0x%8lx)", errors);
		Serial.println(buf);
	}

	return errors;
}

void spi_setup()
{
	SPI.begin();

	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIVIDER_VAL);
	SPI.setDataMode(SPI_MODE1);
	ShiftOutputs output;
	output.master_ics = 0;
	write_shift_out(output);
	SPI.transfer(SDATAC);
	delayMicroseconds(1);
}

void spi_teardown()
{
	SPI.end();
	pinMode(MOSI, OUTPUT);
	digitalWrite(MOSI, LOW);
	pinMode(SCK, OUTPUT);
	digitalWrite(SCK, LOW);
}

struct error_code run_tests()
{
	{
		ShiftOutputs output = ShiftOutputs::power_off();
		output.enable_shield = 1;
		write_shift_out(output);
	}

	delay(STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS);

	if (check_for_short()) {
		return ERROR_BLINK_SHORT;
	}

	digitalWrite(PIN_RESIST_GND_ISO, HIGH);
	delay(STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS);
	if (check_div_gnd_high_fault()) {
		return ERROR_BLINK_GND_HI;	// bail early
	}

	if (check_3v3_bogus_iso_fault()) {
		return ERROR_BLINK_3V3_ISO;	// bail early
	}

	if (check_5v_bogus_iso_fault()) {
		return ERROR_BLINK_VIN_ISO;	// bail early
	}

	digitalWrite(PIN_RESIST_GND_ISO, LOW);
	delay(STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS);
	if (check_div_gnd_low_fault()) {
		return ERROR_BLINK_GND_LOW;	// bail early
	}

	ShiftInputs default_expected;
	default_expected.slave_and_slave_cs = 0;
	default_expected.master_and_master_cs = 0;
	default_expected.i_master = 0;
	default_expected.master = 1;
	default_expected.go_button = 0;
	default_expected.i_cs = 1;
	default_expected.dout = 0;
	default_expected.i_drdy = 1;

	default_expected.mosi_iso = 0;
	default_expected.SCLKiso = 0;
	default_expected.i_csiso = 1;
	default_expected.master_iso = 1;
	default_expected.clk_iso = 0;
	default_expected.i_drdy_iso = 1;
	default_expected.dout_iso = 0;
	default_expected.unused1 = 0;

	default_expected.gpio1 = 0;
	default_expected.gpio2 = 0;
	default_expected.gpio3 = 0;
	default_expected.gpio4 = 0;
	default_expected.daisyin = 0;
	default_expected.biasinv = 0;
	default_expected.unused2 = 0;

	// TODO: compare both shift inputs and SPI inputs
	{
		ShiftOutputs output;
		write_shift_out(output);
		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);

		ShiftInputs expected = default_expected;
		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_FIRST_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.master_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.master_and_master_cs = 1;
		expected.i_cs = 0;
		expected.i_csiso = 0;
		expected.dout = 1;
		expected.dout_iso = 1;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_MCS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.slave_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SCS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.master_ics = 0;
		output.slave_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.master_and_master_cs = 1;
		expected.i_cs = 0;
		expected.i_csiso = 0;
		expected.dout = 1;
		expected.dout_iso = 1;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_BOTH_CS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.master_ics = 0;
		output.slave_ics = 0;
		output.simulate_board_below = 1;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.slave_and_slave_cs = 1;
		expected.i_master = 1;
		expected.master = 0;
		expected.master_iso = 0;
		expected.i_cs = 0;
		expected.i_csiso = 0;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SLAVE_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.simulate_board_below = 1;
		output.master_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.i_master = 1;
		expected.master = 0;
		expected.master_iso = 0;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SLAVE_MCS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.simulate_board_below = 1;
		output.slave_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.slave_and_slave_cs = 1;
		expected.i_master = 1;
		expected.master = 0;
		expected.master_iso = 0;
		expected.i_cs = 0;
		expected.i_csiso = 0;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SLAVE_SCS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		output.simulate_board_below = 1;
		output.master_ics = 0;
		output.slave_ics = 0;
		write_shift_out(output);

		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.slave_and_slave_cs = 1;
		expected.i_master = 1;
		expected.master = 0;
		expected.master_iso = 0;
		expected.i_cs = 0;
		expected.i_csiso = 0;

		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SLAVE_BOTH_CS_SHIFT_IN;
		}
	}

	{
		ShiftOutputs output;
		write_shift_out(output);
		digitalWrite(MOSI, HIGH);
		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.mosi_iso = 1;
		ShiftInputs actual = read_shift_in();
		digitalWrite(MOSI, LOW);
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_MOSI;
		}
	}

	{
		ShiftOutputs output;
		write_shift_out(output);
		digitalWrite(SCK, HIGH);
		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.SCLKiso = 1;
		ShiftInputs actual = read_shift_in();
		digitalWrite(SCK, LOW);
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SCLK;
		}
	}

	{
		ShiftOutputs output;
		write_shift_out(output);
		digitalWrite(SCK, HIGH);
		delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
		ShiftInputs expected = default_expected;
		expected.SCLKiso = 1;
		ShiftInputs actual = read_shift_in();
		digitalWrite(SCK, LOW);
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			return ERROR_BLINK_SCLK;
		}
	}

	// basic electrical connection seems sane
	// next start interacting with ADS129x

	spi_setup();

	SPI.transfer(RREG | 0x00);	// ID is register 0
	SPI.transfer(0);	// number of registers to be read/written
	byte val = 0x1F & SPI.transfer(0);	// Bits[7:5] Not used (Datasheet 40)
	if (val != 0x1E) {
		spi_teardown();
		char buf[80];
		snprintf(buf, 80, "Expected ID:%X, but was:%X", 0x1E, val);
		Serial.println(buf);
		return ERROR_BLINK_CHIP_ID;
	}

	for (int i = 0; i < 4; ++i) {
		uint8_t data = (1 << (4 + i));
		if (1) {	// FIXME Eric's gpio4 is broken
			data |= 0x08;	// FIXME set gpio4 to be an input
		}
		adc_wreg(GPIO, data);

		ShiftInputs expected = default_expected;
		expected.master_and_master_cs = 1;
		expected.i_cs = 0;
		expected.i_csiso = 0;
		expected.mosi_iso = 1;
		switch (i) {
		case 0:
			expected.gpio1 = 1;
			break;
		case 1:
			expected.gpio2 = 1;
			break;
		case 2:
			expected.gpio3 = 1;
			break;
		case 3:
			expected.gpio4 = 1;
			break;
		}
		ShiftInputs actual = read_shift_in();
		bool compare_dout = false;
		if (shift_in_mismatch(&expected, &actual, compare_dout)) {
			spi_teardown();
			return ERROR_BLINK_GPIO;
		}
	}

	{
		ShiftOutputs output;
		output.master_ics = 0;
		// input/ladder
		// A high, b low - read analog data, ensure in approx range
		output.signal_b = 1;
		write_shift_out(output);
	}
	// Power up the internal reference and wait for it to settle
	uint8_t reserved7 = (1L << 7);
	uint8_t reserved4 = (1L << 4);
	uint8_t sps250 = 0x03;
	adc_wreg(CONFIG1, reserved7 | reserved4 | sps250);
	uint8_t reserved6 = (1L << 6);
	uint8_t reserved5 = (1L << 5);
	uint8_t pdrefbuf7 = (1L << 7);
	adc_wreg(CONFIG2, reserved7 | reserved6);
	adc_wreg(CONFIG3, pdrefbuf7 | reserved6 | reserved5);
	adc_wreg(CONFIG4, SINGLE_SHOT);
	for (int i = 1; i <= 8; ++i) {
		adc_wreg(CHnSET + i, ELECTRODE_INPUT);	// | GAIN_12X);
	}
	delay(1500);
	SPI.transfer(START);
	delay(1);
	{
		unsigned long timeout_milliseconds = 200;
		unsigned long start;

		start = millis();
		while (digitalRead(IPIN_DRDY) == HIGH) {
			if ((millis() - start) > timeout_milliseconds) {
				return ERROR_BLINK_NO_DRDY_1;
			}
		}
	}
	{
		ADS1298::Data_frame frame;
		read_data_frame(&frame);
		if (bad_magic(frame)) {
			char buf[80];
			format_data_frame(frame, buf);
			Serial.println(buf);
			return ERROR_BLINK_BAD_MOJO;
		}
		float ladder_resistence = 31500;
		float rung_resistence = 100;
		float io_voltage = 3.3;
		float reference_voltage = 4.5;
		float rung_delta_voltage = io_voltage *
			(rung_resistence/(ladder_resistence));
		unsigned long max_count = 0x7FFFFF;
		float rung_delta_count = (rung_delta_voltage /
			reference_voltage) * max_count;
		float min_ok = rung_delta_count * .9;
		float max_ok = rung_delta_count * 1.1;
		unsigned bad_channels = 0;
		for (unsigned chan = 1; chan <= 8; ++chan) {
			long val = channel_value(frame, chan);
			if(val < min_ok || val > max_ok) {
				++bad_channels;
			}

		}
		if (bad_channels) {
			char buf[80];
			format_data_frame(frame, buf);
			Serial.println(buf);
			for (unsigned chan = 1; chan <= 8; ++chan) {
				long val = channel_value(frame, chan);
				sprintf(buf, "chan[%u]: %ld\n", chan, val);
				Serial.println(buf);
			}
			spi_teardown();
			return ERROR_BLINK_DATA_OOR;
		}
	}

	// flip B high, A low - read analog data, ensure in approx range

	// BIAS_OUT

	// slave board clocking and data

	spi_teardown();

	return ERROR_BLINK_SUCCESS;
}

// for testing the test boards themselves, this function
// can be used to verify behavior and pin states using a
// multi-meter and jumpers
// (without a shield connected)
static void harness_hardware_validation()
{
	ShiftOutputs output;
	ShiftInputs input;
	static bool oddLoop = false;

	oddLoop = !oddLoop;

	output.simulate_board_below = oddLoop;
	output.master_ics = !oddLoop;
	output.slave_ics = oddLoop;
	output.signal_a = !oddLoop;
	output.signal_b = oddLoop;
	output.success_led = !oddLoop;
	output.fault_led = oddLoop;
	output.enable_shield = !oddLoop;
	write_shift_out(output);
	digitalWrite(13, oddLoop ? HIGH : LOW);
	digitalWrite(PIN_RESIST_GND_ISO, oddLoop ? HIGH : LOW);

	int a_vin = analogRead(PIN_DIV_VIN_ISO);
	int a_3v3 = analogRead(PIN_DIV_3V3_ISO);
	int a_gnd = analogRead(PIN_DIV_GND_ISO);
	int a_bia = analogRead(PIN_BIASOUT_FILT);

	char buf[1024];

	sprintf(buf, "[%s] Vin: %5d, 3v3: %5d, Gnd: %5d, BiasOut: %5d",
		oddLoop ? " red " : "green", a_vin, a_3v3, a_gnd, a_bia);

	Serial.println(buf);

	input = read_shift_in();

	Serial.print("P23: ");
	Serial.print(input.mosi_iso);
	Serial.print(", P24: ");
	Serial.print(input.SCLKiso);
	Serial.print(", P25: ");
	Serial.print(input.i_csiso);
	Serial.print(", P26: ");
	Serial.print(input.master_iso);
	Serial.print(", P27: ");
	Serial.print(input.dout_iso);
	Serial.print(", P28: ");
	Serial.print(input.i_drdy_iso);
	Serial.println("");

	Serial.print("P32: ");
	Serial.print(input.gpio1);
	Serial.print(", P33: ");
	Serial.print(input.gpio2);
	Serial.print(", P34: ");
	Serial.print(input.gpio3);
	Serial.print(", P35: ");
	Serial.print(input.gpio4);
	Serial.print(", P36: ");
	Serial.print(input.daisyin);
	Serial.print(", P37: ");
	Serial.print(input.biasinv);
	Serial.println("");

	Serial.print("P38: ");
	Serial.print(input.clk_iso);
	Serial.print(", P39: ");
	Serial.print(input.slave_and_slave_cs);
	Serial.print(", P40: ");
	Serial.print(input.master_and_master_cs);
	Serial.print(", P41: ");
	Serial.print(input.i_master);
	Serial.print(", P42: ");
	Serial.print(input.i_cs);
	Serial.print(", P43: ");
	Serial.print(input.master);
	Serial.println("");

	Serial.print("P44: ");
	Serial.print(input.dout);
	Serial.print(", P45: ");
	Serial.print(input.i_drdy);
	Serial.print(",  Go: ");
	Serial.print(input.go_button);
	Serial.print(",  U1: ");
	Serial.print(input.unused1);
	Serial.print(",  U2: ");
	Serial.print(input.unused2);
	Serial.print(",  U3: ");
	Serial.print(input.unused3);
	Serial.println(".");

	delay(10000);		// wait for 10 seconds
}

void loop()
{
	if (false) {
		harness_hardware_validation();
	} else {
		ShiftInputs input = read_shift_in();
		if (input.go_button) {
			struct error_code error = run_tests();
			if (error.blink_code != ERROR_BLINK_SUCCESS.blink_code) {
				char buf[80];
				sprintf(buf, "Error %u: %s",
					error.blink_code, error.error_txt);
				Serial.println(buf);
				blink_error(error);
			} else {
				Serial.println("SUCCESS!");
				ShiftOutputs output;
				output.success_led = 1;

				write_shift_out(output);

				delay(3000);

				output.enable_shield = 0;
				write_shift_out(output);

				// in real life we'd loop until detected board removed
				delay(3000);
				output.success_led = 0;
				output.fault_led = 0;
				write_shift_out(output);
			}
		}
	}
}
