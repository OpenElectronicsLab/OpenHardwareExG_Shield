/*
  OpenHardwareExG_Shield_Test_Firmware
 */

// some Arduino's use 5.0 ...
#define ANALOG_REFERENCE_VOLTAGE 3.3

// wait this long before attempting to test a short
#define STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS 25

// wait additional time before testing other board features
#define STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS 25

// time to wait between testing state changes
# define DIGITAL_STATE_CHANGE_DELAY_MICROS 2

#include <stdio.h>

struct ShiftOutputs {
    unsigned simulateBoardBelow : 1;
    unsigned master_ics : 1;
    unsigned slave_ics : 1;
    unsigned signalA : 1;
    unsigned signalB : 1;
    unsigned successLED : 1;
    unsigned faultLED : 1;
    unsigned enable_shield : 1;

    ShiftOutputs() {
        simulateBoardBelow = 0;
        master_ics = 1;
        slave_ics = 1;
        signalA = 0;
        signalB = 0;
        successLED = 0;
        faultLED = 0;
        enable_shield = 1;
    }

    static ShiftOutputs power_off() {
        ShiftOutputs result;
	result.master_ics = 0;
        result.slave_ics = 0;
        result.enable_shield = 0;

	return result;
    }
};

struct ShiftInputs {
    unsigned slaveAndSlaveCS : 1;
    unsigned masterAndMasterCS : 1;
    unsigned iMaster : 1;
    unsigned master : 1;
    unsigned goButton : 1;
    unsigned iCS : 1;
    unsigned DOUT : 1;
    unsigned iDRDY : 1;

    unsigned MOSIiso : 1;
    unsigned SCLKiso : 1;
    unsigned iCSiso : 1;
    unsigned master_iso : 1;
    unsigned clk_iso : 1;
    unsigned i_drdy_iso : 1;
    unsigned dout_iso : 1;
    unsigned unused1 : 1;

    unsigned GPIO1 : 1;
    unsigned GPIO2 : 1;
    unsigned GPIO3 : 1;
    unsigned GPIO4 : 1;
    unsigned DAISYIN : 1;
    unsigned BIASINV : 1;
    unsigned unused2 : 1;
    unsigned unused3 : 1;
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
    P1_IN8N = 1, // P1 == 1 for easier debug
    P2_IN7N,
    P3_IN6N,
    P4_IN5N,
    P5_IN4N,
    P6_IN3N,
    P7_IN2N,
    P8_IN1N,
    P9_BIASOUT_FILT, // A5
    P10, // not present (removed during design phase)
    P11_IN8P,
    P12_IN7P,
    P13_IN6P,
    P14_IN5P,
    P15_IN4P,
    P16_IN3P,
    P17_IN2P,
    P18_IN1P,
    P19, // not present (removed during design phase)
    P20, // not present (removed during design phase)
    P21, // not present (removed during design phase)
    P22, // not present (removed during design phase)
    P23_MOSI_ISO, // U2 input E
    P24_SCLK_ISO, // U2 input F
    P25_INV_CS_ISO, // U2 input G
    P26_MASTER_ISO, // U2 input H
    P27_DOUT_ISO, // U2 input A
    P28_INV_DRDY_ISO, // U2 input B
    P29_VIN_ISO, // A1 via voltage divider
    P30_3V3_ISO, // A2 via voltage divider
    P31_GND_ISO, // A3 via voltage divider, A4 via resistor
    P32_GPIO1_ISO, // U1 input E
    P33_GPIO2_ISO, // U1 input F
    P34_GPIO3_ISO, // U1 input G
    P35_GPIO4_ISO, // U1 input H
    P36_DAISY_IN_ISO, // U1 input C
    P37_BIASINV, // U1 input B
    P38_CLK_ISO, // U2 input C
    P39_SLAVE_AND_SLAVE_CS, // U4 input E
    P40_MASTER_AND_MASTER_CS, // U4 input F
    P41_INV_MASTER, // U4 input G
    P42_INV_CS, // U4 input D
    P43_MASTER, // U4 input C
    P44_DOUT, // U4 input B
    P45_INV_DRDY, // U4 input A
    P46_SHIELD_VCC, // toggled on U6 by ENABLE_SHIELD (U3 output H)
    P47_SHIELD_3V3, // toggled on U6 by ENABLE_SHIELD (U3 output H)
    P48_SHIELD_5V, // toggled on U6 by ENABLE_SHIELD (U3 output H)
    P49_ARDUINO_GND, // toggled on U6 by ENABLE_SHIELD (U3 output H)
    P50_MASTER_INV_CS, // U3 input B
    P51_SLAVE_INV_CS, // U3 input C
    P52_MASTER_INV_DRDY, // PIN2
    P53_MISO, // MISO
    P54_MOSI, // MOSI
    P55_SCLK // SCLK
};

enum Delays {
    shiftClockDelay = 1
};

void shiftOut(bool value) {
    digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW); // should be a NOP
    digitalWrite(PIN_SHIFT_OUT, value ? HIGH : LOW);
    delayMicroseconds(shiftClockDelay);
    digitalWrite(PIN_SHIFT_OUT_SRCLK, HIGH);
    delayMicroseconds(shiftClockDelay);
    digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW);
}

void writeShiftOut(const struct ShiftOutputs& output) {
    shiftOut(output.enable_shield);
    shiftOut(output.faultLED);
    shiftOut(output.successLED);
    shiftOut(output.signalB);
    shiftOut(output.signalA);
    shiftOut(output.slave_ics);
    shiftOut(output.master_ics);
    shiftOut(output.simulateBoardBelow ? 0 : 1); // IPIN ~SEND_TO_GND~_MASTER

    digitalWrite(PIN_SHIFT_OUT_RCLK, HIGH);
    delayMicroseconds(shiftClockDelay);
    digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);
}

unsigned shiftIn() {
    unsigned d = digitalRead(PIN_SHIFT_IN);
    digitalWrite(PIN_SHIFT_CLK, HIGH);
    delayMicroseconds(shiftClockDelay);
    digitalWrite(PIN_SHIFT_CLK, LOW);
    delayMicroseconds(shiftClockDelay);
    return d;
}

struct ShiftInputs readShiftIn()
{
    struct ShiftInputs input;

    digitalWrite(IPIN_SHIFT_SH_LD, LOW);
    delayMicroseconds(shiftClockDelay);
    digitalWrite(IPIN_SHIFT_SH_LD, HIGH);
    delayMicroseconds(shiftClockDelay);

    input.goButton = shiftIn();
    input.iMaster = shiftIn();
    input.masterAndMasterCS = shiftIn();
    input.slaveAndSlaveCS = shiftIn();
    input.iCS = shiftIn();
    input.master = shiftIn();
    input.DOUT = shiftIn();
    input.iDRDY = shiftIn();

    input.master_iso = shiftIn();
    input.iCSiso = shiftIn();
    input.SCLKiso = shiftIn();
    input.MOSIiso = shiftIn();
    input.unused1 = shiftIn();
    input.clk_iso = shiftIn();
    input.i_drdy_iso = shiftIn();
    input.dout_iso = shiftIn();

    input.GPIO4 = shiftIn();
    input.GPIO3 = shiftIn();
    input.GPIO2 = shiftIn();
    input.GPIO1 = shiftIn();
    input.unused2 = shiftIn();
    input.DAISYIN = shiftIn();
    input.BIASINV = shiftIn();
    input.unused3 = shiftIn();

    return input;
}


// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin 13 as an output.
    pinMode(13, OUTPUT);

    pinMode(PIN_SHIFT_IN, INPUT);
    pinMode(PIN_SHIFT_CLK, OUTPUT);
    pinMode(IPIN_SHIFT_SH_LD, OUTPUT);

    pinMode(PIN_DIV_VIN_ISO, INPUT); // un-needed?
    pinMode(PIN_DIV_3V3_ISO, INPUT); // un-needed?
    pinMode(PIN_DIV_GND_ISO, INPUT); // un-needed?
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
    writeShiftOut(output);

    // initialize the USB Serial connection
    Serial.begin(115200);
}

bool check_for_short()
{
    unsigned long timeout_milliseconds = 200;
    unsigned long start;

    start = millis();
    while ((millis()-start) < timeout_milliseconds) {
        unsigned d = digitalRead(IPIN_SHIELD_SHORTED);
        if (!d) {
	    ShiftOutputs output = ShiftOutputs::power_off();
            writeShiftOut(output); // disables power to the board
            return true;
        }
    }
    return false;
}

bool check_div_gnd_high_fault()
{
    int gnd = analogRead(PIN_DIV_GND_ISO); // 0 - 1023
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
    int gnd = analogRead(PIN_DIV_GND_ISO); // 0 - 1023
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
    int val0 = analogRead(PIN_DIV_GND_ISO); // 0 - 1023

    // bc <<<"scale=3; 11* ((93 * 3.3)/1023) "
    // 3.300

    char buf[1024];
    sprintf(buf, "PIN_DIV_GND_ISO: %d", val0);
    Serial.println(buf);

    int val1 = analogRead(PIN_DIV_3V3_ISO); // 0 - 1023
    sprintf(buf, "PIN_DIV_3V3_ISO: %d", val1);
    Serial.println(buf);

    /* ANALOG_REFERENCE_VOLTAGE */
    float voltage = 11.0 * (((val1-val0)*3.3)/1023.0);

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
    int val0 = analogRead(PIN_DIV_GND_ISO); // 0 - 1023

    char buf[1024];
    sprintf(buf, "PIN_DIV_GND_ISO: %d", val0);
    Serial.println(buf);

    int val1 = analogRead(PIN_DIV_VIN_ISO); // 0 - 1023
    sprintf(buf, "PIN_DIV_VIN_ISO: %d", val1);
    Serial.println(buf);

    float voltage = 11.0 * (((val1-val0)*3.3)/1023.0);

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
struct error_code ERROR_BLINK_SHORT   = { 0x00001, "Short detected" };
struct error_code ERROR_BLINK_GND_HI  = { 0x00002, "problem with GND iso" };
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
struct error_code ERROR_BLINK_SLAVE_BOTH_CS_SHIFT_IN = { 0x0000D, "SLAVE BOTH CS" };

void blink_error(struct error_code err)
{
	Serial.println(err.error_txt);

	ShiftOutputs errorOutput = ShiftOutputs::power_off();
	errorOutput.faultLED = 1;
	writeShiftOut(errorOutput);

	for (int i = 0; i < 10 /* or board removed */; ++i) {
		// TODO: actually blink this
		errorOutput.faultLED = 1;
		writeShiftOut(errorOutput);
	}

	delay(3000);
	errorOutput.faultLED = 0;
	writeShiftOut(errorOutput);
}

/* the ADS1299 datasheet
   (page 38 of SBAS499A - JULY 2012 - REVISED AUGUST 2012)
   shows that DOUT is undefined at some times, set compare_dout to false
   if comparing during an undefined time */
unsigned long shift_in_mismatch(struct ShiftInputs *expected, struct ShiftInputs *actual, bool compare_dout = true)
{
    unsigned long i=0;
    unsigned long errors=0;
    bool match;
    char buf[255];

    if(expected->slaveAndSlaveCS != actual->slaveAndSlaveCS) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing slaveAndSlaveCS. Expected %u but was %u", expected->slaveAndSlaveCS, actual->slaveAndSlaveCS);
        Serial.println(buf);
    }
    ++i;
    if(expected->masterAndMasterCS != actual->masterAndMasterCS) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing masterAndMasterCS. Expected %u but was %u", expected->masterAndMasterCS, actual->masterAndMasterCS);
        Serial.println(buf);
    }
    ++i;
    if(expected->iMaster != actual->iMaster) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing iMaster. Expected %u but was %u", expected->iMaster, actual->iMaster);
        Serial.println(buf);
    }
    ++i;
    if(expected->master != actual->master) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing master. Expected %u but was %u", expected->master, actual->master);
        Serial.println(buf);
    }
    ++i;
    if( 0 && expected->goButton != actual->goButton) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing goButton. Expected %u but was %u", expected->goButton, actual->goButton);
        Serial.println(buf);
    }
    ++i;
    if(expected->iCS != actual->iCS) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing iCS. Expected %u but was %u", expected->iCS, actual->iCS);
        Serial.println(buf);
    }
    ++i;
    if(compare_dout && expected->DOUT != actual->DOUT) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing DOUT. Expected %u but was %u", expected->DOUT, actual->DOUT);
        Serial.println(buf);
    }
    ++i;
    if(expected->iDRDY != actual->iDRDY) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing iDRDY. Expected %u but was %u", expected->iDRDY, actual->iDRDY);
        Serial.println(buf);
    }
    ++i;

    if(expected->MOSIiso != actual->MOSIiso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing MOSIiso. Expected %u but was %u", expected->MOSIiso, actual->MOSIiso);
        Serial.println(buf);
    }
    ++i;
    if(expected->SCLKiso != actual->SCLKiso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing SCLKiso. Expected %u but was %u", expected->SCLKiso, actual->SCLKiso);
        Serial.println(buf);
    }
    ++i;
    if(expected->iCSiso != actual->iCSiso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing iCSiso. Expected %u but was %u", expected->iCSiso, actual->iCSiso);
        Serial.println(buf);
    }
    ++i;
    if(expected->master_iso != actual->master_iso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing master_iso. Expected %u but was %u", expected->master_iso, actual->master_iso);
        Serial.println(buf);
    }
    ++i;
    if(expected->clk_iso != actual->clk_iso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing clk_iso. Expected %u but was %u", expected->clk_iso, actual->clk_iso);
        Serial.println(buf);
    }
    ++i;
    if(expected->i_drdy_iso != actual->i_drdy_iso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing i_drdy_iso. Expected %u but was %u", expected->i_drdy_iso, actual->i_drdy_iso);
        Serial.println(buf);
    }
    ++i;
    if(compare_dout && expected->dout_iso != actual->dout_iso) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing dout_iso. Expected %u but was %u", expected->dout_iso, actual->dout_iso);
        Serial.println(buf);
    }
    ++i;
    if(0 && expected->unused1 != actual->unused1) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing unused1. Expected %u but was %u", expected->unused1, actual->unused1);
        Serial.println(buf);
    }
    ++i;

    if(expected->GPIO1 != actual->GPIO1) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing GPIO1. Expected %u but was %u", expected->GPIO1, actual->GPIO1);
        Serial.println(buf);
    }
    ++i;
    if(expected->GPIO2 != actual->GPIO2) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing GPIO2. Expected %u but was %u", expected->GPIO2, actual->GPIO2);
        Serial.println(buf);
    }
    ++i;
    if(expected->GPIO3 != actual->GPIO3) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing GPIO3. Expected %u but was %u", expected->GPIO3, actual->GPIO3);
        Serial.println(buf);
    }
    ++i;
    if(expected->GPIO4 != actual->GPIO4) {
        // errors |= (1L<<i); // FIXME: Eric's board is solder-bridged to DRDY
        Serial.println("mismatch comparing GPIO4 (Supressed)");
    }
    ++i;
    if(expected->DAISYIN != actual->DAISYIN) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing DAISYIN. Expected %u but was %u", expected->DAISYIN, actual->DAISYIN);
        Serial.println(buf);
    }
    ++i;
    if(expected->BIASINV != actual->BIASINV) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing BIASINV. Expected %u but was %u", expected->BIASINV, actual->BIASINV);
        Serial.println(buf);
    }
    ++i;
    if(0 && expected->unused2 != actual->unused2) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing unused2. Expected %u but was %u", expected->unused2, actual->unused2);
        Serial.println(buf);
    }
    ++i;
    if(0 && expected->unused3 != actual->unused3) {
        errors |= (1L<<i);
        sprintf(buf, "mismatch comparing unused3. Expected %u but was %u", expected->unused3, actual->unused3);
        Serial.println(buf);
    }
    ++i;

    if(actual->master == actual->iMaster) {
        errors |= (1L<<i);
        Serial.println("inconsistent master, iMaster");
    };
    ++i;

    if(actual->master != actual->master_iso) {
        errors |= (1L<<i);
        Serial.println("inconsistent master, master_iso");
    };
    ++i;

    if(actual->DOUT != actual->dout_iso) {
        errors |= (1L<<i);
        Serial.println("inconsistent dout, dout_iso");
    };
    ++i;

    if(actual->iCS != actual->iCSiso) {
        errors |= (1L<<i);
        Serial.println("inconsistent iCS, iCSiso");
    };
    ++i;

    if(actual->iDRDY != actual->i_drdy_iso) {
        errors |= (1L<<i);
        Serial.println("inconsistent iDRDY, i_drdy_iso");
    };
    ++i;

    if (errors) {
        char buf[80];
        sprintf(buf, "(errors: 0x%8lx)", errors);
        Serial.println(buf);
    }

    return errors;
}

struct error_code run_tests()
{

    {
	ShiftOutputs output = ShiftOutputs::power_off();
        output.enable_shield = 1;
        writeShiftOut(output);
    }

    delay(STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS);

    if(check_for_short()) {
	return ERROR_BLINK_SHORT;
    }

    digitalWrite(PIN_RESIST_GND_ISO, HIGH);
    delay(STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS);
    if(check_div_gnd_high_fault()) {
	return ERROR_BLINK_GND_HI; // bail early
    }

    if(check_3v3_bogus_iso_fault()) {
	return ERROR_BLINK_3V3_ISO; // bail early
    }

    if(check_5v_bogus_iso_fault()) {
	return ERROR_BLINK_VIN_ISO; // bail early
    }

    digitalWrite(PIN_RESIST_GND_ISO, LOW);
    delay(STARTUP_ADDITIONAL_CAPACITOR_CHARGE_DELAY_MILLIS);
    if(check_div_gnd_low_fault()) {
	return ERROR_BLINK_GND_LOW; // bail early
    }

    ShiftInputs default_expected;
    default_expected.slaveAndSlaveCS = 0;
    default_expected.masterAndMasterCS = 0;
    default_expected.iMaster = 0;
    default_expected.master = 1;
    default_expected.goButton = 0;
    default_expected.iCS = 1;
    default_expected.DOUT = 0;
    default_expected.iDRDY = 1;

    default_expected.MOSIiso = 0;
    default_expected.SCLKiso = 0;
    default_expected.iCSiso = 1;
    default_expected.master_iso = 1;
    default_expected.clk_iso = 0;
    default_expected.i_drdy_iso = 1;
    default_expected.dout_iso = 0;
    default_expected.unused1 = 0;

    default_expected.GPIO1 = 0;
    default_expected.GPIO2 = 0;
    default_expected.GPIO3 = 0;
    default_expected.GPIO4 = 0;
    default_expected.DAISYIN = 0;
    default_expected.BIASINV = 0;
    default_expected.unused2 = 0;

    // TODO: compare both shift inputs and SPI inputs
    bool compare_dout = false;
    {
	ShiftOutputs output;
        writeShiftOut(output);
        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);

        ShiftInputs expected = default_expected;
        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_FIRST_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
        output.master_ics=0;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;
        expected.masterAndMasterCS = 1;
        expected.iCS = 0;
        expected.iCSiso = 0;
        expected.DOUT = 1;
        expected.dout_iso = 1;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_MCS_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
        output.slave_ics=0;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_SCS_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
	output.master_ics=0;
        output.slave_ics=0;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;
        expected.masterAndMasterCS = 1;
        expected.iCS = 0;
        expected.iCSiso = 0;
        expected.DOUT = 1;
        expected.dout_iso = 1;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_BOTH_CS_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
        output.master_ics=0;
        output.slave_ics=0;
        output.simulateBoardBelow=1;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;
        expected.slaveAndSlaveCS = 1;
        expected.iMaster = 1;
        expected.master = 0;
        expected.master_iso = 0;
        expected.iCS = 0;
        expected.iCSiso = 0;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_SLAVE_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
        output.simulateBoardBelow=1;
        output.master_ics=0;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;
        expected.iMaster = 1;
        expected.master = 0;
        expected.master_iso = 0;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_SLAVE_MCS_SHIFT_IN;
        }
    }

/*
struct error_code ERROR_BLINK_SLAVE_SHIFT_IN = { 0x0000A, "SLAVE" };
struct error_code ERROR_BLINK_SLAVE_MCS_SHIFT_IN = { 0x0000B, "SLAVE MCS" };
struct error_code ERROR_BLINK_SLAVE_SCS_SHIFT_IN = { 0x0000C, "SLAVE SCS" };
struct error_code ERROR_BLINK_SLAVE_BOTH_CS_SHIFT_IN = { 0x0000D, "SLAVE BOTH CS" };

    {
        ShiftOutputs output;
        output.slave_ics=1;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_SCS_SHIFT_IN;
        }
    }

    {
        ShiftOutputs output;
        output.master_ics=1;
        output.slave_ics=1;
        writeShiftOut(output);

        delayMicroseconds(DIGITAL_STATE_CHANGE_DELAY_MICROS);
        ShiftInputs expected = default_expected;
        expected.masterAndMasterCS = 0;
        expected.DOUT = 1;
        expected.dout_iso = 1;
        expected.iCS = 1;
        expected.iCSiso = 1;

        ShiftInputs actual = readShiftIn();
        if(shift_in_mismatch(&expected, &actual, compare_dout)) {
           return ERROR_BLINK_MSCS_SHIFT_IN;
        }
    }
*/
    return ERROR_BLINK_SUCCESS;
}

// for testing the test boards themselves, this function
// can be used to verify behavior and pin states using a
// multi-meter and jumpers
// (without a shield connected)
static void harness_hardware_validation() {
    ShiftOutputs output;
    ShiftInputs input;
    static bool oddLoop = false;

    oddLoop = !oddLoop;

    output.simulateBoardBelow = oddLoop;
    output.master_ics = !oddLoop;
    output.slave_ics = oddLoop;
    output.signalA = !oddLoop;
    output.signalB = oddLoop;
    output.successLED = !oddLoop;
    output.faultLED = oddLoop;
    output.enable_shield = !oddLoop;
    writeShiftOut(output);
    digitalWrite(13, oddLoop ? HIGH : LOW );
    digitalWrite(PIN_RESIST_GND_ISO, oddLoop ? HIGH : LOW );

    int a_vin = analogRead(PIN_DIV_VIN_ISO);
    int a_3v3 = analogRead(PIN_DIV_3V3_ISO);
    int a_gnd = analogRead(PIN_DIV_GND_ISO);
    int a_bia = analogRead(PIN_BIASOUT_FILT);

    char buf[1024];

    sprintf(buf, "[%s] Vin: %5d, 3v3: %5d, Gnd: %5d, BiasOut: %5d",
        oddLoop ? " red " : "green", a_vin, a_3v3, a_gnd, a_bia);

    Serial.println(buf);

    input = readShiftIn();

    Serial.print("P23: ");
    Serial.print(input.MOSIiso);
    Serial.print(", P24: ");
    Serial.print(input.SCLKiso);
    Serial.print(", P25: ");
    Serial.print(input.iCSiso);
    Serial.print(", P26: ");
    Serial.print(input.master_iso);
    Serial.print(", P27: ");
    Serial.print(input.dout_iso);
    Serial.print(", P28: ");
    Serial.print(input.i_drdy_iso);
    Serial.println("");

    Serial.print("P32: ");
    Serial.print(input.GPIO1);
    Serial.print(", P33: ");
    Serial.print(input.GPIO2);
    Serial.print(", P34: ");
    Serial.print(input.GPIO3);
    Serial.print(", P35: ");
    Serial.print(input.GPIO4);
    Serial.print(", P36: ");
    Serial.print(input.DAISYIN);
    Serial.print(", P37: ");
    Serial.print(input.BIASINV);
    Serial.println("");

    Serial.print("P38: ");
    Serial.print(input.clk_iso);
    Serial.print(", P39: ");
    Serial.print(input.slaveAndSlaveCS);
    Serial.print(", P40: ");
    Serial.print(input.masterAndMasterCS);
    Serial.print(", P41: ");
    Serial.print(input.iMaster);
    Serial.print(", P42: ");
    Serial.print(input.iCS);
    Serial.print(", P43: ");
    Serial.print(input.master);
    Serial.println("");

    Serial.print("P44: ");
    Serial.print(input.DOUT);
    Serial.print(", P45: ");
    Serial.print(input.iDRDY);
    Serial.print(",  Go: ");
    Serial.print(input.goButton);
    Serial.print(",  U1: ");
    Serial.print(input.unused1);
    Serial.print(",  U2: ");
    Serial.print(input.unused2);
    Serial.print(",  U3: ");
    Serial.print(input.unused3);
    Serial.println(".");

    delay(10000);              // wait for 10 seconds
}

void loop() {
    if (false) {
        harness_hardware_validation();
    } else {
	ShiftInputs input = readShiftIn();
	if (input.goButton) {
	    struct error_code error = run_tests();
	    if(error.blink_code != ERROR_BLINK_SUCCESS.blink_code){
		char buf[80];
		sprintf(buf, "Error %u: %s",
                        error.blink_code, error.error_txt);
		Serial.println(buf);
		blink_error(error);
	    } else {
		Serial.println("SUCCESS!");
                ShiftOutputs output;
                output.successLED = 1;

                writeShiftOut(output);

                delay(3000);

                output.enable_shield=0;
                writeShiftOut(output);

                // in real life we'd loop until detected board removed
                delay(3000);
                output.successLED = 0;
                output.faultLED = 0;
                writeShiftOut(output);
	    }
	}
    }
}
