/*
  OpenHardwareExG_Shield_Test_Firmware
 */

// some Arduino's use 5.0 ...
#define ANALOG_REFERENCE_VOLTAGE 3.3

// wait this long before attempting to test a short
#define STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS 25

#include <stdio.h>

struct ShiftOutputs {
    unsigned simulateBoardBelow : 1;
    unsigned masterCS : 1;
    unsigned slaveCS : 1;
    unsigned signalA : 1;
    unsigned signalB : 1;
    unsigned successLED : 1;
    unsigned faultLED : 1;
    unsigned enableShield : 1;

    ShiftOutputs() {
        simulateBoardBelow = 0;
        masterCS = 0;
        slaveCS = 0;
        signalA = 0;
        signalB = 0;
        successLED = 0;
        faultLED = 0;
        enableShield = 0;
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
    shiftOut(output.enableShield);
    shiftOut(output.faultLED);
    shiftOut(output.successLED);
    shiftOut(output.signalB);
    shiftOut(output.signalA);
    shiftOut(output.slaveCS);
    shiftOut(output.masterCS);
    shiftOut(!output.simulateBoardBelow); // IPIN ~SEND_TO_GND~_MASTER

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
    ShiftOutputs output;
    writeShiftOut(output);

    // initialize the USB Serial connection
    Serial.begin(115200);
}

bool check_for_short()
{
    unsigned long timeout_milliseconds = 200;
    unsigned long start;

    delay(STARTUP_CAPACITOR_CHARGE_DELAY_MILLIS);

    start = millis();
    while ((millis()-start) < timeout_milliseconds) {
        unsigned d = digitalRead(IPIN_SHIELD_SHORTED);
        if (!d) {
	    ShiftOutputs errorOutput;
            writeShiftOut(errorOutput); // disables power to the board
            return true;
        }
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

struct error_code ERROR_BLINK_SHORT   = { 0x00000, "Short detected" };
struct error_code ERROR_BLINK_3V3_ISO = { 0x00001, "problem with 3v3 iso" };
struct error_code ERROR_BLINK_VIN_ISO = { 0x00002, "problem with VIN iso" };

void blink_error(struct error_code err)
{
   Serial.println(err.error_txt);

   ShiftOutputs errorOutput;
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

void run_tests()
{
    ShiftOutputs output;

    output.enableShield=1; // powers test board
    writeShiftOut(output);

    if(check_for_short()) {
	blink_error(ERROR_BLINK_SHORT);
	return; // bail early
    }

    if(check_3v3_bogus_iso_fault()) {
	blink_error(ERROR_BLINK_3V3_ISO);
	return; // bail early
    }

    if(check_5v_bogus_iso_fault()) {
	blink_error(ERROR_BLINK_VIN_ISO);
	return; // bail early
    }


    output.successLED = 1;
    writeShiftOut(output);

    delay(3000);

    output.enableShield=0;
    writeShiftOut(output);

    // in real life we'd loop until detected board removed
    delay(3000);
    output.successLED = 0;
    output.faultLED = 0;
    writeShiftOut(output);
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
    output.masterCS = !oddLoop;
    output.slaveCS = oddLoop;
    output.signalA = !oddLoop;
    output.signalB = oddLoop;
    output.successLED = !oddLoop;
    output.faultLED = oddLoop;
    output.enableShield = !oddLoop;
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
	    run_tests();
	}
    }
}
