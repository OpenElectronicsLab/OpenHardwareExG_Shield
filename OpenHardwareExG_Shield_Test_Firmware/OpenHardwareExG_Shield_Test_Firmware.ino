/*
  OpenHardwareExG_Shield_Test_Firmware
 */

struct ShiftOutputs {
    unsigned sendToGround : 1;
    unsigned masterCS : 1;
    unsigned slaveCS : 1;
    unsigned signalA : 1;
    unsigned signalB : 1;
    unsigned successLED : 1;
    unsigned faultLED : 1;
    unsigned enableShield : 1;

    ShiftOutputs() {
        sendToGround = 0;
        masterCS = 0;
        slaveCS = 0;
        signalA = 0;
        signalB = 0;
        successLED = 0;
        faultLED = 0;
        enableShield = 0;
    }
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
    P10, // not present
    P11_IN8P,
    P12_IN7P,
    P13_IN6P,
    P14_IN5P,
    P15_IN4P,
    P16_IN3P,
    P17_IN2P,
    P18_IN1P,
    P19, // not present
    P20, // not present
    P21, // not present
    P22, // not present
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
    shiftOutClockDelay = 1
};

void shiftOut(bool value) {
    digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW); // should be a NOP
    digitalWrite(PIN_SHIFT_OUT, value ? HIGH : LOW);
    delay(shiftOutClockDelay);
    digitalWrite(PIN_SHIFT_OUT_SRCLK, HIGH);
    delay(shiftOutClockDelay);
    digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW);
}

void setShiftOut(const struct ShiftOutputs& output) {
    shiftOut(output.enableShield);
    shiftOut(output.faultLED);
    shiftOut(output.successLED);
    shiftOut(output.signalA);
    shiftOut(output.signalB);
    shiftOut(output.slaveCS);
    shiftOut(output.masterCS);
    shiftOut(output.sendToGround);

    digitalWrite(PIN_SHIFT_OUT_RCLK, HIGH);
    delay(shiftOutClockDelay);
    digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);
}

// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin 13 as an output.
    pinMode(13, OUTPUT);

    // set up the pins for the output shift registers
    pinMode(PIN_SHIFT_OUT_RCLK, OUTPUT);
    pinMode(PIN_SHIFT_OUT_SRCLK, OUTPUT);
    pinMode(PIN_SHIFT_OUT, OUTPUT);
    pinMode(IPIN_SHIFT_OUT_SRCLR, OUTPUT);

    digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);
    digitalWrite(PIN_SHIFT_OUT_SRCLK, LOW);
    digitalWrite(PIN_SHIFT_OUT, LOW);

    // toggle the clear pins to clear the registers
    digitalWrite(IPIN_SHIFT_OUT_SRCLR, LOW);
    delay(shiftOutClockDelay);
    digitalWrite(PIN_SHIFT_OUT_RCLK, HIGH);
    delay(shiftOutClockDelay);
    digitalWrite(IPIN_SHIFT_OUT_SRCLR, HIGH);
    digitalWrite(PIN_SHIFT_OUT_RCLK, LOW);
}

// the loop function runs over and over again forever
void loop() {
    ShiftOutputs output;

    output.sendToGround = 1;
    output.masterCS = 0;
    output.slaveCS = 1;
    output.signalA = 0;
    output.signalB = 1;
    output.successLED = 0;
    output.faultLED = 1;
    output.enableShield = 0;
    setShiftOut(output);
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    output.sendToGround = 0;
    output.masterCS = 1;
    output.slaveCS = 0;
    output.signalA = 1;
    output.signalB = 0;
    output.successLED = 1;
    output.faultLED = 0;
    output.enableShield = 1;
    setShiftOut(output);
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second
}
