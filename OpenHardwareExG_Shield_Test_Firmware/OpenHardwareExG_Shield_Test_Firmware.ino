/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
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
    PIN_SHIFT_OUT_RCLK = 5,
    PIN_SHIFT_OUT_SRCLK = 4,
    IPIN_SHIFT_OUT_SRCLR = 3,
    PIN_SHIFT_OUT = 6
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
