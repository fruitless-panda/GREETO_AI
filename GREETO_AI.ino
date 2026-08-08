// =============================
// Includes
// =============================
#include <Wire.h>
#include "DFRobot_GP8403.h"
#include <Adafruit_BNO08x_RVC.h>

// =============================
// Pin Definitions
// =============================

// BNO085 IMU pins (UART-RVC mode on Serial1)
#define BNO_RX_PIN 34
#define BNO_TX_PIN 17

// DAC I2C pins
#define DAC_SDA 19
#define DAC_SCL 18

// Hall sensor pins
#define H1A 32
#define H1B 33
#define H1C 25

#define H2A 26
#define H2B 27
#define H2C 14

// Motor output pins
#define DIR_M1 4
#define DIR_M2 23


// =============================
// Configuration Constants
// =============================

// Fixed speeds expressed as 0-4095 DAC output values
const int FORWARD_SPEED_LEFT = 1200;
const int FORWARD_SPEED_RIGHT = 1200;


// Uart command buffer
char rxCommand = 0;
// =============================
// Global Variables
// =============================
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
float currentYaw = 0.0;
bool isYawCalibrated = false;
float yawOffset = 0.0;

// Instantiate DAC on the secondary I2C bus (Wire1)
DFRobot_GP8403 dac(&Wire1, 0x58);

enum SystemState
{
    STOPPED,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT
};
SystemState driveState = STOPPED;

int motorOutputLeft = 0;
int motorOutputRight = 0;

volatile long motor1_ticks = 0;
volatile long motor2_ticks = 0;

volatile int lastHall1 = 0;
volatile int lastHall2 = 0;

// =============================
// Interrupt Service Routines
// =============================
void IRAM_ATTR hall1ISR() {
    int state = (digitalRead(H1A) << 2) | (digitalRead(H1B) << 1) | (digitalRead(H1C));
    if (state != lastHall1) {
        motor1_ticks++;
        lastHall1 = state;
    }
}

void IRAM_ATTR hall2ISR() {
    int state = (digitalRead(H2A) << 2) | (digitalRead(H2B) << 1) | (digitalRead(H2C));
    if (state != lastHall2) {
        motor2_ticks++;
        lastHall2 = state;
    }
}

// =============================
// Sensor Functions
// =============================
void readBNO085() {
    BNO08x_RVC_Data heading;

    if (rvc.read(&heading)) {
        if (!isYawCalibrated) {
            yawOffset = heading.yaw;
            isYawCalibrated = true;
            Serial.print("--- BNO085 YAW CALIBRATED! Offset: ");
            Serial.print(yawOffset);
            Serial.println(" ---");
        }

        float adjustedYaw = heading.yaw - yawOffset;

        if (adjustedYaw > 180.0) adjustedYaw -= 360.0;
        if (adjustedYaw < -180.0) adjustedYaw += 360.0;

        currentYaw = adjustedYaw;
    }
}

// =============================
// Motor Control Functions
// =============================
void moveForward() {
    driveState = FORWARD;

    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);

    motorOutputLeft = FORWARD_SPEED_LEFT;
    motorOutputRight = FORWARD_SPEED_RIGHT;
}

void turnLeft() {
    driveState = TURN_LEFT;

    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);

    motorOutputLeft = 0;
    motorOutputRight = FORWARD_SPEED_RIGHT;
}

void turnRight() {
    driveState = TURN_RIGHT;

    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);

    motorOutputLeft = FORWARD_SPEED_LEFT;
    motorOutputRight = 0;
}

void stopMotors() {
    driveState = STOPPED;

    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);

    motorOutputLeft = 0;
    motorOutputRight = 0;
}

void applyMotorOutput() {
    dac.setDACOutVoltage(motorOutputLeft, 0);
    dac.setDACOutVoltage(motorOutputRight, 1);
}

// =============================
// Serial Functions
// =============================
void sendOdometry() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 20) {
        lastPrint = millis();

        Serial.print("E:");
        Serial.print(motor1_ticks);
        Serial.print(",");
        Serial.print(motor2_ticks);
        Serial.print(",");
        Serial.print(currentYaw);
        Serial.print(",");
        Serial.println(driveState);
    }
}


void processUART()
{
    while (Serial.available())
    {
        rxCommand = Serial.read();

        switch (rxCommand)
        {
            case 'F':
                moveForward();
                applyMotorOutput();
                break;

            case 'L':
                turnLeft();
                applyMotorOutput();
                break;

            case 'R':
                turnRight();
                applyMotorOutput();
                break;

            case 'S':
                stopMotors();
                applyMotorOutput();
                break;

            default:
                // Ignore unknown characters
                break;
        }
    }
}
// =============================
// setup()
// =============================
void setup() {
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, BNO_RX_PIN, BNO_TX_PIN);
    if (!rvc.begin(&Serial1)) {
        Serial.println("WARNING: BNO085 not found! Check wiring and P0 pin.");
    } else {
        Serial.println("BNO085 Initialized successfully!");
    }

    Wire1.begin(DAC_SDA, DAC_SCL);
    while (dac.begin() != 0) {
        Serial.println("DAC initialization failed!");
        delay(1000);
    }
    dac.setDACOutRange(dac.eOutputRange10V);

    pinMode(H1A, INPUT_PULLUP);
    pinMode(H1B, INPUT_PULLUP);
    pinMode(H1C, INPUT_PULLUP);
    pinMode(H2A, INPUT_PULLUP);
    pinMode(H2B, INPUT_PULLUP);
    pinMode(H2C, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(H1A), hall1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(H1B), hall1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(H1C), hall1ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(H2A), hall2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(H2B), hall2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(H2C), hall2ISR, CHANGE);

    pinMode(DIR_M1, OUTPUT);
    pinMode(DIR_M2, OUTPUT);
    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);

    stopMotors();
    applyMotorOutput();
}

// =============================
// loop()
// =============================
void loop()
{
    readBNO085();

    
    
    processUART();
    sendOdometry();
}
}

