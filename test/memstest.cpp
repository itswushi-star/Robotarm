#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)
#define NUM_JOINTS 6
#define TOTAL_POSES 6 // Hardcoded sequence length

// Smooth Movement Settings
const int MOVEMENT_DURATION_MS = 1000; // Time taken to travel between poses (in milliseconds)
const int REFRESH_RATE_MS = 20;        // Step update interval (20ms matches 50Hz frequency)

// Structure for managing structural metadata and pin channels
struct JointConfig {
    const char* name;
    uint8_t channel;
    int currentUs; // Tracks live positions globally for smooth interpolation
    int minUs;
    int maxUs;
};

// INITIAL STARTUP STATE & SAFETY LIMITS
JointConfig joints[NUM_JOINTS] = {
    {"Base",        5, 1520, 640,  2600},
    {"Shoulder",    4, 1830, 840,  1990},
    {"Elbow",       3, 2380, 750,  2600},
    {"Wrist Pitch", 1, 1460, 400,  1890},
    {"Wrist Roll",  2, 500,  500,  2600},
    {"Gripper",     0, 1470, 900,  1500}
};

// HARDCODED POSES MATRIX (Extracted directly from your calibrated saves)
// Order matching: {Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper}
const int AUTOMATED_ROUTINE[TOTAL_POSES][NUM_JOINTS] = {
    {1520, 1810, 2380, 1440, 1440, 1470}, // Pose State 1
    {1580, 1330, 2120, 1350, 1520, 1470}, // Pose State 2
    {1580, 1330, 2120, 1350, 1520, 900 }, // Pose State 3
    {1580, 1330, 2120, 1350, 1520, 1500}, // Pose State 4
    {1580, 1370, 1900, 1510, 1480, 1500}, // Pose State 5
    {1240, 1570, 1880, 1310, 1500, 1460}  // Pose State 6
};

void setServoPulseUs(uint8_t channel, int us) {
    double pulseLength = (us / 20000.0) * 4096.0;
    pwm.setPWM(channel, 0, (uint16_t)pulseLength);
}

// Fluid transition controller using Linear Interpolation (LERP)
void smoothMoveToTarget(const int targets[NUM_JOINTS]) {
    int startUs[NUM_JOINTS];
    
    // Cache the exact position the arm is sitting at right now
    for (int j = 0; j < NUM_JOINTS; j++) {
        startUs[j] = joints[j].currentUs;
    }

    int totalSteps = MOVEMENT_DURATION_MS / REFRESH_RATE_MS;

    for (int step = 1; step <= totalSteps; step++) {
        float fraction = (float)step / (float)totalSteps;

        for (int j = 0; j < NUM_JOINTS; j++) {
            // Calculate step-by-step intermediate position path
            int currentTarget = startUs[j] + (int)((targets[j] - startUs[j]) * fraction);
            
            // Constrain tightly to hardware bounds and update driver registers
            joints[j].currentUs = constrain(currentTarget, joints[j].minUs, joints[j].maxUs);
            setServoPulseUs(joints[j].channel, joints[j].currentUs);
        }
        delay(REFRESH_RATE_MS);
    }
}

void executeMacroRoutine() {
    Serial.println("\n>>> STARTING AUTOMATED SEQUENCE (POSES 1 TO 6) <<<");
    
    for (int poseIdx = 0; poseIdx < TOTAL_POSES; poseIdx++) {
        Serial.printf("Executing Pose State [%d/%d]...\n", poseIdx + 1, TOTAL_POSES);
        
        // Command all joints to move smoothly to the next row targets
        smoothMoveToTarget(AUTOMATED_ROUTINE[poseIdx]);
        
        // Hold position for 800ms before starting the next transition
        delay(800); 
    }
    
    Serial.println(">>> AUTOMATED SEQUENCE COMPLETE <<<\n");
    Serial.println("Waiting for input... Press [1] to run again.");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin(21, 22); // ESP32 Pins: SDA=21, SCL=22
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    Serial.println("Initializing robotic arm to safe default startup posture...");
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].currentUs = constrain(joints[i].currentUs, joints[i].minUs, joints[i].maxUs);
        setServoPulseUs(joints[i].channel, joints[i].currentUs);
    }
    
    delay(1500);
    Serial.println("\n=============================================");
    Serial.println(" SYSTEM READY: Press [1] to trigger routine. ");
    Serial.println("=============================================");
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '1') {
            executeMacroRoutine();
        }
    }
}