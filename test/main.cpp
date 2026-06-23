//Calibartion code for determini robot angle. 
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)
#define NUM_JOINTS 6
#define MAX_STEPS 50  // Maximum number of recorded poses

// Smooth Movement Settings
const int MOVEMENT_DURATION_MS = 500; // Time taken to travel between macro poses (in milliseconds)
const int REFRESH_RATE_MS = 20;        // Step update interval (20ms matches 50Hz frequency)

// Struct to handle calibration and dynamic safety limits
struct CalibJoint {
    const char* name;
    uint8_t channel;
    int currentUs;
    int minUs;
    int maxUs;
    int maxDegree;
};

// INITIAL STARTUP VALUES & STRICT PHYSICAL BOUNDARIES
CalibJoint joints[NUM_JOINTS] = {
    {"Base",        5, 1500, 640,  2600, 270},
    {"Shoulder",    4, 1850, 840,  1990, 110},
    {"Elbow",       3, 1480, 750,  2600, 155},
    {"Wrist Pitch", 1, 1500, 400,  1890, 140},
    {"Wrist Roll",  2, 1460,  500,  2600, 180},
    {"Gripper",     0, 900, 900,  1500, 60}
};

int activeJointIndex = 0; 
const int stepSizeUs = 20; // Fixed speed step per keypad entry

// Recording Memory Matrix
int recordedPositions[MAX_STEPS][NUM_JOINTS];
int totalRecordedSteps = 0;

void setServoPulseUs(uint8_t channel, int us) {
    // Formula: (us / 20000.0) * 4096
    double pulseLength = (us / 20000.0) * 4096.0;
    pwm.setPWM(channel, 0, (uint16_t)pulseLength);
}

// Helper to convert current microseconds to real degrees based on limits
float getCalculatedDegree(int index) {
    float rangeUs = joints[index].maxUs - joints[index].minUs;
    float currentOffset = joints[index].currentUs - joints[index].minUs;
    return (currentOffset / rangeUs) * joints[index].maxDegree;
}

void printStatus() {
    Serial.println("\n----------------------------------------------------");
    Serial.printf("ACTIVE JOINT: [%d] %s (PCA9685 Ch: %d)\n", 
                  activeJointIndex, joints[activeJointIndex].name, joints[activeJointIndex].channel);
    Serial.printf("Current Pulse Width: %d us (~%.1f°)\n", 
                  joints[activeJointIndex].currentUs, getCalculatedDegree(activeJointIndex));
    Serial.printf("Limits: [%d us to %d us]\n", joints[activeJointIndex].minUs, joints[activeJointIndex].maxUs);
    Serial.println("----------------------------------------------------");
    Serial.printf("Recorded Steps: %d / %d\n", totalRecordedSteps, MAX_STEPS);
    Serial.println("----------------------------------------------------");
    Serial.println("NUMPAD CONTROLS:");
    Serial.println("  [8] Increase (+20us)   |  [2] Decrease (-20us)");
    Serial.println("  [4] Previous Joint     |  [6] Next Joint");
    Serial.println("  [5] Record Current Pose|  [7] Playback Saved Routine");
    Serial.println("----------------------------------------------------");
}

// Function to move all joints smoothly from their current positions to target positions
void smoothMoveAll(int targets[NUM_JOINTS]) {
    int startUs[NUM_JOINTS];
    
    // Cache the starting positions
    for (int j = 0; j < NUM_JOINTS; j++) {
        startUs[j] = joints[j].currentUs;
    }

    int totalSteps = MOVEMENT_DURATION_MS / REFRESH_RATE_MS;

    // Linearly interpolate (LERP) positions over time
    for (int step = 1; step <= totalSteps; step++) {
        float fraction = (float)step / (float)totalSteps;

        for (int j = 0; j < NUM_JOINTS; j++) {
            // Calculate intermediate pulse width
            int currentTarget = startUs[j] + (int)((targets[j] - startUs[j]) * fraction);
            
            // Apply constraints and update PCA9685
            joints[j].currentUs = constrain(currentTarget, joints[j].minUs, joints[j].maxUs);
            setServoPulseUs(joints[j].channel, joints[j].currentUs);
        }
        
        delay(REFRESH_RATE_MS); // Micro-pause to establish the visual "speed"
    }
}

void playRecording() {
    if (totalRecordedSteps == 0) {
        Serial.println(">>> ERROR: No steps recorded yet! <<<");
        return;
    }
    
    Serial.printf("\n>>> PLAYING BACK %d RECORDED STEPS SMOOTHLY <<<\n", totalRecordedSteps);
    for (int step = 0; step < totalRecordedSteps; step++) {
        Serial.printf("Executing Step %d/%d...\n", step + 1, totalRecordedSteps);
        
        // Pass the target row array to our smooth transition driver
        smoothMoveAll(recordedPositions[step]);
        
        delay(500); // Brief pause delay at the achieved macro step before moving to the next
    }
    Serial.println(">>> PLAYBACK COMPLETE <<<\n");
    printStatus();
}

void recordCurrentPose() {
    if (totalRecordedSteps >= MAX_STEPS) {
        Serial.println(">>> ERROR: Recording Memory Full! <<<");
        return;
    }
    
    // Save state of all 6 joints into the matrix row
    for (int j = 0; j < NUM_JOINTS; j++) {
        recordedPositions[totalRecordedSteps][j] = joints[j].currentUs;
    }
    
    Serial.printf("\n>>> SUCCESS: Saved Pose Step [%d] <<<\n", totalRecordedSteps + 1);
    Serial.println("=============================================");
    Serial.println("   JOINT NAME   |  PULSE WIDTH  |   ANGLE    ");
    Serial.println("---------------------------------------------");
    for (int j = 0; j < NUM_JOINTS; j++) {
        Serial.printf(" %-14s |   %4d us    |   %5.1f°   \n", 
                      joints[j].name, 
                      joints[j].currentUs, 
                      getCalculatedDegree(j));
    }
    Serial.println("=============================================");
    
    totalRecordedSteps++;
    printStatus();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin(21, 22); // ESP32 Pins: SDA=21, SCL=22
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    // Initial setup sanitization logic: Enforce custom boundaries right away
    Serial.println("Initializing all joints to bounded positions...");
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].currentUs = constrain(joints[i].currentUs, joints[i].minUs, joints[i].maxUs);
        setServoPulseUs(joints[i].channel, joints[i].currentUs);
    }
    
    delay(1500);
    printStatus();
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        int stepModifier = 0;
        
        switch (c) {
            case '8': 
                stepModifier = stepSizeUs; 
                break;
            case '2': 
                stepModifier = -stepSizeUs; 
                break;
            case '4': 
                activeJointIndex--;
                if (activeJointIndex < 0) activeJointIndex = NUM_JOINTS - 1;
                printStatus();
                return;
            case '6': 
                activeJointIndex++;
                if (activeJointIndex >= NUM_JOINTS) activeJointIndex = 0;
                printStatus();
                return;
            case '5':
                recordCurrentPose();
                return;
            case '7':
                playRecording();
                return;
            default: 
                return; 
        }

        // Apply safely calculated changes bound specifically to the current targeted active structural component 
        int targetUs = joints[activeJointIndex].currentUs + stepModifier;
        joints[activeJointIndex].currentUs = constrain(targetUs, joints[activeJointIndex].minUs, joints[activeJointIndex].maxUs);

        // Instant update payload send
        setServoPulseUs(joints[activeJointIndex].channel, joints[activeJointIndex].currentUs);
        
        // Live serial feedback printouts
        Serial.printf("Joint %s -> %d us (~%.1f°)\n", 
                      joints[activeJointIndex].name, 
                      joints[activeJointIndex].currentUs, 
                      getCalculatedDegree(activeJointIndex));
    }
}