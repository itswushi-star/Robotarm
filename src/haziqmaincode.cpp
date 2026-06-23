#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)

// User-adjustable movement speed, hard-limited to 60% for safety.
constexpr int MAX_MOVEMENT_SPEED_PERCENT = 60;
int movementSpeedPercent = 30;

// Struct to handle calibration data
struct CalibJoint {
    const char* name;
    uint8_t channel;
    int currentUs;
    int minUs;
    int maxUs;
    int maxDegrees;
};

// INITIAL STARTUP VALUES (Safe middle ground: ~90 degrees for standard servos)
// We use 500us as a baseline start for all joints so nothing clips on boot.
CalibJoint joints[] = {
    {"Base",        5, 1520, 640, 2600, 270},
    {"Shoulder",    4, 1830, 840, 1990, 110},
    {"Elbow",       3, 2380, 750, 2600, 155},
    {"Wrist Pitch", 1, 1460, 400, 1850, 140},
    {"Wrist Roll",  2, 500,  500, 2400, 180},
    {"Gripper",     0, 1470, 830, 1500, 60}
};
const int NUM_JOINTS = 6;
int activeJointIndex = 0; // Starts tracking the Base
bool servoOutputEnabled = true;

constexpr int MAX_ROUTINES = 6;
constexpr int MAX_ROUTINE_STEPS = 30;
struct SavedRoutine {
    char name[21];
    uint8_t stepCount;
    int16_t poses[MAX_ROUTINE_STEPS][NUM_JOINTS];
};
SavedRoutine routines[MAX_ROUTINES] = {};
bool recording = false;
SavedRoutine recordingBuffer = {};
bool routineRunning = false;
int runningRoutine = -1;
int runningStep = 0;
unsigned long routineHoldStarted = 0;
unsigned long routineLastMove = 0;
bool routinePoseReached = false;
// Pointer for ad-hoc playback of the in-memory recording buffer (not saved to flash)
SavedRoutine* adhocRoutine = nullptr;

// Pick-and-place demonstration poses, ordered:
// Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper.
// Adjust these pulse widths with the calibration controls before using a load.
// Standing/home position: Base, Shoulder, Elbow, Wrist Pitch,
// Wrist Roll, Gripper.
const int HOME_POSE[NUM_JOINTS]  = {1500, 1850, 1480, 1500, 1460, 900};
const int PICK_UP[NUM_JOINTS]    = {1540, 1850, 2300, 1520, 1460, 830};
const int PICK_DOWN[NUM_JOINTS]  = {1540, 1850, 2400, 960, 1460, 830};
const int GRAB_POSE[NUM_JOINTS]  = {1540, 1850, 2400, 960, 1460, 1150};
const int LIFT_POSE[NUM_JOINTS]  = {1540, 1850, 2400, 1480, 1460, 1150};
const int PLACE_UP[NUM_JOINTS]   = {640, 1850, 2400, 1200, 1460, 1150};
const int PLACE_DOWN[NUM_JOINTS] = {640, 1850, 2400, 1200, 1460, 1150};
const int RELEASE_POSE[NUM_JOINTS] =
                                  {640, 1850, 2400, 1200, 1460, 1110};

const int* const PICK_PLACE_POSES[] = {
    PICK_UP, PICK_DOWN, GRAB_POSE, LIFT_POSE,
    PLACE_UP, PLACE_DOWN, RELEASE_POSE, HOME_POSE
};
const unsigned long POSE_HOLD_MS[] = {
    1000, 1200, 800, 1200, 1200, 1200, 800, 1500
};
const unsigned long SERVO_STEP_INTERVAL_MS = 20;
const int NUM_PICK_PLACE_POSES =
    sizeof(PICK_PLACE_POSES) / sizeof(PICK_PLACE_POSES[0]);

bool pickPlaceRunning = false;
int pickPlaceStage = 0;
unsigned long poseStartedAt = 0;
unsigned long lastServoStepAt = 0;
bool poseReached = false;

void printStatus();
void printActiveJoint();
void saveCurrentRecording();

void setServoPulseUs(uint8_t channel, int us) {
    // Formula: (us / 20000.0) * 4096
    double pulseLength = (us / 20000.0) * 4096.0;
    if (servoOutputEnabled) {
        pwm.setPWM(channel, 0, (uint16_t)pulseLength);
    }
}

int clampJointPulse(int jointIndex, int us) {
    return constrain(us, joints[jointIndex].minUs, joints[jointIndex].maxUs);
}

int pulseToDegrees(int jointIndex, int us) {
    const CalibJoint& joint = joints[jointIndex];
    const long pulseRange = joint.maxUs - joint.minUs;
    const long offset = clampJointPulse(jointIndex, us) - joint.minUs;
    const int degrees =
        (offset * joint.maxDegrees + pulseRange / 2) / pulseRange;
    // The elbow is installed in the opposite direction to the other joints.
    return jointIndex == 2 ? joint.maxDegrees - degrees : degrees;
}

int degreesToPulse(int jointIndex, int degrees) {
    const CalibJoint& joint = joints[jointIndex];
    degrees = constrain(degrees, 0, joint.maxDegrees);
    if (jointIndex == 2) {
        degrees = joint.maxDegrees - degrees;
    }
    const long pulseRange = joint.maxUs - joint.minUs;
    return joint.minUs +
           (degrees * pulseRange + joint.maxDegrees / 2) / joint.maxDegrees;
}

String currentDegreesJson() {
    String json = "{\"degrees\":[";
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i > 0) {
            json += ",";
        }
        json += String(pulseToDegrees(i, joints[i].currentUs));
    }
    json += "]}";
    return json;
}

void moveJointSlowly(int jointIndex, int targetUs) {
    targetUs = clampJointPulse(jointIndex, targetUs);
    int currentUs = joints[jointIndex].currentUs;
    movementSpeedPercent =
        constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
    // Speed controls both step size and delay while preserving smooth motion.
    int stepSize = map(movementSpeedPercent, 1, 100, 1, 12);
    int stepDelayMs = map(movementSpeedPercent, 1, 100, 25, 1);
    int direction = targetUs >= currentUs ? stepSize : -stepSize;

    while (currentUs != targetUs) {
        currentUs += direction;

        if ((direction > 0 && currentUs > targetUs) ||
            (direction < 0 && currentUs < targetUs)) {
            currentUs = targetUs;
        }

        joints[jointIndex].currentUs = currentUs;
        setServoPulseUs(joints[jointIndex].channel, currentUs);
        delay(stepDelayMs);
        yield();
    }
}

void returnHomeOneJointAtATime() {
    // Move one complete joint at a time in a clearance-first order.
    const int homingOrder[NUM_JOINTS] = {5, 3, 4, 1, 2, 0};
    servoOutputEnabled = true;

    for (int i = 0; i < NUM_JOINTS; i++) {
        const int joint = homingOrder[i];
        Serial.printf("Homing %s (%d/%d)\n",
                      joints[joint].name, i + 1, NUM_JOINTS);
        moveJointSlowly(joint, HOME_POSE[joint]);
        delay(250);
    }
}

void performStartupHoming() {
    constexpr unsigned long HOMING_STAGE_SETTLE_MS = 500;

    // Homing is deliberately blocking. The web server and serial command
    // handling are not started until the arm has reached its safe coordinate.
    servoOutputEnabled = true;
    Serial.println("Starting staged software homing...");

    // Stage 1: create clearance by opening the gripper and retracting the wrist.
    Serial.println("Homing stage 1/3: gripper and wrist clearance");
    moveJointSlowly(5, 1200);             // Gripper open/release
    moveJointSlowly(3, HOME_POSE[3]);     // Wrist pitch
    moveJointSlowly(4, HOME_POSE[4]);     // Wrist roll
    delay(HOMING_STAGE_SETTLE_MS);

    // Stage 2: elevate the arm before allowing the base to rotate.
    Serial.println("Homing stage 2/3: shoulder and elbow elevation");
    moveJointSlowly(1, HOME_POSE[1]);     // Shoulder
    moveJointSlowly(2, HOME_POSE[2]);     // Elbow
    delay(HOMING_STAGE_SETTLE_MS);

    // Stage 3: center the base only after the arm has sufficient clearance.
    Serial.println("Homing stage 3/3: base alignment");
    moveJointSlowly(0, HOME_POSE[0]);     // Base
    delay(HOMING_STAGE_SETTLE_MS);

    Serial.println("Staged software homing complete.");
}

void setAllOutputs(bool enabled) {
    servoOutputEnabled = enabled;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (enabled) {
            setServoPulseUs(joints[i].channel, joints[i].currentUs);
        } else {
            pwm.setPWM(joints[i].channel, 0, 0);
        }
    }
}

void saveRoutines() {
    preferences.putBytes("routines", routines, sizeof(routines));
}

void captureRoutinePose() {
    if (!recording || recordingBuffer.stepCount >= MAX_ROUTINE_STEPS) {
        return;
    }

    if (recordingBuffer.stepCount > 0) {
        const int lastStep = recordingBuffer.stepCount - 1;
        bool unchanged = true;
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            if (recordingBuffer.poses[lastStep][joint] !=
                joints[joint].currentUs) {
                unchanged = false;
                break;
            }
        }
        if (unchanged) {
            return;
        }
    }

    int step = recordingBuffer.stepCount;
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        recordingBuffer.poses[step][joint] = joints[joint].currentUs;
    }
    recordingBuffer.stepCount++;
}

void saveCurrentRecording() {
    if (!recording || recordingBuffer.stepCount == 0) {
        Serial.println("No recorded poses to save. Press 5 to capture at least one pose.");
        return;
    }

    int slot = -1;
    for (int i = 0; i < MAX_ROUTINES; i++) {
        if (routines[i].stepCount == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        Serial.println("No free routine slots available. Delete a task first.");
        return;
    }

    snprintf(recordingBuffer.name, sizeof(recordingBuffer.name), "Task %d", slot + 1);
    routines[slot] = recordingBuffer;
    recording = false;
    saveRoutines();
    Serial.printf("Saved recording to slot %d: %s\n", slot + 1, routines[slot].name);
}

void deleteFirstSavedRoutine() {
    for (int i = 0; i < MAX_ROUTINES; i++) {
        if (routines[i].stepCount > 0) {
            memset(&routines[i], 0, sizeof(SavedRoutine));
            saveRoutines();
            Serial.printf("Deleted saved task slot %d and freed the routine slot.\n", i + 1);
            printStatus();
            return;
        }
    }
    Serial.println("No saved tasks to delete.");
}

void updateRoutinePlayback() {
    if (!routineRunning || (runningRoutine < 0 && adhocRoutine == nullptr)) {
        return;
    }

    SavedRoutine& routine = (runningRoutine >= 0) ? routines[runningRoutine] : *adhocRoutine;
    unsigned long now = millis();
    if (!routinePoseReached) {
        if (now - routineLastMove < 10) {
            return;
        }
        routineLastMove = now;

        bool reached = true;
        const int safeSpeed =
            constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
        int stepSize = map(safeSpeed, 1, 100, 1, 12);
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int safeTarget =
                clampJointPulse(joint, routine.poses[runningStep][joint]);
            int difference = safeTarget - joints[joint].currentUs;
            if (difference != 0) {
                joints[joint].currentUs += constrain(difference, -stepSize, stepSize);
                setServoPulseUs(joints[joint].channel, joints[joint].currentUs);
                reached = false;
            }
        }
        if (reached) {
            routinePoseReached = true;
            routineHoldStarted = now;
        }
        return;
    }

    if (now - routineHoldStarted < 500) {
        return;
    }

    runningStep++;
    if (runningStep >= routine.stepCount) {
        routineRunning = false;
        if (runningRoutine >= 0) {
            runningRoutine = -1;
        } else {
            adhocRoutine = nullptr;
        }
        runningStep = 0;
        Serial.println("Saved task complete; returning home.");
        returnHomeOneJointAtATime();
        Serial.println("Arm returned home; all playback stopped.");
        return;
    }
    routinePoseReached = false;
}

void moveToPose(const int pose[]) {
    if (pose == HOME_POSE) {
        returnHomeOneJointAtATime();
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        moveJointSlowly(i, pose[i]);
    }
}

void startPickAndPlace() {
    pickPlaceRunning = true;
    pickPlaceStage = 0;
    poseReached = false;
    lastServoStepAt = millis();
    Serial.println("\nPick-and-place demonstration started.");
    Serial.println("\n====State Busy====.");
}

bool startFirstSavedRoutine() {
    for (int id = 0; id < MAX_ROUTINES; id++) {
        if (routines[id].stepCount == 0) {
            continue;
        }

        recording = false;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        runningRoutine = id;
        runningStep = 0;
        routinePoseReached = false;
        routineLastMove = millis();
        routineRunning = true;
        Serial.printf("Automatically playing saved task: %s\n",
                      routines[id].name);
        return true;
    }

    Serial.println("No saved task available for automatic playback.");
    return false;
}

void updatePickAndPlace() {
    if (!pickPlaceRunning) {
        return;
    }

    const unsigned long now = millis();

    if (!poseReached) {
        if (now - lastServoStepAt < SERVO_STEP_INTERVAL_MS) {
            return;
        }
        lastServoStepAt = now;

        bool allAtTarget = true;
        const int safeSpeed =
            constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
        const int servoStepUs = map(safeSpeed, 1, 100, 1, 12);
        const int* target = PICK_PLACE_POSES[pickPlaceStage];
        for (int i = 0; i < NUM_JOINTS; i++) {
            const int safeTarget = clampJointPulse(i, target[i]);
            const int difference = safeTarget - joints[i].currentUs;
            if (difference != 0) {
                const int step = constrain(
                    difference, -servoStepUs, servoStepUs);
                joints[i].currentUs += step;
                setServoPulseUs(joints[i].channel, joints[i].currentUs);
                allAtTarget = false;
            }
        }

        if (allAtTarget) {
            poseReached = true;
            poseStartedAt = now;
        }
        return;
    }

    if (now - poseStartedAt < POSE_HOLD_MS[pickPlaceStage]) {
        return;
    }

    pickPlaceStage++;
    if (pickPlaceStage >= NUM_PICK_PLACE_POSES) {
        pickPlaceRunning = false;
        Serial.println("Pick-and-place demonstration complete. Returning to homing mode...");
        // Return to home safely, one joint at a time.
        returnHomeOneJointAtATime();
        Serial.println("====State Idle====.");
        printStatus();
        return;
    }

    poseReached = false;
    lastServoStepAt = now;
}

void printStatus() {
    int savedCount = 0;
    for (int i = 0; i < MAX_ROUTINES; i++) {
        if (routines[i].stepCount > 0) {
            savedCount++;
        }
    }

    Serial.println("\n----------------------------------------------------");
    Serial.printf("ACTIVE JOINT: [%d] %s (PCA9685 Ch: %d)\n",
                  activeJointIndex, joints[activeJointIndex].name, joints[activeJointIndex].channel);
    Serial.printf("Current Pulse Width: %d us\n", joints[activeJointIndex].currentUs);
    Serial.printf("Recording: %s | Steps: %d\n",
                  recording ? "ON" : "OFF", recordingBuffer.stepCount);
    Serial.printf("Saved Tasks: %d / %d\n", savedCount, MAX_ROUTINES);
    Serial.println("----------------------------------------------------");
    Serial.println("NUMPAD CONTROLS:");
    Serial.println("  [8] Increase current joint (+20us)");
    Serial.println("  [2] Decrease current joint (-20us)");
    Serial.println("  [4] Previous joint");
    Serial.println("  [6] Next joint");
    Serial.println("  [5] Capture current pose (starts recording if needed)");
    Serial.println("  [3] Save recording to next free slot");
    Serial.println("  [D] Delete first saved task slot");
    Serial.println("  [1] Return home");
    Serial.println("  [7] Play first saved task");
    Serial.println("  [0] Toggle servo output");
    Serial.println("  [R] Print all joint angles (current pose)");
    Serial.println("  [P] Start pick-and-place demo");
    Serial.println("  [Q] Stop demo and return home");
    Serial.println("----------------------------------------------------");
}

void printActiveJoint() {
    Serial.printf("current joint : %s (%d)\n", joints[activeJointIndex].name, activeJointIndex);
}

void printAllJointAngles() {
    Serial.println("\n========== CURRENT POSE ==========");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.printf("  [%d] %-15s : %5d us\n", i, joints[i].name, joints[i].currentUs);
    }
    Serial.println("=================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    preferences.begin("robot-arm", false);
    if (preferences.getBytesLength("routines") == sizeof(routines)) {
        preferences.getBytes("routines", routines, sizeof(routines));
        bool repairedRoutines = false;
        for (int i = 0; i < MAX_ROUTINES; i++) {
            if (routines[i].stepCount > MAX_ROUTINE_STEPS) {
                memset(&routines[i], 0, sizeof(SavedRoutine));
                repairedRoutines = true;
            } else {
                routines[i].name[20] = '\0';
            }
        }
        if (repairedRoutines) {
            saveRoutines();
        }
    }
    
    Wire.begin(21, 22); // ESP32 Pins: SDA=21, SCL=22
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    // Complete the blocking clearance/elevation/alignment sequence before
    // starting any interface that can accept commands.
    performStartupHoming();

    printStatus();
}


void loop() {
    updatePickAndPlace();
    updateRoutinePlayback();

    if (Serial.available() > 0) {
        char c = Serial.read();
        int step = 0;

        if (c == '\r' || c == '\n') {
            return;
        }

        if (c == 'p' || c == 'P') {
            // If we have an in-memory recording buffer, play it as an ad-hoc routine.
            if (recordingBuffer.stepCount > 0) {
                recording = false;
                pickPlaceRunning = false;
                servoOutputEnabled = true;
                adhocRoutine = &recordingBuffer;
                runningRoutine = -2; // sentinel for adhoc (we also check adhocRoutine ptr)
                runningStep = 0;
                routinePoseReached = false;
                routineLastMove = millis();
                routineRunning = true;
                Serial.println("Playing ad-hoc recorded movement.");
                return;
            }
            // Fallback to predefined pick-and-place demo
            startPickAndPlace();
            return;
        }

        if (c == 'q' || c == 'Q') {
            pickPlaceRunning = false;
            moveToPose(HOME_POSE);
            Serial.println("Demonstration stopped; arm returned home.");
            return;
        }

        if (pickPlaceRunning) {
            // Allow manual override and useful diagnostics while the demo is running.
            if (c == 'r' || c == 'R') {
                printAllJointAngles();
                return;
            }
            pickPlaceRunning = false;
            Serial.println("Pick-and-place demo aborted by manual input.");
        }

        switch (c) {
            case '8':
                step = 20;
                break;
            case '2':
                step = -20;
                break;
            case '4':
                activeJointIndex = (activeJointIndex + NUM_JOINTS - 1) % NUM_JOINTS;
                printStatus();
                printActiveJoint();
                return;
            case '6':
                activeJointIndex = (activeJointIndex + 1) % NUM_JOINTS;
                printStatus();
                printActiveJoint();
                return;
            case '5':
                if (!recording) {
                    recording = true;
                    memset(&recordingBuffer, 0, sizeof(recordingBuffer));
                    Serial.println("Recording started.");
                }
                {
                    uint8_t previousCount = recordingBuffer.stepCount;
                    captureRoutinePose();
                    if (recordingBuffer.stepCount == previousCount) {
                        Serial.println("Pose unchanged or duplicate; not recorded.");
                    } else {
                        Serial.printf("Pose recorded as step %d.\n", recordingBuffer.stepCount);
                    }
                }
                return;
            case '3':
                saveCurrentRecording();
                return;
            case 'd':
            case 'D':
                deleteFirstSavedRoutine();
                return;
            case '1':
                routineRunning = false;
                pickPlaceRunning = false;
                moveToPose(HOME_POSE);
                Serial.println("Arm returned home.");
                return;
            case '7':
                if (!startFirstSavedRoutine()) {
                    Serial.println("No saved task available.");
                }
                return;
            case '0':
                setAllOutputs(!servoOutputEnabled);
                Serial.printf("Servo output %s\n", servoOutputEnabled ? "enabled" : "disabled");
                return;
            case 'r':
            case 'R':
                printAllJointAngles();
                return;
            default:
                return;
        }

        int targetUs = joints[activeJointIndex].currentUs + step;
        targetUs = clampJointPulse(activeJointIndex, targetUs);
        moveJointSlowly(activeJointIndex, targetUs);
        Serial.printf("Joint %s -> %d us\n", joints[activeJointIndex].name,
                      joints[activeJointIndex].currentUs);
    }
}
