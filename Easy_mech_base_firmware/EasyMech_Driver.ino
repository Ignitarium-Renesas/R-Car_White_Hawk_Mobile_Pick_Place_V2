#define USE_IMU 1    // <--- Set to 1 to enable IMU, 0 to fully exclude all IMU code!

#include <Arduino.h>
#include <util/atomic.h>
#include <Wire.h>

#if USE_IMU
#include <ICM_20948.h>   // SparkFun IMU library
#include <Thread.h>
#endif

//-----------------------------------------//
// PIN DEFINITIONS //
//-----------------------------------------//
// Back Left Motor
#define BL_F 9                              // Back Left Forward pin
#define BL_B 8                              // Back Left Backward pin
#define HALL_SENSOR_BL_A_PIN 18             // Back Left Hall Sensor A pin
#define HALL_SENSOR_BL_B_PIN 23             // Back Left Hall Sensor B pin

// Back Right Motor
#define BR_F 7                              // Back Right Forward pin
#define BR_B 6                              // Back Right Backward pin
#define HALL_SENSOR_BR_A_PIN 19             // Back Right Hall Sensor A pin
#define HALL_SENSOR_BR_B_PIN 25             // Back Right Hall Sensor B pin

// Front Left Motor
#define FL_F 11                             // Front Left Forward pin
#define FL_B 10                             // Front Left Backward pin
#define HALL_SENSOR_FL_A_PIN 3             // Front Left Hall Sensor A pin
#define HALL_SENSOR_FL_B_PIN 29            // Front Left Hall Sensor B pin

// Front Right Motor
#define FR_F 5                              // Front Right Forward pin
#define FR_B 4                              // Front Right Backward pin
#define HALL_SENSOR_FR_A_PIN 2             // Front Right Hall Sensor A pin
#define HALL_SENSOR_FR_B_PIN 27            // Front Right Hall Sensor B pin 

// System Pins
#define LED_PIN 13
#define ADC_PIN A0

//-----------------------------------------//
// MOTOR CONSTANTS //
//-----------------------------------------//
#define PULSES_PER_ROTATION 306.0
#define INTERVAL 10000                  // 10ms control loop(100fps)
#define MAX_PWM 100
#define TICK_AVG_WINDOW 10

// MAX_TICK_TIME: Maximum interval (in microseconds) allowed between hall sensor ticks
// If no tick is detected within this time, the motor is considered stopped.
// Used with micros(): checks how long it's been since the last sensor event.
// 1 second = 1,000,000 microseconds, so 500,000UL = 0.5s (half a second)

#define MAX_TICK_TIME 500000UL         // 0.5 second timeout

#define TARGET_SPEED_MAX 60           // Maximum RPM limit
#define MAX_RPM_VALIDATION 300.0      // Maximum RPM validation limit - saturate instead of zero

//-----------------------------------------//
// GLOBAL VARIABLES - BACK LEFT MOTOR //
//-----------------------------------------//
volatile unsigned long lastTickTime_BL = 0;      
volatile unsigned long prevTickTime_BL = 0;
volatile int totalTicks_BL = 0;
volatile int direction_BL = 1;
long odometry_BL = 0;
float measuredSpeed_BL = 0;
float filteredRPM_BL = 0;
volatile unsigned long tickIntervals_BL[TICK_AVG_WINDOW] = {0};
volatile int tickIndex_BL = 0;
volatile int tickCount_BL = 0;
float targetRPM_BL = 0;
int currentPWM_BL = 0;
float lastPWM_BL = 0;
float targetPWM_BL = 0;
float correction = 0;


//-----------------------------------------//
// GLOBAL VARIABLES - BACK RIGHT MOTOR //
//-----------------------------------------//
volatile unsigned long lastTickTime_BR = 0;
volatile unsigned long prevTickTime_BR = 0;
volatile int totalTicks_BR = 0;
volatile int direction_BR = 1;
long odometry_BR = 0;
float measuredSpeed_BR = 0;
float filteredRPM_BR = 0;
volatile unsigned long tickIntervals_BR[TICK_AVG_WINDOW] = {0};
volatile int tickIndex_BR = 0;
volatile int tickCount_BR = 0;
float targetRPM_BR = 0;
int currentPWM_BR = 0;
float lastPWM_BR = 0;
float targetPWM_BR = 0;

//-----------------------------------------//
// GLOBAL VARIABLES - FRONT LEFT MOTOR //
//-----------------------------------------//
volatile unsigned long lastTickTime_FL = 0;
volatile unsigned long prevTickTime_FL = 0;
volatile int totalTicks_FL = 0;
volatile int direction_FL = 1;
long odometry_FL = 0;
float measuredSpeed_FL = 0;
float filteredRPM_FL = 0;
volatile unsigned long tickIntervals_FL[TICK_AVG_WINDOW] = {0};
volatile int tickIndex_FL = 0;
volatile int tickCount_FL = 0;
float targetRPM_FL = 0;
int currentPWM_FL = 0;
float lastPWM_FL = 0;
float targetPWM_FL = 0;

//-----------------------------------------//
// GLOBAL VARIABLES - FRONT RIGHT MOTOR //
//-----------------------------------------//
volatile unsigned long lastTickTime_FR = 0;
volatile unsigned long prevTickTime_FR = 0;
volatile int totalTicks_FR = 0;
volatile int direction_FR = 1;
long odometry_FR = 0;
float measuredSpeed_FR = 0;
float filteredRPM_FR = 0;
volatile unsigned long tickIntervals_FR[TICK_AVG_WINDOW] = {0};
volatile int tickIndex_FR = 0;
volatile int tickCount_FR = 0;
float targetRPM_FR = 0;
int currentPWM_FR = 0;
float lastPWM_FR = 0;
float targetPWM_FR = 0;

//-----------------------------------------//
// CONTROL PARAMETERS //
//-----------------------------------------//
const float Kp = 0.11;             // P Gain - Reduced slightly to minimize overshoot  0.10 Working
const float Ki = 0.010;            // Integral Gain - Increased to eliminate steady-state error  0.010 working
const float Kd = 0.025;            // Derivative Gain - Increased for better damping    0.025 Working

// $FR:0,FL:0,BR:0,BL:0#          // Stop all motors
// $FR:12,FL:12,BR:12,BL:12#      // All motors forward at 20 RPM

// --- PID tracking variables for each motor --- //
float integral_BL = 0.0, previousError_BL = 0.0, filteredDerivative_BL = 0.0;
float integral_BR = 0.0, previousError_BR = 0.0, filteredDerivative_BR = 0.0;
float integral_FL = 0.0, previousError_FL = 0.0, filteredDerivative_FL = 0.0;
float integral_FR = 0.0, previousError_FR = 0.0, filteredDerivative_FR = 0.0;
const float maxIntegral = 80.0;    // Further reduced anti-windup for better transient response
const float derivativeFilter = 0.15; // Increased filtering to reduce derivative kick

//-----------------------------------------//
// SYSTEM VARIABLES //
//-----------------------------------------//
float BAT_VOLT = 0.0;
String inputString = "";
bool stringComplete = false;

//-----------------------------------------//
// IMU Integration //
//-----------------------------------------//
#if USE_IMU
#define AD0_VAL 1
#define WIRE_PORT Wire
ICM_20948_I2C myICU;
Thread imuThread;
unsigned long lastImuPrint = 0;
const unsigned long imuPrintPeriod = 30; //ms

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) Serial3.print("-");
  else Serial3.print(" ");
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 1;
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) tenpow *= 10;
    if (aval < tenpow) Serial3.print("0");
    else break;
  }
  if (val < 0) Serial3.print(-val, decimals);
  else Serial3.print(val, decimals);
}

void printScaledAGMT(ICM_20948_I2C *sensor) {
  Serial3.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2); Serial3.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2); Serial3.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2); Serial3.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2); Serial3.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2); Serial3.print(" ]");
  Serial3.println();
}

//----------- IMU Task (for the thread) ---------------
void imuTask() {
  if (millis() - lastImuPrint >= imuPrintPeriod) {
    if (myICU.dataReady()) {
      myICU.getAGMT();
      printScaledAGMT(&myICU);
    }
    lastImuPrint = millis();
  }
}
#endif



//-----------------------------------------//
// RPM CALCULATION WITH FILTERING - BACK LEFT //
//-----------------------------------------//
float calculateRPM_BL(unsigned long safeLastTickTime) {
    const float alpha = 0.05;
    float avgTickInterval = 0.0;
    float rpm = 0.0;
    unsigned long timeSinceLastTick = micros() - safeLastTickTime;
    // Create local copies of volatile variables inside atomic block
    unsigned long localTickIntervals[TICK_AVG_WINDOW];
    unsigned int localTickIndex = 0;
   
    // Copy the entire array and index atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memcpy((void*)localTickIntervals, (const void*)tickIntervals_BL, sizeof(tickIntervals_BL));
        localTickIndex = tickIndex_BL;
    }

    if (timeSinceLastTick > MAX_TICK_TIME) {
        rpm = 0.0;
        filteredRPM_BL = 0.0;
    } else {
        unsigned long sum = 0;
        int validCount = 0;
        
        for (int i = 0; i < TICK_AVG_WINDOW; i++) {
            unsigned int j = (localTickIndex - i) % TICK_AVG_WINDOW;  // Wrap-around indexing: ensures j is always within positive
            if (localTickIntervals[j] > 0) {
                sum += localTickIntervals[j];
                validCount++;
            }
            if(sum >= MAX_TICK_TIME) break;
        }    
        if (validCount >= 2) {
            avgTickInterval = (float)sum / validCount;
            if (avgTickInterval > 0) {
                rpm = (1.0 / (avgTickInterval / 1000000.0)) * (60.0 / PULSES_PER_ROTATION);
                if (!isfinite(rpm)) {
                    rpm = 0.0;
                } else {
                    rpm = rpm * direction_BL;  // Apply direction sign
                    // Saturate RPM to max validation limit instead of zeroing
                    rpm = constrain(rpm, -MAX_RPM_VALIDATION, MAX_RPM_VALIDATION);
                }
            }
        }
        filteredRPM_BL = (filteredRPM_BL == 0.0) ? rpm : (alpha * rpm + (1.0 - alpha) * filteredRPM_BL);
    }   
    return filteredRPM_BL;
}

//-----------------------------------------//
// RPM CALCULATION WITH FILTERING - BACK RIGHT //
//-----------------------------------------//
float calculateRPM_BR(unsigned long safeLastTickTime) {
    const float alpha = 0.05;
    float avgTickInterval = 0.0;
    float rpm = 0.0;
    unsigned long timeSinceLastTick = micros() - safeLastTickTime;
    // Create local copies of volatile variables inside atomic block
    unsigned long localTickIntervals[TICK_AVG_WINDOW];
    unsigned int localTickIndex = 0;
    // Copy the entire array and index atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memcpy((void*)localTickIntervals, (const void*)tickIntervals_BR, sizeof(tickIntervals_BR));
        localTickIndex = tickIndex_BR;
    }

    if (timeSinceLastTick > MAX_TICK_TIME) {
        rpm = 0.0;
        filteredRPM_BR = 0.0;
    } else {
        unsigned long sum = 0;
        int validCount = 0;
        
        for (int i = 0; i < TICK_AVG_WINDOW; i++) {
            unsigned int j = (localTickIndex - i) % TICK_AVG_WINDOW; // Wrap-around indexing: ensures j is always within positive
            if (localTickIntervals[j] > 0) {
                sum += localTickIntervals[j];
                validCount++;
            }
            if(sum >= MAX_TICK_TIME) break;
        }    
        if (validCount >= 2) {
            avgTickInterval = (float)sum / validCount;
            if (avgTickInterval > 0) {
                rpm = (1.0 / (avgTickInterval / 1000000.0)) * (60.0 / PULSES_PER_ROTATION);
                if (!isfinite(rpm)) {
                    rpm = 0.0;
                } else {
                    rpm = rpm * direction_BR;  // Apply direction sign
                    // Saturate RPM to max validation limit instead of zeroing
                    rpm = constrain(rpm, -MAX_RPM_VALIDATION, MAX_RPM_VALIDATION);
                }
            }
        }
        filteredRPM_BR = (filteredRPM_BR == 0.0) ? rpm : (alpha * rpm + (1.0 - alpha) * filteredRPM_BR);
    }   
    return filteredRPM_BR;
}

//-----------------------------------------//
// RPM CALCULATION WITH FILTERING - FRONT LEFT //
//-----------------------------------------//
float calculateRPM_FL(unsigned long safeLastTickTime) {
    const float alpha = 0.05;
    float avgTickInterval = 0.0;
    float rpm = 0.0;
    unsigned long timeSinceLastTick = micros() - safeLastTickTime;
    // Create local copies of volatile variables inside atomic block  
    unsigned long localTickIntervals[TICK_AVG_WINDOW];
    unsigned int localTickIndex = 0;
    // Copy the entire array and index atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memcpy((void*)localTickIntervals, (const void*)tickIntervals_FL, sizeof(tickIntervals_FL));
        localTickIndex = tickIndex_FL;
    }

    if (timeSinceLastTick > MAX_TICK_TIME) {
        rpm = 0.0;
        filteredRPM_FL = 0.0;
    } else {
        unsigned long sum = 0;
        int validCount = 0;
        
        for (int i = 0; i < TICK_AVG_WINDOW; i++) {
            unsigned int j = (localTickIndex - i) % TICK_AVG_WINDOW;  // Wrap-around indexing: ensures j is always within positive
            if (localTickIntervals[j] > 0) {
                sum += localTickIntervals[j];
                validCount++;
            }
            if(sum >= MAX_TICK_TIME) break;
        }    
        if (validCount >= 2) {
            avgTickInterval = (float)sum / validCount;
            if (avgTickInterval > 0) {
                rpm = (1.0 / (avgTickInterval / 1000000.0)) * (60.0 / PULSES_PER_ROTATION);
                if (!isfinite(rpm)) {
                    rpm = 0.0;
                } else {
                    rpm = rpm * direction_FL;  // Apply direction sign
                    // Saturate RPM to max validation limit instead of zeroing
                    rpm = constrain(rpm, -MAX_RPM_VALIDATION, MAX_RPM_VALIDATION);
                }
            }
        }
        filteredRPM_FL = (filteredRPM_FL == 0.0) ? rpm : (alpha * rpm + (1.0 - alpha) * filteredRPM_FL);
    }   
    return filteredRPM_FL;
}

//-----------------------------------------//
// RPM CALCULATION WITH FILTERING - FRONT RIGHT //
//-----------------------------------------//
float calculateRPM_FR(unsigned long safeLastTickTime) {
    const float alpha = 0.05;
    float avgTickInterval = 0.0;
    float rpm = 0.0;
    unsigned long timeSinceLastTick = micros() - safeLastTickTime;
    // Create local copies of volatile variables inside atomic block
    unsigned long localTickIntervals[TICK_AVG_WINDOW];
    unsigned int localTickIndex = 0;
    // Copy the entire array and index atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memcpy((void*)localTickIntervals, (const void*)tickIntervals_FR, sizeof(tickIntervals_FR));
        localTickIndex = tickIndex_FR;
    }

    if (timeSinceLastTick > MAX_TICK_TIME) {
        rpm = 0.0;
        filteredRPM_FR = 0.0;
    } else {
        unsigned long sum = 0;
        int validCount = 0;
        
        for (int i = 0; i < TICK_AVG_WINDOW; i++) {
            unsigned int j = (localTickIndex - i) % TICK_AVG_WINDOW;  // Wrap-around indexing: ensures j is always within positive 
            if (localTickIntervals[j] > 0) {
                sum += localTickIntervals[j];
                validCount++;
            }
            if(sum >= MAX_TICK_TIME) break;
        }    
        if (validCount >= 2) {
            avgTickInterval = (float)sum / validCount;
            if (avgTickInterval > 0) {
                rpm = (1.0 / (avgTickInterval / 1000000.0)) * (60.0 / PULSES_PER_ROTATION);
                if (!isfinite(rpm)) {
                    rpm = 0.0;
                } else {
                    rpm = rpm * direction_FR;  // Apply direction sign
                    // Saturate RPM to max validation limit instead of zeroing
                    rpm = constrain(rpm, -MAX_RPM_VALIDATION, MAX_RPM_VALIDATION);
                }
            }
        }
        filteredRPM_FR = (filteredRPM_FR == 0.0) ? rpm : (alpha * rpm + (1.0 - alpha) * filteredRPM_FR);
    }   
    return filteredRPM_FR;
}

//-----------------------------------------//
// NON-BLOCKING PWM SMOOTHING UPDATE - BACK LEFT //
//-----------------------------------------//
void updateSmoothPWM_BL(float desiredPWM) {
    const float SMOOTH_THRESHOLD = 12.0;  // Reduced threshold for better responsiveness
    float diff = desiredPWM - lastPWM_BL;
    if (abs(diff) > SMOOTH_THRESHOLD) {
        float step = (SMOOTH_THRESHOLD) * (diff/abs(diff)); // Calculate step size based on smooth threshold
        lastPWM_BL += step;
        lastPWM_BL = constrain(lastPWM_BL, -MAX_PWM, MAX_PWM); 
        setMotorPWM_BL((int)lastPWM_BL);
    } else {
        lastPWM_BL = constrain(desiredPWM, -MAX_PWM, MAX_PWM);
        setMotorPWM_BL((int)lastPWM_BL);
    }
}

//-----------------------------------------//
// NON-BLOCKING PWM SMOOTHING UPDATE - BACK RIGHT //
//-----------------------------------------//
void updateSmoothPWM_BR(float desiredPWM) {
    const float SMOOTH_THRESHOLD = 12.0;  // Reduced threshold for better responsiveness
    float diff = desiredPWM - lastPWM_BR;
    if (abs(diff) > SMOOTH_THRESHOLD) {
        float step = (SMOOTH_THRESHOLD) * (diff/abs(diff));   // Calculate step size based on smooth threshold
        lastPWM_BR += step;
        lastPWM_BR = constrain(lastPWM_BR, -MAX_PWM, MAX_PWM);
        setMotorPWM_BR((int)lastPWM_BR);
    } else {
        lastPWM_BR = constrain(desiredPWM, -MAX_PWM, MAX_PWM);
        setMotorPWM_BR((int)lastPWM_BR);
    }
}

//-----------------------------------------//
// NON-BLOCKING PWM SMOOTHING UPDATE - FRONT LEFT //
//-----------------------------------------//
void updateSmoothPWM_FL(float desiredPWM) {
    const float SMOOTH_THRESHOLD = 12.0;  // Reduced threshold for better responsiveness
    float diff = desiredPWM - lastPWM_FL;
    if (abs(diff) > SMOOTH_THRESHOLD) {
        float step = (SMOOTH_THRESHOLD) * (diff/abs(diff)); // Calculate step size based on smooth threshold
        lastPWM_FL += step;
        lastPWM_FL = constrain(lastPWM_FL, -MAX_PWM, MAX_PWM);
        setMotorPWM_FL((int)lastPWM_FL);
    } else {
        lastPWM_FL = constrain(desiredPWM, -MAX_PWM, MAX_PWM);
        setMotorPWM_FL((int)lastPWM_FL);
    }
}

//-----------------------------------------//
// NON-BLOCKING PWM SMOOTHING UPDATE - FRONT RIGHT //
//-----------------------------------------//
void updateSmoothPWM_FR(float desiredPWM) {
    const float SMOOTH_THRESHOLD = 12.0;  // Reduced threshold for better responsiveness
    float diff = desiredPWM - lastPWM_FR;
    if (abs(diff) > SMOOTH_THRESHOLD) {
        float step = (SMOOTH_THRESHOLD) * (diff/abs(diff)); // Calculate step size based on smooth threshold
        lastPWM_FR += step;
        lastPWM_FR = constrain(lastPWM_FR, -MAX_PWM, MAX_PWM);
        setMotorPWM_FR((int)lastPWM_FR);
    } else {
        lastPWM_FR = constrain(desiredPWM, -MAX_PWM, MAX_PWM);
        setMotorPWM_FR((int)lastPWM_FR);
    }
}

//-----------------------------------------//
// SET MOTOR OUTPUT - BACK LEFT //
//-----------------------------------------//
void setMotorPWM_BL(int pwmVal) {
    pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);
    
    if (pwmVal >= 0) {
        // Forward direction
        analogWrite(BL_F, pwmVal);
        analogWrite(BL_B, 0);
    } else {
        // Reverse direction
        analogWrite(BL_F, 0);
        analogWrite(BL_B, abs(pwmVal));
    }
    currentPWM_BL = pwmVal;
}

//-----------------------------------------//
// SET MOTOR OUTPUT - BACK RIGHT //
//-----------------------------------------//
void setMotorPWM_BR(int pwmVal) {
    pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);
    
    if (pwmVal >= 0) {
        // Forward direction
        analogWrite(BR_F, pwmVal);
        analogWrite(BR_B, 0);
    } else {
        // Reverse direction
        analogWrite(BR_F, 0);
        analogWrite(BR_B, abs(pwmVal));
    }
    currentPWM_BR = pwmVal;
}

//-----------------------------------------//
// SET MOTOR OUTPUT - FRONT LEFT //
//-----------------------------------------//
void setMotorPWM_FL(int pwmVal) {
    pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);
    
    if (pwmVal >= 0) {
        // Forward direction
        analogWrite(FL_F, pwmVal);
        analogWrite(FL_B, 0);
    } else {
        // Reverse direction
        analogWrite(FL_F, 0);
        analogWrite(FL_B, abs(pwmVal));
    }
    currentPWM_FL = pwmVal;
}

//-----------------------------------------//
// SET MOTOR OUTPUT - FRONT RIGHT //
//-----------------------------------------//
void setMotorPWM_FR(int pwmVal) {
    pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);
    
    if (pwmVal >= 0) {
        // Forward direction
        analogWrite(FR_F, pwmVal);
        analogWrite(FR_B, 0);
    } else {
        // Reverse direction
        analogWrite(FR_F, 0);
        analogWrite(FR_B, abs(pwmVal));
    }
    currentPWM_FR = pwmVal;
}

//-----------------------------------------//
// IMMEDIATE MOTOR STOP & RESET - ALL MOTORS //
//-----------------------------------------//
void stopAllMotorsImmediately() {
    // Stop Back Left Motor
    setMotorPWM_BL(0);
    lastPWM_BL = 0;     // Reset smoothing state
    targetPWM_BL = 0;   // Reset target PWM
    for (int i = 0; i < TICK_AVG_WINDOW; i++) tickIntervals_BL[i] = 0;
    tickIndex_BL = 0; tickCount_BL = 0; filteredRPM_BL = 0;
    integral_BL = 0; previousError_BL = 0; filteredDerivative_BL = 0;

    // Stop Back Right Motor
    setMotorPWM_BR(0);
    lastPWM_BR = 0;     // Reset smoothing state
    targetPWM_BR = 0;   // Reset target PWM
    for (int i = 0; i < TICK_AVG_WINDOW; i++) tickIntervals_BR[i] = 0;
    tickIndex_BR = 0; tickCount_BR = 0; filteredRPM_BR = 0;
    integral_BR = 0; previousError_BR = 0; filteredDerivative_BR = 0;

    // Stop Front Left Motor
    setMotorPWM_FL(0);
    lastPWM_FL = 0;       // Reset smoothing state
    targetPWM_FL = 0;     // Reset target PWM
    for (int i = 0; i < TICK_AVG_WINDOW; i++) tickIntervals_FL[i] = 0;
    tickIndex_FL = 0; tickCount_FL = 0; filteredRPM_FL = 0;
    integral_FL = 0; previousError_FL = 0; filteredDerivative_FL = 0;

    // Stop Front Right Motor
    setMotorPWM_FR(0);
    lastPWM_FR = 0;     // Reset smoothing state
    targetPWM_FR = 0;   // Reset target PWM
    for (int i = 0; i < TICK_AVG_WINDOW; i++) tickIntervals_FR[i] = 0;
    tickIndex_FR = 0; tickCount_FR = 0; filteredRPM_FR = 0;
    integral_FR = 0; previousError_FR = 0; filteredDerivative_FR = 0;
}

//-----------------------------------------//
// INTERRUPT ROUTINES FOR TICK COUNT //
//-----------------------------------------//
void onHallSensor_BL_A() {
    unsigned long now = micros();
    if (lastTickTime_BL > 0) {
        unsigned long interval = now - lastTickTime_BL;
        tickIndex_BL = (tickIndex_BL + 1) % TICK_AVG_WINDOW;
        tickIntervals_BL[tickIndex_BL] = interval;  
        if (tickCount_BL < TICK_AVG_WINDOW) tickCount_BL++;
    }
    prevTickTime_BL = lastTickTime_BL;
    lastTickTime_BL = now;
    totalTicks_BL++;
    if (digitalRead(HALL_SENSOR_BL_B_PIN) == HIGH) {
        direction_BL = -1;
        odometry_BL--;
    } else {
        direction_BL = 1;
        odometry_BL++;
    }
}

void onHallSensor_BR_A() {
    unsigned long now = micros();
    if (lastTickTime_BR > 0) {
        unsigned long interval = now - lastTickTime_BR;
        tickIndex_BR = (tickIndex_BR + 1) % TICK_AVG_WINDOW;
        tickIntervals_BR[tickIndex_BR] = interval;  
        if (tickCount_BR < TICK_AVG_WINDOW) tickCount_BR++;
    }
    prevTickTime_BR = lastTickTime_BR;
    lastTickTime_BR = now;
    totalTicks_BR++;
    if (digitalRead(HALL_SENSOR_BR_B_PIN) == HIGH) {
        direction_BR = 1;
        odometry_BR++;
    } else {
        direction_BR = -1;
        odometry_BR--;
    }
}

void onHallSensor_FL_A() {
    unsigned long now = micros();
    if (lastTickTime_FL > 0) {
        unsigned long interval = now - lastTickTime_FL;
        tickIndex_FL = (tickIndex_FL + 1) % TICK_AVG_WINDOW;
        tickIntervals_FL[tickIndex_FL] = interval;  
        if (tickCount_FL < TICK_AVG_WINDOW) tickCount_FL++;
    }
    prevTickTime_FL = lastTickTime_FL;
    lastTickTime_FL = now;
    totalTicks_FL++;
    if (digitalRead(HALL_SENSOR_FL_B_PIN) == HIGH) {
        direction_FL = -1;
        odometry_FL--;
    } else {
        direction_FL = 1;
        odometry_FL++;
    }
}

void onHallSensor_FR_A() {
    unsigned long now = micros();
    if (lastTickTime_FR > 0) {
        unsigned long interval = now - lastTickTime_FR;
        tickIndex_FR = (tickIndex_FR + 1) % TICK_AVG_WINDOW;
        tickIntervals_FR[tickIndex_FR] = interval;  
        if (tickCount_FR < TICK_AVG_WINDOW) tickCount_FR++;
    }
    prevTickTime_FR = lastTickTime_FR;
    lastTickTime_FR = now;
    totalTicks_FR++;
    if (digitalRead(HALL_SENSOR_FR_B_PIN) == HIGH) {
        direction_FR = 1;
        odometry_FR++;
    } else {
        direction_FR = -1;
        odometry_FR--;
    }
}

//-----------------------------------------//
// SERIAL DATA PARSING //
//-----------------------------------------//
void parseData(String data) {
    if (data.startsWith("$") && data.endsWith("#")) {
        data = data.substring(1, data.length() - 1);

        int frIndex = data.indexOf("FR:");
        int flIndex = data.indexOf("FL:");
        int brIndex = data.indexOf("BR:");
        int blIndex = data.indexOf("BL:");

        if (frIndex == -1 || flIndex == -1 || brIndex == -1 || blIndex == -1) {
            Serial3.println("Parsing Error: Tags missing");
            return;
        }
             
        targetRPM_FR = data.substring(frIndex + 3, flIndex - 1).toFloat();
        targetRPM_FL = data.substring(flIndex + 3, brIndex - 1).toFloat();
        targetRPM_BR = data.substring(brIndex + 3, blIndex - 1).toFloat();
        targetRPM_BL = data.substring(blIndex + 3).toFloat();

        targetRPM_FR = constrain(targetRPM_FR, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);
        targetRPM_FL = constrain(targetRPM_FL, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);
        targetRPM_BR = constrain(targetRPM_BR, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);
        targetRPM_BL = constrain(targetRPM_BL, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);

        // Reset PID states when new targets are set
        if (targetRPM_BL == 0) { integral_BL = 0; previousError_BL = 0; filteredDerivative_BL = 0; }
        else { previousError_BL = targetRPM_BL - measuredSpeed_BL; }
        
        if (targetRPM_BR == 0) { integral_BR = 0; previousError_BR = 0; filteredDerivative_BR = 0; }
        else { previousError_BR = targetRPM_BR - measuredSpeed_BR; }
        
        if (targetRPM_FL == 0) { integral_FL = 0; previousError_FL = 0; filteredDerivative_FL = 0; }
        else { previousError_FL = targetRPM_FL - measuredSpeed_FL; }
        
        if (targetRPM_FR == 0) { integral_FR = 0; previousError_FR = 0; filteredDerivative_FR = 0; }
        else { previousError_FR = targetRPM_FR - measuredSpeed_FR; }
    }
}

//-----------------------------------------//
// SERIAL EVENT HANDLERS //
//-----------------------------------------//
void serialEvent3() {
    while (Serial3.available()) {
        char inChar = (char)Serial3.read();
        if (inChar == '$') {
            inputString = "";
        }
        inputString += inChar;
        if (inChar == '#') {
            stringComplete = true;
        }
    }
}

//-----------------------------------------//
// SETUP FUNCTION //
//-----------------------------------------//
void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    
    // Initialize motor driver pins
    pinMode(BL_F, OUTPUT); pinMode(BL_B, OUTPUT);
    pinMode(BR_F, OUTPUT); pinMode(BR_B, OUTPUT);
    pinMode(FL_F, OUTPUT); pinMode(FL_B, OUTPUT);
    pinMode(FR_F, OUTPUT); pinMode(FR_B, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Initialize hall sensor pins
    pinMode(HALL_SENSOR_BL_A_PIN, INPUT_PULLUP); pinMode(HALL_SENSOR_BL_B_PIN, INPUT_PULLUP);
    pinMode(HALL_SENSOR_BR_A_PIN, INPUT_PULLUP); pinMode(HALL_SENSOR_BR_B_PIN, INPUT_PULLUP);
    pinMode(HALL_SENSOR_FL_A_PIN, INPUT_PULLUP); pinMode(HALL_SENSOR_FL_B_PIN, INPUT_PULLUP);
    pinMode(HALL_SENSOR_FR_A_PIN, INPUT_PULLUP); pinMode(HALL_SENSOR_FR_B_PIN, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_BL_A_PIN), onHallSensor_BL_A, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_BR_A_PIN), onHallSensor_BR_A, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_FL_A_PIN), onHallSensor_FL_A, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_FR_A_PIN), onHallSensor_FR_A, RISING);
    
    // ---- IMU INIT -----------
    #if USE_IMU
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    bool initialized = false;
    while (!initialized) {
        myICU.begin(WIRE_PORT, AD0_VAL);
        Serial3.println(myICU.statusString());
        if (myICU.status == ICM_20948_Stat_Ok) {
            initialized = true;
            Serial3.println("IMU initialized!");
        } else {
            Serial3.println("IMU initialization failed. Trying again...");
            delay(500);
        }
    }
    imuThread.onRun(imuTask);
    imuThread.setInterval(imuPrintPeriod);
    #endif
    
    delay(500);
}

//-----------------------------------------//
// MAIN LOOP //
//-----------------------------------------//
void loop() {
    static unsigned long lastPrintTime = micros();
    
    //--------------- RECEIVE TARGET RPM FROM SERIAL ---------------//
    if (stringComplete) {
        parseData(inputString);
        inputString = "";
        stringComplete = false;
    }

    //--------------- SAFETY CHECK: STOP IF ALL TARGETS ARE ZERO ---------------//
    if (targetRPM_BL == 0 && targetRPM_BR == 0 && targetRPM_FL == 0 && targetRPM_FR == 0) {
        stopAllMotorsImmediately();
    }

    //--------------- MAIN CONTROL LOOP (EVERY 10ms) ---------------//
    if (micros() - lastPrintTime >= INTERVAL) {
        unsigned long now = micros();
        
        unsigned long interval = now - lastPrintTime;
        lastPrintTime = now;
        
        // Read battery voltage
        BAT_VOLT = analogRead(ADC_PIN) * (5.0 / 1023.0) * 3.3; // Adjust scaling as needed

        //===============================================//
        // BACK LEFT MOTOR CONTROL BLOCK //
        //===============================================//
        unsigned long safeLastTickTime_BL;
        int safeTotalTicks_BL;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            safeLastTickTime_BL = lastTickTime_BL;
            safeTotalTicks_BL = totalTicks_BL;
        }
        
        float rpm_BL = calculateRPM_BL(safeLastTickTime_BL);
        measuredSpeed_BL = rpm_BL;
        
        if (targetRPM_BL != 0) {
            float error_BL = targetRPM_BL - rpm_BL;
            float dt = interval / 1000000.0;

            // PID Control with improved derivative filtering
            float Pout_BL = Kp * error_BL;
            
            // Integral with improved anti-windup
            integral_BL += Ki * error_BL * dt;
            integral_BL = constrain(integral_BL, -maxIntegral, maxIntegral);
            float Iout_BL = integral_BL;
            
            // Filtered derivative term to reduce noise
            float rawDerivative_BL = (error_BL - previousError_BL) / dt;
            filteredDerivative_BL = derivativeFilter * rawDerivative_BL + (1.0 - derivativeFilter) * filteredDerivative_BL;
            float Dout_BL = Kd * filteredDerivative_BL;
            
            float correction_BL = Pout_BL + Iout_BL + Dout_BL;
            previousError_BL = error_BL;

            // Initialize targetPWM with a better estimate for step response
            if (targetPWM_BL == 0 && targetRPM_BL != 0) {
                targetPWM_BL = targetRPM_BL * 1.1; // Initial estimate: ~1.1 PWM per RPM (handles negative values)
            }
            
            targetPWM_BL += correction_BL;
            correction = correction_BL;
            targetPWM_BL = constrain(targetPWM_BL, -MAX_PWM, MAX_PWM);
            updateSmoothPWM_BL(targetPWM_BL);
            
        }

        //===============================================//
        // BACK RIGHT MOTOR CONTROL BLOCK //
        //===============================================//
        unsigned long safeLastTickTime_BR;
        int safeTotalTicks_BR;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            safeLastTickTime_BR = lastTickTime_BR;
            safeTotalTicks_BR = totalTicks_BR;
        }
        
        float rpm_BR = calculateRPM_BR(safeLastTickTime_BR);
        measuredSpeed_BR = rpm_BR;
        
        if (targetRPM_BR != 0) {
            float error_BR = targetRPM_BR - rpm_BR;
            float dt = interval / 1000000.0;

            // PID Control with improved derivative filtering
            float Pout_BR = Kp * error_BR;
            
            // Integral with improved anti-windup
            integral_BR += Ki * error_BR * dt;
            integral_BR = constrain(integral_BR, -maxIntegral, maxIntegral);
            float Iout_BR = integral_BR;
            
            // Filtered derivative term to reduce noise
            float rawDerivative_BR = (error_BR - previousError_BR) / dt;
            filteredDerivative_BR = derivativeFilter * rawDerivative_BR + (1.0 - derivativeFilter) * filteredDerivative_BR;
            float Dout_BR = Kd * filteredDerivative_BR;
            
            float correction_BR = Pout_BR + Iout_BR + Dout_BR;
            previousError_BR = error_BR;

            // Initialize targetPWM with a better estimate for step response
            if (targetPWM_BR == 0 && targetRPM_BR != 0) {
                targetPWM_BR = targetRPM_BR * 1.1; // Initial estimate: ~1.1 PWM per RPM (handles negative values)
            }
            
            targetPWM_BR += correction_BR;
            targetPWM_BR = constrain(targetPWM_BR, -MAX_PWM, MAX_PWM);
            updateSmoothPWM_BR(targetPWM_BR);
        }

        //===============================================//
        // FRONT LEFT MOTOR CONTROL BLOCK //
        //===============================================//
        unsigned long safeLastTickTime_FL;
        int safeTotalTicks_FL;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            safeLastTickTime_FL = lastTickTime_FL;
            safeTotalTicks_FL = totalTicks_FL;
        }
        
        float rpm_FL = calculateRPM_FL(safeLastTickTime_FL);
        measuredSpeed_FL = rpm_FL;
        
        if (targetRPM_FL != 0) {
            float error_FL = targetRPM_FL - rpm_FL;
            float dt = interval / 1000000.0;

            // PID Control with improved derivative filtering
            float Pout_FL = Kp * error_FL;
            
            // Integral with improved anti-windup
            integral_FL += Ki * error_FL * dt;
            integral_FL = constrain(integral_FL, -maxIntegral, maxIntegral);
            float Iout_FL = integral_FL;
            
            // Filtered derivative term to reduce noise
            float rawDerivative_FL = (error_FL - previousError_FL) / dt;
            filteredDerivative_FL = derivativeFilter * rawDerivative_FL + (1.0 - derivativeFilter) * filteredDerivative_FL;
            float Dout_FL = Kd * filteredDerivative_FL;
            
            float correction_FL = Pout_FL + Iout_FL + Dout_FL;
            previousError_FL = error_FL;

            // Initialize targetPWM with a better estimate for step response
            if (targetPWM_FL == 0 && targetRPM_FL != 0) {
                targetPWM_FL = targetRPM_FL * 1.1; // Initial estimate: ~1.1 PWM per RPM (handles negative values)
            }
            
            targetPWM_FL += correction_FL;
            targetPWM_FL = constrain(targetPWM_FL, -MAX_PWM, MAX_PWM);
            updateSmoothPWM_FL(targetPWM_FL);
        }

        //===============================================//
        // FRONT RIGHT MOTOR CONTROL BLOCK //
        //===============================================//
        unsigned long safeLastTickTime_FR;
        int safeTotalTicks_FR;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            safeLastTickTime_FR = lastTickTime_FR;
            safeTotalTicks_FR = totalTicks_FR;
        }
        
        float rpm_FR = calculateRPM_FR(safeLastTickTime_FR);
        measuredSpeed_FR = rpm_FR;
        
        if (targetRPM_FR != 0) {
            float error_FR = targetRPM_FR - rpm_FR;
            float dt = interval / 1000000.0;

            // PID Control with improved derivative filtering
            float Pout_FR = Kp * error_FR;
            
            // Integral with improved anti-windup
            integral_FR += Ki * error_FR * dt;
            integral_FR = constrain(integral_FR, -maxIntegral, maxIntegral);
            float Iout_FR = integral_FR;
            
            // Filtered derivative term to reduce noise
            float rawDerivative_FR = (error_FR - previousError_FR) / dt;
            filteredDerivative_FR = derivativeFilter * rawDerivative_FR + (1.0 - derivativeFilter) * filteredDerivative_FR;
            float Dout_FR = Kd * filteredDerivative_FR;
            
            float correction_FR = Pout_FR + Iout_FR + Dout_FR;
            previousError_FR = error_FR;

            // Initialize targetPWM with a better estimate for step response
            if (targetPWM_FR == 0 && targetRPM_FR != 0) {
                targetPWM_FR = targetRPM_FR * 1.1; // Initial estimate: ~1.1 PWM per RPM (handles negative values)
            }
            
            targetPWM_FR += correction_FR;
            targetPWM_FR = constrain(targetPWM_FR, -MAX_PWM, MAX_PWM);
            updateSmoothPWM_FR(targetPWM_FR);
        }


            //--------------- SERIAL MONITOR LOGGING (UNCHANGED) ---------------//
            
            Serial.print("RPM_BL: "); Serial.print(rpm_BL); Serial.print(", ");
            Serial.print("TargetRPM_BL: "); Serial.print(targetRPM_BL); Serial.print(", ");
            Serial.print("PWM_BL: "); Serial.print((int)lastPWM_BL); Serial.print(", ");
            Serial.print("Error_BL: "); Serial.print(targetRPM_BL - rpm_BL); Serial.print(", ");

            
            Serial.print("RPM_BR: "); Serial.print(rpm_BR); Serial.print(", ");
            Serial.print("TargetRPM_BR: "); Serial.print(targetRPM_BR); Serial.print(", ");
            Serial.print("PWM_BR: "); Serial.print((int)lastPWM_BR); Serial.print(", ");
            Serial.print("Error_BR: "); Serial.print(targetRPM_BR - rpm_BR); Serial.print(", ");  

            
            Serial.print("RPM_FR: "); Serial.print(rpm_FR); Serial.print(", ");
            Serial.print("TargetRPM_FR: "); Serial.print(targetRPM_FR); Serial.print(", ");
            Serial.print("PWM_FR: "); Serial.print((int)lastPWM_FR); Serial.print(", ");
            Serial.print("Error_FR: "); Serial.print(targetRPM_FR - rpm_FR); Serial.print(", "); 

            
            Serial.print("RPM_FL: "); Serial.print(rpm_FL); Serial.print(", ");
            Serial.print("TargetRPM_FL: "); Serial.print(targetRPM_FL); Serial.print(", ");
            Serial.print("PWM_FL: "); Serial.print((int)lastPWM_FL); Serial.print(", ");
            Serial.print("Error_FL: "); Serial.println(targetRPM_FL - rpm_FL);      
            // Send odometry data
            Serial3.print("$BR:"); Serial3.print(odometry_BR);
            Serial3.print(",BL:"); Serial3.print(odometry_BL);
            Serial3.print(",FL:"); Serial3.print(odometry_FL);
            Serial3.print(",FR:"); Serial3.print(odometry_FR);
            Serial3.print(",BT:"); Serial3.print(BAT_VOLT);
            Serial3.println("#");
            
          
        
    }
    
    // -- IMU Thread running (if enabled) --
    #if USE_IMU
    imuThread.run();
    #endif
}

/*
Example Serial Commands:
$FR:0,FL:0,BR:0,BL:0#          // Stop all motors
$FR:20,FL:20,BR:20,BL:20# 
$FR:-20,FL:-20,BR:-20,BL:-20#     
$FR:15,FL:25,BR:15,BL:25#    // Turn right
$FR:-30,FL:30,BR:-30,BL:30#    // Turn left
$FR:50,FL:50,BR:50,BL:50#      // Move forward at 50 RPM
*/