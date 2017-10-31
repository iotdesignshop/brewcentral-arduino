#include <TimedPID.h>
#include <Wire.h>
#include <DueTimer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MAX31855.h>

#define SERVO_CONTROL_RATE 2  // Polling rate for flow pulse tally and servo response
#define SERVO_CONTROL_TIME (SERVO_CONTROL_RATE * 60)  // Number of updates per minute.
#define SERVO_RELAX_OFFSET 8  // Amount to relax servo closed stop to reduce current draw
#define SERVO_RELAX_TIME 3    // Delay before relaxing servo position
int Servo0Relax = 0;
int Servo1Relax = 0;
float ServoLastTime;
float ServoCurrentTime;
float ServoTimeDelta;

#define FLOW_PER_TICK (0.000367 / 2)  // Double edge counting pulses, thus div 2 here
#define MIXER_FLOW_CORRECTION 0.95
#define MASH_FLOW_CORRECTION 1.0

#define MIXER_SERVO_THROW 184 // Measured tick throw of Futaba servos on 90 degree valve
#define MIXER_SERVO_0_MIN 172  // Closed position for servo 0, Hot
#define MIXER_SERVO_0_MAX (MIXER_SERVO_0_MIN + MIXER_SERVO_THROW)
#define MIXER_SERVO_1_MIN (MIXER_SERVO_1_MAX - MIXER_SERVO_THROW)
#define MIXER_SERVO_1_MAX 510  // Closed position for servo 1, COld
#define MIXER_FLOW_PULSE_PIN 53
#define MIXER_TEMP_PIN 51
#define MIXER_MAX_FLOW_SET 3.0
#define MIXER_KP 0.50
#define MIXER_KD 0.005
#define MIXER_KI 0.0005
int MixerPulseCount;
float MixerFlowSet;
float MixerTempSet;
float MixVolume;
float MixFlowRate;
float MixFlowRatePrevious;
int MixerFlowUnderRun;
float MixerFlowPIDOutput;
int MixerServo0SetPoint;
int MixerServo1SetPoint;
float temp0Set;
float temp1Set;
//m500 off
//m300 on

#define MASH_SERVO_THROW 200
#define MASH_SERVO_MIN 300
#define MASH_SERVO_MAX (MASH_SERVO_THROW + MASH_SERVO_MIN)
#define MASH_FLOW_PULSE_PIN 49
#define MASH_TEMP_DO_PIN  48
#define MASH_TEMP_CS_PIN  50
#define MASH_TEMP_CLK_PIN 52
#define MASH_MAX_FLOW_SET 3.0
#define MASH_KP 0.50
#define MASH_KD 0.005
#define MASH_KI 0.0005
float MashFlowSet;
float MashFlowRate;
float MashFlowRatePrevious;
float MashVolume;
int MashPulseCount;
float MashFlowPIDOutput;
int MashServoSetPoint;
int MashServoRelax;
float MashTemp;

#define TEMP_INT_CUTOFF 10  // Degrees offset before integrator starts running.
#define TEMP_KP 0.110
#define TEMP_KD 0.110
#define TEMP_KI 0.004
float Temp1PIDOutput;
float OldMixTemp = 0;
float MixTemp = 0;

char RxBuffer[5];
TimedPID mixerFlowPID(MIXER_KP, MIXER_KI, MIXER_KD);
TimedPID mashFlowPID(MASH_KP, MASH_KI, MASH_KD);
TimedPID temp1PID(TEMP_KP, TEMP_KI, TEMP_KD);
OneWire  oneWireTemp(MIXER_TEMP_PIN);
DallasTemperature tempSensors(&oneWireTemp);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MAX31855 thermocouple(MASH_TEMP_CLK_PIN, MASH_TEMP_CS_PIN, MASH_TEMP_DO_PIN);

#define SERIAL_TEMP_UPDATE_PERIOD   1000      // Communicate with Android at this frequency (milliseconds)

// -------------------------------------------------------------------------------
// Progam Setup
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("--- Beer Central Adruino PoC In Development ---");
  
  pinMode(MIXER_FLOW_PULSE_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(MIXER_FLOW_PULSE_PIN), MixerPulseISR, CHANGE);
  pinMode(MASH_FLOW_PULSE_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(MASH_FLOW_PULSE_PIN), MashPulseISR, CHANGE);

  pwm.begin();
  pwm.setPWMFreq(60); 
  Timer0.attachInterrupt(Timer0ISR);
  Timer0.setFrequency(SERVO_CONTROL_RATE); 
  Timer0.start(); 
  Timer1.attachInterrupt(Timer1ISR);
  Timer1.setFrequency(1); 
  Timer1.start(); 
  //Timer2.attachInterrupt(Timer2ISR);
  //Timer2.setFrequency(10); 
  //Timer2.start(); 
  Timer3.attachInterrupt(Timer3ISR);
  Timer3.setFrequency(SERVO_CONTROL_RATE); 
  Timer3.start(); 

  mixerFlowPID.setCmdRange(0, MIXER_MAX_FLOW_SET);
  mixerFlowPID.reset();
  mashFlowPID.setCmdRange(0, MASH_MAX_FLOW_SET);
  mashFlowPID.reset();
  temp1PID.setCmdRange(-1, 1);
  temp1PID.reset();
  tempSensors.begin();
  tempSensors.setResolution(11);
  MixerTempSet = tempSensors.getTempCByIndex(0);

  // Set up serial interface to Android
  Serial1.begin(9600);
  
}

// -------------------------------------------------------------------------------
// Flow Meter Pulse Counter
void MixerPulseISR() {
  MixerPulseCount++;
}

//-------------------------------------------------------------------------------
// Flow Meter Pulse Counter
void MashPulseISR() {
  MashPulseCount++;
}


// -------------------------------------------------------------------------------
// Serial1 (Android) Print

#define TAG_TEMP_MIX  "t-mix"
#define TAG_TEMP_MASH "t-msh"

#define TAG_FLOW_MIX  "f-mix"
#define TAG_FLOW_MASH "f-msh"

#define TAG_VOLUME_MIX  "v-mix"
#define TAG_VOLUME_MASH "v-msh"

void sendSerial1(char* tag, float currentVal, float setPoint)
{
  Serial1.print(tag);
  Serial1.print(" ");
  Serial1.print(currentVal);
  Serial1.print(" ");
  Serial1.println(setPoint);
}

void sendSerial1(char* tag, float currentVal)
{
  Serial1.print(tag);
  Serial1.print(" ");
  Serial1.print(currentVal);
}


// -------------------------------------------------------------------------------
// USB/Android Serial Print Timer
void Timer0ISR() {
  //Serial.print(dMixerPulseCount, 0);
  //Serial.print(", ");
  Serial.print("MixF: ");
  Serial.print(MixVolume, 2);
  Serial.print(", ");
  Serial.print(MixFlowRate, 2);
  Serial.print(", ");
  Serial.print(MixerFlowSet, 2);
  Serial.print(", ");
  Serial.print(MixerFlowPIDOutput, 2);
  Serial.print(" MixT: ");
  Serial.print(MixTemp, 2);
  Serial.print(", ");
  Serial.print(MixerTempSet, 2);
  Serial.print(", ");
  Serial.print(Temp1PIDOutput, 2);
  Serial.print(" MashF: ");
  Serial.print(MashVolume, 2);
  Serial.print(", ");
  Serial.print(MashFlowRate, 2);
  Serial.print(", ");
  Serial.print(MashFlowSet, 2);
  Serial.print(", ");
  Serial.print(MashFlowPIDOutput, 2);
  Serial.print(", ");
  Serial.print(MashServoSetPoint);
  Serial.print(" MashT: ");
  Serial.print(MashTemp, 2); 
  Serial.println();

  // Echo to Android controls
  sendSerial1(TAG_TEMP_MIX, MixTemp, MixerTempSet);
  sendSerial1(TAG_TEMP_MASH, MashTemp);
  sendSerial1(TAG_FLOW_MIX, MixFlowRate, MixerFlowSet);
  sendSerial1(TAG_FLOW_MASH, MashFlowRate, MashFlowSet);
  sendSerial1(TAG_VOLUME_MIX, MixVolume);
  sendSerial1(TAG_VOLUME_MASH, MashVolume);
}

// -------------------------------------------------------------------------------
// Serial & USB Input Poll
void Timer1ISR() {
  int RxFrameIndex = 0;
  if (Serial.available()) {
    char inByte = 0;
    while (Serial.available()) {
      inByte = Serial.read();
      RxBuffer[RxFrameIndex] = inByte;
      RxFrameIndex++;
      // Too many bytes in serial buffer.  Flush and reset
      if (RxFrameIndex > 5) {
        RxFrameIndex = -1;
        Serial.println("Extra Bytes in Serial buffer, purging");
        while (Serial.available()) {
          inByte = Serial.read();
        }
      } else {
        char tempBuffer[4];
        tempBuffer[0] = RxBuffer[1];
        tempBuffer[1] = RxBuffer[2];
        tempBuffer[2] = RxBuffer[3];
        tempBuffer[3] = RxBuffer[4];
        if (RxBuffer[0] == 'f') {
          MixerFlowSet = String(tempBuffer).toFloat();
        } else if (RxBuffer[0] == 't') {
          MixerTempSet = String(tempBuffer).toFloat();
        } else if (RxBuffer[0] == 'm') {
          MashFlowSet = String(tempBuffer).toFloat();
        } else if (RxBuffer[0] == 'r') {
          MixVolume = 0;
          MashVolume = 0;
        }
      }
    }
  }

   // Check input on Serial1 (Android)
  char cmdString[64];
  memset(cmdString, 0, 64);
  bool runparser = false;

  if (Serial1.available() > 0)
  {
    int size = Serial1.readBytes(cmdString, 64);
    runparser = (size > 0);

    Serial.print("CMD:");
    Serial.println(cmdString);

    if (runparser)
    {
      // Space delimited
      char* cmd = strtok(cmdString, " ");
      while(cmd != 0)
      {
        // Pick off value
        char* val = strtok(NULL, " ");
        float value = 0.0f;
        
        if (cmd)
        {
          value = atof(val);
        
          if (strcmp(cmd, "t-mix") == 0)
          {
            MixerTempSet = value;
          }
          else if (strcmp(cmd, "f-mix") == 0)
          {
            MixerFlowSet = value;
          }
          else if (strcmp(cmd, "f-msh") == 0)
          {
            MashFlowSet = value;
          }
          else if (strcmp(cmd, "v-mix") == 0)
          {
            MixVolume = value;
          }
          else if (strcmp(cmd, "v-msh") == 0)
          {
            MashVolume = value;
          }

          // Ignore simulator time
          cmd = strtok(NULL, " ");

          // Next cmd
          cmd = strtok(NULL, " ");
        }  
        
      }
  }
    
  }
  
}

// -------------------------------------------------------------------------------
// Periodic Servo Update
void Timer3ISR() {
  ServoLastTime = ServoCurrentTime;
  ServoCurrentTime = millis();
  int MashCorrectedPulseCount = (int)MashPulseCount * MASH_FLOW_CORRECTION;
  int MixerCorrectedPulseCount = (int)MixerPulseCount * MIXER_FLOW_CORRECTION;
  MashPulseCount = 0;
  MixerPulseCount = 0;
  float TimeDelta = ServoCurrentTime - ServoLastTime;
  float FractionOfMinute = 60000 / TimeDelta;

  // Catch micros() roll over. Could cause significant glitch.
  if (ServoCurrentTime < ServoLastTime) {
    // Use previous timeDelta on roll over
    Serial.println(">>> NOTE: MICROS() Roll Over"); 
  } else {
    ServoTimeDelta = (ServoCurrentTime - ServoLastTime);
  }
  // --------------------
  // Mash Pulse Count
  MashFlowRatePrevious = MashFlowRate;
  MashFlowRate = 0.8 * (MashCorrectedPulseCount * FLOW_PER_TICK * FractionOfMinute) + 0.2 * MashFlowRatePrevious;

  float tempCorrection = 0.7 - MashFlowRate;
  if (tempCorrection > 0) {
    MashFlowRate = MashFlowRate * (1 - tempCorrection * 0.3);
    MashVolume += (int)(MashCorrectedPulseCount * (1 - tempCorrection * 0.3)) * FLOW_PER_TICK;
  } else {
    MashVolume += MashCorrectedPulseCount * FLOW_PER_TICK;
  }

  // ---------------------
  // Mash Flow Code
  MashFlowPIDOutput = mashFlowPID.getCmdStep(MashFlowSet, MashFlowRate, ServoTimeDelta);
  MashServoSetPoint = (int)(MASH_SERVO_MAX - ((MashFlowPIDOutput / MASH_MAX_FLOW_SET) * MASH_SERVO_THROW)); 
  if (MashFlowSet == 0.0) {
    MashServoSetPoint = MASH_SERVO_MAX; 
    mashFlowPID.reset();
  }

  if (MashServoSetPoint == MASH_SERVO_MAX) {
    MashServoRelax += 1;
    if (MashServoRelax > SERVO_RELAX_TIME) {
      MashServoRelax = SERVO_RELAX_TIME;
      MashServoSetPoint = MASH_SERVO_MAX - SERVO_RELAX_OFFSET;
    } 
  } else if (MashServoSetPoint == MASH_SERVO_MIN) {
    MashServoRelax += 1;
    if (MashServoRelax > SERVO_RELAX_TIME) {
      MashServoRelax = SERVO_RELAX_TIME;
      MashServoSetPoint = MASH_SERVO_MIN + SERVO_RELAX_OFFSET;
    } 
  } else {
    MashServoRelax = 0;
  }
 
  pwm.setPWM(2, 0, MashServoSetPoint);

  // -----------------------------------------
  // Mixer Code
  MixFlowRatePrevious = MixFlowRate;
  MixFlowRate = 0.8 * (MixerCorrectedPulseCount * FLOW_PER_TICK * FractionOfMinute) + 0.2 * MixFlowRatePrevious;
  
  tempCorrection = 0.7 - MixFlowRate;
  if (tempCorrection > 0) {
    MixFlowRate = MixFlowRate * (1 - tempCorrection * 0.3);
    MixVolume += (int)(MixerCorrectedPulseCount * (1 - tempCorrection * 0.3)) * FLOW_PER_TICK;
  } else {
    MixVolume += MixerCorrectedPulseCount * FLOW_PER_TICK;
  }

  MixerFlowPIDOutput = mixerFlowPID.getCmdStep(MixerFlowSet, MixFlowRate, ServoTimeDelta);
  // Temp Control PID Calculation
  // Temp PID is a ratio adjustment based on temp delta from -1 (full cold), to +1 (full hot)
  Temp1PIDOutput = temp1PID.getCmdStep(MixerTempSet, MixTemp, (ServoTimeDelta / 1000));
  // If TempPID rails, reset error so integral doesn't over accumulate error
  if (abs(MixTemp - MixerTempSet) >= TEMP_INT_CUTOFF) {
    temp1PID.reset();
  }


  if (MixerFlowSet == 0.0) {
    MixerServo0SetPoint = MIXER_SERVO_0_MIN; // Hot valve is Servo 0
    MixerServo1SetPoint = MIXER_SERVO_1_MAX; // Cold valve is Servo 1
    mixerFlowPID.reset();
    temp1PID.reset();
  } else {     
    if (Temp1PIDOutput > 0) {
      // Reduce cold
      temp1Set = MixerFlowPIDOutput * (1 - abs(Temp1PIDOutput));
      temp0Set = MixerFlowPIDOutput;
    } else if (Temp1PIDOutput < 0) {
      // Reduce hot
      temp0Set = MixerFlowPIDOutput * (1 - abs(Temp1PIDOutput));
      temp1Set = MixerFlowPIDOutput;
    } else {
      temp0Set = MixerFlowPIDOutput;
      temp1Set = MixerFlowPIDOutput;
    }
    
    // Cold Servo
    MixerServo0SetPoint = (int)(MIXER_SERVO_0_MIN + ((temp0Set / MIXER_MAX_FLOW_SET) * MIXER_SERVO_THROW)); 
    // Hot Servo
    MixerServo1SetPoint = (int)(MIXER_SERVO_1_MAX - ((temp1Set / MIXER_MAX_FLOW_SET) * MIXER_SERVO_THROW));
  }

  if (MixerServo0SetPoint == MIXER_SERVO_0_MIN) {
    Servo0Relax += 1;
    if (Servo0Relax > SERVO_RELAX_TIME) {
      Servo0Relax = SERVO_RELAX_TIME;
      MixerServo0SetPoint = MIXER_SERVO_0_MIN + SERVO_RELAX_OFFSET;
    } 
  } else {
    Servo0Relax = 0;
  }

  if (MixerServo1SetPoint == MIXER_SERVO_1_MAX) {
    Servo1Relax += 1;
    if (Servo1Relax > SERVO_RELAX_TIME) {
      Servo1Relax = SERVO_RELAX_TIME;
      MixerServo1SetPoint = MIXER_SERVO_1_MAX - SERVO_RELAX_OFFSET;
    } 
  } else {
    Servo1Relax = 0;
  }
  
  pwm.setPWM(0, 0, MixerServo0SetPoint);
  pwm.setPWM(1, 0, MixerServo1SetPoint);


}

// -------------------------------------------------------------------------------
// Main Loop


void loop() {
  // Anytime to fire once pre loop here.
  while(true) {
    tempSensors.requestTemperatures();
    MixTemp = tempSensors.getTempCByIndex(0);
    MashTemp = thermocouple.readCelsius();
    delay(50);
  }
}
