

#define TFT_DC  15
#define TFT_CS 10

#include "SPI.h"
#include <Wire.h>
#include "ILI9341_t3.h"
#define ScreenColor ILI9341_BLACK

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);


#define XA_OFFSET_H      0x06 
#define YA_OFFSET_H      0x08
#define ZA_OFFSET_H      0x0A
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43


#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define MPU6050_ADDRESS  0x68
#define RR               0  //  0=RR, 1=RL, 2=FR, 3=FL
#define RL               1
#define FR               2
#define FL               3



enum Action{
  LAY,
  LRC,
  STAND,
  LIFTLEGS,
  PEE,
  TEST,
  POINT,
  WALK,
  SIT,
  WAVE,
  WAIT,
  PREWALK,
  JUMP,
  PUSHUP,
};

struct Point {
  int x;
  int y;
};

struct Leg {
  int x;
  int y;
  int startX;
  int startY;
  int targetX;
  int targetY;
  int dur;
  int moveType;
  int height;
  int currComp;
  int targetComp;
  int twistComp;
  long startTime;
};

struct LegCmd{
  int targetX;
  int targetY;
  int moveType;
};

struct Stage{
  LegCmd legCmd[4];
  int dur;
  float tPitch;
  float tRoll;
  int steerAng;
  int headAng;
  boolean repeat;
  int autolevel; //0=off, 1=fullRange, 2=halfRange
};


struct Command {
  Action cmd;
  int dur;
};

// global variables for accelerometer
int Gscale = 0;
int Ascale = 0;
float aRes = 2.0/32768.0; // scale resolutions per LSB for the sensors
float gRes = 250.0/32768.0;
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer

// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float pitch, yaw, roll;
float Pitch, Roll;
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion


// servo pin declarations
const int servoRRTop = 4;
const int servoRRBottom = 6;  
const int servoRLTop = 3;     
const int servoRLBottom = 5;  
const int servoFRTop = 23;    
const int servoFRBottom = 20; 
const int servoFLTop = 22;    
const int servoFLBottom = 21; 
const int servoHead = 9;
const int servoSteer = 25;
const int MPUInt = 2;
const int eyeTrig = 16;
const int eyeEcho = 17;

// minor servo trim adjustments, less than 1 tooth on gear. each tooth=3
float trimRRTop = 2;
float trimRRBottom = 6;
float trimRLTop = -22;
float trimRLBottom = 22;
float trimFRTop = -26;
float trimFRBottom = 0;
float trimFLTop = -28;
float trimFLBottom = -16;
float trimRangeRRBottom = 4.45;
float trimRangeRLBottom = 4.10;
float trimRangeFRBottom = 4.35;
float trimRangeFLBottom = 4.10;
float trimRangeRRTop = 4.00;
float trimRangeRLTop = 4.16;
float trimRangeFRTop = 4.12;
float trimRangeFLTop = 4.06;
int headCenterVal = 684;
int centerVal = 614;
int zeroVal = 414;


// global variables for range calculations
long startTime = 0;
long elapsedTime = 0;
boolean dataReady = false;


// global variables for range servo sweeping
const int radarPts = 5;  // number of points on each side of center. Center is always included.
const int sweepRange = 280;  //head servo position max value
const int sampleSize = 6;  //number of samples to use at each position
const int sweepTimeout = 40;  //maximum time(ms) to wait for radar return
const int rangeLimit = 10;  //maximum value allowed of radar return
float cAng = 0;    //stores angle of radar direction, used for drawing map
int headPos = 0;   //position in radar[] array
int headDir = 1;   //direction to move along Pos[] while sweeping
int headStage = 0;  //increment through each minor step of measuring the range at each Pos[]
uint32_t sampleAvg = 0;  //the average value of samples taken at each head Pos[]
int sampleCount = 0;   //counter to use correct number of samples
long radar[radarPts*2+1];      // current values of radar map
long headStartTime = 0;   //used to measure delays
long motionDelay = (200/radarPts);   //time to wait for servo movement
        

// drawing variables
Point legPoints[4][4];    //points on leg for drawing
Point radarPoints[radarPts*2+1] ;


// program commmand variables
Leg legs[5];  //legs[4] = waist servo
Command commands[15];
Stage stage[15];
String cmdStr = "";
String nextCmdStr = "";
long actionStartTime = 0;
bool actionComplete = true;
int stageCount;
int stageIdx;
int currentCommand = 0;
bool sweepOn = false;
int autolevelOn = 0;  //0=off, 1=fullRange, 2=tightRange
bool walking = false;
int rideHeight = 250;

// timer variables
int stageTime = 0; //timer to wait for each movement to complete
long TimeNow = 0; // marks current time in various functions
int drawTimer = 0;   //counter used to determine when to draw to tft
long TrigTimer = 0; //timer to produce a 10us pulse for eyes




//***************************************************SETUP**********************************************
void setup() {
    
    //startup program
    commands[0].cmd = LAY;
    commands[0].dur = 0;
    commands[1].cmd = WAIT;
    commands[1].dur = 1000;
    commands[2].cmd = SIT;
    commands[2].dur = 0;
    commands[3].cmd = WAVE;
    commands[3].dur = 0;
    commands[4].cmd = JUMP;
    commands[4].dur = 0;
    commands[5].cmd = STAND;
    commands[5].dur = 0;
    commands[6].cmd = LIFTLEGS;
    commands[6].dur = 0;
    commands[7].cmd = LRC;
    commands[7].dur = 1000;
    commands[8].cmd = WAIT;
    commands[8].dur = 500;
    commands[9].cmd = PEE;
    commands[9].dur = 500;
    commands[10].cmd = PREWALK;
    commands[10].dur = 0;
    commands[11].cmd = WALK;
    commands[11].dur = 0;
    
    //override for testing
    
    commands[0].cmd = STAND;
    commands[0].dur = 0;
    commands[1].cmd = WAIT;
    commands[1].dur = 1000;
    commands[2].cmd = JUMP;
    commands[2].dur = 10000;
    
    commands[3].cmd = STAND;
    commands[3].dur = 500000;
    commands[4].cmd = WAIT;
    commands[4].dur = 5000000;
    
 
    
    // define servo pins
    pinMode(servoRRTop, OUTPUT);
    pinMode(servoRRBottom, OUTPUT);
    pinMode(servoRLTop, OUTPUT);
    pinMode(servoRLBottom, OUTPUT);
    pinMode(servoFRTop, OUTPUT);
    pinMode(servoFRBottom, OUTPUT);
    pinMode(servoFLTop, OUTPUT);
    pinMode(servoFLBottom, OUTPUT);
    pinMode(servoHead, OUTPUT);
    pinMode(servoSteer, OUTPUT);


    // set the PWM frequency to the range of a R/C servo, 50 is normal, 100 should work fine (more resolution)
    analogWriteFrequency(servoRRTop, 100);
    analogWriteFrequency(servoRRBottom, 100);
    analogWriteFrequency(servoRLTop, 100);
    analogWriteFrequency(servoRLBottom, 100);
    analogWriteFrequency(servoFRTop, 100);
    analogWriteFrequency(servoFRBottom, 100);
    analogWriteFrequency(servoFLTop, 100);
    analogWriteFrequency(servoFLBottom, 100);
    analogWriteFrequency(servoHead, 100); 
    analogWriteFrequency(servoSteer, 100); 
    analogWriteResolution(12);

    // setup the HC-SR04 Range Finder
    pinMode(eyeTrig, OUTPUT);
    pinMode(eyeEcho, INPUT);
    digitalWrite(eyeTrig, LOW);
    attachInterrupt(eyeEcho, pulseEnd, FALLING);
    
    
    // start the tft screen
    tft.begin();
    tft.fillScreen(ScreenColor);
    tft.setRotation(1);
    tft.fillRect(200,0, 119, 239, ILI9341_BLUE);
    tft.setCursor(220,5);
    tft.print("Roll");
    tft.setCursor(275,5);
    tft.print("Pitch");
    tft.setCursor(210,60);
    tft.print("Heading");
    tft.fillRect(205,180, 110, 55, ILI9341_WHITE);
          
    
    // setup the MPU
    Wire.begin();
    pinMode(MPUInt, INPUT);
    digitalWrite(MPUInt, LOW);
    attachInterrupt(MPUInt, readMPU, RISING);
    calibrateMPU(gyroBias, accelBias);   // Calibrate gyro and accelerometers, load biases in bias registers  
    initMPU6050();     // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // set initial values
    analogWrite(servoHead, headCenterVal);
    for (int i = 0; i < 4; i++) {
      legs[i].x = 0;
      legs[i].y = rideHeight;
      legs[i].height = rideHeight;
      legs[i].twistComp = 0;
    }
    legs[4].x = centerVal;
}


//***************************Interrupt Routines********************************


//   range finder signal return, mark time and return
void pulseEnd() {
  elapsedTime = millis() - startTime;
  dataReady = true;  
}
 
//Gyro acquired new values, Quickly grab the data and return
void readMPU() {
   if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // reset interrupt
     readAccelData(accelCount);  // Read the x/y/z adc values
     readGyroData(gyroCount);  // Read the x/y/z adc values
   }
}

//********************************LOOP*******************************************



void loop() {

    //calibrateDown();
    //calibrateUp();
    //calibrateCenter();

    //calculate the current yaw, pitch, and roll
    processGyro();

    //loads the next command into stage[] buffer for execution
    loadCommand();

    //executes each move command at the correct intervals
    executeCommand(); 

    // adjust the height compensation of the legs to maintain level attitude
    autolevel();
    
    //calculates and moves servos in small increments to keep sync
    moveServo();



    // radar routine to build radar map
    if(sweepOn) sweep();

    // print out position data to screen
    if ( ++drawTimer > 10) {
      drawTimer = 0;
      drawGyroData();
    }


}


float adjPitch;
float adjRoll;
float targetRoll, targetPitch;
float rollTrim = -0.035;
float pitchTrim = -0.01; 
float compRate = 4.0;


void autolevel() {
  float avgRoll, avgPitch;
  int bound;
  //calculate new value
  avgRoll = (ay + rollTrim + targetRoll)*compRate;
  avgPitch = (ax + pitchTrim + targetPitch)*compRate * 1.5;
  Roll = avgRoll*20;
  Pitch = avgPitch*20;

  if(autolevelOn > 0) {
  
    if(abs(avgPitch) > 0.3) avgPitch *= 1.2;
    if(abs(avgRoll) > 0.5) avgRoll *= 1.4;
    if(avgRoll > 4) avgRoll = 4; //prevent death wobble
    if(avgRoll < -4) avgRoll = -4; //prevent death wobble
    
  
    /*
    if((abs(avgRoll) < 0.0005) && (abs(avgPitch) < 0.05)) {
      q[0] = 1;
      q[1] = 0;
      q[2] = 0;
      q[3] = 0;
    }
    */
    
    adjRoll += avgRoll;
    adjPitch += avgPitch;
    
    bound = (autolevelOn==1)? 50:40;    //stay within motion bounds
    if (adjRoll > bound) adjRoll = bound;
    if (adjRoll < -bound) adjRoll = -bound;
    if (adjPitch > bound) adjPitch = bound;
    if (adjPitch < -bound) adjPitch = -bound;
  
    legs[RR].currComp = adjRoll - adjPitch;
    legs[FR].currComp = adjRoll + adjPitch;
    legs[RL].currComp = -adjPitch - adjRoll;
    legs[FL].currComp = adjPitch - adjRoll;
  } else {
    legs[RR].currComp = 0;
    legs[FR].currComp = 0;
    legs[RL].currComp = 0;
    legs[FL].currComp = 0;
  }
}


void drawGyroData(){

      // Roll and Pitch boxes
      tft.fillRect(205, 15, 50,30, ILI9341_WHITE);
      tft.fillRect(265, 15, 50,30, ILI9341_WHITE);
      tft.setTextSize(3);
      tft.setTextColor(ILI9341_BLACK);
      if((int)Roll < 0) tft.fillRect(208,30,8,3,ILI9341_BLACK);
      if((int)Pitch < 0) tft.fillRect(268,30,8,3,ILI9341_BLACK);
      tft.setCursor(220, 20);
      tft.print(abs((int)Roll));
      tft.setCursor(280,20);
      tft.print(abs((int)Pitch));

      // Heading
      tft.fillCircle(230,90,20,ILI9341_WHITE);
      tft.setTextSize(1);
      tft.setTextColor(ILI9341_BLACK);
      tft.setCursor(225,88);     
      tft.print(-(int)yaw);
      float yawCos = cos(yaw * (PI / 180));
      float yawSin = sin(yaw * (PI / 180));
      tft.drawLine(230 - (yawSin *10),90 - (yawCos * 10),230 - (yawSin * 19), 90 - (yawCos * 19), ILI9341_RED);
      
      //command
      tft.fillRect(70,10,129,20,ScreenColor);
      tft.setCursor(70,10);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      tft.print(cmdStr);
      tft.setTextSize(1);
      
}

/*
 * set zero trim for Left side, trim range for right side
 */
void calibrateDown() { //check center, Legs down
      analogWrite(servoRRTop, centerVal + trimRRTop);
      analogWrite(servoRRBottom, zeroVal + (90 * trimRangeRRBottom) + trimRRBottom); 
      analogWrite(servoRLTop, centerVal + trimRLTop);
      analogWrite(servoRLBottom, zeroVal + trimRLBottom); 
      analogWrite(servoFRTop, centerVal + trimFRTop);
      analogWrite(servoFRBottom, zeroVal + (90 * trimRangeFRBottom) + trimFRBottom); 
      analogWrite(servoFLTop, centerVal + trimFLTop);
      analogWrite(servoFLBottom, zeroVal + trimFLBottom); 
      analogWrite(servoHead, headCenterVal);
      analogWrite(servoSteer, centerVal);
      
}
/*
 * set zero trim for Right side, trim range for left side
 */
void calibrateUp() {  //zero angle for bottom, top at 45 deg down, legs up
      analogWrite(servoRRTop, centerVal - (90 * trimRangeRRTop) + trimRRTop);
      analogWrite(servoRRBottom, zeroVal + trimRRBottom); 
      analogWrite(servoRLTop, centerVal + (90 * trimRangeRLTop) + trimRLTop);
      analogWrite(servoRLBottom, zeroVal + (90 * trimRangeRLBottom) + trimRLBottom); 
      analogWrite(servoFRTop, centerVal - (90 * trimRangeFRTop) + trimFRTop);
      analogWrite(servoFRBottom, zeroVal + trimFRBottom); 
      analogWrite(servoFLTop, centerVal + (90 * trimRangeFLTop) + trimFLTop);
      analogWrite(servoFLBottom, zeroVal + (90 * trimRangeFLBottom) + trimFLBottom); 
      analogWrite(servoHead, headCenterVal);
      analogWrite(servoSteer, centerVal);
      
}
void calibrateCenter() {
       setSteer(centerVal, 1); 
       analogWrite(servoSteer, centerVal);
       for (int i = 0; i < 4; i++) {
        moveArm(i, 0, 250);
       }
     
       
}


void moveServo() {
  TimeNow = millis();
  for (int leg = 4; leg >= 0; leg--) {
    
    float progress, subProgress;
    long timePassed = TimeNow - legs[leg].startTime;
    progress = (float)timePassed / (float)legs[leg].dur;   //the percentage of progress completion from 0 to 1
    
    if (progress > 1) {
      // action has been comleted
      legs[leg].x = legs[leg].targetX;
      legs[leg].y = legs[leg].targetY;
    } else {
      int distX, distY, yVal;
      switch (legs[leg].moveType) {
        case 1:          //linear movement from point start(x,y) to target(x,y)
          distX = (legs[leg].targetX - legs[leg].startX) * progress;
          distY = (legs[leg].targetY - legs[leg].startY) * progress;
          legs[leg].x = legs[leg].startX + distX;
          legs[leg].y = legs[leg].startY + distY;
          break;
        case 2:  //a step movement, up->forward->down
          if (progress < 0.15) {  //move foot down 20
            subProgress = progress * 6.667;
            distY = 10 * subProgress;
            legs[leg].y = (legs[leg].startY + distY);
          } else {
            
            if (progress < 0.4) {  //move foot up to (startX, 155), remove currComp
              subProgress = (progress-.15) * 4;
              distY = (legs[leg].startY - 155 + legs[leg].currComp) * subProgress;
              legs[leg].y = (legs[leg].startY - distY);
            } else {
              if (progress < 0.55) {  //move to (0,155), no currComp
                subProgress = (progress - 0.4) * 6.667;
                distX = legs[leg].startX * subProgress;
                legs[leg].x = legs[leg].startX - distX;
                
              } else {
                if (progress < 0.7) {  //move forward and up along arc, no currComp
                  subProgress = (progress - 0.55) * 6.667;
                  distX = (legs[leg].targetX * 0.75) * subProgress;
                  legs[leg].x = distX;
                  legs[leg].y = sqrt(24025 - (distX * distX)) - legs[leg].currComp;
                  
                } else {  //move to (targetX,targetY) along arc, include currComp again
                  subProgress = (progress - 0.7) * 3.333;
                  distX = sin(subProgress * (PI/2)) * (legs[leg].targetX/2);
                  float startY = sqrt(24025 - ((legs[leg].targetX * 0.75)*(legs[leg].targetX * 0.75)));
                  distY = sin(subProgress * (PI/2)) * (legs[leg].targetY - startY + legs[leg].currComp);
                  legs[leg].x = distX + (legs[leg].targetX * 0.75);
                  legs[leg].y = startY + distY - legs[leg].currComp;
                
                }
              }
            }
          }
          break;
        case 3:  //lift and place
          distX = (legs[leg].targetX - legs[leg].startX) * progress;
          distY = (legs[leg].targetY - legs[leg].startY) * progress;
          legs[leg].x = legs[leg].startX + distX;
          legs[leg].y = legs[leg].startY + distY - (80*sin(PI*progress));
          break;
      }
    }

    if (leg == 4) {
      int twist = (0.18)*(legs[4].x - centerVal);
      legs[RL].twistComp = -twist;
      legs[FR].twistComp = -twist;
      legs[FL].twistComp = twist;
      legs[RR].twistComp = twist;

      analogWrite(servoSteer, legs[4].x);  //38 center
    } else {
       
      moveArm(leg, (legs[leg].x + legs[leg].twistComp), (legs[leg].y + legs[leg].currComp));
    }
  }

}


/*
 * Array Helper Functions for easily loading values
 */

void setLeg(int leg, int x, int y, int dur, int type) {
  legs[leg].targetX = x;
  legs[leg].targetY = y;
  legs[leg].startX = legs[leg].x;
  legs[leg].startY = legs[leg].y;
  legs[leg].dur = dur;
  legs[leg].moveType = type;
  legs[leg].startTime = millis();
}

void setSteer(int x, int dur){
  legs[4].targetX = x;
  legs[4].targetY = 0;
  legs[4].startX = legs[4].x;
  legs[4].startY = 0;
  legs[4].dur = dur;
  legs[4].moveType = 1;
  legs[4].startTime = millis();
}

void loadStageLeg(int sIdx, int legIdx,  int tX, int tY, int type) {
  stage[sIdx].legCmd[legIdx].targetX = tX;
  stage[sIdx].legCmd[legIdx].targetY = tY;
  stage[sIdx].legCmd[legIdx].moveType = type;
}
/*
 * sIdx - index of stage[] to load into
 * dur - amount of time(ms) that the movement should take
 * tPitch - target pitch value to balance on
 * tRoll - target roll value to balance on
 * steerAng - raw servo value for the waist angle. centerVal is center
 * headAng - raw servo value for head servo. headServoVal is center
 * repeating - true will cause next movement to reset to movement 0 of current command.
 * level - sets autolevel state. 
 */
void loadStageVar(int sIdx, int dur, float tPitch, float tRoll, int steerAng, int headAng, boolean repeating, int level) {
  stage[sIdx].dur = dur;
  stage[sIdx].tPitch = tPitch;
  stage[sIdx].tRoll = tRoll;
  stage[sIdx].steerAng = steerAng;
  stage[sIdx].headAng = headAng;
  stage[sIdx].repeat = repeating;
  stage[sIdx].autolevel = level;
}


/*
 * This function will wait until the current movement has completed, based on time elapsed, and then load the
 * next movement of the current command into memory. Once all movements have been completed,
 * it sets the actionComplete flag to true.
 */
void executeCommand() {
  TimeNow = millis();
  if (TimeNow > (actionStartTime + stageTime)) {
    actionStartTime = TimeNow;
    stageTime = stage[stageIdx].dur;
    cmdStr = nextCmdStr;
    for (int i = 0; i < 4; i++){
      setLeg(i, stage[stageIdx].legCmd[i].targetX, stage[stageIdx].legCmd[i].targetY, 
                                stage[stageIdx].dur, stage[stageIdx].legCmd[i].moveType);
    }
    setSteer(stage[stageIdx].steerAng, stage[stageIdx].dur);
    targetPitch = stage[stageIdx].tPitch;
    targetRoll = stage[stageIdx].tRoll;
    autolevelOn = stage[stageIdx].autolevel;
    if(stage[stageIdx].headAng == 0){
      sweepOn = true;
    } else {
      analogWrite(servoHead, stage[stageIdx].headAng);
      sweepOn = false;
    }
    if (stage[stageIdx].repeat) stageIdx = -1;
  
    if (++stageIdx == stageCount) {
      stageIdx = 0;
      actionComplete = true;
    }
  }
}

/*
 * This function will load the next command into memory. This is done by loading each stage of 
 * movements into the stage[] buffer. Global variable 'stageCount' marks size of buffer for command. 
 * Resets the actionComplete to false.
 */
void loadCommand() {
  if (actionComplete) {
    actionComplete = false;
 
    int stepHi, step2, step1, stepLo;
    float rollAmt, pitchAmt;
    stepHi = 150;
    stepLo = -150;
    step2 = (stepHi - ((stepHi-stepLo)/3));
    step1 = (stepHi - 2*((stepHi-stepLo)/3));
    
    Command cmd = commands[currentCommand];
    currentCommand++;
    switch (cmd.cmd) {
      case TEST:
        nextCmdStr = "Test";
        stageCount = 2;
        rollAmt = 0.1;
        pitchAmt = 0.08;
        
        loadStageLeg(0, RR, stepHi, rideHeight, 1);  //stepping
        loadStageLeg(0, RL, step1, rideHeight, 1);
        loadStageLeg(0, FR, stepLo, rideHeight, 1);
        loadStageLeg(0, FL, step2, rideHeight, 1);  
        loadStageVar(0, 2500, pitchAmt, rollAmt, centerVal-150, 0, false, 2);
  
        loadStageLeg(1, RR, step2, rideHeight, 1);
        loadStageLeg(1, RL, stepLo, rideHeight, 1);
        loadStageLeg(1, FR, stepHi, rideHeight, 2); //stepping
        loadStageLeg(1, FL, step1, rideHeight, 1);  
        loadStageVar(1, 2500, pitchAmt, rollAmt, centerVal+150, 0, true, 2);

        break;

      case JUMP:
        nextCmdStr = "Jump";
        stageCount = 9;
        loadStageLeg(0, RR, 300, 200, 1);
        loadStageLeg(0, RL, 300, 200, 1);
        loadStageLeg(0, FR, stepLo, rideHeight, 1);
        loadStageLeg(0, FL, stepLo, rideHeight, 1);  
        loadStageVar(0, 1000, 0, 0, centerVal, 0, false, 0);

        loadStageLeg(1, RR, 100, 350, 1);
        loadStageLeg(1, RL, 100, 350, 1);
        loadStageLeg(1, FR, stepLo, rideHeight, 1);
        loadStageLeg(1, FL, stepLo, rideHeight, 1);  
        loadStageVar(1, 500, 0, 0, centerVal, 0, false, 0);
        
        loadStageLeg(2, RR, 100, 350, 1);
        loadStageLeg(2, RL, 100, 350, 1);
        loadStageLeg(2, FR, 300, 100, 1);
        loadStageLeg(2, FL, 300, 100, 1);  
        loadStageVar(2, 500, 0, 0, centerVal, 0, false, 0);

//        arm stuff
        loadStageLeg(3, RR, 100, 350, 1);
        loadStageLeg(3, RL, 100, 350, 1);
        loadStageLeg(3, FR, 150, 250, 1);
        loadStageLeg(3, FL, 300, 100, 1);  
        loadStageVar(3, 500, 0, 0, centerVal, 0, false, 0);

        loadStageLeg(4, RR, 100, 350, 1);
        loadStageLeg(4, RL, 100, 350, 1);
        loadStageLeg(4, FR, 300, 100, 1);
        loadStageLeg(4, FL, 150, 250, 1);  
        loadStageVar(4, 500, 0, 0, centerVal, 0, false, 0);
        
        loadStageLeg(5, RR, 100, 350, 1);
        loadStageLeg(5, RL, 100, 350, 1);
        loadStageLeg(5, FR, 150, 250, 1);
        loadStageLeg(5, FL, 300, 100, 1);  
        loadStageVar(5, 500, 0, 0, centerVal, 0, false, 0);

        loadStageLeg(6, RR, 100, 350, 1);
        loadStageLeg(6, RL, 100, 350, 1);
        loadStageLeg(6, FR, 300, 100, 1);
        loadStageLeg(6, FL, 150, 250, 1);  
        loadStageVar(6, 500, 0, 0, centerVal, 0, false, 0);
        

//      back down

        loadStageLeg(7, RR, 300, 100, 1);
        loadStageLeg(7, RL, 300, 100, 1);
        loadStageLeg(7, FR, stepHi, rideHeight, 1);
        loadStageLeg(7, FL, stepHi, rideHeight, 1);  
        loadStageVar(7, 1000, 0, 0, centerVal, 0, false, 0);

        for (int i = 0; i < 4; i++)
        loadStageLeg(8, i, 0, rideHeight, 1);
        loadStageVar(8, 1000, 0, 0, centerVal, 0, false, 1);


  
        break;

      case PUSHUP:
        nextCmdStr = "Pushup";
        stageCount = 5;
        for (int i = 0; i < 4; i++){
          loadStageLeg(0, i, 0, 155, 1); 
          loadStageLeg(1, i, 0, rideHeight+100, 1); 
          loadStageLeg(2, i, 0, 155, 1); 
          loadStageLeg(3, i, 0, rideHeight+100, 1); 
          loadStageLeg(4, i, 0, rideHeight, 1); 
          
        }
        loadStageVar(0, 500, 0, 0, centerVal, 0, false, 0);
        loadStageVar(1, 500, 0, 0, centerVal, 0, false, 0);
        loadStageVar(2, 500, 0, 0, centerVal, 0, false, 0);
        loadStageVar(3, 500, 0, 0, centerVal, 0, false, 0);
        loadStageVar(4, 500, 0, 0, centerVal, 0, false, 1);

        break;

  
      case PREWALK:
        nextCmdStr = "Pre-walk";
        stageCount = 8;
    
        //***************Front Right**********************
        loadStageLeg(0, RR, step2, rideHeight, 1);
        loadStageLeg(0, RL, step2, rideHeight, 1);
        loadStageLeg(0, FR, step2, rideHeight, 1);
        loadStageLeg(0, FL, step2, rideHeight, 1);
        loadStageVar(0, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);

        loadStageLeg(1, RR, step2, rideHeight, 1);
        loadStageLeg(1, RL, step2, rideHeight, 1);
        loadStageLeg(1, FR, stepHi, 150, 1);
        loadStageLeg(1, FL, step2, rideHeight, 1);
        loadStageVar(1, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);

        loadStageLeg(2, RR, step2, rideHeight, 1);
        loadStageLeg(2, RL, step2, rideHeight, 1);
        loadStageLeg(2, FR, stepHi, rideHeight, 1);
        loadStageLeg(2, FL, step2, rideHeight, 1);
        loadStageVar(2, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);
        
        //***************Front Left**********************
        loadStageLeg(3, RR, step2, rideHeight, 1);
        loadStageLeg(3, RL, step2, rideHeight, 1);
        loadStageLeg(3, FR, stepHi, rideHeight, 1);
        loadStageLeg(3, FL, step2, rideHeight, 1);
        loadStageVar(3, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(4, RR, step2, rideHeight, 1);
        loadStageLeg(4, RL, step2, rideHeight, 1);
        loadStageLeg(4, FR, stepHi, rideHeight, 1);
        loadStageLeg(4, FL, step1, 150, 1);
        loadStageVar(4, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(5, RR, step2, rideHeight, 1);
        loadStageLeg(5, RL, step2, rideHeight, 1);
        loadStageLeg(5, FR, stepHi, rideHeight, 1);
        loadStageLeg(5, FL, step1, rideHeight, 1);
        loadStageVar(5, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);
        
        
        //*****************Rear Left*********************
        loadStageLeg(6, RR, step1, rideHeight, 1);
        loadStageLeg(6, RL, step1, rideHeight, 1);
        loadStageLeg(6, FR, step2, rideHeight, 1);
        loadStageLeg(6, FL, stepLo, rideHeight, 1);
        loadStageVar(6, 500, -0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(7, RR, step1, rideHeight, 1);
        loadStageLeg(7, RL, stepHi, rideHeight, 2);
        loadStageLeg(7, FR, step2, rideHeight, 1);
        loadStageLeg(7, FL, stepLo, rideHeight, 1);
        loadStageVar(7, 500, -0.015, -0.05, centerVal+280, headCenterVal, false,2);

        break;
      
      case WALK:
        nextCmdStr = "Walk";
        stageCount = 4;
        
        rollAmt = 0.1;
        pitchAmt = 0.08;
        loadStageLeg(0, RR, stepLo, rideHeight, 1);
        loadStageLeg(0, RL, step2, rideHeight, 1);
        loadStageLeg(0, FR, step1, rideHeight, 1);
        loadStageLeg(0, FL, stepHi, rideHeight, 2);  //stepping
        loadStageVar(0, 500, pitchAmt, -rollAmt, centerVal-150, 0, false, 2);
  
        loadStageLeg(1, RR, stepHi, rideHeight, 2);  //stepping
        loadStageLeg(1, RL, step1, rideHeight, 1);
        loadStageLeg(1, FR, stepLo, rideHeight, 1);
        loadStageLeg(1, FL, step2, rideHeight, 1);  
        loadStageVar(1, 500, -pitchAmt, rollAmt, centerVal-150, 0, false, 2);
  
        loadStageLeg(2, RR, step2, rideHeight, 1);
        loadStageLeg(2, RL, stepLo, rideHeight, 1);
        loadStageLeg(2, FR, stepHi, rideHeight, 2); //stepping
        loadStageLeg(2, FL, step1, rideHeight, 1);  
        loadStageVar(2, 500, pitchAmt, rollAmt, centerVal+150, 0, false, 2);
  
        loadStageLeg(3, RR, step1, rideHeight, 1);
        loadStageLeg(3, RL, stepHi, rideHeight, 2); //stepping
        loadStageLeg(3, FR, step2, rideHeight, 1);
        loadStageLeg(3, FL, stepLo, rideHeight, 1);  
        loadStageVar(3, 500, -pitchAmt, -rollAmt, centerVal+150, 0, true, 2);
        break;
      
      case POINT:
        nextCmdStr = "Point";
        stageCount = 2;
        
        loadStageLeg(0, RR, 150,180, 1);
        loadStageLeg(0, RL, 150, 180, 1);
        loadStageLeg(0, FR, stepHi, rideHeight, 1);
        loadStageLeg(0, FL, 350, 50, 1);
        loadStageVar(0, 500, 0, 0, centerVal, 0, false, 0);

        loadStageLeg(1, RR, 0, rideHeight, 1);
        loadStageLeg(1, RL, 0, rideHeight, 1);
        loadStageLeg(1, FR, 0, rideHeight, 1);
        loadStageLeg(1, FL, 0, rideHeight, 1);
        loadStageVar(1, 500, 0, 0, centerVal, 0, false, 1);
        break;
        
      case WAIT:
        nextCmdStr = "Wait";
        stageCount = 1;
        for (int i = 0; i < 4; i++)
          loadStageLeg(0, i, legs[i].targetX, legs[i].targetY, 1);
        loadStageVar(0, cmd.dur, targetPitch, targetRoll, legs[4].targetX, (sweepOn? 0:headCenterVal), false, autolevelOn);
        break;
      
      case LAY:
        nextCmdStr = "Lay";
        stageCount = 1;
        loadStageLeg(0, RR, 200, 120, 1);
        loadStageLeg(0, RL, 200, 120, 1);
        loadStageLeg(0, FR, 200, 80, 1);
        loadStageLeg(0, FL, 200, 80, 1);
        loadStageVar(0, 1000, 0, 0, centerVal, headCenterVal, false, 0);
        break;
        
      case LRC:
        nextCmdStr = "LRC";
        stageCount = 5;
        for (int sIdx = 0; sIdx < stageCount; sIdx++) {
          for (int legIdx = 0; legIdx < 4; legIdx++) {
            loadStageLeg(sIdx, legIdx, 0, rideHeight, 1);
          }
        }
        loadStageVar(0, 300, 0, 0, centerVal, headCenterVal, false, 1);
        loadStageVar(1, 500, 0, 0, centerVal-280, headCenterVal, false, 1);
        loadStageVar(2, 500, 0, 0, centerVal+280, headCenterVal, false, 1);
        loadStageVar(3, 500, 0, 0, centerVal-280, headCenterVal, false, 1);
        loadStageVar(4, 300, 0, 0, centerVal, headCenterVal, false, 1);
        break;
        
      case STAND:
        nextCmdStr = "Stand";
        stageCount = 1;
        for (int i = 0; i < 4; i++)
          loadStageLeg(0, i, 0, rideHeight, 1);
        loadStageVar(0, 1000, 0, 0, centerVal, 0, false, 1);
        break;
        
      case LIFTLEGS:
        nextCmdStr = "Lift Legs";
        stageCount = 13;
        
        //******************Rear Right***************
        loadStageLeg(0, RR, step1, rideHeight, 1);
        loadStageLeg(0, RL, step1, rideHeight, 1);
        loadStageLeg(0, FR, step1, rideHeight, 1);
        loadStageLeg(0, FL, step1, rideHeight, 1);
        loadStageVar(0, 500, -0.015, 0.05, centerVal-280, headCenterVal+280, false,2);

        loadStageLeg(1, RR, 0, 150, 1);
        loadStageLeg(1, RL, step1, rideHeight, 1);
        loadStageLeg(1, FR, step1, rideHeight, 1);
        loadStageLeg(1, FL, step1, rideHeight, 1);
        loadStageVar(1, 500, -0.015, 0.05, centerVal-280, headCenterVal+280, false,2);

        loadStageLeg(2, RR, step1, rideHeight, 1);
        loadStageLeg(2, RL, step1, rideHeight, 1);
        loadStageLeg(2, FR, step1, rideHeight, 1);
        loadStageLeg(2, FL, step1, rideHeight, 1);
        loadStageVar(2, 500, -0.015, 0.05, centerVal-280, headCenterVal+280, false,2);
        
        //*****************Rear Left*********************
        loadStageLeg(3, RR, step1, rideHeight, 1);
        loadStageLeg(3, RL, step1, rideHeight, 1);
        loadStageLeg(3, FR, step1, rideHeight, 1);
        loadStageLeg(3, FL, step1, rideHeight, 1);
        loadStageVar(3, 500, -0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(4, RR, step1, rideHeight, 1);
        loadStageLeg(4, RL, 0, 150, 1);
        loadStageLeg(4, FR, step1, rideHeight, 1);
        loadStageLeg(4, FL, step1, rideHeight, 1);
        loadStageVar(4, 500, -0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(5, RR, step1, rideHeight, 1);
        loadStageLeg(5, RL, step1, rideHeight, 1);
        loadStageLeg(5, FR, step1, rideHeight, 1);
        loadStageLeg(5, FL, step1, rideHeight, 1);
        loadStageVar(5, 500, -0.015, -0.05, centerVal+280, headCenterVal, false,2);
        
        //***************Front Left**********************
        loadStageLeg(6, RR, step2, rideHeight, 1);
        loadStageLeg(6, RL, step2, rideHeight, 1);
        loadStageLeg(6, FR, step2, rideHeight, 1);
        loadStageLeg(6, FL, step2, rideHeight, 1);
        loadStageVar(6, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(7, RR, step2, rideHeight, 1);
        loadStageLeg(7, RL, step2, rideHeight, 1);
        loadStageLeg(7, FR, step2, rideHeight, 1);
        loadStageLeg(7, FL, 0, 150, 1);
        loadStageVar(7, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);

        loadStageLeg(8, RR, step2, rideHeight, 1);
        loadStageLeg(8, RL, step2, rideHeight, 1);
        loadStageLeg(8, FR, step2, rideHeight, 1);
        loadStageLeg(8, FL, step2, rideHeight, 1);
        loadStageVar(8, 500, 0.015, -0.05, centerVal+280, headCenterVal, false,2);

        //***************Front Right**********************
        loadStageLeg(9, RR, step2, rideHeight, 1);
        loadStageLeg(9, RL, step2, rideHeight, 1);
        loadStageLeg(9, FR, step2, rideHeight, 1);
        loadStageLeg(9, FL, step2, rideHeight, 1);
        loadStageVar(9, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);

        loadStageLeg(10, RR, step2, rideHeight, 1);
        loadStageLeg(10, RL, step2, rideHeight, 1);
        loadStageLeg(10, FR, 0, 150, 1);
        loadStageLeg(10, FL, step2, rideHeight, 1);
        loadStageVar(10, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);

        loadStageLeg(11, RR, step2, rideHeight, 1);
        loadStageLeg(11, RL, step2, rideHeight, 1);
        loadStageLeg(11, FR, step2, rideHeight, 1);
        loadStageLeg(11, FL, step2, rideHeight, 1);
        loadStageVar(11, 500, 0.015, 0.05, centerVal-280, headCenterVal, false,2);

        //**************STAND********************
        loadStageLeg(12, RR, 0, rideHeight, 1);
        loadStageLeg(12, RL, 0, rideHeight, 1);
        loadStageLeg(12, FR, 0, rideHeight, 1);
        loadStageLeg(12, FL, 0, rideHeight, 1);
        loadStageVar(12, 500, 0.0, 0.0, centerVal, headCenterVal, false,1);
        break;
    
      case SIT:
        nextCmdStr = "Sit";
        stageCount = 1;
        loadStageLeg(0, RR, 200, 180, 1);
        loadStageLeg(0, RL, 200, 180, 1);
        loadStageLeg(0, FR, 0, 350, 1);
        loadStageLeg(0, FL, 0, 350, 1);
        loadStageVar(0, 1000, 0, 0, centerVal, headCenterVal, false, 0);
        break;
      
      case WAVE:
        nextCmdStr = "Wave";
        stageCount = 6;
        for (int sIdx = 0; sIdx < stageCount; sIdx++) {
          loadStageLeg(sIdx, RR, 150, 180, 1);
          loadStageLeg(sIdx, RL, 150, 180, 1);
          loadStageLeg(sIdx, FL, -50, 350, 1);
          loadStageVar(sIdx, 500, 0, 0, centerVal, headCenterVal + 280, false, 0);
        }
        loadStageLeg(0, FR, 150, 100, 1);
        loadStageLeg(1, FR, 150, 200, 1);
        loadStageLeg(2, FR, 150, 100, 1);
        loadStageLeg(3, FR, 150, 200, 1);
        loadStageLeg(4, FR, 150, 100, 1);
        loadStageLeg(5, FR, legs[FR].targetX, legs[FR].targetY, 1);
        break;

      case PEE:
        nextCmdStr = "Pee";
        stageCount = 5;
        
        loadStageLeg(0, RR, stepLo, rideHeight, 1);
        loadStageLeg(0, RL, stepLo, rideHeight, 1);
        loadStageLeg(0, FR, stepLo, rideHeight, 1);
        loadStageLeg(0, FL, stepLo, rideHeight, 1);
        loadStageVar(0, 500, -0.1, 0.15, centerVal, headCenterVal, false,2);

        loadStageLeg(1, RR, stepHi, 50, 2);
        loadStageLeg(1, RL, stepLo, rideHeight, 1);
        loadStageLeg(1, FR, stepLo, rideHeight, 1);
        loadStageLeg(1, FL, stepLo, rideHeight, 1);
        loadStageVar(1, 1000, -0.1, 0.15, centerVal, headCenterVal, false,2);

        loadStageLeg(2, RR, stepHi, 50, 1);
        loadStageLeg(2, RL, stepLo, rideHeight, 1);
        loadStageLeg(2, FR, stepLo, rideHeight, 1);
        loadStageLeg(2, FL, stepLo, rideHeight, 1);
        loadStageVar(2, 1500, -0.1, 0.15, centerVal, headCenterVal, false,2);

        loadStageLeg(3, RR, stepLo, rideHeight, 1);
        loadStageLeg(3, RL, stepLo, rideHeight, 1);
        loadStageLeg(3, FR, stepLo, rideHeight, 1);
        loadStageLeg(3, FL, stepLo, rideHeight, 1);
        loadStageVar(3, 500, -0.1, 0.15, centerVal, headCenterVal, false,2);


        loadStageLeg(4, RR, 0, rideHeight, 1);
        loadStageLeg(4, RL, 0, rideHeight, 1);
        loadStageLeg(4, FR, 0, rideHeight, 1);
        loadStageLeg(4, FL, 0, rideHeight, 1);
        loadStageVar(4, 500, 0, 0, centerVal, headCenterVal, false,2);


    } 
  } 
}





void sweep() {

  switch (headStage) {
    case 0:
      //send servo command
      analogWrite(servoHead, (headCenterVal + headPos*(sweepRange / radarPts)));
      startTime = millis();
      headStage = 1; break;
      
    case 1:
      //wait for servo to move
      TimeNow = millis();
      if (TimeNow > (startTime + motionDelay)) headStage = 2; 
      break;
     
    case 2:
      //set trig to high
      digitalWrite(eyeTrig, HIGH);
      dataReady = false;
      startTime = micros();
      headStage = 3; 
      break;
      
    case 3:
      // wait 10us and drop trigger
      if (micros() >  (startTime + 10)) {
        digitalWrite(eyeTrig, LOW);
        startTime = millis();
        headStage = 4; 
      }
      break;
    case 4:
      
      if (millis() > (startTime + sweepTimeout)){
        elapsedTime = sweepTimeout;
        dataReady = true;
      }
      // data is ready, repeat to obtain samples and use the average
      if (dataReady) {
        if (sampleCount++ < sampleSize) {
          sampleAvg += elapsedTime;
          headStage = 2;
        } else {
          sampleAvg /= sampleSize;
          radar[headPos+radarPts] = sampleAvg;
          if(sampleAvg > rangeLimit) sampleAvg = rangeLimit; //reduces radar range to this limit, (switch to const int)
          sampleCount = 0; //reset for next cycle
          
          // erase old lines
          for (int i = 0; i < (radarPts*2); i++){
            tft.drawLine(260 + radarPoints[i].x, 235 - radarPoints[i].y, 
                    260 + radarPoints[i+1].x, 235 - radarPoints[i+1].y, ILI9341_WHITE);
          }
          tft.drawLine(260, 235, 260 + sin(cAng)*50, 235-cos(cAng)*50, ILI9341_WHITE);
          
          //calculate new value
          cAng = headPos*(60.0/radarPts)*(PI/180);
          radarPoints[headPos+radarPts].x = (int)(sin(cAng)*sampleAvg*(50/rangeLimit)); 
          radarPoints[headPos+radarPts].y = (int)(cos(cAng)*sampleAvg*(50/rangeLimit));
          
          // draw new lines
          for (int i = 0; i < (radarPts*2); i++){
            tft.drawLine(260 + radarPoints[i].x, 235 - radarPoints[i].y, 
                    260 + radarPoints[i+1].x, 235 - radarPoints[i+1].y, ILI9341_BLACK);
          }
          tft.drawLine(260, 235, 260 + sin(cAng)*50, 235-cos(cAng)*50, ILI9341_RED);


          // adjust headPos to scan back and forth
          headPos+=headDir;
          if(abs(headPos) == radarPts)
            headDir = -headDir;
          

            
          headStage = 0;
        }
        break;
      }
      
  }
  
}

void processGyro() {
    ay = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ax = (float)accelCount[1]*aRes; // - accelBias[1];   
    az = (float)accelCount[2]*aRes; // - accelBias[2];  
    gy = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gx = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   

   //IMU algorithm
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
    
}







// moves the leg servos to the correct servo angles to place the foot exactly
// at coordinate x,y
// scaled up by 100, measured in inches, displacement from the shoulder joint
// armIdx 0=RR, 1=RL, 2=FR, 3=FL

void moveArm(int armIdx, int x, int y) {
  float dist, dist2, armDeg, shoDeg, ElbowX, ElbowY, scaleX, scaleY;
  float tenDeg, tendonX, tendonY, servoX, servoY, servoDeg;

  //set scale size for drawing to screen
  float legScale = 0.2;
  int legSize = (int)(100.0 * legScale);

  //determine position on screen, (which quadrant)
  int yPos = (armIdx % 2 == 0)? 150 : 50;
  int xPos = (armIdx < 2)? 50 : 150;

  // Erase old leg
  tft.drawLine(legPoints[armIdx][0].x, legPoints[armIdx][0].y, legPoints[armIdx][1].x, legPoints[armIdx][1].y, ScreenColor);
  tft.drawLine(legPoints[armIdx][0].x, legPoints[armIdx][0].y, xPos, yPos, ScreenColor);
  tft.fillRect(legPoints[armIdx][1].x, legPoints[armIdx][1].y, 60 * legScale, 20 * legScale, ScreenColor);
  tft.drawLine(legPoints[armIdx][2].x, legPoints[armIdx][2].y, legPoints[armIdx][3].x, legPoints[armIdx][3].y, ScreenColor);
  tft.drawLine(legPoints[armIdx][2].x, legPoints[armIdx][2].y, xPos + (80.0 * legScale), yPos - (80.0 * legScale), ScreenColor);

  // do all the math....law of cosines
  scaleX = ((float)x / 100);
  scaleY = ((float)y / 100);
  dist = sqrt(scaleX*scaleX + scaleY*scaleY);
  armDeg = acos((4.29 + (dist*dist)) / (5 * dist)); //Angle of forearm, part1 Shoulder to Elbow// law of cosines
  armDeg += atan2(scaleX, scaleY);                  //Angle of forearm, part2 0 to Shoulder

  ElbowX = scaleX - (sin(armDeg)*2.5);
  ElbowY = scaleY - (cos(armDeg)*2.5);
  tendonX = scaleX - (sin(armDeg)*1.8);
  tendonY = scaleY - (cos(armDeg)*1.8);
  shoDeg = atan2(ElbowY, -ElbowX);  //Angle from 0,0 to ElbowX,ElbowY is shoulder angle

  dist2 = sqrt(pow(0.8 - tendonX, 2) + pow(0.8 + tendonY, 2));
  tenDeg = acos((5.7903 + (dist2*dist2)) / (5 * dist2));    //angle of tendon 
  tenDeg += atan2(0.8 - tendonX, 0.8 + tendonY);
  servoX = tendonX + (sin(tenDeg) * 2.5);
  servoY = tendonY - (cos(tenDeg) * 2.5);
  servoDeg = atan2(servoY + 0.8, servoX - 0.8);

  // calculate each point of the leg for drawing it to the screen
  legPoints[armIdx][0].x = xPos + (ElbowX * legSize);
  legPoints[armIdx][0].y = yPos + (ElbowY * legSize);
  legPoints[armIdx][1].x = xPos + (x * legScale);
  legPoints[armIdx][1].y = yPos + (y * legScale);
  legPoints[armIdx][2].x = xPos + (servoX * legSize);
  legPoints[armIdx][2].y = yPos + (servoY * legSize);
  legPoints[armIdx][3].x = xPos + (tendonX * legSize);
  legPoints[armIdx][3].y = yPos + (tendonY * legSize);

  // now draw leg on screen
  tft.drawLine(legPoints[armIdx][0].x, legPoints[armIdx][0].y, legPoints[armIdx][1].x, legPoints[armIdx][1].y, ILI9341_RED);
  tft.drawLine(legPoints[armIdx][0].x, legPoints[armIdx][0].y, xPos, yPos, ILI9341_RED);
  tft.fillRect(legPoints[armIdx][1].x, legPoints[armIdx][1].y, 60 * legScale, 20 * legScale, ILI9341_RED);
  tft.drawLine(legPoints[armIdx][2].x, legPoints[armIdx][2].y, legPoints[armIdx][3].x, legPoints[armIdx][3].y, ILI9341_BLUE);
  tft.drawLine(legPoints[armIdx][2].x, legPoints[armIdx][2].y, xPos + (80.0 * legScale), yPos - (80.0 * legScale), ILI9341_BLUE);

  // write the correct value to each servo pin and write Label on screen
  tft.setCursor(xPos - 10, yPos - 10);
  if (drawTimer == 0) tft.fillRect(xPos - 10, yPos - 10, 30, 10, ScreenColor);

  switch(armIdx) {
    case 0:
      analogWrite(servoRRTop, centerVal + trimRRTop - (int)(servoDeg * (180.0/PI) * trimRangeRRTop));
      analogWrite(servoRRBottom, zeroVal + trimRRBottom + (int)(shoDeg * (180.0/PI) * trimRangeRRBottom)); 
      if (drawTimer == 0) tft.print("RR"); 
      break;
    case 1:
      analogWrite(servoRLTop, centerVal + trimRLTop + (int)(servoDeg * (180.0/PI) * trimRangeRLTop));
      analogWrite(servoRLBottom, zeroVal + trimRLBottom + (int)((90.0 - (shoDeg * (180.0/PI))) * trimRangeRLBottom)); 
      if (drawTimer == 0) tft.print("RL"); 
      break;
    case 2:
      analogWrite(servoFRTop, centerVal + trimFRTop - (int)(servoDeg * (180.0/PI) * trimRangeFRTop));
      analogWrite(servoFRBottom, zeroVal + trimFRBottom + (int)(shoDeg * (180.0/PI) * trimRangeFRBottom));
      if (drawTimer == 0) tft.print("FR");
      break;
    case 3:
      analogWrite(servoFLTop, centerVal + trimFLTop + (int)(servoDeg * (180.0/PI) * trimRangeFLTop));
      analogWrite(servoFLBottom, zeroVal + trimFLBottom + (int)((90.0 - (shoDeg * (180.0/PI))) * trimRangeFLBottom));
      if (drawTimer == 0) tft.print("FL");

      break;
  }

}

static inline int8_t sign(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

//******************************************GYRO/ACCELEROMETER CODE******************************************************

void initMPU6050()
{  
 // Initialize MPU6050 device

  // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz sample rate 
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x02);    
   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}


void calibrateMPU(float * dest1, float * dest2) {  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  Serial.print("1");
  delay(100);  

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00); 
  delay(200);
  
  // Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
  }
 
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
  dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
  dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void readAccelData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void readGyroData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx=0, gbiasy=0, gbiasz=0;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;

    // Normalise accelerometer measurement
    ax += pitchTrim;  //add back the bias for true level
    ay += rollTrim;
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
