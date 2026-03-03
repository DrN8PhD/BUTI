/***************************************************
 *       ITSY BITSY M4 WITH CUSTOM PCB             *
 *     1.8" LCD TFT ST7735 DISPLAY 128X160.        *
 *             REVISED 19FEB2026 NRT               *
 *   FOR USE WITH SOFTWARE INTERFACE BY OJVR       *  
 ***************************************************/

#include <Wire.h>
#include "bitmaps.h"
#include "FlashStorage.h"
#include "stepper.h"
#include "avr/dtostrf.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "FreeSansBold7pt7b.h"
#include "FreeSansBold8pt7b.h"
#include "FreeSansBold9pt7b.h"
#include <Adafruit_ST7735.h>  // Hardware-specific library

/*ADC Setup*/
ADS1115 ads;
#define txdx1 0

//TFT Setup
#define TFT_CS A4
#define TFT_DC A3
#define TFT_RST 13  // Or set to -1 and connect to Arduino RESET pin
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

//Rotary Encoder Setup and interrups
#define pinA 10
#define pinB 11
#define enSW 12
volatile byte aFlag = 0;        //  let's us know when we're expecting a rising edge on pinDT to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;        //  let's us know when we're expecting a rising edge on pinCLK to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
int encoderPos;                 //  this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile uint32_t reading = 0;  //  somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
uint32_t maskA;
uint32_t maskB;
uint32_t maskAB;
volatile uint32_t *port;
bool box;
int runState = 0;
int captureChoice;
int count;
float tempPos;

void fpinA() {
  noInterrupts();
  reading = *port & maskAB;
  if ((reading == maskAB) && aFlag) {  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos--;
    box = !box;
    bFlag = 0;  //reset flags for the next turn
    aFlag = 0;  //reset flags for the next turn
  } else if (reading == maskB)
    bFlag = 1;
  interrupts();  //signal that we're expecting pinCLK to signal the transition to detent from free rotation
}
void fpinB() {
  noInterrupts();
  reading = *port & maskAB;
  if (reading == maskAB && bFlag) {  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos++;
    box = !box;
    bFlag = 0;  //reset flags for the next turn
    aFlag = 0;  //reset flags for the next turn
  } else if (reading == maskA)
    aFlag = 1;
  interrupts();  //signal that we're expecting pinDT to signal the transition to detent from free rotation
}

//Stepper Setup
float stepAngle = 1.8;  //in deg. Change depending on stepper; usually 0.9 or 1.8 degress
float acmeLead = 8;     //in mm. Change based on acme lead; 8 mm is common. This is NOT pitch; that's space between threads.
float stepDistance;
int stepsDeform;
float selPreload = 0.000;
Stepper stepper(0, 9, 7, 5, 2, 3);
int zeroing;
bool home = false;

//Parameters for constant rate and manual step configurations
float deform;
int GapTime;
int FwdGapTime;
int RevGapTime;
int expType;
int cycleCount = 0;
int cycleCountTemp = 0;
int counter = 0;
float tracker;  //This tracks the distance traveled after preload in ConstVelCycle
unsigned long previousTimer = 0;
int step = 0;
unsigned long currentTimer;
float conTension;
bool counts = true;
char receivedChar;
bool newData;
int armType;
float hiCounter;
float diamCounter;

//Lighting Pins and camera
#define CamTrig A2
#define ledPin 13
int interval = 10;  //time for the camera to trigger high
int cameraFrame = 0;

//FORCE sensor calibration an setup
int CST_FORCE_IN_MIN = 0;            // Raw value calibration lower point
int CST_FORCE_IN_MAX = 65535;         // Raw value calibration upper point
double CST_FORCE_OUT_MIN = 0.00;     // FORCE calibration lower point
double CST_FORCE_OUT_MAX = 1000.00;  // FORCE calibration upper point. This is in mN. This is about the max it can do.
double CAL_FORCE_MIN = 0.00;
double CAL_FORCE_MAX = 5.00;

/*Flash storage definitions and matrices*/
struct cal_matrix {
  bool valid;
  float _lowSel;
  float _highSel;
  int _lowADC;
  int _highADC;
  float _wireDiam;
  float _preload;
  float _rateFwd;
  float _rateRev;
  int _dfrm;
  int _numCycles;
  int _timeDelay;
  int _numSamples;
  int _filterWeight;
} calib;

float lowSel;
float highSel;
int lowADC;
int highADC;
float wireDiam;
float preload;
float rateFwd;
float rateRev;
int dfrm;
int numCycles;
int timeDelay;                 //  Change this to alter the time delay for wheatstone bridge output. *
int numSamples;                //  Change this to alter the number of samples averaged.              *
int filterWeight;              //  Change this to alter the weighting of the averaging filter.         *


FlashStorage(calibrate, cal_matrix);

unsigned long previousMillis = 0;        //  Resetting the clock to determine how much time has elapsed.
unsigned long currentMillis;
unsigned long startMillis;
bool UseStartTime = true;
char number[8];
char distTravel[8];
char force[6];
char cycles[3];
char time[12];
char serialOutput[36];
char preloadBuff[32];
char defDistBuff[32];
char defPercentBuff[32];
char stepsBuff[32];
char fwdBuff[32];
char revBuff[32];
char numCyclesBuff[32];
char wireBuff[32];
char debugging[64];
double avg;
double currentTime;
float setDef;

void setup() {
  Serial.begin(460800);
  ads.begin();
  ads.setGain(GAIN_ONE);
  pinMode(enSW, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinA, INPUT_PULLUP);
  attachInterrupt(pinA, fpinA, CHANGE);  //Sets interrupt for rotary encoder so that it works with above functions;
  attachInterrupt(pinB, fpinB, CHANGE);  //RISING for the black rotary encoder; CHANGE for blue one.
  maskA = digitalPinToBitMask(pinA);
  maskB = digitalPinToBitMask(pinB);
  maskAB = maskA | maskB;
  port = portInputRegister(digitalPinToPort(pinA));
  stepper.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(CamTrig, OUTPUT);
  digitalWrite(CamTrig, LOW);
  digitalWrite(ledPin, HIGH);
  tft.initR(INITR_GREENTAB);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(false);
  calib = calibrate.read();
  loadingFromFlash();
  
  bootup();
  chooseMode();
  calibration();
  ads.linearCal(lowADC, highADC, lowSel, highSel);
  lighting();
  ZeroChoice();
  preloading();
  delay(200);
  testType();
  cameraCapture();
  clickBegin();
  
  setDef = preload + deform;
  stepDistance = stepper.setStepFrac(8, stepAngle, acmeLead);
  stepsDeform = deform / stepDistance;
  FwdGapTime = (1000000 / (rateFwd / stepDistance));
  RevGapTime = (1000000 / (rateRev / stepDistance));
  printWords(7, 1, 6, 122, ST77XX_RED, "STOPPED");
  tft.drawRect(2, 108, 158, 20, ST77XX_RED);
  rowNames();
}
void loop() {
    if (runState == 0) {
      isRunning();
    }
    else if (runState == 1) {
      isStopping();
    }
    else if (runState == 2) {
      FixDefSteps();
    }
}
/*******************************Utility Functions *******************************/

// void checkingSave() {
//   sprintf(debugging, "lowSel=%0.2f, highSel=%0.2f, lowADC=%d, highADC=%d, wireDiam=%0.2f, preload=%0.3f, rateFwd=%0.3f, rateRev=%0.3f, deform=%d, numCycles=%d", lowSel, highSel, lowADC, highADC, wireDiam, preload, rateFwd, rateRev, dfrm, numCycles);
//   Serial.println(debugging);
// }

void loadingFromFlash() {
  lowSel = calib._lowSel;
  highSel = calib._highSel;
  lowADC = calib._lowADC;
  highADC = calib._highADC;
  wireDiam = calib._wireDiam;
  preload = calib._preload;
  rateFwd = calib._rateFwd;
  rateRev = calib._rateRev;
  dfrm = calib._dfrm;
  numCycles = calib._numCycles;
  timeDelay = calib._timeDelay;
  numSamples = calib._numSamples;
  filterWeight = calib._filterWeight;
}

void writingToFlash() {
  calib._lowSel = lowSel;
  calib._highSel = highSel;
  calib._lowADC = lowADC;
  calib._highADC = highADC;
  calib._wireDiam = wireDiam;
  calib._preload = preload;
  calib._rateFwd = rateFwd;
  calib._rateRev = rateRev;
  calib._dfrm = dfrm;
  calib._numCycles = numCycles;
  calib._timeDelay = timeDelay;
  calib._numSamples = numSamples;
  calib._filterWeight = filterWeight;
}

void autoloading() {
  initScreen();
  int ticker = 0;
  encoderPos = 0;
  char buffer[7];
  char buffer2[7];
  stepDistance = stepper.setStepFrac(8, stepAngle, acmeLead);
  stepper.setStepFrac(8, stepAngle, acmeLead);
  while (digitalRead(enSW)) {
    ticker = stepper.load(1, 50000, 1);
    averaging(numSamples);
    preload = selPreload + (ticker * stepDistance);
    sprintf(buffer, "%.3f ", preload);
    sprintf(buffer2, "%.2f ", avg);
    printWords(0, 2, 80, 87, ST77XX_WHITE, buffer);
    printWords(0, 2, 80, 106, ST77XX_WHITE, buffer2);
  }
  while (digitalRead(enSW) == 0) {
    selPreload = preload;
  }
}

void advancedSettings() {
  tft.fillScreen(ST7735_BLACK);
  initScreenAdv();
  selectArmType();
  selectWireDiam();
  selectFilterWeight();
  selectTimeDelay();
  selectNumSamples();
  calib.valid = true;
  delay(100);
  bootup();
}

void selectArmType() {
  encoderPos = 0;
  const char *startMenu[] = {"Wire", "Jaws" };
  while (digitalRead(enSW)) {
    armType = encoderPos;
    encoderLimit(0,1);
    listBox(99, 27, 50, 14, ST77XX_BLACK);
    printWords(9, 1, 100, 39, ST77XX_WHITE, startMenu[armType]);
  }
  while (digitalRead(enSW) == 0) {
    if (armType == 0) {
      wireDiam = 0.65;
      delay(100);
    }
    if (armType == 1) {
      wireDiam = 0.08;
    }
  printWords(9, 1, 100, 39, 0xfb2c, startMenu[armType]);
  }
}

void selectWireDiam() {
  encoderPos = 0;
  char buffer[6];
  diamCounter = wireDiam;
  while (digitalRead(enSW)) {
    diamCounter = wireDiam + (encoderPos * 0.01);
    sprintf(buffer, " %.2f", diamCounter);
    printWords(0, 2, 100, 47, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    wireDiam = diamCounter * 2;
    printWords(0, 2, 100, 47, 0xfb2c, buffer);
    delay(100);
  }
}

void selectFilterWeight() {
  encoderPos = (filterWeight / 3);
  char buffer[3];
  int filterCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    filterCounter = (encoderPos * 3);
    sprintf(buffer, " %3d", filterCounter);
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    filterWeight = filterCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectTimeDelay() {
  encoderPos = (timeDelay / 5);
  char buffer[4];
  int delayCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    delayCounter = (encoderPos * 5);
    sprintf(buffer, "% 4d", delayCounter);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    timeDelay = delayCounter;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void selectNumSamples() {
  encoderPos = numSamples;
  char buffer[3];
  int sampleCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    sampleCounter = encoderPos;
    sprintf(buffer, " %3d", sampleCounter);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    numSamples = sampleCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void averaging(int q) {
  double avgSample = ads.measure(txdx1);
  for (int i = 0; i < q; i++) {
    avg = avg + (avgSample - avg) / filterWeight;
  }
}

void bootup() {
  int h = 128, w = 160, row, col, buffidx = 0;
  for (row = 0; row < h; row++) {
    for (col = 0; col < w; col++) {
      tft.drawPixel(col, row, pgm_read_word(logo + buffidx));
      buffidx++;
    }
  }
  printWords(0, 1, 120, 112, ST77XX_RED, "v5.0");
  if (calib.valid == false) {
    delay(1000);
    tft.fillRect(0, 100, 160, 28, ST77XX_BLACK);
    printWords(7, 1, 1, 120, ST77XX_YELLOW, "First Complete Setup");
    numSamples = 5;
    timeDelay = 50;
    filterWeight = 9;
    delay(1000);
    advancedSettings();  
  }
}

void calibration() {
  const char *myMenu[] = { "Load", "New", "N/A" };
  encoderPos = 0;
  initScreen();
  while (digitalRead(enSW)) {
    encoderLimit(0, 2);
    listBox(79, 27, 80, 14, ST77XX_BLACK);
    printWords(9, 1, 80, 39, ST77XX_WHITE, myMenu[encoderPos]);
  }
  while (digitalRead(enSW) == 0) {
    printWords(9, 1, 80, 39, 0xfb2c, myMenu[encoderPos]);
  }
  if (encoderPos == 0) {
    if (calib.valid == true) {
      calWords();
      calNums();
      delay(200);
      offset();
      tft.fillScreen(ST77XX_BLACK);
      initScreen();
      printWords(9, 1, 80, 39, 0xfb2c, "Done");
      delay(100);
    }
    else {
      calibration();
    }
  }
  if (encoderPos == 1) {
    delay(250);
    calWords();
    forceLowADC();
    forceLowSel();
    forceHighADC();
    forceHighSel();
    delay(100);
    offset();
    delay(100);
    tft.fillScreen(ST77XX_BLACK);
    calib.valid = true;
    
    initScreen();
    printWords(9, 1, 80, 39, 0xfb2c, "Done");
    delay(1000);
  }
  if (encoderPos == 2) {
    tft.fillScreen(ST77XX_BLACK);
    lowADC = 980;
    lowSel = 0;
    highADC = 1624;
    highSel = 5*9.81;
    calWords();
    calNums();
    offset();
    tft.fillScreen(ST77XX_BLACK);
    initScreen();
    calib.valid = true;
    printWords(9, 1, 80, 39, 0xfb2c, "Done");
  }
  writingToFlash();
}

void chooseMode() {
  const char *modeMenu[] = {"Turn to Select","Run Exp", "Advanced" };
  encoderPos = 0;
  while (digitalRead(enSW)) {
    encoderLimit(0, 2);
    listBox(0, 110, 100, 28, ST77XX_BLACK);
    printWords(7, 1, 7, 120, ST77XX_WHITE, modeMenu[encoderPos]);  
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      bootup();
      chooseMode();
    }
    if (switchChoice == 1) {
      printWords(7, 1, 7, 120, 0xfb2c, modeMenu[encoderPos]);  
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 2) {
          printWords(7, 1, 7, 120, 0xfb2c, modeMenu[encoderPos]);  
      delay(500);
      advancedSettings();
      tft.fillScreen(ST77XX_BLACK);
    }
  }
}

void calNums() {
  printNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, lowADC);
  printNumber(2, 90, 46, ST77XX_WHITE, ST77XX_BLACK, lowSel);
  printNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, highADC);
  printNumber(2, 90, 86, ST77XX_WHITE, ST77XX_BLACK, highSel);
}

void calWords() {
  tft.fillScreen(ST77XX_BLACK);
  printWords(9, 1, 36, 19, 0x64df, "Calibration");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Min ADC:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Sel Min:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max ADC:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Sel Max:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Offset:");
}

void cameraCapture() {
  const char *myString[] = {"None", "Every", "1 in 5", "1 in 10", "1 in 15", "1 in 20", "1 in 25", "1 in 30"};
  encoderPos = 0;
  cameraMenu();
  while (digitalRead(enSW)) {
    encoderLimit(0, 7);
    listBox(80, 46, 80, 18, ST77XX_BLACK);
    printWords(9, 1, 80, 59, ST77XX_WHITE, myString[encoderPos]);
  }    
  while (digitalRead(enSW) == 0) {
    if (encoderPos == 0) {captureChoice = 0;}
    if (encoderPos == 1) {captureChoice = 1;}
    if (encoderPos == 2) {captureChoice = 5;}
    if (encoderPos == 3) {captureChoice = 10;}
    if (encoderPos == 4) {captureChoice = 15;}
    if (encoderPos == 5) {captureChoice = 20;}
    if (encoderPos == 6) {captureChoice = 25;}
    if (encoderPos == 7) {captureChoice = 30;}
    printWords(9, 1, 80, 59, 0xfb2c, myString[encoderPos]);
  }
}

void cameraMenu() {
  tft.fillScreen(ST77XX_BLACK);
  printWords(9, 1, 10, 19, 0x64df, "Camera Settings");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Capture:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  tft.drawFastHLine(2, 104, 158, 0xfe31);
}

void clickBegin() {
  while (digitalRead(enSW)) {
    printWords(8, 1, 36, 118, 0x64df, "Click to Begin");
  }
  while (digitalRead(enSW) == 0) {
    writingToFlash();
    delay(200);
    calibrate.write(calib);
    delay(200);
    tft.fillScreen(ST7735_BLACK);
    delay(200);
    runState = 1;
    UseStartTime = true;
  }
  encoderPos = 0;
}

void constVelocity() {
  tft.fillScreen(ST77XX_BLACK);
  encoderPos = 0;
  printWords(9, 1, 4, 19, 0x64df, "Constant Velocity");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Rate Fwd:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 58, 0x04d3, "Rate Rev:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Deform:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 98, 0x04d3, "# Cyles:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  delay(200);
  SelRateFwd();
  delay(200);
  SelRateRev();
  delay(200);
  SelDeform();
  delay(200);
  SelNumCycles();
  delay(200);
  header();
}

void constTension() {
  tft.fillScreen(ST77XX_BLACK);
  encoderPos = 0;
  printWords(9, 1, 4, 19, 0x64df, "Constant Tension");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Strain");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 58, 0x04d3, "(in mN)");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  delay(200);
  SelTension();
  delay(200);
}
void ConstVelCycle() {
    currentTimer = micros();
  if ((numCycles > 0) && (cycleCountTemp < numCycles)) {
    if (counts == true) {
      if (currentTimer - previousTimer >= FwdGapTime && (tracker <= setDef)) {
        tracker = tracker + stepDistance;
        stepper.move(1, 0, 1);
        previousTimer = currentTimer;
      }
      if (tracker >= setDef) {
        counts = false;
      }
    }
    if (counts == false) {
      if (currentTimer - previousTimer >= RevGapTime && (tracker >= preload)) {
        tracker = tracker - stepDistance;
        stepper.move(1, 0, -1);
        previousTimer = currentTimer;
      }
      if (tracker <= preload) {
        cycleCountTemp++;
        cycleCount++;
        counts = true;
      }
    }
  }
  if (numCycles == 0) {
    if (counts == true) {
      if (currentTimer - previousTimer >= FwdGapTime && (tracker <= setDef)) {
        tracker = tracker + stepDistance;
        stepper.move(1, 0, 1);
        previousTimer = currentTimer;
      }
      if (tracker > setDef) {
        counts = false;
      }
    }
    if (counts == false) {
      if (currentTimer - previousTimer >= RevGapTime && (tracker >= preload)) {
        tracker = tracker - stepDistance;
        stepper.move(1, 0, -1);
        previousTimer = currentTimer;
      }
      if (tracker <= preload) {
        cycleCountTemp++;
        cycleCount++;
        counts = true;
      }
    }
  }
    if ((numCycles > 0) && (cycleCountTemp == numCycles)) {
    cycleCount = numCycles;
    sprintf(cycles, "%d", cycleCount);
    printWords(0, 2, 100, 83, ST77XX_WHITE, cycles);
    tft.fillScreen(ST7735_BLACK);
    runState = 1;
    printWords(7, 1, 6, 122, ST77XX_RED, "STOPPED");
    tft.drawRect(2, 108, 158, 20, ST77XX_RED);
  }
}

void encoderLimit(int min, int max) {
  if (encoderPos <= min) { 
    encoderPos = min; 
  }
  if (encoderPos >= max) {
    encoderPos = max;
  }
}

void FixDefSteps() {
  if (tracker <= setDef) {
    tracker = tracker + stepDistance;
    stepper.move(1, 1, 1);
    hiLow();
    averaging(1);
    sprintf(serialOutput, "%.2f, %d, %.3f, %d, %.3f", currentTime, count, tracker, cycleCount, avg);
    Serial.println(serialOutput);
    previousTimer = currentTimer;
  }
  else if (tracker > setDef) {
    cycleCountTemp++;
    cycleCount++;
    setDef = tracker + deform;
    runState = 0;
  }
}

void forceHighADC() {
  int firstADC = 0;
  int avgHigh;
  uint8_t i;
  int samplesHigh[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesHigh[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgHigh = 0;
    for (i = 0; i < numSamples; i++) {
      avgHigh += samplesHigh[i];
    }
    avgHigh /= numSamples;
    firstADC = avgHigh;
    printNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, firstADC);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    highADC = firstADC;
  }
}

void forceHighSel() {
  encoderPos = 0;
  tempPos = CAL_FORCE_MAX;
  while (digitalRead(enSW)) {
    char buffer[10];
    tempPos = CAL_FORCE_MAX + (encoderPos * 0.1);
    sprintf(buffer, "%.1f ", tempPos);
    printWords(0, 2, 102, 86, ST77XX_WHITE, buffer);
    hiCounter = tempPos * 9.81;
  }
  while (digitalRead(enSW) == 0) {
    highSel = hiCounter;
    delay(200);
  }  
}

void forceLowADC() {
  calWords();
  int firstADC = 0;
  int avgLow;
  uint8_t i;
  int samplesLow[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesLow[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgLow = 0;
    for (i = 0; i < numSamples; i++) {
      avgLow += samplesLow[i];
    }
    avgLow /= numSamples;
    firstADC = avgLow;
    printNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, firstADC);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    lowADC = firstADC;
    delay(200);
  }
}

void forceLowSel() {
  encoderPos = CAL_FORCE_MIN;
  char buffer[10];
  float loCounter;
  while (digitalRead(enSW)) {
    sprintf(buffer, "%d ", encoderPos);
    printWords(0, 2, 102, 46, ST77XX_WHITE, buffer);
    loCounter = encoderPos;
  }
  while (digitalRead(enSW) == 0) {
    lowSel = loCounter;
    printWords(0, 2, 102, 46, ST77XX_WHITE, buffer);
  }  
}

void header() {
  sprintf(preloadBuff, "Preload = %.3f mm", preload);
  sprintf(defDistBuff, "Deform = %.3f mm", deform);
  sprintf(defPercentBuff, "Deform = %d %%", dfrm);
  sprintf(stepsBuff, "# of Steps = %d", stepsDeform);
  sprintf(fwdBuff, "Rate Fwd = %.3f mm/sec", rateFwd);
  sprintf(revBuff, "Rate Rev = %.3f mm/sec", rateRev);
  sprintf(numCyclesBuff, "Cycles = %d", numCycles);
  sprintf(wireBuff, "Wire diameter = %.3f mm", wireDiam);
  String Columns = "Time,Frame,Distance,Cycle,Force";
  Serial.println("----------------------------------------");
  Serial.println(preloadBuff);
  Serial.println(defDistBuff);
  Serial.println(defPercentBuff);
  Serial.println(stepsBuff);
  Serial.println(fwdBuff);
  Serial.println(revBuff);
  Serial.println(numCyclesBuff);
  Serial.println(wireBuff);
  Serial.println("----------------------------------------");
  Serial.println(Columns);
}

void hiLow() {
  int now = interval;
  while (now--) {
    digitalWrite(CamTrig, HIGH);
  }
  digitalWrite(CamTrig, LOW);
}

void homing() {
  currentTimer = micros();
  if (currentTimer - previousTimer >= RevGapTime && (tracker >= preload)) {
        tracker = tracker - stepDistance;
        stepper.move(1, 0, -1);
        previousTimer = currentTimer;
      }
  if (tracker <= preload) {
    switchCount();
    home = false;
  }
}

void initScreen() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calib:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Lights:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Zero:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Preload:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Type:");
}

void initScreenAdv() {
  printWords(9, 1, 6, 19, 0x64df, "Advanced Setup");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Arm Type:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Diam (mm):");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Weighting:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Delay (ms):");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Samples:");
}

void lighting() {
  const char *myString[] = { "On", "Off" };
  encoderPos = 0;
  while (digitalRead(enSW)) {
    encoderLimit(0, 1);
    listBox(80, 46, 80, 14, ST77XX_BLACK);
    printWords(9, 1, 80, 59, ST77XX_WHITE, myString[encoderPos]);
    if (encoderPos == 0) {
      digitalWrite(ledPin, HIGH);
    } 
    else if (encoderPos == 1) {
      digitalWrite(ledPin, LOW);
    }
  }
  while (digitalRead(enSW) == 0) {
    printWords(9, 1, 80, 59, 0xfb2c, myString[encoderPos]);
  }
}

void listBox(uint8_t posX, uint8_t posY, uint8_t wide, uint8_t high, uint16_t fontColor) {
  if (box == true) {
    tft.fillRect(posX, posY, wide, high, fontColor);
    box = false;
  }
}

void offset() {
  float samplesOffset[numSamples];
  float avgSample;
  uint8_t i;
  float tempAvg;
  encoderPos = 0;
  int tempLowADC = lowADC;
  int tempHighADC = highADC;
  while (digitalRead(enSW)) {
    tempLowADC = (lowADC - (encoderPos * 10));
    tempHighADC = (highADC - (encoderPos * 10));
    for (i = 0; i < numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, lowSel, highSel);
      samplesOffset[i] = ads.measure(txdx1);
      delay(10);
    }
    avgSample = 0;
    for (i = 0; i < numSamples; i++) {
      avgSample += samplesOffset[i];
    }
    avgSample /= numSamples;
    tempAvg = avgSample;
    char tempOffset[10];
    sprintf(tempOffset, "%.1f ", tempAvg);
    printWords(0, 2, 90, 106, ST77XX_WHITE, tempOffset);
  }
  while (digitalRead(enSW) == 0) {
    lowADC = tempLowADC;
    highADC = tempHighADC;
  }
}

void printWords(byte font, int fontSize, int posX, int posY, uint16_t fontColor, const char *words) {
  if (font == 9) {
    tft.setFont(&FreeSansBold9pt7b);
  } else if (font == 7) {
    tft.setFont(&FreeSansBold7pt7b);
  } else if (font == 8) {
    tft.setFont(&FreeSansBold8pt7b);
  } else if (font == 0) {
    tft.setFont();
  }
  tft.setTextSize(fontSize);
  tft.setCursor(posX, posY);
  tft.setTextColor(fontColor, ST77XX_BLACK);
  tft.print(words);
}

void printNumber(int fontSize, int posX, int posY, uint16_t fontColor, uint16_t fontBkg, double num) {
  tft.setFont();
  tft.setTextSize(fontSize);
  tft.setTextColor(fontColor, fontBkg);
  tft.setCursor(posX, posY);
  dtostrf(num, 6, 1, number);
  tft.print(number);
}

void rowNames() {
  printWords(9, 1, 2, 19, 0x64df, "Force");
  tft.drawFastHLine(2, 25, 158, 0xfe31);
  printWords(9, 1, 2, 44, 0x64df, "Length");
  tft.drawFastHLine(2, 52, 158, 0xfe31);
  printWords(9, 1, 2, 70, 0x64df, "Time");
  tft.drawFastHLine(2, 79, 158, 0xfe31);
  printWords(9, 1, 2, 96, 0x64df, "Cycle");
}

void preloading() {
  stepDistance = stepper.setStepFrac(8, stepAngle, acmeLead);
  selPreload = preload;
  encoderPos = 0;
  char buffer[7];
  char buffer2[7];
  while (digitalRead(enSW)) {
    int posPreload = encoderPos;
    averaging(numSamples);
    delay(100);
    preload = selPreload + (encoderPos * stepDistance);
    sprintf(buffer, "%.3f ", preload);
    sprintf(buffer2, "%.2f ", avg);
    printWords(0, 2, 80, 87, ST77XX_WHITE, buffer);
    printWords(0, 2, 80, 106, ST77XX_WHITE, buffer2);
    if (posPreload > encoderPos) {
      stepper.move(1, 2, -1);
    } else if (posPreload < encoderPos) {
      stepper.move(1, 2, 1);
    }
    posPreload = encoderPos;
  }
  while (digitalRead(enSW) == 0) {
    tracker = preload;
    delay(100);
    printWords(0, 2, 80, 87, 0xfb2c, buffer);
    printWords(0, 2, 80, 106, ST77XX_BLACK, buffer2);
  }
}

void testType() {
  encoderPos = 0;
  const char *myMenu[] = { "Velocity", "Step", "Strain" };
  while (digitalRead(enSW)) {
    encoderLimit(0,2);
    listBox(79, 105, 80, 23, ST77XX_BLACK);
    printWords(9, 1, 80, 118, ST77XX_WHITE, myMenu[encoderPos]);
    expType = encoderPos;
  }
  while (digitalRead(enSW) == 0) {
    if (expType == 0) {
      Serial.println("Experiment Type: Constant Velocity");
      constVelocity();
    }
    if (expType == 1) {
      Serial.println("Experiment Type: Stress Relaxation");
      Steps();
    }
    if (expType == 2) {
      Serial.println("Experiment Type: Constant Tension");
      constTension();
    }
  }
}

void Steps() {
  tft.fillScreen(ST77XX_BLACK);
  encoderPos = 0;
  printWords(9, 1, 4, 19, 0x64df, "Step Deformation");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Rate Fwd:");
  printWords(9, 1, 90, 39, 0xfb2c, "High");
  tft.drawFastHLine(2, 46, 158, 0xfe31);
  printWords(9, 1, 2, 58, 0x04d3, "Rate Rev:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 82, 0x04d3, "Deform:");
  tft.drawFastHLine(2, 85, 158, 0xfe31);
  delay(200);
  SelRateRev();
  delay(200);
  SelDeform();
  delay(200);
  step = 1;
}

void SelDeform() {
  char buffer[4];
  encoderPos = dfrm;
  while (digitalRead(enSW)) {
    sprintf(buffer, "%u%% ", encoderPos);
    printWords(0, 2, 90, 68, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    printWords(0, 2, 90, 68, 0xfb2c, buffer);
    dfrm = encoderPos;
    deform = preload * (dfrm * 0.01);
    stepsDeform = deform / stepDistance;
  }
}

void SelNumCycles() {
  char buffer[3];
  encoderPos = numCycles;
  while (digitalRead(enSW)) {
    if (encoderPos < 0) {
      encoderPos = 0;
    }
    sprintf(buffer, "%u ", encoderPos);
    printWords(0, 2, 90, 88, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    printWords(0, 2, 90, 88, 0xfb2c, buffer);
    numCycles = encoderPos;
  }
}

void SelRateFwd() {
  char buffer[7];
  float rate;
  encoderPos = 20;
  while (digitalRead(enSW)) {
    if (encoderPos < 0) {
      encoderPos = 0;
    }
    rate = 0.005 * encoderPos;
    sprintf(buffer, "%.3f ", rate);
    printWords(0, 2, 90, 27, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    printWords(0, 2, 90, 27, 0xfb2c, buffer);
    if (rate <= 10.000) {
      rateFwd = rate;
    } else {
      rateFwd = 10.000;
    }
  }
}

void SelRateRev() {
  char buffer[7];
  float rate = rateFwd;
  while (digitalRead(enSW)) {
    rate = 0.005 * encoderPos;
    sprintf(buffer, "%.3f ", rate);
    printWords(0, 2, 90, 48, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    printWords(0, 2, 90, 48, 0xfb2c, buffer);
    if (rate <= 10.000) {
      rateRev = rate;
    } else {
      rateRev = 10.000;
    }
    encoderPos = 0;
  }
}

void strain() {
  stepper.setStepFrac(16, stepAngle, acmeLead);
  averaging(numSamples);
  float tensionDiff = 0.00;
  tensionDiff = (conTension - avg);
  if (tensionDiff <= -5) {
    stepper.move(1, 5, -1);
    tracker += stepDistance;
  } else if (tensionDiff >= 5) {
    stepper.move(1, 5, 1);
    tracker -= stepDistance;
  } else {
  stepper.move(0, 5, -1);
  }
}

void switchCount() {
  count = 0;
  cycleCount = 0;
  sprintf(cycles, "%d", cycleCount);
  printWords(0, 2, 100, 83, ST77XX_RED, cycles);
}

void SelTension() {
  // tft.fillScreen(ST77XX_BLACK);
  encoderPos = 0;
  char buffer[5];
  float ConStrain;
  encoderPos = 0;
  while (digitalRead(enSW)) {
    if (encoderPos < 0) {
      encoderPos = 0;
    }
    ConStrain = encoderPos;
    sprintf(buffer, "%.1f ", ConStrain);
    printWords(0, 2, 90, 27, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    printWords(0, 2, 90, 27, 0xfb2c, buffer);
    if (ConStrain <= 20.0) {
      conTension = ConStrain;
    } else {
      conTension = 20.0;
    }
  }
  step = 2;
}

void ZeroChoice() {
  const char *zero[] = { "No", "Yes" };
  encoderPos = 0;
  while (digitalRead(enSW)) {
    encoderLimit(0, 1);
    if (encoderPos == 0) {
      listBox(80, 66, 80, 14, ST77XX_BLACK);
      printWords(9, 1, 80, 79, ST77XX_WHITE, zero[encoderPos]);
      zeroing = encoderPos;
    }
    if (encoderPos == 1) {
      listBox(80, 66, 80, 14, ST77XX_BLACK);
      printWords(9, 1, 80, 79, ST77XX_WHITE, zero[encoderPos]);
      zeroing = encoderPos;
    }
  }
  while (digitalRead(enSW) == 0) {
    printWords(9, 1, 80, 79, 0xfb2c, zero[zeroing]);
  }
  if (zeroing == 0) {
    if (preload == 0) {
      ZeroSelect();
      autoloading();  
    }
  }
  if (zeroing == 1) {
    ZeroSelect();
    autoloading();
  }
}

double zeroForce() {
  double zeroOffset[numSamples];
  double avgSample;
  double zeroAvg;
  uint8_t i;
  for (i = 0; i < numSamples; i++) {
    ads.linearCal(lowADC, highADC, lowSel, highSel);
    zeroOffset[i] = ads.measure(txdx1);
    delay(10);
  }
  avgSample = 0;
    for (i = 0; i < numSamples; i++) {
    avgSample += zeroOffset[i];
  }
    avgSample /= numSamples;
    zeroAvg = avgSample;
    return zeroAvg;
}

void ZeroSelect() {
  char buffer[7];
  int selZero = 0;
  encoderPos = 0;
  stepDistance = stepper.setStepFrac(8, stepAngle, acmeLead);
  while (digitalRead(enSW)) {
    avg = zeroForce();
    if (selZero > encoderPos) {
      stepper.move(1, 5, -1);
      selZero = encoderPos;
    }
    if (selZero < encoderPos) {
      stepper.move(1, 5, 1);
      selZero = encoderPos;
    }
    sprintf(buffer, "%.2f ", avg);
    printWords(0, 2, 80, 106, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    tft.fillRect(80, 66, 80, 15, ST77XX_BLACK);
    printWords(9, 1, 80, 79, 0xfb2c, "Zeroed");
    selPreload = wireDiam;
  }
}

void recvOneChar() {
 if (Serial.available() > 0) {
 receivedChar = Serial.read();
 newData = true;
 }
}

void showNewData() {
 if (newData == true) {
  if (receivedChar == 'G') {
    listBox(79, 109, 79, 18, ST77XX_BLACK);
    tft.fillScreen(ST7735_BLACK);
    printWords(7, 1, 6, 122, ST77XX_GREEN, "RUNNING");
    tft.drawRect(2, 108, 158, 20, ST77XX_GREEN);
    tft.fillRect(79, 109, 79, 18, ST77XX_BLACK);
    cycleCountTemp = 0;
    runState = 0;
    encoderPos = 0;
    box = !box;
  }
  else if (receivedChar == 'S') {
      runState = 1;
      tft.fillScreen(ST7735_BLACK);
      tft.drawRect(2, 108, 158, 20, ST77XX_RED);
      printWords(7, 1, 6, 122, ST77XX_RED, "STOPPED");
      box = !box;
  }
  else if (receivedChar == 'R') {
    tft.fillScreen(ST7735_BLACK);
    rowNames();
    runState = 0;
    UseStartTime = true;
  }
  else if (receivedChar == 'Z') {
    runState = 2;
  }
 newData = false;
 }
}

/*******************************Core Functions *******************************/
void isRunning() {
  recvOneChar();
  showNewData();
  const char *myMenu[] = { "PAUSE?", "STEP" };
  if (step == 0) {
    ConstVelCycle();
  }
  if (step == 1) {
  }
  if (step == 2) {
    strain();
  }
  if (UseStartTime == true) {
    startMillis = millis();
    UseStartTime = false;
  }
  currentMillis = millis() - startMillis;
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  if (currentMillis - previousMillis >= timeDelay) {
    currentTime = (currentMillis / 1000.00);
    encoderLimit(0, 1);
    listBox(79, 109, 79, 18, ST77XX_BLACK);
    printWords(7, 1, 88, 122, ST77XX_YELLOW, myMenu[encoderPos]);
    averaging(numSamples);
    cameraFrame++;
    sprintf(time, "%.2f", currentTime);
    sprintf(distTravel, "%.2f", tracker);
    sprintf(force, "%.2f", avg);
    sprintf(cycles, "%d", cycleCount);
    printWords(0, 2, 64, 6, ST77XX_WHITE, force);
    printWords(0, 2, 100, 31, ST77XX_WHITE, distTravel);
    printWords(0, 2, 64, 57, ST77XX_WHITE, time);
    printWords(0, 2, 100, 83, ST77XX_WHITE, cycles);
    if (captureChoice > 0 && cameraFrame == captureChoice) {
      count++;
      hiLow();
      cameraFrame = 0;
    }
    sprintf(serialOutput, "%.2f, %d, %.3f, %d, %.3f", currentTime, count, tracker, cycleCount, avg);
    Serial.println(serialOutput);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if ((step == 0) && (switchChoice == 0)) {
      runState = 1;
      tft.fillScreen(ST7735_BLACK);
      printWords(7, 1, 6, 122, ST77XX_RED, "STOPPED");
      tft.drawRect(2, 108, 158, 20, ST77XX_RED);
    } if ((step == 0) && (switchChoice == 1)) {
    } if ((step == 1) && (switchChoice == 0)) {
      runState = 1;
      tft.fillScreen(ST7735_BLACK);
      printWords(7, 1, 6, 122, ST77XX_RED, "STOPPED");
      tft.drawRect(2, 108, 158, 20, ST77XX_RED);
    } if ((step == 1) && (switchChoice == 1)) {
      runState = 2;
    } if ((step == 2) && (switchChoice == 1)) {
    }
  }
}

void isStopping() {
  recvOneChar();
  showNewData();
  const char *myMenu[] = { "UNPAUSE", "HOME", "REPEAT", "RESET" };
  if (home == true) {
    homing();
  }
  currentMillis = millis();
  encoderLimit(0, 3);
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  printWords(7, 1, 88, 122, ST77XX_YELLOW, myMenu[encoderPos]);
  rowNames();
    if (currentMillis - previousMillis >= timeDelay) {
      averaging(numSamples);
      sprintf(force, "%.2f", avg);
      sprintf(distTravel, "%.2f", tracker);
      printWords(0, 2, 64, 6, ST77XX_RED, force);
      printWords(0, 2, 100, 31, ST77XX_RED, distTravel);
      printWords(0, 2, 50, 57, ST77XX_RED, "      --- ");
      printWords(0, 2, 100, 83, ST77XX_RED, cycles);
      previousMillis = currentMillis;
    }
  while (digitalRead(enSW) == 0) {
    if (encoderPos == 0) {
      tft.fillRect(79, 109, 79, 18, ST77XX_BLACK);
      cycleCountTemp = 0;
      runState = 0;
      tft.fillScreen(ST7735_BLACK);
      printWords(7, 1, 6, 122, ST77XX_GREEN, "RUNNING");
      tft.drawRect(2, 108, 158, 20, ST77XX_GREEN);
      rowNames();
    }
    else if (encoderPos == 1) {
      home = true;
      UseStartTime = true;
      tft.fillRect(79, 109, 79, 18, ST77XX_BLACK);
    }
    else if (encoderPos == 2) {
      cycleCount = 0;
      cycleCountTemp = 0;
      runState = 0;
      UseStartTime = true;
      setDef = preload + deform;
      tft.fillScreen(ST7735_BLACK);
      printWords(7, 1, 6, 122, ST77XX_GREEN, "RUNNING");
      tft.drawRect(2, 108, 158, 20, ST77XX_GREEN);
      rowNames();
    }
    else if (encoderPos == 3) {
      delay(100);
      NVIC_SystemReset();
    }
  }
}
