/*
 * Alex Vassallo
 */

#include "SPI.h"
#include "ILI9341_t3.h"
#include "SdFat.h"
#include "SDFatUtil.h"


#define ScreenColor ILI9341_BLACK

// For the Adafruit shield, these are the default.
#define TFT_DC  9
#define TFT_CS 10
#define SD_CS 2

#define SELECT_PIN 21
#define MODE_PIN 22
#define SAMPLES_PER_FILE 60


ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);


Sd2Card card;
uint32_t cardSizeBlocks;
uint16_t cardCapacityMB;

// cache for SD block
cache_t cache;

// MBR information
uint8_t partType;
uint32_t relSector;
uint32_t partSize;

// Fake disk geometry
uint8_t numberOfHeads;
uint8_t sectorsPerTrack;

// FAT parameters
uint16_t reservedSectors;
uint8_t sectorsPerCluster;
uint32_t fatStart;
uint32_t fatSize;
uint32_t dataStart;

// constants for file system structure
uint16_t const BU16 = 128;
uint16_t const BU32 = 8192;

//  strings needed in file system structures
char noName[] = "NO NAME    ";
//char fat16str[] = "FAT16   ";
char fat32str[] = "FAT32   ";

const int sampleSize = 200;
const int sampleBufSize = 4;  //some code needs to be changed still to accept this parameter
int boxWidth = 8;  //# of pixels for each box

// Upon DMA complete, immediately copied over from the buffer (dataBuf) that DMA was filling into a Sample
struct Sample{
  uint16_t data[sampleSize];
  int dy[sampleSize];
  byte zeroDY[sampleSize];
  int avg = 0;
};
Sample samples[sampleBufSize];   // round-robin buffer for raw data
int sIdx = 0;        // current index of complete data (DMA just filled it)
int prevIdx = (sampleBufSize -1);
int DMAIdx = 0;      // current index for DMA to fill buffer


// This contains a formatted copy of a captured rhythym
struct Rhythym{
  uint16_t data[320];  //holds 320 points, or 0.8sec of data
  int dy[320];    //slope of each point
  byte zeroDY[320];  //marks where dy/dx crosses zero
  int low = 0;
  int high = 0;
  int idx = 0;
  int origin = 0;
  int p = 0;
  int q = 0;
  int r = 0;
  int s = 0;
  int t = 0;
  bool valid = false;
  int heartRate = 0;
};
Rhythym rhythyms[10];
int rIdx = 9;

boolean dataReady = false;     // true when new data is available 
boolean drawReady = false;      // ready to draw live data point
uint16_t tLines[320];      //used to draw live trace lines on display
uint16_t rLines[160];      // used to draw rhythym on display
uint16_t dLines[160];      // used to draw dy/dx on display
uint16_t sdLines[320];    //draws lines on the SD card recall display 
uint16_t sdData[320];

int oldVLine = 0;          // used to draw a Vertical Line, for debug
int oldHLine = 0;          // used to draw a Horizontal Line, for debug
int vScale = 124/boxWidth;    // used to scale the data to match given voltage scale
float hScale = (boxWidth / 16.0);   //used to scale the data to match given time scale
int cAvg = 2500;  //current average of most recent set of data. 2500=normal avg for startup
int cSum = 0;  //total sum of all elements in most recent set of data
int c1Idx = 0;  //current index of DMA progress for live data trace
int c2Idx = -1;  //current index of DMA progress for live data trace. Neg # causes initial delay
int c3Idx = -2;  //current index of DMA progress offset by -1 for accessing filtered data
int tIdx = 0;  //index for drawing live data to display
float tIncr = 0.0; //index for scaling for proper time interval when drawing live trace

int heartRate = 0;
int beepIdx = -1;
int beepOrigin = -1;
int samplesPerSecond = 400;

int sampleNumber = 0;  //number of samples in current record file
int fileNumber = -1;   //which file is currently being written to
char* uniqueID = "AVMK"; //file header identifier
SdFat sd;
SdFile file;

bool readyToWrite = false;
bool dataValid = false;
bool SDCardExist = false;
bool drawSDdata = false;
bool drawMenuReady = false;
int invalidDataCount = 2;
int activeMenu = -1;

int avgHR = 0;

int retVal = 0;

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ScreenColor);
  SDInit();

  drawGrid();

  pinMode(SELECT_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  digitalWrite(SELECT_PIN, HIGH);
  digitalWrite(MODE_PIN, HIGH);
  attachInterrupt(SELECT_PIN, but2_int, FALLING);
  attachInterrupt(MODE_PIN, but1_int, FALLING);
  
  adcInit(0, samples[0].data, sampleSize, 400);

}

long but1_time = 0;
long but2_time = 1;
boolean but1_pressed = false;
boolean but2_pressed = false;

/*
 * Int routine for menu button
 */
void but1_int(){
  if (millis() > but1_time){
    but1_time = millis() + 300;
    drawMenuReady = true;
    but1_pressed = true;  
  }
}

/*
 * Int routine for select button
 */
void but2_int(){
  if (millis() > but2_time){
    but2_time = millis() + 300;
    drawMenuReady = true;
    but2_pressed = true;  
  }
}

/*
 * Verify that the SD card is installed, check for a FAT table, and initialize
 * the SDFAT object for writing and reading.
 */
void SDInit() {
  SDCardExist = card.begin(SD_CS, SPI_HALF_SPEED);
  
  if (!SDCardExist){
    SDCardExist = false;
    tft.print("no card");
    return;
  } 
  cardSizeBlocks = card.cardSize();
  
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    tft.print("SDFat fail");
    return;
  }
  SDCardExist = true;
  

  if (cardSizeBlocks == 0) {
    tft.print("damaged card");
    return;
  }
  cardCapacityMB = (cardSizeBlocks + 2047)/2048;
  
  //tft.setCursor(0,0);
  //tft.print(eraseCard());
  //tft.print(formatCard());
  //tft.print(cardCapacityMB);
}

void loop() {
  int steps=0;
  while(dataReady || drawReady) {
    if (drawReady) {
      drawReady = false;
      if(!drawSDdata) drawTrace(1);
    }
  
    if (dataReady) {
      switch(steps++) {
        case 0: filterData(6); break;  //will filter sample[sIdx], except the last (n+1) elements
        case 1: scanForQRS(); break;
        case 2: drawRhythym(rIdx); break;
        case 3: checkButtons(); break;
        case 4: drawMenu(); break;
        case 5: writeToSD(); break;
        case 6: sendBlueTooth(); dataReady = false; break;
      }
    }
  }

}

/*
 * This function is called by the loop and checks if the interupt for either button
 * has been called.  If a button has been pressed, it determines the desired action
 * and calls the appropriate functions
 */
void checkButtons(){
  if (but1_pressed){
    but1_pressed = false;
    if (++activeMenu > 3) activeMenu = -1;
  }
  if (but2_pressed){
    but2_pressed = false;
    switch(activeMenu){
      case -1: if(drawSDdata) {
                  readData(); 
                  drawData();
              }
              break;
      
      case 0: readyToWrite=!readyToWrite;
              if(readyToWrite) {
                startWritingToNewFile();  
              } else {
                file.println("EOF");
                file.close();
                sampleNumber = 0;
              }
              
              break;
              
      case 1: if (!drawSDdata){
                clearMonitorLines();
              } else {
                clearSDLines();
              }
              drawSDdata = !drawSDdata;
              if(drawSDdata) {
                openFile(0);
                readData();
                drawData();
              } else{
                drawMsg("live data");
              }
              break;
      case 2: drawMsg("Erasing SD Card",0);
              retVal = eraseCard();
              if(retVal > 0){
                drawMsg("Erase Fail ", retVal);
              } else {
                drawMsg("Erase Complete",0);
              }
              break;
      case 3: drawMsg("Formating SD Card", 0);
              retVal = formatCard();
              if(retVal > 0) {
                drawMsg("Format Fail ", retVal);
              } else {
                drawMsg("Format Complete", retVal);
              }
              break;
    }
    activeMenu=-1;
  }
}

/*
 * This function will remove all trace lines from the live trace function
 * This is used when switching from live data to SD card data
 */
void clearMonitorLines() {
  
  for(int i = 0; i < 319; i++) {
    //erase old line and repair grid underneath
    tft.drawLine(i, tLines[i], i+1 , tLines[i + 1], ScreenColor);
    repairGrid(i, tLines[i], i+1, tLines[i + 1]);
  }
}

/*
 * This functio will remove all SD card lines that have been drawn from
 * stored data. This functio should be called when switching from stored
 * data to live data
 */
void clearSDLines() {
  for(int i = 0; i < 319; i++) {
    //erase old line and repair grid underneath
    tft.drawLine(i, sdLines[i], i+1 , sdLines[i + 1], ScreenColor);
    repairGrid(i, sdLines[i], i+1, sdLines[i + 1]);
  }
}



void drawMenu() {
  if(drawMenuReady && (activeMenu >= 0)) {
    drawMenuReady = false;
    for (int i = 0; i < 4; i++) {
      tft.drawRect(161,(i*14),158, 13, ILI9341_BLUE);
      tft.fillRect(162,(i*14)+1, 156, 12, ILI9341_WHITE);
      tft.setCursor(180,(i*14)+3);
      tft.setTextColor(ILI9341_BLUE);
      switch(i){
        case 0: if(readyToWrite) {
                  tft.print("Stop Recording");
                } else {
                  tft.print("Record Data (30 Sec)"); 
                }
                break;
        case 1: if (drawSDdata) {
                  tft.print("Display Live Data");
                } else {
                  tft.print("Display Data from SD"); 
                }
                break;
        case 2: tft.print("Erase SD Card"); break;
        case 3: tft.print("Format SD Card"); break;
      }
      tft.setTextColor(ILI9341_WHITE);
      if (i == activeMenu){
        tft.drawFastHLine (165, (i*14)+7, 10, ILI9341_BLUE);
        tft.drawLine(170, (i*14)+3, 175, (i*14)+7, ILI9341_BLUE);
        tft.drawLine(170, (i*14)+11, 175, (i*14)+7, ILI9341_BLUE);
      }
    }
    
  } else {
    if(activeMenu == -1) drawStatus();
  }
}

/*
 * This function will draw the information into the top right quadrant of the screen
 * This is what should be displayed when the menu is not present.
 */
void drawStatus(){
  tft.fillRect(161,0, 158, 70, ScreenColor);
  tft.setCursor(270,0);
  tft.print("SD Card: ");
  tft.setCursor(280,10);
  if(SDCardExist){
    if(cardCapacityMB>0) {
      tft.print("Ready");
    } else {
      tft.print("Bad");
    }
  } else {
    tft.print("None");
  }

  //print out BPM
  tft.setCursor(170,15);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_CYAN);
  tft.print(avgHR);
  tft.setTextSize(1);
  tft.print("bpm");
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
}

/*
 * Helper function to write a message to the screen
 */
void drawMsg(String msg){
  drawMsg(msg, 0);
}

/*
 * Writes a message to the screen at the bottom of the status window
 */
void drawMsg(String msg, int val){
  tft.fillRect(180, 70, 139,10, ScreenColor);
  tft.setCursor(180, 70);
  tft.print(msg);
  if(val>0)tft.print(val);
}

/*
 * This function will draw data that is read in from the SD card. 320 points 
 * at a time.
 */
void drawData() {
  //erase old lines
  int avg = 0;
  for (int i = 1; i < 320; i++){
    avg+=sdData[i];
    tft.drawLine(i-1, sdLines[i-1], i, sdLines[i], ScreenColor);
  }
  avg = avg/320;
  //redraw the grid
  drawGrid();
  //set old lines as new lines and draw them
  for (int i = 0; i < 320; i++){
    sdLines[i] = 180 - ((sdData[i] - avg) / vScale);
    if(i>0) tft.drawLine(i-1, sdLines[i-1], i, sdLines[i], ILI9341_RED);
  }
}
/*
 * Write the processed signal data to the SD card. It checks that the SD card is 
 * installed, that a valid heartbeat rhythm is present, and it stops after 60 samples 
 * have been recorded (30 sec @ 400Hz)
 */
void writeToSD(){
  if(!SDCardExist){
    drawMsg("No SD Card");
    return;
  }
  if(readyToWrite && !dataValid) {
    drawMsg("Waiting for rhythym");
    return;
  }
  if(readyToWrite && dataValid) {
    //write current sample
    
    int indexToRecord = prevIdx;
    for (int i = 0; i < sampleSize; i+=8) {
      for (int j = 0; j < 8; j++) {
        if(j>0) file.print(", ");
        file.print(samples[indexToRecord].data[i+j]);
      }
      file.println();
    }
    sampleNumber++;
    tft.fillRect(180,70,139,10, ScreenColor);
    tft.setCursor(180,70);
    tft.print("Saving ");
    tft.print(sampleNumber/2);
    tft.print("sec");
    
    //switch to next file if full
    if(sampleNumber > SAMPLES_PER_FILE) {
      file.println("EOF");
      file.close();
      sampleNumber = 0;
      readyToWrite = false;
      drawMsg("Saving Complete");
    }
  }
}

/*
 * Starts a new file to record SD data onto. Also closes the previous file.
 */
void startWritingToNewFile() {
  char fileName[15];
  char header[13];
  
  //create new data file
  fileNumber++;

  //set header
  if(fileNumber < 10) {
    sprintf(header, "%s%s%s%d", uniqueID, "0", "0", fileNumber); 
  } else if(fileNumber < 100) {
    sprintf(header, "%s%s%d", uniqueID, "0", fileNumber);
  } else {
    sprintf(header, "%s%d", uniqueID, fileNumber);
  }
  sprintf(fileName, "%s%d.txt", uniqueID, fileNumber); 
  if (!file.open(fileName, O_CREAT | O_APPEND | O_RDWR)) {
    //file can't be created
  }
  
  //add header
  file.print(header);
  file.println(", 400");
}

int fileBeingRead = 0;
ifstream sdin;
int line_buffer_size = 100;

/*
 * Uses a file Number to open the associated file for reading
 */
void openFile(int fileNum){
  char buffer[line_buffer_size];
  char fileName[15];
  
  //calculate filename and open stream
  sprintf(fileName, "%s%d.txt", uniqueID, fileNum); 
  
  // open input file
  sdin.open(fileName);
  
  //get header  *could use info if we want
  sdin.getline(buffer, line_buffer_size, '\n');
  
  //display filename to screen
  tft.fillRect(180,70,139,10, ScreenColor);
  tft.setCursor(180, 70);
  tft.print("Displaying ");
  tft.print(fileName);

}


/*
 * This function will read 320 points of data from the SD card. It begins at File
 * Number 0, and reads the next 320 points each time this function is called. When each  
 * file is completely read, it shifts to the next file number. When
 * the next file does not exist, it does nothing.
 */
  
void readData() {

  int k = 0;
  
  char buffer[line_buffer_size];
  int lineData[8];
  char* tokens;
  int sampleNum = 0;

  if (sdin.is_open()) {

    while(!sdin.fail()) {
      sdin.getline(buffer, line_buffer_size, '\n');
      if ((String)buffer == "EOF"){
        sdin.close();
        openFile(++fileBeingRead);
        sdin.getline(buffer, line_buffer_size, '\n');
      }
      tokens = strtok(buffer, ", ");
      
      for(int i = 0; i < 8; i++) {
        lineData[i] = ((String)tokens).toInt();
        sdData[sampleNum + i] = (uint16_t)lineData[i];
        tokens = strtok(NULL, ", ");
      }
      
      sampleNum+=8;
      if(sampleNum >= 320) break;
      
      
      tft.fillRect(180, 0, 60, 10, ScreenColor);
      tft.setCursor(180, 0);
      tft.print(k++);

    } 
    if(sdin.fail()) {
      sdin.close();
      openFile(++fileBeingRead);
    }

  }
}

/*
 * Coulda Woulda Shoulda
 */
void sendBlueTooth(){
  
}




int dyDir = 0;
 /* This function uses an interpolation method to flatten the samples[sIdx] that was
  *  just filled by DMA. It works in the following way... For each point in the sample,
  *  I find the average slope from that point to the next 5 points of data (skipping the
  *  adjacent point), and I overwrite the adjacent point to the right with a new value 
  *  that is on that average slope. Each pass overlaps the previous set of sample data 
  *  since the final n+1 points of it can not be calculated during the first pass. This 
  *  means the end of samples[sIdx] will not be filtered, but samples[prevIdx] will be 
  *  entirely filtered.
  *  
  *  While filtering, this function will also store the slope at each point in samples[prevIdx].dy[]
  *  It will also calculate the average value of samples[sIdx].data[] and store in samples[sIdx].avg
  *  
  *  n = number of points to use for calculating the average slope
  */
void filterData(int n){
  int origin;
  uint16_t* target;
  int* dyTarget;
  byte* dyZeroTarget;
  float slope = 0;
  int sum = 0;
  for (int scan = -(n+1); scan < (sampleSize - (n+1)); scan++) {
    origin = (scan < 0)? samples[prevIdx].data[sampleSize + scan] : samples[sIdx].data[scan];
    target = (scan < -1)? &samples[prevIdx].data[sampleSize + (scan + 1)] : &samples[sIdx].data[scan + 1];
    dyTarget = (scan < -1)? &samples[prevIdx].dy[sampleSize + (scan + 1)] : &samples[sIdx].dy[scan + 1];
    dyZeroTarget = (scan < -1)? &samples[prevIdx].zeroDY[sampleSize + (scan + 1)] : &samples[sIdx].zeroDY[scan + 1];
    sum += origin;
    //scan n points to gather slopes
    for (int pointIdx = (scan + 2); pointIdx < (scan + n + 2); pointIdx++) {
      if (pointIdx < 0) {
        slope += ((float)((int)samples[prevIdx].data[sampleSize + pointIdx] - origin) / (float)n);
      } else {
        slope += ((float)(samples[sIdx].data[pointIdx] - origin) / (float)n);
      }
    }
    slope /= n;
    cAvg = sum / sampleSize;
    samples[sIdx].avg = cAvg;
    
    *dyZeroTarget = (((slope > 0) && (dyDir == 0)) || ((slope <= 0) && (dyDir == 1)))? 1:0;
    dyDir = (slope > 0)? 1:0;
    *dyTarget = (int)slope;
    *target = origin + (int)slope;
  }
}
  
  
int areaUnderCurve = 0;
int width = 0;
int Px,Qx,Rx,Sx,Tx=0;
int PQ,QR,RS,ST;
int PQArea,QRArea,RSArea,STArea;
boolean cndMsg = false;
/*
 * This function will track the points at every spot where the derivate is zero. These points
 * are the characteristic points of a heartbeat rhythym. 
 * For each spot where the dy is zero, we assign it as point T in our heartbeat, and
 * let all the other points be aligned along all the previous spots where dy was zero.
 * Then we attempt to pass that combination of values through a filter to determine if that
 * combination of points does match the requirements of a heartbeat rhythym. 
 * Once a heartbeat is found, it is stored in rhythyms[rIdx+1] and further calculations are done.
 */
void scanForQRS() {
  invalidDataCount++;  //counts the number of consecutive samples with no detected rhythym
  for (int scan = 0; scan < sampleSize; scan++) {
    width++;
    areaUnderCurve+=samples[prevIdx].dy[scan];
    
    if (samples[prevIdx].zeroDY[scan]==1){
      PQ = QR;
      PQArea = QRArea;
      QR = RS;
      QRArea = RSArea;
      RS = ST;
      RSArea = STArea;
      ST = width;
      STArea = areaUnderCurve;
      Px=Qx+width;
      Qx=Rx+width;
      Rx=Sx+width;
      Sx=width;
      width = 0;
      areaUnderCurve = 0;
      int PVal = (int)valAtOffset(prevIdx, scan, -Px, NULL, NULL);
      int QVal = (int)valAtOffset(prevIdx, scan, -Qx, NULL, NULL);
      int RVal = (int)valAtOffset(prevIdx, scan, -Rx, NULL, NULL);
      int SVal = (int)valAtOffset(prevIdx, scan, -Sx, NULL, NULL);
      int TVal = (int)samples[prevIdx].data[scan];
      
      if(abs(QVal - samples[prevIdx].avg) < 200) {
        if(((QR+RS)<70) && ((QR+RS) > 30)) {
          if (((QRArea - RSArea) > 600)){
            if((PVal < RVal)&&(PQ < 60))  {
              invalidDataCount = 0;

              
              
              //find the array alignment at the Rx value
              int Ridx = scan-Rx;
              int Sidx = prevIdx;
              if (Ridx < 0) {
                Ridx += sampleSize;
                Sidx = (Sidx+3)%4;
              }
              int prevrIdx = rIdx;
              captureRhythym(Sidx, Ridx);  //copies into rhythym array, aligned by QRS at 120


              
              // adjust P and T points if too narrow
              if (PQ < 20) {
                int incr = (20 - PQ);
                while (incr++ < 60) {
                  byte zyZero = 0;
                  valAtOffset(prevIdx, (scan - Px), -incr, NULL, &zyZero);
                  if (zyZero > 0) {
                    PQ += incr;
                    break;
                  }
                }
              }
              if (ST < 40) {
                int incr = (50 - PQ);
                while (incr++ < 60) {
                  byte zyZero = 0;
                  valAtOffset(prevIdx, scan, incr, NULL, &zyZero);
                  if (zyZero > 0) {
                    ST += incr;
                    break;
                  }
                }
              }

              //store characteristic points into rhythym struct
              rhythyms[rIdx].p = 120-(PQ+QR);
              rhythyms[rIdx].q = 120-QR;
              rhythyms[rIdx].r = 120;
              rhythyms[rIdx].s = 120+RS;
              rhythyms[rIdx].t = 120+(RS+ST);

              //calculate heart rate
              
              if(dataValid && rhythyms[prevrIdx].valid){
                int lastIdx = rhythyms[prevrIdx].idx;
                heartRate=0;
                heartRate+=(rhythyms[rIdx].origin - rhythyms[prevrIdx].origin);
                while(lastIdx != rhythyms[rIdx].idx) {
                  heartRate+=sampleSize;
                  if (++lastIdx==sampleBufSize)lastIdx=0;
                }
                
                rhythyms[rIdx].heartRate = (400 * 60) / heartRate;
              } else {
                rhythyms[rIdx].heartRate = 0; //no previous heartbeat rhythym for timing.
              }
              
              avgHR = getAvgHR();
              int PR = PQ+QR;
              int QS = QR+RS;
              if(cndMsg) {
                tft.fillRect(40, 90, 240, 20, ScreenColor);
                repairGrid(40,90,280,110);
              }
              cndMsg = false;
              tft.setCursor(40, 90);
              tft.setTextSize(2);
              tft.setTextColor(ILI9341_RED);
              if(PR >= 40 && PR <= 90 && QS < 70 && avgHR > 0) {
                if(avgHR < 60) {
                  tft.print("BRADYCARDIA DETECTED");
                  cndMsg = true;
                }
                if(avgHR > 100) {
                  tft.print("TACHYCARDIA DETECTED");
                  cndMsg = true;
                }
              }
              tft.setTextSize(1);
              tft.setTextColor(ILI9341_WHITE);
            }
          }
        }
      }
    }
  }
  dataValid = (invalidDataCount <= 2)? true:false;
  
}

/*
 * This function averages the heartrate of all captured rhythms.
 * If a rhythm has no preceeding rhythm in time, the heartrate is zero and
 * should not be used in calculations.
 */
int getAvgHR() {
  int n = 0;
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    if ((rhythyms[i].valid) && (rhythyms[i].heartRate > 0)) {
      n++;
      sum += rhythyms[i].heartRate;
    }
  }
  
  return (sum / n);
}

 /*
 * 
 * Used to combine adjacent samples of data into an aligned heartbeat rhythym
 * when the R peak is detected at point sample[idx].data[origin]
 * The R peak should be exactly at point 120 in rhythym array
 */
void captureRhythym(int idx, int origin) {
  uint16_t newVal;
  
  if(++rIdx == 10) rIdx = 0;  //increment newest rhythym index
  rhythyms[rIdx].low = samples[idx].data[origin];  //preset averages
  rhythyms[rIdx].high = samples[idx].data[origin];
  rhythyms[rIdx].idx = idx;
  rhythyms[rIdx].origin = origin;
  rhythyms[rIdx].valid = true;
  
  for (int dCount = 0; dCount < 320; dCount++) {
    
    newVal = valAtOffset(idx, origin, (dCount-119), &rhythyms[rIdx].dy[dCount], &rhythyms[rIdx].zeroDY[dCount]);
    rhythyms[rIdx].data[dCount] = newVal;

     //set maximum and minimum values for scaling and alignment
    if (newVal > rhythyms[rIdx].high) rhythyms[rIdx].high = newVal;  
    if (newVal < rhythyms[rIdx].low) rhythyms[rIdx].low = newVal;
  }
}

/*
 * Helper function to return a value that is some offset from
 * a specified point, i.e. sample[idx].data[origin + off].
 * This will wrap around to adjacent samples as needed.
 *         (sample[idx].data[origin] +- some offset)
 */
uint16_t valAtOffset(int idx, int origin, int off, int* dyVal, byte* dyZeroVal) {
  int dest = origin + off;
  int idxShift = (dest/sampleSize);  
  if (dest < 0) idxShift--;
  dest = dest - (idxShift * sampleSize); 
  idxShift = (idx + idxShift) % 4;
  if (idxShift < 0) idxShift += 4;
  
  if(dyVal) *dyVal = samples[idxShift].dy[dest];
  if(dyZeroVal) *dyZeroVal = samples[idxShift].zeroDY[dest];
  return (samples[idxShift].data[dest]);
}

int px, py, qx, qy, rx, ry, sx, sy, tx, ty;
/*
 * This function draws white boxes around upper data area and displays rhythym[idx] 
 * into the left pane of data area on display.
 * note: (vScale*2) is not formatted properly, it just works for now
 */
void drawRhythym(int idx) {
 if ((idx < 0) || rhythyms[idx].valid==false) return;
 
 //erase old lines and labels
  for (int i = 1; i < 160; i++) {
    tft.drawLine(i - 1, rLines[i-1], i, rLines[i], ScreenColor);
  }
    tft.fillRect(px,py,20,10,ScreenColor);
    tft.fillRect(qx,qy,20,10,ScreenColor);
    tft.fillRect(rx,ry,20,10,ScreenColor);
    tft.fillRect(sx,sy,20,10,ScreenColor);
    tft.fillRect(tx,ty,20,10,ScreenColor);

  //draw white boxes
  tft.drawFastVLine(160,0,80,ILI9341_WHITE);
  tft.drawFastHLine(0,80,320,ILI9341_WHITE);

  // translate rhythym points into pixel coordinates
  px=rhythyms[idx].p/2;
  py=60-((rhythyms[idx].data[rhythyms[idx].p]-rhythyms[idx].low)/(vScale*2))-10; 
  qx=rhythyms[idx].q/2;
  qy=60-((rhythyms[idx].data[rhythyms[idx].q]-rhythyms[idx].low)/(vScale*2))+5; 
  rx=rhythyms[idx].r/2;
  ry=60-((rhythyms[idx].data[rhythyms[idx].r]-rhythyms[idx].low)/(vScale*2))-10;
  sx=rhythyms[idx].s/2;
  sy=60-((rhythyms[idx].data[rhythyms[idx].s]-rhythyms[idx].low)/(vScale*2))+5;
  tx=rhythyms[idx].t/2;
  ty=60-((rhythyms[idx].data[rhythyms[idx].t]-rhythyms[idx].low)/(vScale*2))-10;
  if(py<0)py=0;
  if(qy<0)qy=0;
  if(ry<0)ry=0;
  if(sy<0)sy=0;
  if(ty<0)ty=0;

   //draw new labels
  tft.setCursor(px, py);
  tft.print("P");
  tft.setCursor(qx, qy);
  tft.print("Q");
  tft.setCursor(rx, ry);
  tft.print("R");
  tft.setCursor(sx, sy);
  tft.print("S");
  tft.setCursor(tx, ty);
  tft.print("T");
  
  
  //draw new heartbeat rhythym lines 
  for (int i = 0; i < 160; i++) {
    rLines[i] = 60 - ((rhythyms[idx].data[i*2]-rhythyms[idx].low)/(vScale*2));
    if(i>0) tft.drawLine(i - 1, rLines[i-1], i, rLines[i], ILI9341_RED);
  } 


}

/*
 * Used to draw a quick Horizontal line for debug
 */
void drawHLine(int y) {
  tft.drawFastHLine(0, oldHLine, 320, ScreenColor);
  tft.drawFastHLine(0, y, 320, ILI9341_YELLOW); 
  oldHLine = y;
}

/*
 * Used to draw a quick Vertical line for debug
 */
void drawVLine(int x) {
  tft.drawFastVLine(oldVLine, 60, 30, ScreenColor);
  tft.drawFastVLine(x, 60, 30, ILI9341_YELLOW); 
  oldVLine = x;
}






int tAvg = 0;  // the current average that the trace is using to vertically offset the data
              // the trace should be centered around line #180
/*
 * This function will draw each new point of data to a trace on the display. Should be
 * called when drawReady=true.
 * @type - 0 for LIVE data, 1 for FILTERED data
 */
void drawTrace(int type) {
  analogWrite(3,0);
    //horizontal scale is created by delaying at proper intervals
    //tIncr and tIdx should increment at different rates to achieve scaling
  if ((int)tIncr >= tIdx) {
    
    //erase old line and repair grid underneath
    if ((tIdx < 319) && (tLines[tIdx] > 0)){
      tft.drawLine(tIdx, tLines[tIdx], tIdx +1 , tLines[tIdx + 1], ScreenColor);
      repairGrid(tIdx, tLines[tIdx], tIdx+1, tLines[tIdx + 1]);
    }
  
    //set new vertical offset and grab new data
    if(tIdx == 0)tAvg=cAvg;  //only change the vertical offset at the beginning of trace
    if (type==0)tLines[tIdx] = 180 + (-(samples[c1Idx].data[c2Idx] - tAvg) / vScale);
    if (type==1)tLines[tIdx] = 180 + (-(samples[c3Idx].data[c2Idx] - tAvg) / vScale);

    //keep within bounds 
    if (tLines[tIdx] < 81) tLines[tIdx] = 81;
    if (tLines[tIdx] > 239) tLines[tIdx] = 239;
    
    //draw new lines
    if (tIdx == 0) tft.drawLine(318, tLines[318], 319, tLines[319], ILI9341_RED);
    if (tIdx > 0) tft.drawLine(tIdx -1, tLines[tIdx-1], tIdx, tLines[tIdx], ILI9341_WHITE);
    if (tIdx > 1) tft.drawLine(tIdx -2, tLines[tIdx-2], tIdx-1, tLines[tIdx-1], ILI9341_RED);

    //beep when beepIdx and beepOrigin is reached
    if (dataValid && (c3Idx == beepIdx) && (c2Idx == beepOrigin)) {
      analogWrite(3, 130);
    }
    //cycle index
    if (++tIdx == 320) {
      tIdx = 0;
      tIncr = 0.0;
    }
  }
  tIncr += hScale;
  
}







/*
 * This function will draw the entire grid onto the screen with the specified boxWidth
 * It will leave the first 80 rows blank for the rhythym and data area
 */
void drawGrid() {
  
  for (int xPos = 0; xPos < 320; xPos+=boxWidth){
    if (xPos % (boxWidth * 5) == 0) {
      tft.fillRect(xPos - 1, 81, 3, 159, ILI9341_BLUE);
      if ((xPos > 80) && (xPos < 240))
        tft.fillRect(0, xPos - 1, 320, 3, ILI9341_BLUE);
    } else {
      tft.drawFastVLine(xPos, 81, 159, ILI9341_BLUE);
      if ((xPos > 80) && (xPos < 240) )
        tft.drawFastHLine(0, xPos, 320, ILI9341_BLUE);
    }
  }
  //draw white boxes
  tft.drawFastVLine(160,0,80,ILI9341_WHITE);
  tft.drawFastHLine(0,80,320,ILI9341_WHITE);
}

/*
 * This function will only draw the portion of the grid that exists within the
 * rectangle specified. Used to reduce overhead time of erasing old trace lines.
 * x2 must be greater than x1.
 */
void repairGrid(int x1, int y1, int x2, int y2) {

  //make sure y2 > y1
  if(y2 < y1) {
    int swap = y2;
    y2 = y1;
    y1 = swap;
  }
  for (int xPos = x1; xPos <= x2; xPos++){
    if ((xPos % boxWidth == 0) || (xPos % (boxWidth * 5) == 1) || (xPos % (boxWidth * 5) == ((boxWidth * 5) - 1))){
      tft.drawFastVLine(xPos, y1, y2-y1+1, ILI9341_BLUE);
    }
  }
  for (int yPos = y1; yPos <= y2; yPos++){
    if ((yPos % boxWidth == 0) || (yPos % (boxWidth * 5) == 1) || (yPos % (boxWidth * 5) == ((boxWidth * 5) - 1))){
      tft.drawFastHLine(x1, yPos, x2-x1+1, ILI9341_BLUE);
    }
  }
}


/*
 * This function is called by the DMA interupt (when the buffer is full)
 * 
 * Sets sIdx to the index of the newest set of data, i.e. sample[sIdx].data, and
 * sets the next location for DMA to copy data into. The buffer is set up in a 
 * round robin style, overwriting the oldest data. prevIdx and DMAIdx are also
 * incremented.
 * 
 * dataReady is set to true when new data is available
 */
void dma_ch2_isr() {
  prevIdx = sIdx;   //track previous data set index
  sIdx = DMAIdx;    //mark newest data set index
  if (++DMAIdx == sampleBufSize) DMAIdx = 0;  //runs faster than DMAIdx=(DMAIdx+1)%sampleBufSize
  DMA_TCD2_DADDR = samples[DMAIdx].data;  //set new DMA target
  DMA_CINT = 2; // Clear interrupt request for channel 2
  dataReady = true;
}

/*
 * This function is called every time a new element of data is ready.
 * Runs parallel to actual DMA position. Should always match. Eliminates race condition.
 * 
 * c#Idx - the indexes of the newest peice of data in the buffer that DMA is writing to.
 *         i.e. sample[c1Idx].data[c2Idx], unfiltered raw data
 * c3Idx - previous buffer, i.e sample[c3Idx].data[c2Idx], for filtered data
 *        
 *        
 * drawReady - set to true in order to trigger a live trace in real time.
 */
void adc0_isr() {
  if(++c2Idx == sampleSize){
    c2Idx=0;
    if(++c1Idx == sampleBufSize) c1Idx=0;
    if(++c3Idx == sampleBufSize) c3Idx=0;
  }
  drawReady = true;

}


 #define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
  | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))
/*
 * This function will setup the PDB to trigger the ADC at the given frequency 
 * and DMA will automatically copy the results of ADC into the buffer specified.
 * The DMA will copy ADC values sequentially into buff[], from 0 to (buffSize - 1)
 * 
 * Once the buffer is full, DMA will trigger a software interupt by calling dma_ch2_isr().
 * Each time the ADC completes 1 piece of data, it both triggers DMA and calls adc0_isr(). 
 * 
 * uint32_t pin - ADC source pin, must be 0-13(A0-A13)
 * uint16_t buff[] - target buffer for DMA
 * int buffSize  - the size of the buffer. Number of elements, not bytes.
 * int freq  - the frequency at which the PDB triggers the ADC, in HZ.
 */

void adcInit(unsigned int pin, uint16_t buff[], int buffSize, int freq) {

        
        // Configure the ADC and run at least one software-triggered
        // conversion.  This completes the self calibration stuff and
        // leaves the ADC in a state that's mostly ready to use
        analogReadRes(12);
        analogReadAveraging(4);
        uint32_t sum = analogRead(pin);
        sum++; //removes "unused variable" warning message
        //enable adc interrupt
        ADC0_SC1A |= ADC_SC1_AIEN;
        NVIC_ENABLE_IRQ(IRQ_ADC0);

        // set the programmable delay block to trigger the ADC at given frequency
        SIM_SCGC6 |= SIM_SCGC6_PDB;
        PDB0_MOD = (F_BUS / 128 / 10 / freq);  // 48 MHz / 128 / 10 / 1 Hz = 37500
        PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
        PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
        PDB0_CH0C1 = 0x0101;

        // enable the ADC for hardware trigger and DMA
        ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;

        // set up a DMA channel to store the ADC data
        SIM_SCGC7 |= SIM_SCGC7_DMA;
        SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
        DMA_CR = 0;
        DMA_TCD2_SADDR = &ADC0_RA;
        DMA_TCD2_SOFF = 0;
        DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
        DMA_TCD2_NBYTES_MLNO = 2;
        DMA_TCD2_SLAST = 0;
        DMA_TCD2_DADDR = buff;
        DMA_TCD2_DOFF = 2;
        DMA_TCD2_CITER_ELINKNO = buffSize;
        DMA_TCD2_DLASTSGA = -(buffSize * 2);
        DMA_TCD2_BITER_ELINKNO = buffSize;
        DMA_TCD2_CSR = DMA_TCD_CSR_INTMAJOR;
        DMAMUX0_CHCFG2 = DMAMUX_DISABLE;
        DMAMUX0_CHCFG2 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;
        DMA_SERQ = 2;
        NVIC_ENABLE_IRQ(IRQ_DMA_CH2);
}


/*/////////////////////////////////////////////////////////////////////////////////////*/
// SD Card Functions Below


uint32_t const ERASE_SIZE = 262144L;
// flash erase all data
int eraseCard() {
  uint32_t firstBlock = 0;
  uint32_t lastBlock;

  do {
    lastBlock = firstBlock + ERASE_SIZE - 1;
    if (lastBlock >= cardSizeBlocks) {
      lastBlock = cardSizeBlocks - 1;
    }
    if (!card.erase(firstBlock, lastBlock)){
      return 1;
    }
    firstBlock += ERASE_SIZE;
  } while (firstBlock < cardSizeBlocks);
  
  if (!card.readBlock(0, cache.data)) {
    return 2;
  }
  return 0;
}

int formatCard() {
  retVal = initSizes();
  if (retVal > 0) return retVal;
  if (card.type() != SD_CARD_TYPE_SDHC) {
    return 12;
  } else {
    retVal = makeFat32();
    if(retVal > 0) return retVal;
  }
  return 0;
}

// write cached block to the card
uint8_t writeCache(uint32_t lbn) {
  return card.writeBlock(lbn, cache.data);
}

// initialize appropriate sizes for SD capacity
int initSizes() {
  if (cardCapacityMB <= 6) {
    return 11;
  } else if (cardCapacityMB <= 16) {
    sectorsPerCluster = 2;
  } else if (cardCapacityMB <= 32) {
    sectorsPerCluster = 4;
  } else if (cardCapacityMB <= 64) {
    sectorsPerCluster = 8;
  } else if (cardCapacityMB <= 128) {
    sectorsPerCluster = 16;
  } else if (cardCapacityMB <= 1024) {
    sectorsPerCluster = 32;
  } else if (cardCapacityMB <= 32768) {
    sectorsPerCluster = 64;
  } else {
    // SDXC cards
    sectorsPerCluster = 128;
  }

  sectorsPerTrack = cardCapacityMB <= 256 ? 32 : 63;

  if (cardCapacityMB <= 16) {
    numberOfHeads = 2;
  } else if (cardCapacityMB <= 32) {
    numberOfHeads = 4;
  } else if (cardCapacityMB <= 128) {
    numberOfHeads = 8;
  } else if (cardCapacityMB <= 504) {
    numberOfHeads = 16;
  } else if (cardCapacityMB <= 1008) {
    numberOfHeads = 32;
  } else if (cardCapacityMB <= 2016) {
    numberOfHeads = 64;
  } else if (cardCapacityMB <= 4032) {
    numberOfHeads = 128;
  } else {
    numberOfHeads = 255;
  }
  return 0;
}

// format and write the Master Boot Record
int writeMbr() {
  clearCache(true);
  part_t* p = cache.mbr.part;
  p->boot = 0;
  uint16_t c = lbnToCylinder(relSector);
  if (c > 1023) {
    return 14;
    //MBR CHS
  }
  p->beginCylinderHigh = c >> 8;
  p->beginCylinderLow = c & 0XFF;
  p->beginHead = lbnToHead(relSector);
  p->beginSector = lbnToSector(relSector);
  p->type = partType;
  uint32_t endLbn = relSector + partSize - 1;
  c = lbnToCylinder(endLbn);
  if (c <= 1023) {
    p->endCylinderHigh = c >> 8;
    p->endCylinderLow = c & 0XFF;
    p->endHead = lbnToHead(endLbn);
    p->endSector = lbnToSector(endLbn);
  } else {
    // Too big flag, c = 1023, h = 254, s = 63
    p->endCylinderHigh = 3;
    p->endCylinderLow = 255;
    p->endHead = 254;
    p->endSector = 63;
  }
  p->firstSector = relSector;
  p->totalSectors = partSize;
  if (!writeCache(0)) {
    return 15;
    //write MBR
  }
  return 0;
}

// return cylinder number for a logical block number
uint16_t lbnToCylinder(uint32_t lbn) {
  return lbn / (numberOfHeads * sectorsPerTrack);
}

// return head number for a logical block number
uint8_t lbnToHead(uint32_t lbn) {
  return (lbn % (numberOfHeads * sectorsPerTrack)) / sectorsPerTrack;
}

// return sector number for a logical block number
uint8_t lbnToSector(uint32_t lbn) {
  return (lbn % sectorsPerTrack) + 1;
}

// format the SD as FAT32
int makeFat32() {
  int retVal;
  uint32_t nc;
  relSector = BU32;
  for (dataStart = 2 * BU32;; dataStart += BU32) {
    nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
    fatSize = (nc + 2 + 127)/128;
    uint32_t r = relSector + 9 + 2 * fatSize;
    if (dataStart >= r) {
      break;
    }
  }
  // error if too few clusters in FAT32 volume
  if (nc < 65525) {
    return 3;
    //Bad cluster count
  }
  reservedSectors = dataStart - relSector - 2 * fatSize;
  fatStart = relSector + reservedSectors;
  partSize = nc * sectorsPerCluster + dataStart - relSector;
  // type depends on address of end sector
  // max CHS has lbn = 16450560 = 1024*255*63
  if ((relSector + partSize) <= 16450560) {
    // FAT32
    partType = 0X0B;
  } else {
    // FAT32 with INT 13
    partType = 0X0C;
  }
  retVal = writeMbr();
  if(retVal > 0) return retVal;
  clearCache(true);

  fat32_boot_t* pb = &cache.fbs32;
  pb->jump[0] = 0XEB;
  pb->jump[1] = 0X00;
  pb->jump[2] = 0X90;
  for (uint8_t i = 0; i < sizeof(pb->oemId); i++) {
    pb->oemId[i] = ' ';
  }
  pb->bytesPerSector = 512;
  pb->sectorsPerCluster = sectorsPerCluster;
  pb->reservedSectorCount = reservedSectors;
  pb->fatCount = 2;
  pb->mediaType = 0XF8;
  pb->sectorsPerTrack = sectorsPerTrack;
  pb->headCount = numberOfHeads;
  pb->hidddenSectors = relSector;
  pb->totalSectors32 = partSize;
  pb->sectorsPerFat32 = fatSize;
  pb->fat32RootCluster = 2;
  pb->fat32FSInfo = 1;
  pb->fat32BackBootBlock = 6;
  pb->driveNumber = 0X80;
  pb->bootSignature = EXTENDED_BOOT_SIG;
  pb->volumeSerialNumber = volSerialNumber();
  memcpy(pb->volumeLabel, noName, sizeof(pb->volumeLabel));
  memcpy(pb->fileSystemType, fat32str, sizeof(pb->fileSystemType));
  // write partition boot sector and backup
  if (!writeCache(relSector)
      || !writeCache(relSector + 6)) {
    return 4;
    //sdError("FAT32 write PBS failed");
  }
  clearCache(true);
  // write extra boot area and backup
  if (!writeCache(relSector + 2)
      || !writeCache(relSector + 8)) {
    return 5;
    //sdError("FAT32 PBS ext failed");
  }
  fat32_fsinfo_t* pf = &cache.fsinfo;
  pf->leadSignature = FSINFO_LEAD_SIG;
  pf->structSignature = FSINFO_STRUCT_SIG;
  pf->freeCount = 0XFFFFFFFF;
  pf->nextFree = 0XFFFFFFFF;
  // write FSINFO sector and backup
  if (!writeCache(relSector + 1)
      || !writeCache(relSector + 7)) {
    return 6;
    //sdError("FAT32 FSINFO failed");
  }
  retVal = clearFatDir(fatStart, 2 * fatSize + sectorsPerCluster);
  if (retVal > 0) return retVal;
  clearCache(false);
  cache.fat32[0] = 0x0FFFFFF8;
  cache.fat32[1] = 0x0FFFFFFF;
  cache.fat32[2] = 0x0FFFFFFF;
  // write first block of FAT and backup for reserved clusters
  if (!writeCache(fatStart)
      || !writeCache(fatStart + fatSize)) {
    return 7;
    //sdError("FAT32 reserve failed");
  }
  return 0;
}

// zero cache and optionally set the sector signature
void clearCache(uint8_t addSig) {
  memset(&cache, 0, sizeof(cache));
  if (addSig) {
    cache.mbr.mbrSig0 = BOOTSIG0;
    cache.mbr.mbrSig1 = BOOTSIG1;
  }
}

// zero FAT and root dir area on SD
int clearFatDir(uint32_t bgn, uint32_t count) {
  clearCache(false);
  if (!card.writeStart(bgn, count)) {
    return 8;
    //sdError("Clear FAT/DIR writeStart failed");
  }
  for (uint32_t i = 0; i < count; i++) {
    if (!card.writeData(cache.data)) {
      return 9;
      //sdError("Clear FAT/DIR writeData failed");
    }
  }
  if (!card.writeStop()) {
    return 10;
    //sdError("Clear FAT/DIR writeStop failed");
  }
  return 0;
}

uint32_t volSerialNumber() {
  return (cardSizeBlocks << 8) + micros();
}
