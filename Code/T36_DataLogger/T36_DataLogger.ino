// Data logger based on a FIFO to decouple SD write latency from data
// acquisition timing.
//
// Modified for Teensy 3.6    
//
// The FIFO uses two semaphores to synchronize between tasks.
////#include <SPI.h>

#include <ChibiOS_ARM.h>
////#include <SdFat.h>
#include <SD.h>
//
// interval between points in units of 1000 usec
const uint16_t intervalTicks = 1;
//------------------------------------------------------------------------------
// SD file definitions
////const uint8_t sdChipSelect = SS;
const int chipSelect = BUILTIN_SDCARD;
////SdFat sd; Instantiated as SD by SD.h
////SdFile file;
File dataFile;    // I don't think this needs to be global,but I'll let it be for now.
//------------------------------------------------------------------------------
// Fifo definitions

// size of fifo in records
const size_t FIFO_SIZE = 200;

// count of data records in fifo
SEMAPHORE_DECL(fifoData, 0);

// count of free buffers in fifo
SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

// data type for fifo item
struct FifoItem_t {
  uint32_t usec;
  int value;
  int error;
};
// array of data items
FifoItem_t fifoArray[FIFO_SIZE];
//------------------------------------------------------------------------------
// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waThread1, 32);

static THD_FUNCTION(Thread1, arg) {
  // index of record to be filled
  size_t fifoHead = 0;

  // count of overrun errors
  int error = 0;

  // dummy data
  int count = 0;

  while (1) {
    chThdSleep(intervalTicks);
    // get a buffer
    if (chSemWaitTimeout(&fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
      // fifo full indicate missed point
      error++;
      continue;
    }
    FifoItem_t* p = &fifoArray[fifoHead];
    p->usec = micros();

    // replace next line with data read from sensor such as
    // p->value = analogRead(0);
    p->value = count++;

    p->error = error;
    error = 0;

    // signal new data
    chSemSignal(&fifoData);

    // advance FIFO index
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
  }
}
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // wait for USB Serial
  while (!Serial) {}

  ////Serial.println(F("type any character to begin"));
  Serial.println("type any character to begin");
  while (!Serial.available());

  // open file
  ////if (!sd.begin(sdChipSelect)
  ////   || !file.open("DATA.CSV", O_CREAT | O_WRITE | O_TRUNC)) {
  ////  Serial.println(F("SD problem"));
  ////  sd.errorHalt();
  //// }
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  dataFile = SD.open("DATA.CSV", FILE_WRITE);    // if card is OK, then open should work

  if (!SD.exists("DATA.CSV")) {
    Serial.println("DATA file not present");
  }

  Serial.println("card initialized.");
  // throw away input
  while (Serial.available()) {
    Serial.read();
    delay(10);
  }
  Serial.println(F("type any character to end"));

  // start kernel
  chBegin(mainThread);
  while (1);
}
//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {
  // FIFO index for record to be written
  size_t fifoTail = 0;

  // time in micros of last point
  uint32_t last = 0;

  // remember errors
  bool overrunError = false;

  // start producer thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  // start SD write loop
  while (!Serial.available()) {   // Checks for a character typed
    // wait for next data point
    chSemWait(&fifoData);

    FifoItem_t* p = &fifoArray[fifoTail];
    if (fifoTail >= FIFO_SIZE) fifoTail = 0;

    // if the file is available, write to it:
    //if (SD.exists("DATA.CSV")) {
    if (dataFile) {
      // print interval between points
      if (last) {
        dataFile.print(p->usec - last);
      } else {
        dataFile.write("NA");
      }
      last = p->usec;
      dataFile.write(',');
      dataFile.print(p->value);
      dataFile.write(',');
      dataFile.println(p->error);

      // remember error
      if (p->error) overrunError = true;

      // release record
      chSemSignal(&fifoSpace);

      // advance FIFO index
      fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("DATA file not existing");
      break;
    }
  }
  // close file, print stats and stop
  // if the file is available, close it:
  if (SD.exists("DATA.CSV")) {
 // if (dataFile) {
    dataFile.close();
 }
  // if the file isn't open, pop up an error:
 else {
    Serial.println("error opening data file");
  }
  Serial.println("Done");
  Serial.print("Thread1 unused stack: ");
  Serial.println(chUnusedStack(waThread1, sizeof(waThread1)));
  Serial.print("Heap/Main unused: ");
  Serial.println(chUnusedHeapMain());
  if (overrunError) {
    Serial.println();
    Serial.println("** overrun errors **");
  }
  while (1);
}
//------------------------------------------------------------------------------
void loop() {
  // not used
}
