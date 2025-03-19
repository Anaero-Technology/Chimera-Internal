/* Version 0.4
Changes this version:
-Switched method of reading  values from sensors
*/

#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <Wire.h>
#include <ESP32AnalogRead.h>
#include "USB.h"        
#include "USBCDC.h"     

const int averageCount = 16;

//Valve pins for each of the channels (and flush)
const int solenoidValvePins[16] = { 21, 34, 35, 36, 16, 15, 14, 13, 12, 11, 10, 9, 5, 6, 7, 8 };
//Pins used for analogread for sensors
const int co2Pin = 4;
const int ch4Pin = 1;

//SD card pin configuration
const int SD_CS = 40;
const int SD_MOSI = 39;
const int SD_SCK = 38;
const int SD_MISO = 37;

//Maximum possible value for an unsigned ulong (used for handling overflow)
const unsigned long ULONGMAX = 0UL - 1UL;

//Timestamp of last event druing valve sequence
unsigned long lastAction = 0;

//Default values for timing
//Amount of time to keep valve open while reading (ms)
unsigned long openDuration = 30000;
//Amount of time to wait before starting flushing (ms)
unsigned long beforeFlush = 1000;
//How long to flush for (ms)
unsigned long flushDuration = 5000;
//How ling to wait before opening the next valve (ms)
unsigned long waitBetween = 2000;
//Time between reading the sensor (ms) (currently unused, may be removed completely)
unsigned long readInterval = 1000;
//Time to flush after calibration gasses
unsigned long calFlushDuration = 15000;

//Array of booleans to determine which channels are currently being checked
bool inService[15] = { true, true, true, true, true, true, true, true, true, true, true, true, true, true, true };

//Most recent 5 readings from each sensor (averaged 4 values each)
int ch4Values[5] = { 0, 0, 0, 0, 0 };
int co2Values[5] = { 0, 0, 0, 0, 0 };
//Where the next value is being placed
int currentValueIndex = 0;
//The 5 values up to and including the peak value for each sensor
int ch4ValuesPeak[5] = { 0, 0, 0, 0, 0 };
int co2ValuesPeak[5] = { 0, 0, 0, 0, 0 };
//The set of values to be averaged from each sensor to produce a data point
int ch4ValueSet[averageCount];
int co2ValueSet[averageCount];
//Current index within the set of values - used to determine if an average needs to be taken and reset
int setPosition = 0;

//The set of values to be averaged from each sensor to produce a calibration data point
int calCh4ValueSet[averageCount];
int calCo2ValueSet[averageCount];
//Current index within the set of calibration averaging valuves
int calSetPosition = 0;

//Maximum reading values for each sensor
int ch4Max = 0;
int co2Max = 0;

//Which step in the sequence is the current one
int currentState = 0;
//Which valve in the sequence is being tested
int currentValve = 0;

//The flush position
const int flushValve = 15;

//Pins to use for controlling solenoid power
const int positive = 26;
const int negative = 33;

//Calibration arrays - used to convert sensor integer to percentages
double ch4Calibration[4];
double co2Calibration[4];

//If the current calibration being handled is methane
bool currentCalDataCh4 = false;
//Array to hold received calibration values
double newCalibration[4];

//Current line array in the file being read
char fileLine[100];
//Position in the line array
int linePosition = 0;

//Current message received via serial
char currentMessage[100];
int currentMsgPos = 0;

//Array to store message split into parts
char msgParts[3][33];

//Array to store the requested file name
char fileToDownload[33];
int downloadTimeout = 5;

//Booleans to store current calibration states
bool calibrated = false;
bool calibrating = false;
bool readingCalibration = false;
//Booleans to know what kind of message to send back once calibration points are done
bool checkingCh4 = false;
bool checkingCo2 = false;
bool flushingCalibration = false;
//Maximum values found during calibration read
int calCh4Max = 0;
int calCo2Max = 0;
//Timing for calibration read
unsigned long calWaitTime;
//To store the current time
unsigned long currentTime;
//Change in time since last check
unsigned long timeDifference;

//Strings to store names of files being written to
char outputFileName[28] = "/files/eventLog_200001.csv";
char extraOutputFileName[28] = "/files/pointLog_200001.csv";
//Where to start changing the strings for different dates
const int outputFilePos = 16;

//Create the real time clock
RTC_DS3231 rtc;

// Create an USB Serial port
USBCDC USBSerial;               // Create Object @@@@ USB_Update Branch

//Arrays to store the time as text and numbers
char timeStamp[19];
int timeParts[6];
//Whether setup was successful
bool setupCorrectly = false;

//Arrays to hold most recent percentage for each reactor - for use by interface to recall last data
int previousCh4Percent[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int previousCo2Percent[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int valveOpen = -1;

ESP32AnalogRead adc;

void setup() {
  //Change the attenuation on the Analogue to Digital converter
  //analogSetAttenuation(ADC_6db);
  //Start a serial connection (used for usb port)
  Serial.begin(115200);
  USBSerial.begin();              // Start USB serial Communication
  USB.begin();                    // Start USB Native Port
  delay(2000);
  Serial.write("Starting\n");
  Wire.begin(2, 3);
  //Start the real time clock
  if (!rtc.begin()) {
    //Serial.write("RTC failed to start.\n");
    USBSerial.write("RTC failed to start.\n");  // Send the messages by USB too (Or replace the serial)
  } else {
    //Start the SPI bus
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    //Try and connect to the SD card
    if (!SD.begin(SD_CS)) {
      USBSerial.write("Error, could not find SD card.\n");
    } else {
      USBSerial.write("SD card connected\n");
      //Check if the data file folder exists and create it if not
      if (!SD.exists("/files")) {
        USBSerial.write("Creating Folder.\n");
        SD.mkdir("/files");
        USBSerial.write("Folder Created.\n");
      } else {
        USBSerial.write("Folder already exists.\n");
      }
      //Setup completed successfully
      setupCorrectly = true;
    }
  }

  //Setup the pins for the solenoid power control
  pinMode(positive, OUTPUT);
  pinMode(negative, OUTPUT);
  //Set both to high as default
  digitalWrite(positive, HIGH);
  digitalWrite(negative, HIGH);

  //Iterate through pins
  for (int i = 0; i < 16; i++) {
    //Setup pin as output and high
    pinMode(solenoidValvePins[i], OUTPUT);
    digitalWrite(solenoidValvePins[i], HIGH);
    //Cycle power pins
    digitalWrite(positive, HIGH);
    digitalWrite(negative, LOW);
    delay(10);
    digitalWrite(positive, HIGH);
    digitalWrite(negative, HIGH);
    delay(10);
    //Close the valve
    digitalWrite(solenoidValvePins[i], LOW);
  }

  USBSerial.write("Starting calibration.\n");
  //Read calibration values if possible
  readCalibration();
  if (calibrated) {
    USBSerial.write("Calibrated Successfully.\n");
  } else {
    USBSerial.write("Calibration read failed.\n");
  }
  //Output the read calibration values
  USBSerial.write("Methane:\n");
  for (int i = 0; i < 4; i++) {
    USBSerial.println(ch4Calibration[i], 6);
  }
  USBSerial.write("\nCarbon Dioxide:\n");
  for (int i = 0; i < 4; i++) {
    USBSerial.println(co2Calibration[i], 6);
  }

  //Read the timing and in service files
  readTiming();
  USBSerial.write("Timings read\n");
  readInService();
  USBSerial.write("In Service read\n");

  //Setup the arrays for averaging values
  for (int i = 0; i < averageCount; i++) {
    ch4ValueSet[i] = 0;
    co2ValueSet[i] = 0;
  }

  //Get the valve position from the file
  currentValve = readValvePosition();
  //If it is not a valid valve
  if (currentValve < 0 || currentValve > 14 || !inService[currentValve]) {
    currentValve = firstValve();
    if (currentValve != -1) {
      //Store the new valve position
      writeValvePosition();
    }
  }
  if (setupCorrectly) {
    USBSerial.write("Setup Correctly\n");
  }
}

uint32_t getSecondsSince() {
  /*Returns the number of seconds since the unix epoch*/
  if (setupCorrectly) {
    uint32_t epochTime = rtc.now().unixtime();
    return epochTime;
  }

  return -1;
}

void getTime() {
  /*Get the time from the RTC and convert to text*/
  DateTime timeNow = rtc.now();
  timeParts[5] = timeNow.second();
  timeParts[4] = timeNow.minute();
  timeParts[3] = timeNow.hour();
  timeParts[2] = timeNow.day();
  timeParts[1] = timeNow.month();
  timeParts[0] = timeNow.year();

  //Buffer to hold current number
  char buff[5];
  int timePos = 0;

  //Iterate through each part
  for (int part = 0; part < 6; part = part + 1) {
    //Convert to a c string
    itoa(timeParts[part], buff, 10);
    bool done = false;
    //Iterate through characters
    for (int ch = 0; ch < 5 && !done; ch = ch + 1) {
      //If the end
      if (buff[ch] == '\0') {
        done = true;
      } else {
        //If still within the buffer
        if (timePos < 18) {
          //Add character to the buffer
          timeStamp[timePos] = buff[ch];
          timePos = timePos + 1;
        }
      }
    }

    //Add a space if there are still more values to add
    if (part != 5 && timePos < 18) {
      timeStamp[timePos] = '/';
      timePos = timePos + 1;
    }
  }
  //Store year and month as char arrays
  char yearCh[5];
  char monthCh[3];
  itoa(timeParts[0], yearCh, 10);
  itoa(timeParts[1], monthCh, 10);
  //Iterate and add year to file name parts
  for (int i = 0; i < 5; i++) {
    outputFileName[outputFilePos + i] = yearCh[i];
    extraOutputFileName[outputFilePos + i] = yearCh[i];
  }
  //If the month is more than 10
  if (timeParts[1] > 9) {
    //Write character into file names
    for (int i = 0; i < 3; i++) {
      outputFileName[outputFilePos + i + 4] = monthCh[i];
      extraOutputFileName[outputFilePos + i + 4] = monthCh[i];
    }
  } else {
    //Add month number with leading 0 to file names
    outputFileName[outputFilePos + 4] = '0';
    outputFileName[outputFilePos + 5] = monthCh[0];
    extraOutputFileName[outputFilePos + 4] = '0';
    extraOutputFileName[outputFilePos + 5] = monthCh[0];
  }
  //Add terminator character
  timeStamp[timePos] = '\0';
}

void getMemoryData() {
  /*Find how much memory is present on the board and how much has been used and output this information via serial*/
  if (setupCorrectly) {
    //Get the total number of bytes available to the file system
    double totalBytes = SD.totalBytes();
    //Get the number of bytes being used to store information
    double usedBytes = SD.usedBytes();
    //Buffers used to convert the integers into c strings
    char totalBuff[65];
    char usedBuff[65];
    //Convert to c strings so that the numbers can be sent via serial
    int length1 = sprintf(totalBuff, "%.0f", totalBytes);
    int length2 = sprintf(usedBuff, "%.0f", usedBytes);

    //Send message
    USBSerial.write("memory ");
    USBSerial.write(totalBuff);
    USBSerial.write(" ");
    USBSerial.write(usedBuff);
    USBSerial.write("\n");
  }
}

void listFiles() {
  /*Outputs a list of files one line at a time preceded by the keyword: file*/
  if (setupCorrectly) {
    getMemoryData();
    char sizeBuff[33];
    USBSerial.write("file start\n");
    //Open the root directory
    File root = SD.open("/files");
    //Open the first file
    File currentFile = root.openNextFile();
    while (currentFile) {
      //If there is a file
      if (currentFile and !currentFile.isDirectory()) {
        int fileSize = currentFile.size();
        itoa(fileSize, sizeBuff, 10);
        //Send the message to give the file name
        char fileName[33];
        strcpy(fileName, currentFile.name());
        USBSerial.write("file ");
        USBSerial.write(fileName);
        USBSerial.write(" ");
        USBSerial.write(sizeBuff);
        USBSerial.write("\n");
      }
      //Open the next file
      currentFile = root.openNextFile();
    }
    //Close the root location
    root.close();
  }
  //Send signal to indicate that it has finished (whether it was able to send files or not)
  USBSerial.write("done files\n");
}

void downloadFile() {
  /*Send the file as a download via serial then resume the Arduino communication*/
  if (SD.exists(fileToDownload)) {
    bool restarting = false;
    if (currentState == 1) {
      closeValve(currentValve);
      resetValues();
      currentState = 0;
      restarting = true;
    }
    char lastChar;
    //Open the file to send
    File fileToSend = SD.open(fileToDownload, FILE_READ);
    //Get the number of characters in the file
    int charNumber = fileToSend.available();
    char charNumberBuffer[33];
    itoa(charNumber, charNumberBuffer, 10);
    //Send message to indicate that the file is about to be transferred, with the file name
    USBSerial.write("download start ");
    USBSerial.write(fileToDownload);
    USBSerial.write(" ");
    USBSerial.write(charNumberBuffer);
    USBSerial.write("\n");

    uint32_t lastTime = getSecondsSince();
    bool terminated = false;
    //If there is data to send
    if (charNumber > 0) {
      //keyword to indicate a line is being sent
      USBSerial.write("download ");
    }
    //Iterate through each character index
    for (int cha = 0; cha < charNumber && !terminated; cha = cha + 1) {
      //Read the character
      char nextChar = fileToSend.read();
      char next[2] = { nextChar, '\0' };
      //Write the character
      USBSerial.write(next);
      //If this is the end of a line
      if (nextChar == '\n' && cha != charNumber - 1) {
        //Message received to indicate that it is ready for the next item
        char waitMessage[64];
        int waitIndex = 0;
        bool waiting = true;
        //Repeat until no longer waiting for a response
        while (waiting) {
          //Get the current time
          uint32_t currentTime = getSecondsSince();
          //If the timeout is exceeded
          if (currentTime - lastTime > downloadTimeout) {
            //Stop waiting - failed, terminate the data transfer
            waiting = false;
            terminated = true;
          }

          //While there are characters to read from python
          while (USBSerial.available()) {
            //Read character
            char msgChar = '\0';
            msgChar = USBSerial.read();
            //If it is a new line
            if (msgChar == '\n') {
              //If the message is 'next' - to indicate that it is ready for the next item
              if (strcmp(waitMessage, "next") == 0) {
                //No longer waiting, update the time
                waiting = false;
                lastTime = currentTime;
              }
              //Reset message and index
              waitMessage[0] = '\0';
              waitIndex = 0;
              //If it is any other character and within the message length
            } else if (waitIndex < 63) {
              //Add the character
              waitMessage[waitIndex] = msgChar;
              //Add termination character and increment counter
              waitMessage[waitIndex + 1] = '\0';
              waitIndex = waitIndex + 1;
            }
          }
        }

        //If the download did not time out
        if (!terminated) {
          //Keyword to indicate the next line
          USBSerial.write("download ");
        }
      }
      //Store the final character that was sent
      lastChar = nextChar;
    }

    //If the file did not terminate with a newline send one to terminate the command (and there was something to send)
    if (lastChar != '\n' && charNumber > 0 && !terminated) {
      USBSerial.write("\n");
    }

    //Close the file
    fileToSend.close();
    if (!terminated) {
      //Send message to indicate that file has been downloaded fully
      USBSerial.write("download stop\n");
    } else {
      //Send message to indicate that something went wrong
      USBSerial.write("download failed\n");
    }

    if(restarting){
      lastAction = millis();
    }

  } else {
    //Message to indicate that the requested file does not exist
    USBSerial.write("failed download nofile\n");
  }
}

void readSerial() {
  /*Read data if it is available*/
  if (USBSerial.available()) {
    //Read the next character from serial
    char c = USBSerial.read();
    //If it is a new line
    if (c == '\n') {
      //If there is other stored text
      if (currentMsgPos != 0) {
        //Add termination character
        currentMessage[currentMsgPos] = '\0';
        currentMsgPos = 0;
        //Process the received message
        processMessage();
      }
    } else {
      //If there is still space to store more characters
      if (currentMsgPos < 99) {
        //Add the character at counter position and increment counter
        currentMessage[currentMsgPos] = c;
        currentMsgPos = currentMsgPos + 1;
      }
    }
  }
}

void outputCalibration() {
  /*Send the current calibration values via serial*/
  //Send the start of the calibration data message
  USBSerial.write("currentcal ");
  //Iterate methane
  for (int i = 0; i < 4; i++) {
    //Send the calibration part
    USBSerial.print(ch4Calibration[i]);
    USBSerial.print(" ");
  }
  //Iterate carbon dioxide
  for (int i = 0; i < 4; i++) {
    //Send the calibration part followed by a space or new line for the last value
    if (i != 3) {
      USBSerial.print(co2Calibration[i]);
      USBSerial.print(" ");
    } else {
      USBSerial.println(co2Calibration[i]);
    }
  }
}

void processMessage() {
  /*Handle the incoming message and respond appropriately*/
  int part = 0;
  int cha = 0;
  bool done = false;
  //Split into parts based on space (3 maximum)
  //Iterate through characters
  for (int chaPos = 0; chaPos < 100 && !done; chaPos++) {
    //Get the character
    char c = currentMessage[chaPos];
    //If it is the end of the message
    if (c == '\0') {
      //Add terminator to the end of this and every following part
      msgParts[part][cha] = '\0';
      for (int p = part + 1; p < 3; p++) {
        msgParts[p][0] = '\0';
      }
      //Finished splitting text
      done = true;
      //If it is a space
    } else if (c == ' ') {
      //Add terminator to current part
      msgParts[part][cha] = '\0';
      //Move to next part
      part = part + 1;
      cha = 0;
      //If there are not any more parts then finished
      if (part > 2) {
        done = true;
      }
    } else {
      //If the current character has not exceeded the length limit of the current part
      if (cha < 32) {
        //Add it to the part and increment counter
        msgParts[part][cha] = c;
        cha = cha + 1;
      }
    }
  }

  //If it is an info request, respond with calibrating status
  if (strcmp(msgParts[0], "info") == 0) {
    USBSerial.write("info ");
    if (calibrating) {
      USBSerial.write("true ");
    } else {
      USBSerial.write("false ");
    }
    USBSerial.println(valveOpen);
  }
  //If it is a request for the file names, send the list of names
  else if (strcmp(msgParts[0], "files") == 0) {
    listFiles();
  }
  //If it is asking to set the clock time
  else if (strcmp(msgParts[0], "timeset") == 0) {
    //If currently in calibration mode
    if (calibrating) {
      //Buffer to store the values
      char timeValues[6][5];
      //Array to store the integer version of the values
      int timeValuesInt[6];

      bool timeDone = false;
      int value = 0;
      int index = 0;
      //Iterate through the characters in the message
      for (int tc = 0; tc < 33 && !timeDone; tc++) {
        //If the end has been reached
        if (msgParts[1][tc] == '\0') {
          timeDone = true;
        } else {
          //If this is a separator
          if (msgParts[1][tc] == ',') {
            //Add terminator to the value
            timeValues[value][index] = '\0';
            //Move on to next value if there is one
            if (value < 5) {
              value = value + 1;
              index = 0;
            } else {
              //If there is not a next value - finished
              timeDone = true;
            }
          } else {
            //Add the character to the value
            timeValues[value][index] = msgParts[1][tc];
            //Move to next position
            index = index + 1;
            //Limit to prevent out of range
            if (index > 4) {
              index = 4;
            }
          }
        }
      }
      //Add terminator character to all following values
      timeValues[value][index] = '\0';
      for (int v = value + 1; v < 6; v = v + 1) {
        timeValues[v][0] = '\0';
      }

      //Iterate through and convert to integers
      for (int i = 0; i < 6; i = i + 1) {
        timeValuesInt[i] = atoi(timeValues[i]);
      }
      //Set the time from the values
      setRTCTime(timeValuesInt[0], timeValuesInt[1], timeValuesInt[2], timeValuesInt[3], timeValuesInt[4], timeValuesInt[5]);
      USBSerial.write("done timeset\n");
    } else {
      USBSerial.write("failed timeset notcalibrating\n");
    }
  }
  //If it is a request for the time in the clock
  else if (strcmp(msgParts[0], "timeget") == 0) {
    //Get the time
    getTime();
    //Ouput the time over serial
    USBSerial.write("time ");
    USBSerial.print(timeParts[0]);
    USBSerial.print(",");
    USBSerial.print(timeParts[1]);
    USBSerial.print(",");
    USBSerial.print(timeParts[2]);
    USBSerial.print(",");
    USBSerial.print(timeParts[3]);
    USBSerial.print(",");
    USBSerial.print(timeParts[4]);
    USBSerial.print(",");
    USBSerial.println(timeParts[5]);
  }
  //If it is a request to start calibrating
  else if (strcmp(msgParts[0], "startcal") == 0) {
    //If not in calibration mode
    if (!calibrating) {
      //Change to calibration mode
      calibrating = true;
      //Close valve and reset state if needed
      if (currentState == 1) {
        closeValve(currentValve);
      }
      currentState = 0;
      USBSerial.write("done startcal\n");
    } else {
      USBSerial.write("failed startcal calibrating\n");
    }
    //Send the stored calibration
    outputCalibration();
  }
  //If it is a request to end calibration
  else if (strcmp(msgParts[0], "endcal") == 0) {
    //If currently in calibration mode
    if (calibrating) {
      //Return to normal mode
      calibrating = false;
      USBSerial.write("done endcal\n");
    } else {
      USBSerial.write("failed endcal notcalibrating\n");
    }
  }
  //If it is a request to download a file
  else if (strcmp(msgParts[0], "download") == 0) {
    //Copy the file name to be downloaded
    for (int ch = 0; ch < 33; ch++) {
      fileToDownload[ch] = msgParts[1][ch];
    }
    //Download the file
    downloadFile();
  }
  //If it is a request to delete a file
  else if (strcmp(msgParts[0], "delete") == 0) {
    //If the file exists
    if (SD.exists(msgParts[1])) {
      //Delete the file
      SD.remove(msgParts[1]);
      //Send signal that file was removed successfully
      USBSerial.write("done delete\n");
    } else {
      //Send signal that the file did not exist
      USBSerial.write("failed delete nofile\n");
    }
  }
  //If it is a request to perform a calibration point
  else if (strcmp(msgParts[0], "point") == 0) {
    //If in calibration mode
    if (calibrating) {
      //If not already reading a point
      if (!readingCalibration) {
        //Store the time to wait for
        int waitTime = atoi(msgParts[2]);
        //Convert to UL and use default value if invalid
        if (waitTime == 0) {
          calWaitTime = 3000UL;
        } else {
          calWaitTime = (unsigned long)waitTime;
        }
        //If checking methane
        if (strcmp(msgParts[1], "ch4") == 0) {
          //Store correct gasses
          checkingCh4 = true;
          checkingCo2 = false;
          resetCalibrationValues();
          //Start reading
          readingCalibration = true;
          //Open the valve
          openValve(0);
          //Store time
          lastAction = millis();
          //If checking carbon dioxide
        } else if (strcmp(msgParts[1], "co2") == 0) {
          //Store correct gasses
          checkingCo2 = true;
          checkingCh4 = false;
          resetCalibrationValues();
          //Start reading
          readingCalibration = true;
          //Open the valve
          openValve(0);
          //Store time
          lastAction = millis();
        } else {
          USBSerial.write("failed point invalidtype\n");
        }
      } else {
        USBSerial.write("failed point alreadychecking\n");
      }
    } else {
      USBSerial.write("failed point notcalibrating\n");
    }
  }
  //If it is a request to set the calibration of either type
  else if (strcmp(msgParts[0], "setch4cal") == 0 || strcmp(msgParts[0], "setco2cal") == 0) {
    USBSerial.write("Found calibration set command\n");
    //If in calibration mode
    if (calibrating) {
      USBSerial.write("Setting calibration\n");
      //Check if setting methane or carbon dioxide
      currentCalDataCh4 = strcmp(msgParts[0], "setch4cal") == 0;
      //Create char buffer to store data
      char* calPart;
      //Split based on |
      calPart = strtok(msgParts[1], "|");
      //If there is a part
      if (calPart != NULL) {
        //Convert to floating point value
        newCalibration[0] = atof(calPart);
      } else {
        //Use default as 0
        newCalibration[0] = 0.0;
      }
      //Split again
      calPart = strtok(NULL, "|");
      //Convert to float and store if found
      if (calPart != NULL) {
        newCalibration[1] = atof(calPart);
      } else {
        newCalibration[1] = 0.0;
      }
      //Split again
      calPart = strtok(msgParts[2], "|");
      //Convert to float and store if found
      if (calPart != NULL) {
        newCalibration[2] = atof(calPart);
      } else {
        newCalibration[2] = 0.0;
      }
      //Split again
      calPart = strtok(NULL, "|");
      //Convert to float and store if found
      if (calPart != NULL) {
        newCalibration[3] = atof(calPart);
      } else {
        newCalibration[3] = 0.0;
      }

      //Iterate through calibration points
      for (int i = 0; i < 4; i++) {
        //Store in methane or carbon dioxide based on command
        if (currentCalDataCh4) {
          ch4Calibration[i] = newCalibration[i];
        } else {
          co2Calibration[i] = newCalibration[i];
        }
      }
      //Store calibration in file
      writeCalibration(currentCalDataCh4);
      USBSerial.write("done calset\n");
    } else {
      USBSerial.write("failed calset notcalibrating\n");
    }
    //Output the current calibration via serial
    outputCalibration();
  }
  //If it is a request to set the valve timings
  else if (strcmp(msgParts[0], "timingset") == 0) {
    //If in calibration mode
    if (calibrating) {
      //convert to integers
      int openReceived = atoi(msgParts[1]);
      int flushReceived = atoi(msgParts[2]);
      //Check neither value was 0
      if (openReceived == 0) {
        USBSerial.write("failed timingset noopen\n");
      } else {
        if (flushReceived == 0) {
          USBSerial.write("failed timingset noflush\n");
        } else {
          //Store the values as UL
          openDuration = (unsigned long)openReceived;
          flushDuration = (unsigned long)flushReceived;
          //Not used currently
          readInterval = (unsigned long)floor(openDuration / 60);
          //Update the timing file
          writeTiming();
          USBSerial.write("done timingset\n");
        }
      }
    } else {
      USBSerial.write("failed timingset notcalibrating\n");
    }
  }
  //If it is a request for the current valve timings
  else if (strcmp(msgParts[0], "timingget") == 0) {
    //If in calibration mode
    //if (calibrating) {
      //Send the valve timings
      USBSerial.print("timing ");
      USBSerial.print(openDuration);
      USBSerial.print(" ");
      USBSerial.println(flushDuration);
    //} else {
      //USBSerial.write("failed timingget notcalibrating\n");
    //}
  }
  //If it is a request for the in service state
  else if (strcmp(msgParts[0], "serviceget") == 0) {
    //Write the service data
    USBSerial.write("service");
    for (int i = 0; i < 15; i++) {
      if (inService[i]) {
        USBSerial.write(" 1");
      } else {
        USBSerial.write(" 0");
      }
    }
    USBSerial.write("\n");
  }
  //If it is a request to set the in service state
  else if (strcmp(msgParts[0], "serviceset") == 0) {
    //If in calibration mode
    if (calibrating) {
      //Iterate through message
      for (int i = 0; i < 15; i++) {
        //If it is not a 0 it is being used
        inService[i] = msgParts[1][i] != '0';
      }
      //Update in service file
      writeInService();
      USBSerial.write("done serviceset\n");
    } else {
      USBSerial.write("failed serviceset notcalibrating\n");
    }
  }
  //If it is a request for past data points
  else if (strcmp(msgParts[0], "getpast") == 0) {
    //Send each of the stored data points
    USBSerial.print("pastdata ");
    //Iterate points
    for(int i = 0; i < 15; i++){
      //Send methane
      USBSerial.print(previousCh4Percent[i]);
      USBSerial.print(" ");
      //If it is not the last
      if(i < 14){
        //Send carbon dioxide
        USBSerial.print(previousCo2Percent[i]);
        USBSerial.print(" ");
      }else{
        //Send carbon dioxide with new line
        USBSerial.println(previousCo2Percent[i]);
      }
    }
  }
}

void setRTCTime(int y, int m, int d, int h, int mi, int s) {
  /*Set the current time of the RTC*/
  rtc.adjust(DateTime(y, m, d, h, mi, s));
}

void readCalibration() {
  /*Read the calibration that is stored in the file*/
  //Number of values read for each of the gasses
  int ch4CalCount = 0;
  int co2CalCount = 0;

  //Reset the calibration stored in memory
  for (int i = 0; i < 4; i++) {
    ch4Calibration[i] = 0.0;
    co2Calibration[i] = 0.0;
  }

  //If the methane calibration file exists
  if (SD.exists("/ch4Calibration.txt")) {
    //Open the file
    File currentFile = SD.open("/ch4Calibration.txt", FILE_READ);
    int line = 0;
    linePosition = 0;
    //For every character in the file
    while (currentFile.available()) {
      //Reach the character
      char c = currentFile.read();
      //If it is a new line
      if (c == '\n') {
        //If characters have not been read this line
        if (linePosition == 0) {
          //Set calibration value to 0.0
          ch4Calibration[line] = 0.0;
        } else {
          //Add terminator character
          fileLine[linePosition] = '\0';
          //If the value is still within the calibration length
          if (line < 4) {
            //Convert to float
            ch4Calibration[line] = atof(fileLine);
            //Increment number of methane read
            ch4CalCount = line + 1;
          }
        }
        //Increment line and reset position
        line = line + 1;
        linePosition = 0;
      } else {
        //If it isn't a return or tab
        if (c != '\t' && c != '\r') {
          //If there is space for the character
          if (linePosition < 99) {
            //Add the character to the stored line
            fileLine[linePosition] = c;
            //Increment position
            linePosition = linePosition + 1;
          }
        }
      }
    }
    //Close the file
    currentFile.close();
  } else {
    //Write blank methane calibration
    writeCalibration(true);
  }
  //If the carbon dioxide calibration file exists
  if (SD.exists("/co2Calibration.txt")) {
    //Open the file
    File currentFile = SD.open("/co2Calibration.txt", FILE_READ);
    int line = 0;
    linePosition = 0;
    //Iterate for each character within the file
    while (currentFile.available()) {
      //Read the character
      char c = currentFile.read();
      //If the character is a new line
      if (c == '\n') {
        //If characters have not been read this line
        if (linePosition == 0) {
          //Set the calibration value to 0.0
          ch4Calibration[line] = 0.0;
        } else {
          //Add the termination character
          fileLine[linePosition] = '\0';
          //If it is within calibration length
          if (line < 4) {
            //Convert to float
            co2Calibration[line] = atof(fileLine);
            //Increment carbon dioxide count
            co2CalCount = line + 1;
          }
        }
        //Increment line and reset position
        line = line + 1;
        linePosition = 0;
      } else {
        //If the character is not a tab or return
        if (c != '\t' && c != '\r') {
          //If there is still space in the line
          if (linePosition < 99) {
            //Add the character to the stored line
            fileLine[linePosition] = c;
            //Increment line position
            linePosition = linePosition + 1;
          }
        }
      }
    }
    //Close the file
    currentFile.close();
  } else {
    //Write the blank carbon dioxide calibration
    writeCalibration(false);
  }

  //If methane and carbon dioxide read at least 1 value
  calibrated = ch4CalCount > 1 && co2CalCount > 1;
}

void writeCalibration(bool methane) {
  /*Write the calibration for methane (true) or carbon dioxide (false) to the appropriate file*/
  if (methane) {
    //Open the methane file (clear and write)
    File currentFile = SD.open("/ch4Calibration.txt", FILE_WRITE);
    //Iterate through methane calibration
    for (int i = 0; i < 4; i++) {
      //Write the number as a line
      currentFile.println(ch4Calibration[i], 4);
    }
    //Close the file
    currentFile.close();
  } else {
    //Open the carbon dioxide file (clear and write)
    File currentFile = SD.open("/co2Calibration.txt", FILE_WRITE);
    //Iterate through carbon dioxide calibration
    for (int i = 0; i < 4; i++) {
      //Write the number as a line
      currentFile.println(co2Calibration[i], 4);
    }
    //Close the file
    currentFile.close();
  }
}

void readTiming() {
  /*Read the valve timing from the file*/
  //If the file exists
  if (SD.exists("/timings.txt")) {
    //Open the file
    File currentFile = SD.open("/timings.txt", FILE_READ);
    int line = 0;
    linePosition = 0;
    bool setOpen = false;
    bool setFlush = false;
    //Iterate through the characters in the file
    while (currentFile.available()) {
      //Read the character
      char c = currentFile.read();
      //If it is a new line
      if (c == '\n') {
        //If other characters have been read
        if (linePosition != 0) {
          //Add terminator character
          fileLine[linePosition] = '\0';
          //Convert line to integer
          int value = atoi(fileLine);
          //If the value is more than 0
          if (value > 0) {
            //If it is the first line
            if (line == 0) {
              //Assign the time to keep the valve open for
              openDuration = (unsigned long)value;
              //Calculate the interval between readings - not currently used (may be removed)
              readInterval = (unsigned long)floor(openDuration / 60);
              //A value for open time has been set
              setOpen = true;
              //If it is the second line
            } else if (line == 1) {
              //Assign the time to flush between tests for
              flushDuration = (unsigned long)value;
              //A value for the flush has been set
              setFlush = true;
            }
          }
        }
        //Increment line and reset position in line
        line = line + 1;
        linePosition = 0;
      } else {
        //If it is not a tab or return
        if (c != '\t' && c != '\r') {
          //If there is still space in the line
          if (linePosition < 99) {
            //Store the character and increment the position
            fileLine[linePosition] = c;
            linePosition = linePosition + 1;
          }
        }
      }
    }
    //Close the file
    currentFile.close();
    //If either value was not read
    if (!(setOpen && setFlush)) {
      //Write the current set of values into the file
      writeTiming();
    }
  } else {
    //Write the default values so the file exists
    writeTiming();
  }
}

void writeTiming() {
  /*Write the timing into the file*/
  //Open the file
  File currentFile = SD.open("/timings.txt", FILE_WRITE);
  //Write the open time and flush time on separate lines
  currentFile.println(openDuration);
  currentFile.println(flushDuration);
  //Close the file
  currentFile.close();
  USBSerial.write("Timing file written\n");
}

void readInService() {
  /*Read the file that stores the valve in service information*/
  if (SD.exists("/inService.txt")) {
    //Open the file to read
    File currentFile = SD.open("/inService.txt", FILE_READ);
    int charIndex = 0;
    //Iterate through every character in the file
    while (currentFile.available()) {
      //Read the file
      char c = currentFile.read();
      //For the first 15 characters
      if (charIndex < 15) {
        //If it is a 0
        if (c == '0') {
          //Channel not in service
          inService[charIndex] = false;
        } else {
          //Channel in service
          inService[charIndex] = true;
        }
      }
      //Increment character index
      charIndex = charIndex + 1;
    }
    //Close the file
    currentFile.close();
  } else {
    //Write the channel in service data so it exists
    writeInService();
  }
}

void writeInService() {
  /*Write the current in service data to the file*/
  //Opent the file to write
  File currentFile = SD.open("/inService.txt", FILE_WRITE);
  //Iterate through channel index
  for (int i = 0; i < 15; i++) {
    //If the channel is in service
    if (inService[i]) {
      //Write a 1
      currentFile.write('1');
    } else {
      //Write a 0
      currentFile.write('0');
    }
  }
  //Close the file
  currentFile.close();
}

int readValvePosition() {
  /*Read which valve in sequence has been reached*/
  //If the file exists
  if (SD.exists("/currentValve.txt")) {
    //Open the file to read
    File currentFile = SD.open("/currentValve.txt", FILE_READ);
    //Array to store read data
    char textBuffer[3];
    int charPos = 0;
    //White there are characters to read and not reached the maximum length
    while (currentFile.available() && charPos < 2) {
      //Read the next character
      char c = currentFile.read();
      //Store the character
      textBuffer[charPos] = c;
      //Increment counter
      charPos = charPos + 1;
    }
    //Add terminator character
    textBuffer[charPos] = '\0';
    //Convert to integer
    int valveNumber = atoi(textBuffer);
    //If the valve number is a valid value
    if (valveNumber > -1 && valveNumber < 15) {
      //Return the read value
      return valveNumber;
    }
  }
  //Indicate that a value could not be read
  return -1;
}

void writeValvePosition() {
  /*Write the current valve index to the file*/
  //Open the file, write and close again
  File currentFile = SD.open("/currentValve.txt", FILE_WRITE);
  currentFile.write(currentValve);
  currentFile.close();
}

void openValve(int valveNumber) {
  /*Open a specific valve*/
  //If the valve is a valid channel number
  if (valveNumber > -1 && valveNumber < 16) {
    //Set the solenoid pin high
    digitalWrite(solenoidValvePins[valveNumber], HIGH);
    //Set the positive pin to low and negative to high
    digitalWrite(positive, LOW);
    digitalWrite(negative, HIGH);
    //Short delay
    delay(100);
    //Set the solenoid pin high
    digitalWrite(solenoidValvePins[valveNumber], HIGH);
    //Set the positive and negative pins to high
    digitalWrite(positive, HIGH);
    digitalWrite(negative, HIGH);
    Serial.write("valve ");
    Serial.print(valveNumber);
    Serial.write(" opened\n");

    valveOpen = valveNumber;
  }
}

void closeValve(int valveNumber) {
  /*Close a specific valve*/
  //If the valve is a valid channel number
  if (valveNumber > -1 && valveNumber < 16) {
    //Set the solenoid pin high
    digitalWrite(solenoidValvePins[valveNumber], HIGH);
    //Set the potitive pin to high and the negative pin to low
    digitalWrite(positive, HIGH);
    digitalWrite(negative, LOW);
    //Short delay
    delay(100);
    //Set the solenoid pin to low
    digitalWrite(solenoidValvePins[valveNumber], LOW);
    USBSerial.write("valve ");
    USBSerial.print(valveNumber);
    USBSerial.write(" closed\n");

    if(valveNumber == valveOpen){
      valveOpen = -1;
    }
  }
}

void resetValues() {
  /*Reset the stored peak and average values*/
  //Iterate through peak values and reset
  for (int i = 0; i < 5; i++) {
    ch4Values[i] = 0;
    co2Values[i] = 0;
    ch4ValuesPeak[i] = 0;
    co2ValuesPeak[i] = 0;
  }
  //Iterate through averaging values and reset
  for (int i = 0; i < averageCount; i++) {
    ch4ValueSet[i] = 0;
    co2ValueSet[i] = 0;
  }
  //Reset the current positions and maximums
  currentValueIndex = 0;
  ch4Max = 0;
  co2Max = 0;
  setPosition = 0;
}

void readValues() {
  /*Read the values from the sensors*/
  //Start converting carbon dioxide value
  //int co2Value = analogRead(co2Pin);
  adc.attach(co2Pin);
  int co2Value = adc.readMiliVolts();
  //Delay to allow A to D to process
  delay(20);
  //Start converting methane value
  //int ch4Value = analogRead(ch4Pin);
  adc.attach(ch4Pin);
  int ch4Value = adc.readMiliVolts();
  //Delay to allow A to D to process
  delay(20);
  //Store value in averaging array
  ch4ValueSet[setPosition] = ch4Value;
  co2ValueSet[setPosition] = co2Value;
  //Increment store position
  setPosition = setPosition + 1;
  //If count has been reached for calculating the average
  if (setPosition >= averageCount) {
    //Average stored values and add to data
    calculateValues();
    //Reset average counter position
    setPosition = 0;
  }
}

void calculateValues() {
  /*Average read values and store*/
  int totalCh4 = 0;
  int totalCo2 = 0;
  //Iterate through values being averaged
  for (int i = 0; i < averageCount; i++) {
    //Sum values
    totalCh4 = totalCh4 + ch4ValueSet[i];
    totalCo2 = totalCo2 + co2ValueSet[i];
  }
  //Divide by count to find average
  int ch4Result = floor(totalCh4 / averageCount);
  int co2Result = floor(totalCo2 / averageCount);

  //If the current peak exceeds start
  if (currentValueIndex > 4) {
    //Iterate through items after the first
    for (int i = 1; i < 5; i++) {
      //Move down one place
      ch4Values[i - 1] = ch4Values[i];
      co2Values[i - 1] = co2Values[i];
    }
    //Add new values to the end
    ch4Values[4] = ch4Result;
    co2Values[4] = co2Result;
  } else {
    //Add values to current end position
    ch4Values[currentValueIndex] = ch4Result;
    co2Values[currentValueIndex] = co2Result;
    //Increment current position
    currentValueIndex = currentValueIndex + 1;
  }

  //If a new peak methane has been found
  if (ch4Result > ch4Max) {
    //Store the new maximum value
    ch4Max = ch4Result;
    //Iterate through peak length
    for (int i = 0; i < 5; i++) {
      //Store current values in peak
      ch4ValuesPeak[i] = ch4Values[i];
    }
  }
  //If a new peak carbon dioxide has been found
  if (co2Result > co2Max) {
    //Store the new maximum value
    co2Max = co2Result;
    //Iterate through peak length
    for (int i = 0; i < 5; i++) {
      //Store current values in peak
      co2ValuesPeak[i] = co2Values[i];
    }
  }
}

void writeData() {
  /*Write the current data point to the SD card and via serial*/
  double actualCh4 = 0.0;
  double actualCo2 = 0.0;
  //Convert peak values to percentages using current calibration
  actualCh4 = ch4Calibration[0] + (ch4Calibration[1] * ch4Max) + (ch4Calibration[2] * ch4Max * ch4Max) + (ch4Calibration[3] * ch4Max * ch4Max * ch4Max);
  actualCo2 = co2Calibration[0] + (co2Calibration[1] * co2Max) + (co2Calibration[2] * co2Max * co2Max) + (co2Calibration[3] * co2Max * co2Max * co2Max);

  //Calculate attempt at subtract and multiply normalisation
  double subtractModifier = ((actualCh4 + actualCo2) - 100) / 2.0;
  //If not flushing
  if(currentState != 3){
    //Get the current time
    getTime();
    //Open the current store file
    File currentFile = SD.open(outputFileName, FILE_WRITE);
    //Move to the end of the file
    currentFile.seek(currentFile.size());
    //Add the time, valve number, maximum CH4, maximum CO2, percentage CH4, percentage CO2 and normalised versions
    currentFile.print(timeStamp);
    currentFile.print(',');
    currentFile.print(currentValve + 1);
    currentFile.print(',');
    currentFile.print(ch4Max);
    currentFile.print(',');
    currentFile.print(co2Max);
    currentFile.print(',');
    currentFile.print(actualCh4);
    currentFile.print(',');
    currentFile.println(actualCo2);
    //currentFile.print(',');
    //currentFile.print(actualCh4 - subtractModifier);
    //currentFile.print(',');
    //currentFile.println(actualCo2 - subtractModifier);
    currentFile.close();
  }
  //Send the valve, maximums and A to D values via serial followed by the peak values
  USBSerial.print("dataPoint ");
  if (currentState != 3){
    USBSerial.print(currentValve);
  }else{
    USBSerial.print("15");
  }
  Serial.print(' ');
  Serial.print(ch4Max);
  Serial.print(' ');
  Serial.print(co2Max);
  Serial.print(' ');
  Serial.print(actualCh4);
  Serial.print(' ');
  Serial.print(actualCo2);
  for (int i = 0; i < 5; i++) {
    USBSerial.print(' ');
    USBSerial.print(ch4ValuesPeak[i]);
  }
  for (int i = 0; i < 5; i++) {
    USBSerial.print(' ');
    USBSerial.print(co2ValuesPeak[i]);
  }
  USBSerial.print('\n');

  if (currentState != 3){
    //Open the extra file to store the peak values
    File extraFile = SD.open(extraOutputFileName, FILE_WRITE);
    //Move to the end of the file
    extraFile.seek(extraFile.size());
    //Write the time, valve and each of the peak values
    extraFile.print(timeStamp);
    extraFile.print(',');
    extraFile.print(currentValve + 1);
    extraFile.print(",ch4");
    for (int i = 0; i < 5; i++) {
      extraFile.print(',');
      extraFile.print(ch4ValuesPeak[i]);
    }
    extraFile.print(",");
    extraFile.println(ch4Max);
    //Write the time, valve and each of the peak values
    extraFile.print(timeStamp);
    extraFile.print(',');
    extraFile.print(currentValve + 1);
    extraFile.print(",co2");
    for (int i = 0; i < 5; i++) {
      extraFile.print(',');
      extraFile.print(co2ValuesPeak[i]);
    }
    extraFile.print(",");
    extraFile.println(co2Max);
    extraFile.close();
    Serial.write("All file writes done\n");

    //Store integer versions for retrieval later
    previousCh4Percent[currentValve] = floor(actualCh4);
    previousCo2Percent[currentValve] = floor(actualCo2);
  }
}

int firstValve() {
  /*Find the first valve in the sequence*/
  int valve = -1;
  //Iterate through the valve numbers
  for (int i = 0; i < 15; i++) {
    //If a valve has not been found
    if (valve == -1) {
      //If the valve is in service
      if (inService[i]) {
        //Store the valve
        valve = i;
      }
    }
  }
  //Return the first valve that was found
  return valve;
}

int nextValve() {
  /*Find the next valve in the sequence*/
  int valve = -1;
  //Iterate through the whole sequence and then 1 more
  for (int i = 1; i < 16; i++) {
    //If a valve has not been found
    if (valve == -1) {
      //Get next valve position
      int newValve = i + currentValve;
      //If the valve is outside of the range
      if (newValve > 14) {
        //Move back round to start of sequence
        newValve = newValve - 15;
      }
      //If valve is in service
      if (inService[newValve]) {
        //Store the valve to be used next
        valve = newValve;
      }
    }
  }
  //Return the valve if one was found
  return valve;
}

bool checkValve() {
  /*Check if the current test valve is in service*/
  //If it is a valid valve counter
  if (currentValve > -1 && currentValve < 15) {
    //If the valve is in service
    if (inService[currentValve]) {
      //Return that the valve is in service
      return true;
    }
  }
  //This valve is not in service
  return false;
}

void updateValves() {
  /*Update the sequence of the valves being serviced*/
  //If currently after flushing and before valve opening
  if (currentState == 0) {
    //If time has elapsed
    if (timeDifference >= waitBetween) {
      //Open Valve
      openValve(currentValve);
      //Store time of last action
      lastAction = millis();
      //Move to valve open state
      currentState = 1;
    }
  }
  //If valve open and being tested
  else if (currentState == 1) {
    //If time has elapsed
    if (timeDifference >= openDuration) {
      //Close Valve
      closeValve(currentValve);
      //If currently setup
      if (setupCorrectly) {
        //If the valve is valid
        if (currentValve > -1 && currentValve < 15) {
          //Write the data to the file and via serial
          writeData();
        }
        //Reset the stored data values
        resetValues();
      }
      //Select the next valve to be used
      currentValve = nextValve();
      //If there is a valve
      if (currentValve != -1) {
        //Store the current valve in the file
        writeValvePosition();
      }
      //Store the time of the last action
      lastAction = millis();
      //Move to pre flush state
      currentState = 2;
    } else {
      //Read the current sensor values and perform average if enough have been tested
      readValues();
    }
  }
  //If in pre flush state
  else if (currentState == 2) {
    //If time has elapsed
    if (timeDifference >= beforeFlush) {
      //Open Flush
      openValve(flushValve);
      //Store the last action time
      lastAction = millis();
      //Move to flushing state
      currentState = 3;
    }
  }
  //If in flushing state
  else if (currentState == 3) {
    //If time has elapsed
    if (timeDifference >= flushDuration) {
      //Close Flush
      closeValve(flushValve);
      writeData();
      resetValues();
      //Store the time of the last action
      lastAction = millis();
      //Move to the pre valve open state
      currentState = 0;
    } else {
      readValues();
    }
  }
}

void resetCalibrationValues() {
  /*Reset all the information about calibration logging*/
  calCh4Max = 0;
  calCo2Max = 0;
  resetCalibrationAverages();
}

void resetCalibrationAverages() {
  /*Reset the average data points for calibration*/
  for (int i = 0; i < averageCount; i++) {
    calCh4ValueSet[i] = 0;
    calCo2ValueSet[i] = 0;
  }
  calSetPosition = 0;
}

void updateCalibrationRead() {
  /*Update the sequence for the calibration being tested*/
  //If not currently flushing
  if (!flushingCalibration) {
    //Take sensor readings
    //int co2Value = analogRead(co2Pin);
    adc.attach(co2Pin);
    int co2Value = adc.readMiliVolts();
    delay(20);
    //int ch4Value = analogRead(ch4Pin);
    adc.attach(ch4Pin);
    int ch4Value = adc.readMiliVolts();
    delay(20);

    //Store in array to be averaged
    calCh4ValueSet[calSetPosition] = ch4Value;
    calCo2ValueSet[calSetPosition] = co2Value;

    //Increment counter
    calSetPosition = calSetPosition + 1;

    //If array is full
    if (calSetPosition >= averageCount) {
      int ch4Total = 0;
      int co2Total = 0;
      //Sum values
      for (int i = 0; i < averageCount; i++) {
        ch4Total = ch4Total + calCh4ValueSet[i];
        co2Total = co2Total + calCo2ValueSet[i];
      }
      //Divide by count to get average value
      int ch4Average = floor(ch4Total / averageCount);
      int co2Average = floor(co2Total / averageCount);

      //If average values exceed maximum stored - update to new maximum
      if (ch4Average > calCh4Max) {
        calCh4Max = ch4Average;
      }
      if (co2Average > calCo2Max) {
        calCo2Max = co2Average;
      }

      //Reset the averages
      resetCalibrationAverages();
    }

    //If calibration time exceeded
    if (timeDifference >= calWaitTime) {
      //Close valve and open flush
      closeValve(0);
      openValve(15);
      //Currently flushing, store start time
      flushingCalibration = true;
      lastAction = millis();
    }
  } else {
    //If time for flushing exceeded
    if (timeDifference >= calFlushDuration) {
      //Close the flush valve
      closeValve(15);
      //Output the calibration point via serial
      USBSerial.write("calpoint ");
      //For methane
      if (checkingCh4) {
        USBSerial.write("ch4 ");
        USBSerial.println(calCh4Max);
      }
      //For carbon dioxide
      else if (checkingCo2) {
        USBSerial.write("co2 ");
        USBSerial.println(calCo2Max);
      }
      flushingCalibration = false;
      readingCalibration = false;
      //Reset all calibration memory
      resetCalibrationValues();
    }
  }
}

void loop() {
  /*Repeat while the device is powered*/
  //Get the time from the internal clock
  currentTime = millis();
  timeDifference = 0;
  //If a rollover has occurred
  if (currentTime < lastAction) {
    //Elapsed is the current time plus when something last occurred taking away the maximum value possible
    timeDifference = currentTime + lastAction - ULONGMAX;
  } else {
    //Elapsed is difference
    timeDifference = currentTime - lastAction;
  }
  //If all the setup worked
  if (setupCorrectly) {
    //If not in calibration mode
    if (!calibrating) {
      //Update the valve sequence
      updateValves();
    } else {
      //If currently taking a calibration point
      if (readingCalibration) {
        //Update the calibration valve sequence
        updateCalibrationRead();
      }
    }
  }
  //Read incoming messages from serial
  readSerial();
}
