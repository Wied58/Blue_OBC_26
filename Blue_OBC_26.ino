//Blue_OBC
//  
// 1 Reads characters from serial stream to String and prints the Strings to serial
// 2 adds getting String length, writing String to byte array and print the
// String and GGA byteArray
// 3 replaces the commas with asterisks in the byte array
// 4 adds reading values to first comma to get literal value
// 5 Reads the entire line so you can choose the field you would like based on the starting comma
// value in the if. 
// 6 does not print the comma and quits reading the line after hitting the second comma
// 7 No longer sends string length to function
// 8 moves the String convert to bytes to the function and has better varible names.
// 9 returns time field from function
// 10 Sets inString to names names of NMEA sentance names. Gets the time from RMC. Passes UTC to flash 
// function to blink onboard led to show life. 
// 11 Adds pressure sensor
// 12 Adds Xbee
// 13 
// 14 Adds Acceleromter - or how you spell it. 
// 15 moves flashing LED to pin 14 to open pin 13 for SPI Sd card. 
// 16 Replaces openlog with SD
// 17 Adds LEDS to mimic camera switching
// 18 Adds a solid LED for Sd card malfunction. 
// May 9th Flight
// 19 Change output to single sentance
// 20 Add Radio in to the single sentance
// 21 JUNKed Fix radio single sentance in the file, replace if-then-else with swtich-case. remove $FH sentance headers
// 22 Write Xbee character to string
// 23 polish of 22
// 24 Port for Teensy 3.6, 9250
// 25 is for Green due it has teensy 3.0
//

//http://arduino.stackexchange.com/questions/21846/how-to-convert-string-to-byte-array

#include <Wire.h>
#include <math.h>

// Pressure sensor setup
#define ADDRESS 0x76 //0x77 Pressure sensor address
uint32_t D1 = 0;
uint32_t D2 = 0;
float dT = 0;
int32_t TEMP = 0;
float OFF = 0; 
float SENS = 0; 
float P = 0;
float T2  = 0;
float OFF2  = 0;
float SENS2 = 0;
uint16_t C[7];
float Temperature;
float Pressure;

#include "MPU9250.h"
MPU9250 myIMU;


int led = 14; //blink the green led to show life
int noSd = 15; // Light the red LED in case the SD card does not open file
int camera1 = 3;  // Mimic camera 1
int camera2 = 4;  // Mimic camera 2

String inString = ""; // GPS port incoming data buffer


//Set up SD Card

#include <SD.h>
File myFile;
const int chipSelect = BUILTIN_SDCARD;

void setup() {

Serial.begin(57600);  
Serial1.begin(57600); // GPS
Serial2.begin(9600);  //Xbee

pinMode(led, OUTPUT); //blink the onbord led to show life
pinMode(noSd, OUTPUT); //Light the red LED in case the SD card does not open file
pinMode(camera1, OUTPUT); //turn on camera 1 
pinMode(camera2, OUTPUT); //turn on camera 2

  // Initialize SdFat or print a detailed error message and halt
  // If halt light LED noSd
  // Use half speed like the native library.
  // change to SPI_FULL_SPEED for more performance.

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    digitalWrite(noSd, HIGH);
    Serial.println("error opening test.txt");
    //SD.initErrorHalt(); from old card writer
    return;
  }
  Serial.println("File initialization done.");

  // open the file for write at end like the Native SD library
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  Serial.println("\nWriting to test.txt...");
  myFile.println("\nTIME,LAT,LAT_DIR,LONG,LONG_DIR,ALT,SATS,QUALITY,DATE,TEMP,PRESSURE,ACCX,ACCY,ACCZ,TEMP2,RIN");



// Pressure sernsor stuff
PORTC |= (1 << 4); // Disable internal pullups, 10Kohms are on the breakout
PORTC |= (1 << 5); // Disable internal pullups, 10Kohms are on the breakout
Wire.begin();
initial(ADDRESS);

myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
}

void loop() {

  

  

  // Receive data from com port
  while (Serial1.available() > 0) {
    int inChar = Serial1.read();

    if (inChar != '\n') { // Not end of line, white character to string
      inString += (char)inChar;
    } // End of Not end of line, white character to string
    
    else { //Else 1 (if end of line) 

      String nmeaId = inString.substring(0,6);
      
      if (nmeaId == "$GPGGA") { //GGA if
       String nmeaSen = inString;   
       //Serial.println(nmeaSen);
       //myFile.print(nmeaSen);  
       String TIME=getNmeaField(nmeaSen,2); // Get the 2nd (Time) field 
       Serial.print(TIME);
       Serial.print(",");
       myFile.print(TIME);
       myFile.print(",");
       String LAT=getNmeaField(nmeaSen,3); // Get the 2nd (Time) field 
       Serial.print(LAT);
       Serial.print(",");
       myFile.print(LAT);
       myFile.print(",");
       String LAT_DIR=getNmeaField(nmeaSen,4); // Get the 2nd (Time) field 
       Serial.print(LAT_DIR);
       Serial.print(",");
       myFile.print(LAT_DIR);
       myFile.print(",");
       String LONG=getNmeaField(nmeaSen,5); // Get the 2nd (Time) field 
       Serial.print(LONG);
       Serial.print(",");
       myFile.print(LONG);
       myFile.print(",");
       String LONG_DIR=getNmeaField(nmeaSen,6); // Get the 2nd (Time) field 
       Serial.print(LONG_DIR);
       Serial.print(",");
       myFile.print(LONG_DIR);
       myFile.print(",");
       String ALT=getNmeaField(nmeaSen,10); // Get the 2nd (Time) field 
       Serial.print(ALT);
       Serial.print(",");
       myFile.print(ALT);
       myFile.print(",");
       String SATS=getNmeaField(nmeaSen,8); // Get the 2nd (Time) field 
       Serial.print(SATS);
       Serial.print(",");
       myFile.print(SATS);
       myFile.print(",");
       String QUALITY=getNmeaField(nmeaSen,7); // Get the 2nd (Time) field 
       Serial.print(QUALITY);
       Serial.print(",");
       myFile.print(QUALITY);
       myFile.print(",");
       radio(TIME, LAT, LAT_DIR, LONG, LONG_DIR, ALT, SATS, QUALITY);
       flash(TIME);
      } //End of GGA if
      else if (nmeaId == "$GPRMC") { //RMC if
        String nmeaSen = inString;   
        //Serial.println(nmeaSen); 
        //myFile.print(nmeaSen);  
        String UTC=getNmeaField(nmeaSen,2); // Get the 2nd (Time) field
        String DATE=getNmeaField(nmeaSen,10); // Get the 2nd (Time) field
     //   String RLAT=getNmeaField(nmeaSen,4); // Get the 2nd (Time) field
        Serial.print(DATE);
        myFile.print(DATE);


      //  flash(UTC);
        pressure(UTC);
        accel();
     //   radio(RLAT);


                
        Serial.println("");
        myFile.println("");

        myFile.flush(); 
        
      } // End of RMC else if
      else { //Print action else
        Serial.println(nmeaId);
      } // end of print action else

      inString = "";
    } // End of else 1

  } // End of while Serial available
//Serial.println("Hello");
} // End of main loop


/************************************************************************************/
/*******         Acceleromter  - or how you spell it.              ******************/
/************************************************************************************/

void accel()
{


myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
myIMU.getAres(); 

 // Now we'll calculate the accleration value into actual g's
// This depends on scale being set
myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

Serial.print(",");
Serial.print(myIMU.ax);
Serial.print(",");
Serial.print(myIMU.ay);
Serial.print(",");
Serial.print(myIMU.az);
Serial.print(",");

myFile.print(",");
myFile.print(",");
myFile.print(myIMU.ax);
myFile.print(",");
myFile.print(myIMU.ay);
myFile.print(",");
myFile.print(myIMU.az);
myFile.print(",");


myIMU.tempCount = myIMU.readTempData();  // Read the adc values
// Temperature in degrees Centigrade
myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
// Print temperature in degrees Centigrade
Serial.print(myIMU.temperature, 1);
myFile.print(myIMU.temperature, 1);



}

/************************************************************************************/
/*******         End of Acceleromter  - or how you spell it.          ***************/
/************************************************************************************/

/************************************************************************************/
/******************           Xbee department                 ***********************/
/************************************************************************************/
// Receive data from com port
//  while (Serial1.available() > 0) {
//    int inChar = Serial1.read();
// if (inChar != '\n') { // Not end of line, white character to string
//      inString += (char)inChar;
//       } // End of Not end of line, white character to string

void radio(String gpsTIME, String gpsLAT, String gpsLAT_DIR, String gpsLONG, String gpsLONG_DIR, String gpsALT, String gpsSATS, String gpsQUALITY)
{
     char getData;
     String xbeeString = ""; // Xbee port incoming data buffer
     String cameraString = ""; // Xbee port incoming data buffer

    while (Serial2.available()){ 
          getData = Serial2.read();
        //  if (getData != '!') { // Not end of line, white character to string
           xbeeString += (char)getData;
        //  }
        
      //Serial2.print(xbeeString);

    }

    
    
    if(xbeeString == "A"){
          digitalWrite(camera1, HIGH);
          digitalWrite(camera2, LOW);

          //Serial.print("a");

          Serial2.print(gpsTIME);
          Serial2.print(",");
          Serial2.print(gpsLAT);
          Serial2.print(",");
          Serial2.print(gpsLAT_DIR);
          Serial2.print(",");
          Serial2.print(gpsLONG);
          Serial2.print(",");
          Serial2.print(gpsLONG_DIR);
          Serial2.print(",");
          Serial2.print(gpsALT);
          Serial2.print(",");
          Serial2.print(gpsSATS);
          Serial2.print(",");
          Serial2.print(gpsQUALITY);
          Serial2.println();
          
    } //End of A if
    
    else if(xbeeString == "B"){
          digitalWrite(camera1, LOW); 
          digitalWrite(camera2, HIGH);
            
          Serial3.print("b");
    } // End of B else if
    
    else if(xbeeString.startsWith ("@RT")){
         Serial.print(",");
         Serial.print(xbeeString);

         Serial3.print("xbeeString");
    } // End of RT else if
          
    else if(xbeeString == "@SE!"){
         Serial.print(",");
         Serial.print(xbeeString);

         Serial3.print("xbeeString");
    } // End of RT else if

    else if(xbeeString == "@SD!"){
         Serial.print(",");
         Serial.print(xbeeString);

         Serial3.print("xbeeString");
     } // End of RT else if
     
     else if(xbeeString == "@AE!"){
         Serial.print(",");
         Serial.print(xbeeString);

         Serial3.print("xbeeString");
     } // End of RT else if

     else if(xbeeString == "@AD!"){
         Serial.print(",");
         Serial.print(xbeeString);

         Serial3.print("xbeeString");
     } // End of RT else if
                   
     else {
         xbeeString = 'N';
     }
    
   //Serial.print(",");
   //Serial.print(xbeeString);

   myFile.print(",");
   myFile.print(xbeeString);

    
    

   
}
        
/************************************************************************************/
/******************           End of Xbee department          ***********************/
/************************************************************************************/



/************************************************************************************/
/******************                 getNmeaField              ***********************/
/*                       return a field from a NMEA sentance                        */
/************************************************************************************/
String getNmeaField(String gpsSen, int fieldNo)
{
  int j = 0;
  int comma = 0;
  byte gpsBytes[100];
  int bufSize=gpsSen.length(); //get the length of the string
  gpsSen.getBytes(gpsBytes, bufSize); // convert type String to array of characters called ggabytes
  String field;
  do
  {
    if (gpsBytes[j] == ',')
    {
      comma ++;
    } // End of comma if  
    
    if (comma == (fieldNo -1) && gpsBytes[j] != ',') //this is value you want, but don't write the comma
    {
    field += char(gpsBytes[j]);
    } // End of this is value you want, but don't write the comma to the string if!
      
    j++; 
       
  } while (comma != fieldNo); // bails afer 2nd comma
 
  return(field);

}

/************************************************************************************/
/******************                End of getNmeaField        ***********************/
/************************************************************************************/


/************************************************************************************/
/******************            Pressure Sensor functions           ******************/
/************************************************************************************/


void pressure(String gpsTime)
{  


 D1 = getVal(ADDRESS, 0x48); // Pressure raw
 D2 = getVal(ADDRESS, 0x58);// Temperature raw

 dT   = (float)D2 - ((uint32_t)C[5] * 256);
 OFF  = ((float)C[2] * 131072) + ((dT * C[4]) / 64);
 SENS = ((float)C[1] * 65536) + (dT * C[3] / 128);

 TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;
  
 if(TEMP < 2000) // if temperature lower than 20 Celsius 
  {
   T2=pow(dT,2)/2147483648;
   OFF2=61*pow((TEMP-2000),2)/16;
   SENS2=2*pow((TEMP-2000),2);
    
   if(TEMP < -1500) // if temperature lower than -15 Celsius 
    {
     OFF2=OFF2+15*pow((TEMP+1500),2);
     SENS2=SENS2+8*pow((TEMP+1500),2);
    } // end if temperature lower than -15 Celsius

    TEMP = TEMP - T2;
    OFF = OFF - OFF2; 
    SENS = SENS - SENS2;
    
    } // if temperature lower than 20 Celsius 
  
    Temperature = (float)TEMP / 100; 
  
    P  = (D1 * SENS / 2097152 - OFF) / 32768;
    
    Pressure = (float)P / 100;
  

    //Serial.print("$FHPRS,");
    //Serial.print(gpsTime);
    Serial.print(",");
    Serial.print(Temperature);
    Serial.print(",");
    Serial.print(Pressure);

    //myFile.print("$FHPRS,");
    //myFile.print(gpsTime);
    myFile.print(",");
    myFile.print(Temperature);
    myFile.print(",");
    myFile.print(Pressure);
  
  /* RESET THE CORECTION FACTORS */
  
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
} // end of pressure

/************************************************************************************/

long getVal(int address, byte code) // for pressure sensor
{
  unsigned long ret = 0;
  Wire.beginTransmission(address);
  Wire.write(code);
  Wire.endTransmission();
  delay(10);
  // start read sequence
  Wire.beginTransmission(address);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)3);
  if (Wire.available() >= 3)
  {
    ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  }
  else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
} // end of getVal

/************************************************************************************/

void initial(uint8_t address) // for pressure sensor
{

  Serial.println();
  Serial.println("PRESSURE SENSOR PROM COEFFICIENTS");

  Wire.beginTransmission(address);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);


  for (int i=0; i<6  ; i++) {

    Wire.beginTransmission(address);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission();

    Wire.beginTransmission(address);
    Wire.requestFrom(address, (uint8_t) 6);
    delay(1);
    if(Wire.available())
    {
       C[i+1] = Wire.read() << 8 | Wire.read();
    }
    else {
      Serial.println("Error reading PROM 1"); // error reading the PROM or communicating with the device
    }
    Serial.println(C[i+1]);
  }
  Serial.println();
} // end of initital

/************************************************************************************/
/******************       End of Pressure Sensor functions         ******************/
/************************************************************************************/



/************************************************************************************/
/******************                flash                      ***********************/
/*             blink the onbord led to show life every other pass                   */
/************************************************************************************/

void flash(String gpsTime) //blink the onbord led to show life every other pass
{ 
  
  int blink = gpsTime.toInt() % 2;
  
  if (blink != 0)
  {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else    
  {          
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  }
}


/************************************************************************************/
/******************                End of flash               ***********************/
/************************************************************************************/

