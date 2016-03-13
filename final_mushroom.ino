// This is a basic snapshot sketch using the VC0706 library.
// On start, the Arduino will find the camera and SD card and
// then snap a photo, saving it to the SD card.
// Public domain.

// If using an Arduino Mega (1280, 2560 or ADK) in conjunction
// with an SD card shield designed for conventional Arduinos
// (Uno, etc.), it's necessary to edit the library file:
//   libraries/SD/utility/Sd2Card.h
// Look for this line:
//   #define MEGA_SOFT_SPI 0
// change to:
//   #define MEGA_SOFT_SPI 1
// This is NOT required if using an SD card breakout interfaced
// directly to the SPI bus of the Mega (pins 50-53), or if using
// a non-Mega, Uno-style board.

int led1 = 2;
int led2 = 3;
int led3 = 4;
int led4 = 5;
int led5 = 6;

int led6 = 7;
int led7 = 8;
int led8 = 9;
int led9 = 10;
int led10 = 11;
int led11 = 12;
int led12 = 13;

int mostureSensor = 0;
int analogPin = 4;
int val = 0;
int humiditySensor = 5;

int photocellPin = 1;     // the cell and 10K pulldown are connected to a1
    int photocellReading;     // the analog reading from the sensor divider
    int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
    int LEDbrightness;

#include <Wire.h>
#include "SparkFunHTU21D.h"

HTU21D myHumidity;





#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

// comment out this line if using Arduino V23 or earlier
#include <SoftwareSerial.h>         

// uncomment this line if using Arduino V23 or earlier
// #include <NewSoftSerial.h>       

// SD card chip select line varies among boards/shields:
// Adafruit SD shields and modules: pin 10
// Arduino Ethernet shield: pin 4
// Sparkfun SD shield: pin 8
// Arduino Mega w/hardware SPI: pin 53
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
#define chipSelect 10

// Pins for camera connection are configurable.
// With the Arduino Uno, etc., most pins can be used, except for
// those already in use for the SD card (10 through 13 plus
// chipSelect, if other than pin 10).
// With the Arduino Mega, the choices are a bit more involved:
// 1) You can still use SoftwareSerial and connect the camera to
//    a variety of pins...BUT the selection is limited.  The TX
//    pin from the camera (RX on the Arduino, and the first
//    argument to SoftwareSerial()) MUST be one of: 62, 63, 64,
//    65, 66, 67, 68, or 69.  If MEGA_SOFT_SPI is set (and using
//    a conventional Arduino SD shield), pins 50, 51, 52 and 53
//    are also available.  The RX pin from the camera (TX on
//    Arduino, second argument to SoftwareSerial()) can be any
//    pin, again excepting those used by the SD card.
// 2) You can use any of the additional three hardware UARTs on
//    the Mega board (labeled as RX1/TX1, RX2/TX2, RX3,TX3),
//    but must specifically use the two pins defined by that
//    UART; they are not configurable.  In this case, pass the
//    desired Serial object (rather than a SoftwareSerial
//    object) to the VC0706 constructor.

// Using SoftwareSerial (Arduino 1.0+) or NewSoftSerial (Arduino 0023 & prior):
#if ARDUINO >= 100
// On Uno: camera TX connected to pin 2, camera RX to pin 3:
SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
// On Mega: camera TX connected to pin 69 (A15), camera RX to pin 3:
//SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
#else
NewSoftSerial cameraconnection = NewSoftSerial(2, 3);
#endif

Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

// Using hardware serial on Mega: camera TX conn. to RX1,
// camera RX to TX1, no SoftwareSerial object is required:
//Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

void setup() {

  // When using hardware SPI, the SS pin MUST be set to an
  // output (even if not connected or used).  If left as a
  // floating input w/SPI on, this can cause lockuppage.
  #if !defined(SOFTWARE_SPI)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if(chipSelect != 53) pinMode(53, OUTPUT); // SS on Mega
#else
  if(chipSelect != 10) pinMode(10, OUTPUT); // SS on Uno, etc.
#endif
#endif

  Serial.begin(9600);

  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);

  pinMode(led6, OUTPUT);
  pinMode(led7, OUTPUT);
  pinMode(led8, OUTPUT);
  pinMode(led9, OUTPUT);
  
  myHumidity.begin();
  
 Serial.println("VC0706 Camera snapshot test");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }  
  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);          // small

 

}

void loop() {



  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  Serial.println("Snap in 1 hour...");
  delay(3600000);

  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");

  
  photocellReading = analogRead(photocellPin);  
     
      Serial.print("Analog reading = ");
      Serial.println(photocellReading);     // the raw analog reading
     
      // LED gets brighter the darker it is at the sensor
      // that means we have to -invert- the reading from 0-1023 back to 1023-0
      photocellReading = 1023 - photocellReading;
      //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
      LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
      analogWrite(LEDpin, LEDbrightness);

  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  
  // read the input on analog pin 0:
  int sensorValue1 = analogRead(mostureSensor);
  val = analogRead(analogPin);
  int sensorValue3 = analogRead(humiditySensor);
  // print out the value you read:

  Serial.println(sensorValue1);
  Serial.println(temp);
  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");

  Serial.println();
  
 if (sensorValue1 >= 0 && sensorValue1 <= 500)
 {
 digitalWrite(led1, HIGH);
 digitalWrite(led4, HIGH);
 digitalWrite(led3, LOW);
 //digitalWrite(led4, LOW);
 //digitalWrite(led5, HIGH);
}
else if (sensorValue1 >= 341 && sensorValue1 <= 680)
 {
 digitalWrite(led6, HIGH);
 digitalWrite(led3, HIGH);
 digitalWrite(led4, HIGH);
 //digitalWrite(led4, HIGH);
 //digitalWrite(led5, HIGH);
}  
else if (sensorValue1 > 680 )
 {
 digitalWrite(led1, HIGH);
 digitalWrite(led3, HIGH);
 digitalWrite(led4, LOW);
 //digitalWrite(led4, HIGH);
 //digitalWrite(led5, HIGH);
}    
/* else if (sensorValue1 >= 341 && sensorValue1 <= 680)
 {
 digitalWrite(led1, LOW);
 digitalWrite(led2, HIGH);
 digitalWrite(led3, HIGH);
 digitalWrite(led4, HIGH);
 digitalWrite(led5, HIGH);
}
else if (sensorValue1 >= 0 && sensorValue1 <= 340)
 {
 digitalWrite(led1, LOW);
 digitalWrite(led2, HIGH);
 digitalWrite(led3, HIGH);
 digitalWrite(led4, HIGH);
 digitalWrite(led5, HIGH);
} */
 
 
 if (temp >= 31)
 {
 digitalWrite(led6, HIGH);
 digitalWrite(led7, LOW);
 digitalWrite(led8, HIGH);

}
else if (temp >= 15 && temp <31 )
 {
 digitalWrite(led6, HIGH);
 digitalWrite(led7, HIGH);
 digitalWrite(led8, LOW);

}
else if (temp <15)
 {
 digitalWrite(led6, HIGH);
 digitalWrite(led7, HIGH);
 digitalWrite(led8, HIGH);

}




 if (photocellReading >= 900)
 {
 digitalWrite(led10, HIGH);
 digitalWrite(led5, LOW);
 digitalWrite(led6, LOW);

}
else if (photocellReading >= 500 && photocellReading <500 )
 {
 digitalWrite(led10, HIGH);
 digitalWrite(led11, HIGH);
 digitalWrite(led12, HIGH);

}
else if (photocellReading <500)
 {
 digitalWrite(led10, HIGH);
 digitalWrite(led6,  LOW);
 digitalWrite(led5,  LOW);

}



  delay(3600000);        // delay 1 second between reads

}

