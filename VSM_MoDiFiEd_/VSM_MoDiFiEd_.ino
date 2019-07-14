/*-------------------------------------------------- VEHICLE SAFETY MODULE ---------------------------------------------------------
---------------------------------------------------- RAHUL, PRIYANKA AND MEGHA -----------------------------------------------------
---------------------------------------------------- PUDDU, GUDDY AND MAGGIE -------------------------------------------------------
---------------------------------------------------- CMR INSTITUTE OF TECHNOLOGY ---------------------------------------------------*/

#include <TembooCoAPEdgeDevice.h>
#include <TembooMQTTEdgeDevice.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
SoftwareSerial gsm(9, 8);
SoftwareSerial serial_connection(10, 11); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
int fsrAnalogPin1 = A0; // FSR is connected to analog 0
int fsrAnalogPin2 = A1;
int LEDpin = 7;       // connect Red LED to pin 11 (PWM pin)
int fsrReading1;      // the analog reading from the FSR resistor divider
int fsrReading2;
int LEDbrightness;

const int buttonPin = 6;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status


void setup()
{
  Serial.begin(9600); // connect serial
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  gsm.begin(9600);// connect gsm modem

  pinMode(LEDpin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}
void loop()
{
  while(serial_connection.available())//While there are characters to come from the GPS
  {
   gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
   //Get the latest info from the gps object which it derived from the data sent by the GPS unit
   Serial.println("Satellite Count:");
   Serial.println(gps.satellites.value());
   Serial.println("Latitude:");
   Serial.println(gps.location.lat(), 6);
   Serial.println("Longitude:");
   Serial.println(gps.location.lng(), 6);
   Serial.println("Speed MPH:");
   Serial.println(gps.speed.mph());
   Serial.println("Altitude Feet:");
   Serial.println(gps.altitude.feet());
   Serial.println("");

  }
  fsrReading1 = analogRead(fsrAnalogPin1);
  Serial.print("left side bumber = ");
  Serial.println(fsrReading1);
  fsrReading2 = analogRead(fsrAnalogPin2);
  Serial.print("Right Side bumber = ");
  Serial.println(fsrReading2);
  delay(500);


  //LEDbrightness = map(fsrReading1, 0, 1023, 0, 255);
  // analogWrite(LEDpin, LEDbrightness);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  if (fsrReading1 > 50 || fsrReading2 > 50)
  {
    if (fsrReading1 < 100 || fsrReading2 < 100)
    {
      Serial.println("OBSTACLE IS APPROACHING NEAR");
      delay(1000);
      Serial.println("\nCAR MET WITH AN ACCIDENT--- PLEASE TAKE APPROPRIATE ACTION !!!!!!!");
    }
    else if (fsrReading1 < 300 || fsrReading2 < 300)
    {
      Serial.println("A MAJOR ACCIDENT HAS OCCURED--- AND THE CONDITION IS CRITICAL--- !!!!!!!!");
    }
    delay(5000);  //delay for the push button for 10 seconds //   SOS SWITCH   //

    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH)
    {
      gsm.print("\r");
      delay(1000);
      gsm.print("AT+CMGF=1\r");
      delay(1000);
      gsm.print("AT+CMGS=\"+919741598719\"\r");    //NUMBER OF GUDDY
      delay(1000);
      gsm.print("AMBULANCE CALL TERMINATED ");            
      delay(1000);
      Serial.write(0x1A);
      delay(1000);
    }
    else
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////
      gsm.print("\r");
      delay(1000);
      gsm.print("AT+CMGF=1\r");
      delay(1000);
      gsm.print("AT+CMGS=\"+919741598719\"\r");    //MESSAGE TO ME
      delay(1000);
      gsm.print("-------#VEHICLE SAFETY MODULE#-------\n-------RAHUL, PRIYANKA AND MEGHA-------\n-------CMRIT-------\n");
      gsm.println("-------#AMBULANCE 108#-------------\n A MAJOR ACCIDENT AS OCCURED IN THE NEAR BY AREA, PLEASE RUSH TO THE SPOT ASAP");
      gsm.println("THE LATITUDE IS FOUND TO BE-->");
      gsm.print(gps.location.lat(), 6);  //LATITUTE VALUE
      gsm.println("\nTHE LONGITUDE IS FOUND TO BE-->");
      gsm.print(gps.location.lng(), 6);   //LONGITUDE VALUE   
      gsm.print("\nTHE EXACT LOCATION IS--> https://www.google.ca/maps/place/"); //LINK THAT AS TO BE SENT THROUGH SMS
      gsm.print(gps.location.lat(), 6);
      gsm.print(",");
      gsm.print(gps.location.lng(), 6);
      delay(10000);    
      gsm.write(0x1A);
      delay(10000);                   

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      gsm.print("\r");
      delay(1000);
      gsm.println("AT+CMGF=1\r");
      delay(1000);
      gsm.println("AT+CMGS=\"+918310178946\"\r");    //MESSAGE TO THE NEAR BY AMBULANCE
      delay(1000);
      gsm.println("------#FAMILY MEMBER#-------YOUR FAMILY MEMBER AS MET WITH AN ACCIDENT !!!!! PLEASE TRY CONTACTING THEM AS SOON AS POSSIBLE"); 
      gsm.println("THE LATITUDE IS FOUND TO BE-->");
      gsm.print("12.9072");     
      gsm.println("\nTHE LONGITUDE IS FOUND TO BE-->");
      gsm.print("77.5656");        
      gsm.print("\nTHE LOCATION IS--> https://www.google.ca/maps/place/"); //LINK THAT AS TO BE SENT THROUGH SMS
      gsm.print("12.9072");
      gsm.print(",");
      gsm.print("77.5656");
      delay(10000);
      gsm.write(0x1A);
      delay(10000);                  


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      gsm.println("\r");
      delay(1000);
      gsm.print("AT+CMGF=1\r");
      delay(1000);
      gsm.print("AT+CMGS=\"+919481123159\"\r");    //MESSAGE TO THE NEAR BY POLICE STATION
      delay(1000);
      gsm.println("-------#POLICE STATION 100#-------\n A MAJOR ACCIDENT AS OCCURED IN THE NEAR BY AREA, PLEASE RUSH TO THE SPOT AS SOON AS POSSIBLE WITH APPROPRIATE CREW MEMBERS");
      gsm.println("THE LATITUDE IS FOUND TO BE-->");
      gsm.print("12.9072");  //LATITUTE VALUE
      gsm.println("\nTHE LONGITUDE IS FOUND TO BE-->");
      gsm.print("77.5656");   //LONGITUDE VALUE
      gsm.print("\nTHE EXACT LOCATION WHERE ACCIDENT AS OCCURED IS--> https://www.google.ca/maps/place/"); //LINK THAT AS TO BE SENT THROUGH SMS
      gsm.print("12.9072");
      gsm.print(",");
      gsm.print("77.5656");
      delay(10000);
      gsm.write(0x1A);
      delay(10000);                  

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
      Serial.println("msg sent\n");

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
  }
  else
  {
    Serial.println(" ");
  }
}



  /*
  $GPRMC,183729,A,3907.356,N,12102.482,W,000.0,360.0,080301,015.5,E*6F
  $GPRMB,A,,,,,,,,,,,,V*71
  $GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75
  $GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D
  $GPGSV,2,1,08,02,43,088,38,04,42,145,00,05,11,291,00,07,60,043,35*71
  $GPGSV,2,2,08,08,02,145,00,09,46,303,47,24,16,178,32,26,18,231,43*77
  $PGRME,22.0,M,52.9,M,51.0,M*14
  $GPGLL,3907.360,N,12102.481,W,183730,A*33
  $PGRMZ,2062,f,3*2D
  $PGRMM,WGS 84*06
  $GPBOD,,T,,M,,*47
  $GPRTE,1,1,c,0*07
  $GPRMC,183731,A,3907.482,N,12102.436,W,000.0,360.0,080301,015.5,E*67
  $GPRMB,A,,,,,,,,,,,,V*71
*/












