//Reverse Geocache v1.0
//Øystein Smith feb. 2011

#include <LiquidCrystal.h>
#include <NewSoftSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <math.h>
#include "EEPROMWrite.h"

#define EARTH_RADIUS 6378.14f
#define DEG_TO_RAD 0.0174532925f

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
NewSoftSerial gpsSerial(8, 9);
Servo lockservo;
TinyGPS gps;

int a = 0;
int shutdownpin = 10;
int ADDR_TRIES = 0;
int ADDR_SOLVED = 1;
int ADDR_TOGGLE = 2;
int ADDR_FLAT = 4;
int ADDR_FLON = 6;
int tries = 0;
int setuppin = 12;
int locationprog = 0;
unsigned long gpsfixdelay = 60000;
unsigned long fix_age;
float flat,flon;
float distancefromtarget = 100.0;

//Servo, endeposisjoner
int servolocked = 160;
int servoopen = 120;

//Target koordinater
//WGS 84 - desimal
float progTargetlat = 59.18712;
float progTargetlon = 9.59560;
//Ant. forsøk
int ntries = 11;
//Toleranse
float distancetolerance = 0.1;

//location struct for reading and writing float coordinates to EEPROM
struct config_t
{
  float targetlat;
  float targetlon;
} location;
	
void setup()
{
  lcd.begin(16, 2);
  lcd.print("   -SCANNING-");
  gpsSerial.begin(4800);
  pinMode (shutdownpin, OUTPUT);
  pinMode (setuppin, INPUT);
  digitalWrite (shutdownpin, LOW);
  //lockservo.attach(11);
  //lockLid();
  int i = EEPROM_readAnything(10, location);
  if(digitalRead(setuppin) == LOW)
  {
    openLid(); //backdoor
    initializeEEPROM();
    delay(2000);
    lockLid();
    shutdown();
  }
}

void loop()
{
  //Enable location programming mode
  if(digitalRead(setuppin) == LOW)
  {
    locationprog = 1;
    lcd.setCursor(0,1);
    lcd.print("for new location");
  }
  //Check if all tries are used
  tries = EEPROM.read(ADDR_TRIES);
  if(tries == 0)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("No more tries.");
    lcd.setCursor(0,1);
    lcd.print("Locked forever!");
    delay (5000);
    shutdown();    
  }
  //Check if puzzle is solved
  if(EEPROM.read(ADDR_SOLVED) == 1)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" ACCESS GRANTED!");
    //Lock/Open lid if solved and button is pressed
    if(EEPROM.read(ADDR_TOGGLE) == 1)
    {
      EEPROM.write(ADDR_TOGGLE, 0);
      lcd.setCursor(3,1);
      lcd.print("Locking...");
      delay(2000);
      lockLid();
      delay(1000);
      shutdown();
    }
    else
    {
      openLid();
      EEPROM.write(ADDR_TOGGLE, 1);
      delay(3000);
      shutdown();
    }
  }
  //If puzzle is not solved and there is still tries left
  if(EEPROM.read(ADDR_SOLVED) == 0)
  {
    //Calculate distance to target
    if( distanceTo(location.targetlat, location.targetlon, &distancefromtarget))
    {
      //If box has reached target destination, open
      if(distancefromtarget < distancetolerance)
      {
        EEPROM.write(ADDR_SOLVED, 1);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" ACCESS GRANTED!");
        openLid();
      }
      //If not, subtract one try from counter and print distancefromtarget to lcd
      else
      {
        tries--;
        EEPROM.write(ADDR_TRIES, tries);
        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print("ACCES DENIED!");
        lcd.setCursor(1,1);
        lcd.print(tries);
        lcd.print(" Tries left");
        delay(4000);
        lcd.clear();
        lcd.setCursor(5,0);
        lcd.print(distancefromtarget);
        lcd.print("km");       
      }
      delay(10000);
      shutdown();
    }
    else
    {
      //Search for signal for 65seconds before shutting down
      if (millis() >= 65000)
      {
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("NO SIGNAL!");
        delay(3500);
        shutdown();
      }
    }
  }
}
void initializeEEPROM()
{
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("INITIALIZING");
  lcd.setCursor(5,1);
  lcd.print("EEPROM");
  EEPROM.write(ADDR_TRIES, ntries);
  EEPROM.write(ADDR_SOLVED, 0);
  EEPROM.write(ADDR_TOGGLE, 0);
  delay(1000);
  lcd.clear();
  lcd.print(" Reset original");
  lcd.setCursor(4,1); lcd.print("location?");
  delay(1000);
  if(digitalRead(setuppin) == LOW)
  {
    location.targetlat = progTargetlat;
    location.targetlon = progTargetlon;
    EEPROM_writeAnything(10, location);
  }
}
/*
* Tries to calculate the distance to the specified lat+lon, using the GPS sensor.
* Returns true if the sensor returned a valid reading.
* On successful return, result will hold the great circle distance to the specified point.
*/
bool distanceTo( float targetLat, float targetLon, float* result)
{
  int numFixes = 0;
  while (gpsSerial.available())
  {
    if (feedgps())
    {
      gps.f_get_position(&flat, &flon, &fix_age);
      //Debug, prints the gps cooordinates on the lcd
      /*
      lcd.clear();
      lcd.print("lat:"); lcd.print(flat);
      lcd.setCursor(0,1);
      lcd.print("lon:"); lcd.print(flon);
      lcd.print(" age:"); lcd.print(fix_age);
      delay (5000);
      */
      if (fix_age > 60000) continue;
      if (locationprog == 1){locationSwitch();}
      flat = fabs(flat);
      flon = fabs(flon);
      float dist = gcd( flat * DEG_TO_RAD, flon * DEG_TO_RAD, targetLat * DEG_TO_RAD, targetLon * DEG_TO_RAD );
      *result = dist;
      return true;
    }
  }
  return false;
}
//Calculate distance form lat1/lon1 to lat2/lon2 using haversine formula
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns float distance in km
float gcd(float lat1, float lon1, float lat2, float lon2)
{
float dlon, dlat, a, c;
float dist = 0.0;
dlon = lon2 - lon1;
dlat = lat2 - lat1;
a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);
c = 2 * atan2(sqrt(a), sqrt(1-a));

dist = EARTH_RADIUS * c;
return(dist);
}

bool feedgps()
{
  while (gpsSerial.available())
  {
    if (gps.encode(gpsSerial.read()))
    {
      return true;
    }    
  }
  return false;
}
//Locationswitching
void locationSwitch()
{
  if (fix_age > 60000) return;
  //EEPROM.write(ADDR_FLAT, flat);
  //EEPROM.write(ADDR_FLON, flon);
  location.targetlat = flat;
  location.targetlon = flon;
  EEPROM_writeAnything(10, location);
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("NEW LOCATION");
  lcd.setCursor(0,1);
  lcd.print("N"); lcd.print(flat, 4); lcd.print(" E"); lcd.print(flon, 4);
  delay(8000);
  shutdown();
}
void lockLid()
{
  gpsSerial.end();
  lockservo.attach(11);
  delay(10);
  lockservo.write(servolocked);
  delay(200);
  lockservo.detach();
  delay(100);
  gpsSerial.begin(4800);
}
void openLid()
{
  gpsSerial.end();
  lockservo.attach(11);
  delay(10);
  lockservo.write(servoopen);
  delay(200);
  lockservo.detach();
  gpsSerial.begin(4800);
}
void shutdown()
{
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("SHUTTING");
  lcd.setCursor(6, 1);
  lcd.print("DOWN");
  delay(2500);
  digitalWrite (shutdownpin, HIGH);
  while(digitalRead(setuppin == HIGH))
  {}
}
