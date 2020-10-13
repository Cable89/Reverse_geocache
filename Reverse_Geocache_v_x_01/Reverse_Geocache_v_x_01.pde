/*
Reverse Geocache
*/

#include <LiquidCrystal.h>
#include <NewSoftSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <math.h>

#define EARTH_RADIUS 6378.14f
#define DEG_TO_RAD 0.0174532925f

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
NewSoftSerial gpsSerial(8, 9);
Servo lockservo;
TinyGPS gps;

int a = 0;
//int buttonpin = 12;
int shutdownpin = 10;
float distancetolerance = 0.1;
int ADDR_TRIES = 0;
int ADDR_SOLVED = 1;
int ADDR_TOGGLE = 2;
int tries = 0;
int servolocked = 160;
int servoopen = 120;
int setuppin = 12;
unsigned long gpsfixdelay = 60000;
//WGS 84 - desimal
float targetlat = 59.133540;
float targetlon = 9.654600;
float distancefromtarget = 100.0;
	
void setup()
{
  lcd.begin(16, 2);
  lcd.print("   -SCANNING-");
  gpsSerial.begin(4800);
  //pinMode(buttonpin, INPUT);
  pinMode (shutdownpin, OUTPUT);
  pinMode (setuppin, INPUT);
  digitalWrite (shutdownpin, LOW);
  lockservo.attach(11);
  lockLid();
  if(digitalRead(setuppin) == LOW  )
  {
    openLid(); //backdoor
    initializeEEPROM();
    lockLid();
    shutdown();
  }
}

void loop()
{
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
  if(EEPROM.read(ADDR_SOLVED) == 1)
  {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" ACCESS GRANTED!");
    //if(EEPROM.read(ADDR_TOGGLE) == 1)
    //{
      //openLid();
      //EEPROM.write(ADDR_TOGGLE, 2);
      //delay(5000);
      //shutdown();
    //}
    if(EEPROM.read(ADDR_TOGGLE) == 1)
    {
      EEPROM.write(ADDR_TOGGLE, 0);
      lcd.setCursor(0,1);
      lcd.print("   Locking...");
      delay(2000);
      lockLid();
      delay(3000);
      shutdown();
    }
    else
    {
      openLid();
      EEPROM.write(ADDR_TOGGLE, 1);
      delay(5000);
      shutdown();
    }
  }
  if(EEPROM.read(ADDR_SOLVED) == 0)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   -SCANNING-");
    //sjekk om GPS fix
    if( distanceTo(targetlat, targetlon, &distancefromtarget))
    {
      //if box has reached target destination
      if(distancefromtarget < distancetolerance)
      {
        EEPROM.write(ADDR_SOLVED, 1);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" ACCESS GRANTED!");
        openLid();
      }
      else
      {
        tries--;
        EEPROM.write(ADDR_TRIES, tries);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" ACCES DENIED!");
        lcd.setCursor(1,1);
        lcd.print(tries);
        lcd.print(" Tries left");
        delay(3000);
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
      delay(1000);
      a++;
      if (a >= 65)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("   NO SIGNAL!");
        delay(3500);
        shutdown();
      }
    }
  }
}
void initializeEEPROM()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("  INITIALIZING");
  lcd.setCursor(0,1);
  lcd.print("     EEPROM");
  EEPROM.write(ADDR_TRIES, 11);
  EEPROM.write(ADDR_SOLVED, 0);
  EEPROM.write(ADDR_TOGGLE, 0);
  delay(2000);
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
    int c = gpsSerial.read();
    if (gps.encode(c))
    {
      unsigned long fix_age;
      float flat,flon;
      gps.f_get_position(&flat, &flon, &fix_age);
      //Debug, prints the gps cooordinates on the lcd
      //lcd.clear();
      //lcd.setCursor(0,0);
      //lcd.print("lat:"); lcd.print(flat);
      //lcd.setCursor(0,1);
      //lcd.print("lon:"); lcd.print(flon);
      //lcd.print(" age:"); lcd.print(fix_age);
      //delay (20000);
      if (fix_age > 60000) continue;
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
void lockLid()
{
  lockservo.write(servolocked);
}
void openLid()
{
  lockservo.write(servoopen);
}
void shutdown()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    SHUTTING");
  lcd.setCursor(0, 1);
  lcd.print("      DOWN");
  delay(2500);
  digitalWrite (shutdownpin, HIGH);
  while(digitalRead(setuppin == HIGH))
  {}
}
