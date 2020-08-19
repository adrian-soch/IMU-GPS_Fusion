#include <TinyGPS++.h>
#include <Haversine.h>

static const uint32_t GPSBaud = 9600;
double cur_lat;
double cur_lon;
double prev_lat;
double prev_lon;
double dist;

// The TinyGPS++ object
TinyGPSPlus gps;
Haversine haversine;


void setup()
{
  SerialUSB.begin(115200);
  Serial1.begin(GPSBaud);
}

void loop()
{
  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
       if(gps.speed.isUpdated() && gps.speed.isValid()){
        
        cur_lon = gps.location.lng();
        cur_lat = gps.location.lat();
        dist = haversine.calcDistance(cur_lat, cur_lon, prev_lat, prev_lon);
        SerialUSB.println("Speed = " + String(gps.speed.kmph()) + "    Distance = " + String(dist));
       }
    }
  }  
      
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    SerialUSB.println(F("No GPS detected: check wiring."));
    while(true);
  }

  prev_lat = cur_lat;
  prev_lon = cur_lon;
  displayInfo();
}

void displayInfo()
{
  SerialUSB.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    SerialUSB.print(gps.date.month());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.day());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.year());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.hour());
    SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.minute());
    SerialUSB.print(F(":"));
    if (gps.time.second() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.second());
    SerialUSB.print(F("."));
    if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.centisecond());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.println();
}
