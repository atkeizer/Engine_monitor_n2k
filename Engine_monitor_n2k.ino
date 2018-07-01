
/* 
 *  Version 1.0
 *  
 *  20180701
 *  
 */



#include <Arduino.h>
#include <NMEA2000_CAN.h> 
#include <N2kMessages.h>

#include "tc_lib.h"

using namespace arduino_due;

#define CAPTURE_TIME_WINDOW 2500000 // usecs

capture_tc0_declaration();
auto& capture_pin2=capture_tc0;

void setup() {
  capture_pin2.config(CAPTURE_TIME_WINDOW);
  Serial.begin(115200);
  analogReadResolution(12);
  NMEA2000.SetProductInformation("12345678", // Manufacturer's Model serial code
                                 101, // Manufacturer's product code
                                 "Engine Monitor",    // Manufacturer's Model ID
                                 "1.0 (2018-07-01)",  // Manufacturer's Software version code
                                 "1.0 (2018-07-01)"   // Manufacturer's Model version
                                 );
  NMEA2000.SetDeviceInformation(1, 130, 75, 2046 );
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,22);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.Open();
}

void loop() {
  SendN2kRapidData();
  SendN2kSlowData();
  NMEA2000.ParseMessages();
  SendIsoAddressClaim();
}

void SendIsoAddressClaim()
{
static unsigned long LastSent=millis();
   if ( LastSent+2000<millis() ) {
      LastSent=millis();
      NMEA2000.SendIsoAddressClaim();
   }
}

double ReadRPM() {
  double rpm=0;
  uint32_t status,duty,period;
  status=capture_pin2.get_duty_and_period(duty,period);
  // 42 pulses per rotation
  // period in uS
  if ( period ) 
     rpm = (1000000/((double)period/(double)capture_pin2.ticks_per_usec()) / 42 ) * 60;
  Serial.print("RPM   ");
  Serial.println(rpm, 1);
  return rpm;
}  

double ReadOilTemp(){
  return ReadCoolantTemp();
}


double interpolate(double x, double x0, double x1, double y0, double y1){ // simple linear interpolation
  return (x-x1)/(x0-x1)*y0 + (x-x0)/(x1-x0)*y1;
}

double ReadCoolantTemp(){
  double temp=0, r;
  double curve[] = { 1037,608,367,229,147,98,67,47,34,26, 19, 14.76, 11.46, 9.01, 7.15}; // VDO curve 3 (temp = index * 10)
  // 330R pull up
  double a = (double)analogRead(A1)/4096;
  r =  a * 330 / (1-a);
  int i=0;
  while (i<14 && temp==0){
     if ( r > curve[i] ) temp=interpolate(r, curve[i], curve[i+1], i*10, i*10+10);
     i++;
  }
  Serial.print("Analog A1 ");
  Serial.print((double)analogRead(A1),2);
  Serial.print(" R ");
  Serial.print(r,2);
  Serial.print(" TEMP  ");
  Serial.println(temp,2);
  return CToKelvin(temp);
}

double ReadOilPressure() {
  double kpa=0, r;
  // 1k8 pull up
  // at  0 kPa R = 10 ohm
  // at 10 kPa R = 180 ohm
  double a = (double)analogRead(A0)/4096;
  r = a * 1800 / (1-a);
  kpa = 1000 * interpolate(r, 10, 180, 0, 10); 
  if (kpa < 0) kpa = 0;
  Serial.print("Analog A0 ");
  Serial.print((double)analogRead(A0),2);
  Serial.print(" R ");
  Serial.print(r,2);
  Serial.print("  OILP  ");
  Serial.println(kpa,2);
  return kpa;
}



#define SlowDataUpdatePeriod 1000

void SendN2kSlowData() {
  static unsigned long SlowDataUpdated=millis();
  tN2kMsg N2kMsg;

  if ( SlowDataUpdated+SlowDataUpdatePeriod<millis() ) {
    SlowDataUpdated=millis();

    // Alternator
    SetN2kDCStatus(N2kMsg,1,1,N2kDCt_Alternator,20,63,5420,0.21);
    NMEA2000.SendMsg(N2kMsg);


    // Engine dynamic parameters  Oil pressure, Oil temp, Coolant temp, Alternator voltage, Fuel rate (1.0), Hours
    SetN2kEngineDynamicParam(N2kMsg,1,ReadOilPressure(),ReadOilTemp(),ReadCoolantTemp(),14.21,5.67,hToSeconds(1137.55));
    NMEA2000.SendMsg(N2kMsg);
  }
}

#define RapidDataUpdatePeriod 95

void SendN2kRapidData() {
  static unsigned long RapidDataUpdated=millis();
  tN2kMsg N2kMsg;

  if ( RapidDataUpdated+RapidDataUpdatePeriod<millis() ) {
    RapidDataUpdated=millis();
    SetN2kEngineParamRapid(N2kMsg,1, ReadRPM() );
    NMEA2000.SendMsg(N2kMsg);
  }
}
