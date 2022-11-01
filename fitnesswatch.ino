#include <Wire.h>
#include <DFRobot_LIS2DH12.h>

#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_I2C.h"
#include "Wire.h"
#include "SPI.h"
#include "math.h"
#include "bmp3_defs.h"

/*If there is no need to calibrate altitude, comment this line*/
#define CALIBRATE_Altitude

/*Create a bmp388 object to communicate with IIC.*/
DFRobot_BMP388_I2C bmp388;

float seaLevel, saved_alt;
int steps, flights, saved_pos_x, saved_pos_y, saved_pos_z;

float FLIGHT_DIFF = 2.5;
int STEP_DIFF = 575;
int LED_PORT = A0;
int STEPS_GOAL = -1;

bool printing = false;

//include blynk library for USB connection
#include <BlynkSimpleSerialBLE.h>

//Edit the token here inside the "" to match your apps token.
//find your token by going to the Project Settings (nut icon) on blynk app.
char auth[] = "HccQSaRd1PIynpRvsq1_dI8z24VrjEpi";

DFRobot_LIS2DH12 LIS; //Accelerometer

void alt(void) {
  float new_alt = bmp388.readCalibratedAltitude(seaLevel);
  
  float diff = new_alt - saved_alt;
  if (abs(diff) > FLIGHT_DIFF) {
    flights++;
    saved_alt = new_alt;
    
    Blynk.virtualWrite(V1, flights);
  }
  delay(100);

  
  if (printing) {
    Serial.print("calibrate Altitude : ");
    Serial.print(new_alt);
    Serial.println(" m");
      
    Serial.print("Flights: ");
    Serial.println(flights);
    Serial.print("Diff: ");
    Serial.println(diff);
  }
}

void acc(void)
{
  int16_t x, y, z;
  
  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);

  float diff_x = x - saved_pos_x;
  float diff_y = y - saved_pos_y;
  float diff_z = z - saved_pos_z;

  float total = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
  
  if (total > STEP_DIFF) {
    steps++;
    saved_pos_x = x;
    saved_pos_y = y;
    saved_pos_z = z;
    Blynk.virtualWrite(V0, steps);
  }
  
  if (printing) {
    Serial.print("Acceleration x: "); //print acceleration
    Serial.print(x);
    Serial.print(" mg \ty: ");
    Serial.print(y);
    Serial.print(" mg \tz: ");
    Serial.print(z);
    Serial.println(" mg");
    
    Serial.print("Steps: ");
    Serial.println(steps);
  }
}

void clear_data(void) {
  // alt
  saved_alt = bmp388.readCalibratedAltitude(seaLevel);
  flights = 0;
  STEPS_GOAL = -1;

  // acc
  steps = 0;
  int16_t x, y, z;
  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  saved_pos_x = x;
  saved_pos_y = y;
  saved_pos_z = z;
     

  Blynk.virtualWrite(V0, steps);
  Blynk.virtualWrite(V1, flights);
}

void check_led(void) {
  if (steps >= STEPS_GOAL) {
    digitalWrite(LED_PORT, HIGH);
  } else {
    digitalWrite(LED_PORT, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Blynk.begin(Serial, auth);
  
  bmp388.set_iic_addr(BMP3_I2C_ADDR_SEC);
  /* Initialize bmp388*/
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }
  /*You can use an accurate altitude to calibrate sea level air pressure. 
   *And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
   *In this case,525.0m is chendu accurate altitude.
   */
  delay(100);
  seaLevel = bmp388.readSeaLevel(525.0);

  Wire.begin();
  while(!Serial);
  delay(100);
  
  // Set measurement range
  // Ga: LIS2DH12_RANGE_2GA
  // Ga: LIS2DH12_RANGE_4GA
  // Ga: LIS2DH12_RANGE_8GA
  // Ga: LIS2DH12_RANGE_16GA
  while(LIS.init(LIS2DH12_RANGE_16GA) == -1){  //Equipment connection exception or I2C address error
    Serial.println("No I2C devices found");
    delay(1000);
  }
  clear_data();
  // check_led
  pinMode(LED_PORT, OUTPUT);
}

BLYNK_WRITE(V2) {
  clear_data();
}
BLYNK_WRITE(V3) {
  STEPS_GOAL += param.asInt();
  
  Blynk.virtualWrite(V5, STEPS_GOAL);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  acc();
  alt();
  check_led();
}
