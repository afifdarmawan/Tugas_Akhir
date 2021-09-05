#include <Wire.h> 
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
//#include "NewPing.h"
#include <LiquidCrystal_I2C.h>  
#include <math.h>
#include <Servo.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // variable kalman

/* IMU Data */
double accX, accY, accZ;
double gyroX ; //variable gyro sudut x

double gyroXangle; //variable sudut menggunakan gyro 
double kalAngleX ; //variable sudut menggunakan KalmanFilter

double roll; //variable angle roll -|-

int distanceP; //variable us p
long durationP; //variable us p
int distanceX; //variable us x
long durationX; //variable us p
int tinggi; //variable rumus trigonometri
int tinggiNoFilter; //variable rumus trigonometri 
double hitungTANKalman; //variable rumus sudut tan
double hitungTANNoKalman; //variable rumus sudut tan

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int servo = 8; //pin data 
int awal = 180;
int offset = 10;
int nilai ;
double kalibrasi;
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

const int  ultra_trigP = 7;
const int  ultra_echoP = 6;
const int  ultra_trigX = 9;
const int  ultra_echoX = 10;

//NewPing sensorUltraP(ultra_trigP, ultra_echoP,300);
//NewPing sensorUltraX(ultra_trigX, ultra_echoX,200);
// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(20,4); 
  lcd.clear();

  myservo.attach(servo);
  pinMode(ultra_trigP,OUTPUT);
  pinMode(ultra_echoP,INPUT);
  pinMode(ultra_trigX,OUTPUT);
  pinMode(ultra_echoX,INPUT);
  
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif*/

  kalmanX.setAngle(roll); // Set starting angle
  //kalmanY.setAngle(pitch);
  gyroXangle = roll;
  //gyroYangle = pitch;
  //compAngleX = roll;
  //compAngleY = pitch;

  timer = micros();
}
void servoLurus(){
  nilai=awal-offset-kalAngleX;
  myservo.write(nilai); //sudut yang diputar
}
  
void ultrasonicX() {
  digitalWrite(ultra_trigX, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ultra_trigX, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigX, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationX = pulseIn(ultra_echoX, HIGH);
  
  // Calculating the distance
  distanceX = durationX*0.034/2;
  //distanceP = sensorUltraP.ping_cm(); //hitung jarak
  //distanceX = sensorUltraX.ping_cm(); //hitung jarak
}

void ultrasonicP() {
  digitalWrite(ultra_trigP, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ultra_trigP, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigP, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationP = pulseIn(ultra_echoP, HIGH);
  
  // Calculating the distance
  distanceP= durationP*0.034/2; //keecepatan rambat suara 
  //distanceP = sensorUltraP.ping_cm(); //hitung jarak
  //distanceX = sensorUltraX.ping_cm(); //hitung jarak
}

void bacaGyro(){
    /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  //gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  //gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //pitch = atan(-accX / sqrt(accY * accY + accZ * acc=Z)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif*/

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  //double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    //compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    //gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else*/
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  //if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    //kalmanY.setAngle(pitch);
    //compAngleY = pitch;
    //kalAngleY = pitch;
    //gyroYangle = pitch;
  //} else
    //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  //if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  //compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  //Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  //if (gyroYangle < -180 || gyroYangle > 180)
    //gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  //Serial.print(gyroY); Serial.print("derajat");
  //Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif
}

void hitungTinggi() {
  tinggi = distanceX  * hitungTANKalman  + distanceP; //sudut tan dgn kalman
  tinggiNoFilter = distanceX  * hitungTANNoKalman + distanceP; //sudut tan tanpa kalman
}

void kalibrasisdt(){
  kalibrasi = gyroXangle + 8 ;
}
void loop() {
  bacaGyro();
  ultrasonicP();
  ultrasonicX();
  hitungTinggi();
  servoLurus();
  kalibrasisdt();

//==========================================================================//  
//------------------SERIAL_PRINT--------------------------------------------//  
//==========================================================================//  
  Serial.print("SudutwithKalman :");
  Serial.print(kalAngleX);
  Serial.print("Derajat");  

  Serial.print("\t"); //tab
  Serial.print("SudutNoKalman :");
  Serial.print(gyroXangle);
  Serial.print("Derajat");

  Serial.print("\t");
  Serial.print("SudutKalibrasi :");
  Serial.print(kalibrasi);
//==========================================================================//  
//--------------------------------------------------------------//  
  Serial.println();
//--------------------------------------------------------------//
  //perhitungan sudut tan
  hitungTANKalman = tan(kalAngleX * 0.0174532925199433) ; //dengan kalman filter
  Serial.print("Tan :");
  Serial.print(kalAngleX);
  Serial.print("= ");
  Serial.print(hitungTANKalman);
 
  
  //perhitungan sudut tan
  hitungTANNoKalman = tan(gyroXangle * 0.0174532925199433) ; //tanpa filter kalman filter
  Serial.print("Tan :");
  Serial.print(gyroXangle);
  Serial.print("=  ");
  Serial.print(hitungTANNoKalman);
  
//==========================================================================//  
//--------------------------------------------------------------//  
  Serial.println();
//--------------------------------------------------------------//  
  
  Serial.print("X :");
  Serial.print(distanceX);
  Serial.print("CM");

  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("X :");
  Serial.print(distanceX);
  Serial.print("CM");
//==========================================================================//  
//--------------------------------------------------------------//
  Serial.println();
//--------------------------------------------------------------//  
  
  Serial.print("P :");
  Serial.print(distanceP);
  Serial.print("CM");

  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("P :");
  Serial.print(distanceP);
  Serial.print("CM");
//==========================================================================//  
//--------------------------------------------------------------//  
  Serial.println();
//--------------------------------------------------------------//
  Serial.print("Hasil dgn Filter:");
  Serial.print(tinggi);
  Serial.print("CM");

  Serial.print("\t");
  Serial.print("\t");
  Serial.print("Hasil Tanpa Filter:");
  Serial.print(tinggiNoFilter);
  Serial.print("CM");
//==========================================================================// 
//--------------------------------------------------------------//  
  Serial.println();
//--------------------------------------------------------------//

//==========================================================================//  
//------------------LCD_PRINT--------------------------------------------//  
//==========================================================================// 
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.clear();
  lcd.print("P:");
  lcd.setCursor(2,0); //Start at character 0 on line 0
  lcd.print(distanceP);
  lcd.setCursor(6,0); //Start at character 0 on line 0
  lcd.print("cm");
 
//--------------------------------------------------------------
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print("X:");
  lcd.setCursor(2,1); //Start at character 2 on line 1
  lcd.print(distanceX);
  lcd.setCursor(6,1); //Start at character 2 on line 1
  lcd.print("cm");
  
//--------------------------------------------------------------
  lcd.setCursor(0,2); //Start at character 6 on line 0
  lcd.print("SUDUT:");
  lcd.setCursor(7,2); //Start at character 10 on line 0
  lcd.print(kalAngleX);
  lcd.setCursor(14,2); //Start at character 2 on line 1
  lcd.print("drjt");
  
//-------------------------------------------------------------- 
  lcd.setCursor(0,3); //Start at character 0 on line 1
  lcd.print("HASIL:");
  lcd.setCursor(7,3); //Start at character 2 on line 1
  lcd.print(tinggi);
  lcd.setCursor(12,3); //Start at character 2 on line 1
  lcd.print("cm");
    
//--------------------------------------------------------------  

//#if 0 // Set to 1 to print the temperature
  //Serial.print("\t");

  //double temperature = (double)tempRaw / 340.0 + 36.53;
  //Serial.print(temperature); Serial.print("\t");
//#endif

  Serial.println();
  delay(300);
  
}


