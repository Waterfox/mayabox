#include "quaternionFilters.h"
#include "MPU9250.h"
#include "Adafruit_VL6180X.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <PWMServo.h>


#define AHRS false         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

#define LED_NEO_PIN 1
#define NEO_NUMPIXELS 48
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0


MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
Adafruit_VL6180X vl = Adafruit_VL6180X();
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_NUMPIXELS, LED_NEO_PIN, NEO_GRB + NEO_KHZ800);


float LEDarr[48][2] = {{-.875,1.0},{-.75,1.0},{-.625,1.0},{-.5,1.0},{-.375,1.0},{-.25,1.0},{-.125,1.0},{0,1.0},{.125,1.0},{.25,1.0},{.375,1.0},{.5,1.0},{.625,1.0},{.75,1.0},{.875,1.0},
                      {1.0,0.8},{1.0,0.6},{1.0,0.4},{1.0,0.2},{1.0,0},{1.0,-0.2},{1.0,-0.4},{1.0,-0.6},{1.0,-0.8},
                      {.875,-1.0},{.75,-1.0},{.625,-1.0},{.5,-1.0},{.375,-1.0},{.25,-1.0},{.125,-1.0},{0,-1.0},{-.125,-1.0},{-.25,-1.0},{-.375,-1.0},{-.5,-1.0},{-.625,-1.0},{-.75,-1.0},{-.875,-1.0},
                      {-1.0,-0.8},{-1.0,-0.6},{-1.0,-0.4},{-1.0,-0.2},{-1.0,0},{-1.0,0.2},{-1.0,0.4},{-1.0,0.6},{-1.0,0.8}};


const int estopbuttonPin=2;
const int ksw1Pin = 3;
const int ksw2Pin = 4;
const int button1Pin = 9;
const int button1LedPin = 8;
const int button2Pin = 11;
const int button2LedPin = 10;
const int button3Pin = 13;
const int button3LedPin = 12;
const int button4Pin = 14;
const int button4LedPin = 15;
const int button5Pin = 16;
const int button5LedPin = 17;
const int led1Pin =5;
const int led2Pin =6;
const int led3Pin =7;
const int redbuttonPin= 20;
const int bigbuttonPin = 0;
PWMServo myservo; 
const int servoPin = 23;
const int motorinA = 22;
const int motorinB = 21;

int lastrange = 186;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(11520);
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);
 if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

     if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

        // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    if(SerialDebug)
    {
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
      Serial.println("AK8963 mag biases (mG)");
      Serial.println(myIMU.magBias[0]);
      Serial.println(myIMU.magBias[1]);
      Serial.println(myIMU.magBias[2]);

      Serial.println("AK8963 mag scale (mG)");
      Serial.println(myIMU.magScale[0]);
      Serial.println(myIMU.magScale[1]);
      Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
  }
  /*
  while (!Serial) 
  {
    delay(1);
  }
  */
  //VL6180X RANGE SENSOR
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) 
  {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
  //NEO PIXEL INIT
  pixels.begin(); 

//BUTTONS
  pinMode(estopbuttonPin,INPUT);
  pinMode(ksw1Pin,INPUT);
  pinMode(ksw2Pin,INPUT);
  pinMode(button1Pin,INPUT);
  pinMode(button1LedPin,OUTPUT);
  pinMode(button2Pin,INPUT);
  pinMode(button2LedPin,OUTPUT);
  pinMode(button3Pin,INPUT);
  pinMode(button3LedPin,OUTPUT);
  pinMode(button4Pin,INPUT);
  pinMode(button4LedPin,OUTPUT);
  pinMode(button5Pin,INPUT);
  pinMode(button5LedPin,OUTPUT);
  pinMode(led1Pin,OUTPUT);
  pinMode(led2Pin,OUTPUT);
  pinMode(led3Pin,OUTPUT);
  pinMode(redbuttonPin,INPUT);
  pinMode(bigbuttonPin,0);
  pinMode(motorinA,OUTPUT);
  pinMode(motorinB,OUTPUT);
  myservo.attach(servoPin);


}

void loop() {

  // READ BUTTONS
  int ES = digitalRead(estopbuttonPin);
  int ksw1 = digitalRead(ksw1Pin);
  int ksw2 = digitalRead(ksw2Pin);
  int but1 = digitalRead(button1Pin);
  int but2 = digitalRead(button2Pin);
  int but3 = digitalRead(button3Pin);
  int but4 = digitalRead(button4Pin);
  int but5 = digitalRead(button5Pin);
  int redbut = digitalRead(redbuttonPin);
  int bigbut = digitalRead(bigbuttonPin);

  //KSW
  if (ksw1 == false && ksw2 == false){
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, LOW);
  }
  else if  (ksw1 == true && ksw2 == false){
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, LOW);
  }
  else if  (ksw2 == true && ksw1 == true){
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, HIGH);
  }
  // SILVER Push buttons and LEDS
  if (but1 == HIGH){
    digitalWrite(button1LedPin, HIGH);
  }
  else{
    digitalWrite(button1LedPin, LOW);
  }
  if (but2 == HIGH){
    digitalWrite(button2LedPin, HIGH);
  }
  else{
    digitalWrite(button2LedPin, LOW);
  }
  if (but3 == HIGH){
    digitalWrite(button3LedPin, HIGH);
  }
  else{
    digitalWrite(button3LedPin, LOW);
  }
  if (but4 == HIGH){
    digitalWrite(button4LedPin, HIGH);
  }
  else{
    digitalWrite(button4LedPin, LOW);
  }
  if (but5 == HIGH){
    digitalWrite(button5LedPin, HIGH);
  }
  else{
    digitalWrite(button5LedPin, LOW);
  }
  // 
  if (bigbut == HIGH && redbut == HIGH && ES == true)
  {
    digitalWrite(motorinA,LOW);
    digitalWrite(motorinB,HIGH);
  }
  else  if (bigbut == HIGH && redbut == LOW && ES == true)
  {
    digitalWrite(motorinA,HIGH);
    digitalWrite(motorinB,LOW);
  }
  else  if (bigbut == LOW)
  {
    digitalWrite(motorinA,LOW);
    digitalWrite(motorinB,LOW);
  }
  //Serial.print("redBut ");Serial.println(redbut);

  // put your main code here, to run repeatedly:
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  //VL6180X Range Sensor
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  //uint8_t range = vl.readRange();
  //uint8_t status = vl.readRangeStatus();
  int range = vl.readRange();
  int status = vl.readRangeStatus();
  //NEOPIXELS

  //myIMU.ax Gs, measured from -1 to 1
  //myIMU.ay Gs, measured from -1 to 1
  //LEDarr
  //Serial.println(status);
  if (ES==true)
  {
    int red = 0;
    if (status > 0){
      red = 8;
    }
    else {
      red = (185-range)/2;
    }
    
    for(int i=0;i<NEO_NUMPIXELS;i++){
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      float r = 1/sqrt(pow(LEDarr[i][0]-(-myIMU.ay),2)+pow(LEDarr[i][1]-(-myIMU.ax),2));
      pixels.setPixelColor(i, pixels.Color(red,0,int(max(r-0.5,0)*30.0)));
    }

    if ((abs(range - lastrange) > 5) && (but1 == HIGH))
    {
      myservo.write(max(10,min(range,175)));
    }
    lastrange=range;
  }
  else
  {
    for(int i=0;i<NEO_NUMPIXELS;i++){
      pixels.setPixelColor(i, pixels.Color(50,0,0));
    }
  }
  pixels.show(); // This sends the updated pixel color to the hardware.


  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 250)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
        Serial.print("Lux: "); Serial.println(lux);
        if (status == VL6180X_ERROR_NONE) {
          Serial.print("Range: "); Serial.println(range);
        }
      }
      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
}
