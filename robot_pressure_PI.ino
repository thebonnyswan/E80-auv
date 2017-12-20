/********
 * Lab 1 Sketch
 * Author: Apoorva Sharma (asharma@hmc.edu) 
 * Edited by: Josephine Wong (jowong@hmAsssGPS.h>
*/

/* Libraries */
#include <MotorDriver.h>
#include <Params.h>
#include <SdFat.h>
#include <MyLogger.h>
#include <StateEstimator.h>
#include <SPI.h>
#include <Wire.h>
// GPS
#include <TinyGPS.h>
#include <SensorGPS.h>

// Adafruit 9DOF IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SensorIMU.h>

// ADC Sampler
#include <ADCSampler.h>

/* Global Variables */
#define LOOP_INTERVAL 100 // in ms
IntervalTimer controlTimer;
MotorDriver motorDriver(MOTOR_L_FORWARD,MOTOR_L_REVERSE,MOTOR_R_FORWARD,MOTOR_R_REVERSE,MOTOR_V_FORWARD,MOTOR_V_REVERSE, MOTOR_V2_FORWARD, MOTOR_V2_REVERSE); 

// Sensors
// IMU
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified();
SensorIMU imu(&dof, &accel, &mag, &gyro);

ADCSampler adc;

//GPS
HardwareSerial Uart = HardwareSerial(); // pin 1 = rx (from gps tx), pin 2 = tx (from gps rx)
SensorGPS gps(&Uart);

// Logger ASI
SdFat sd;
SdFile file;
MyLogger logger(sd,file);

// State Estimator
StateEstimator stateEstimator;

// Path
#define NUM_WAYPOINTS 3

#define NUM_STATES 3

// modes, etc
#define NAVIGATION_MODE 0
#define DIVING_MODE 1
#define RECOVERY_MODE 2
#define HOVER_TIME 90000

int mode = NAVIGATION_MODE;

double WAYPOINT_X = 0;
double WAYPOINT_Y = 0;

size_t curridx = 0;
int currentWayPoint = 0;
//double wayPoints[NUM_WAYPOINTS][NUM_STATES] = {{0,-15},{0,0},{0,0}}; //actually right motor
float e_accum = 0.0;

#define NUM_DIVING_STAGES 3
size_t diving_stage = 0;
float depths[NUM_DIVING_STAGES] = {1.5, 1.75, 2};
unsigned long timer = 0;



unsigned long last_trans = 0;

// setup(): initializes logger and motor pins
void setup() {
  Serial.begin(115200);
  delay(2000); // Wait to ensure computer monitor is ready
  Serial.println(F("Serial connection started")); 
  Serial.println("");
  Serial.print("\nLogger: Initializing SD card...");

  // check if the card is present and can be initialized
  if(!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    Serial.println("Card failed, or not present");
    // don't do anything more
    return;
  }
  Serial.println("Card initialized");

  /* Initialize the Logger */
  logger.include(&stateEstimator);
  logger.include(&gps);
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.include(&adc);
  logger.init();

  /* Initialise the sensors */
  //gps.init();
  Serial.println("initialized gps");
  imu.init();
  Serial.println("initialized imu");
  
  //-------Initialize motor pins--------//
  pinMode(MOTOR_L_FORWARD,OUTPUT);
  pinMode(MOTOR_L_REVERSE,OUTPUT);
  pinMode(MOTOR_R_FORWARD,OUTPUT);
  pinMode(MOTOR_R_REVERSE,OUTPUT);
  pinMode(MOTOR_V_FORWARD,OUTPUT);
  pinMode(MOTOR_V_REVERSE,OUTPUT);
  pinMode(MOTOR_V2_FORWARD,OUTPUT);
  pinMode(MOTOR_V2_REVERSE,OUTPUT);

  pinMode(LOOP_LED,OUTPUT);
  Serial.println("starting control loop");
  Serial.println("Press any character to stop logging");
  last_trans = millis();
  controlTimer.begin(controlLoop, LOOP_INTERVAL*1000);
}



// controlLoop(): updates motor output at every LOOP_INTERVAL 
void controlLoop(void) {
  
  // Gather data from IMU and GPS
  imu.read(); // this is a sequence of blocking I2C read calls
  imu.printState(); // only uncomment this for debugging
  gps.read(); // this is a sequence of UART reads, bounded by a time
  gps.printState();

  adc.updateSample();
  adc.printSample();

  // Calculate x and y position from GPS
  latlongToXY(&stateEstimator.state.x, &stateEstimator.state.y, gps.state.lon, gps.state.lat);
  double uL = 0;
  double uR = 0;
  double uV = 0;
  
  updateMode(&mode, stateEstimator.state.x, stateEstimator.state.y);
  Serial.print("mode is: "); Serial.println(mode);
  // NAVIGATION MODE
  if (mode == NAVIGATION_MODE) {
    //Serial.print(stateEstimator.state.x); Serial.print(' '); Serial.println(stateEstimator.state.y);
  
    // Determine desired point to track in path
    //currentWayPoint = updatePointToTrack(currentWayPoint, stateEstimator.state.x, stateEstimator.state.y);
    //Serial.println(currentWayPoint);
  
    // Calculate the required control signals
    //PControl(&uL, &uR, WAYPOINT_X, WAYPOINT_Y, stateEstimator.state.x, stateEstimator.state.y, imu.state.orientation.heading);
  }

  // DIVING MODE
  if (mode == DIVING_MODE) {
    float z_des = depths[diving_stage];
    float z = v_to_depth(analogRead(14)); //A00
    uL = 0;
    uR = 0;
    e_accum = e_accum + (z_des-z);
    DPIControl(&uV, z_des, z, e_accum);
    if (millis()-timer > HOVER_TIME) {
      diving_stage++;
      timer = millis();
      e_accum = 0;
    }
  }

  // RECOVERY MODE
  if (mode == RECOVERY_MODE){
    float z_des = 0.1;
    float z = v_to_depth(analogRead(14)); //A00
    e_accum = e_accum + (z_des-z);
    DPIControl(&uV, z_des, z, e_accum);
    //PControl(&uL, &uR, WAYPOINT_X, WAYPOINT_Y, stateEstimator.state.x, stateEstimator.state.y, imu.state.orientation.heading);
  }


  
  motorDriver.left = uL;
  motorDriver.right = uR;
  motorDriver.vertical = uV;
  motorDriver.apply();
  motorDriver.printState();

  digitalWrite(LOOP_LED,1);
  logger.log();
}

void updateMode(int* mode, float x, float y)
{
  if (*mode == NAVIGATION_MODE) {
    float dist = sqrt(pow(x-WAYPOINT_X,2) + pow(y-WAYPOINT_Y,2)); 
    float wayPointThreshold = 6.0;
//    if (dist < wayPointThreshold) {
//      *mode = DIVING_MODE;
//      timer = millis();
//    }

      if (millis() >120000) {
        *mode = DIVING_MODE;
        timer = millis();
      }
  }
     else if (*mode == DIVING_MODE) {
      if (diving_stage >= NUM_DIVING_STAGES) {
        *mode = RECOVERY_MODE;
      }
    }
    else if (*mode == RECOVERY_MODE) {
      // just keep swimmmmming. just keeep swiimmmmminggggg
    }
}

int updatePointToTrack(int currentPoint, float x, float y)
{
  float dist = sqrt(pow(x-WAYPOINT_X,2) + pow(y-WAYPOINT_Y,2)); 
  float wayPointThreshold = 6.0;
  //Serial.println(dist); 
  
  if (dist < wayPointThreshold && currentPoint < NUM_WAYPOINTS )
    currentPoint ++;
    
  return currentPoint;
}

void latlongToXY(float *x, float *y, double lon, double lat)
{
  
  double RADIUS_OF_EARTH_M = 6371000;
  // middle of Phake Lake 34.109108, -117.712624
  double orig_lat =  34109108;//34.103835; //Scripp's Pool: 34.103823, -117.708062
  double orig_lon = -117712624;//-117.708172; //Pitzer's Pool: 34.103952, -117.703778
  //34109463;//34106465;//34.103835;
  //double orig_lon =  -117712776;//-117712488;//-117.708172;
  float cosOrigLat = cos(orig_lat/1000000.0*M_PI/180.0);
  
  *x = (lon-orig_lon)*M_PI/180.0*RADIUS_OF_EARTH_M/1000000.0*cosOrigLat;
  *y = (lat-orig_lat)*M_PI/180.0*RADIUS_OF_EARTH_M/1000000.0;
  //Serial.print(lat); Serial.print(' '); Serial.println(lon);


}

void PControl(double * uL, double * uR, float x_des, float y_des, float x, float y, double heading)
{
  float K_P = 20.0;
  float yaw_des = angleDiff(atan2(y_des - y, x_des - x));
  float yaw = angleDiff((-heading+90)/180.0*PI -0.5);
  float u = K_P*angleDiff(yaw_des - yaw);
  //Serial.print(yaw_des); Serial.print(' ');Serial.print(yaw); Serial.print(' ');Serial.print
  //(-heading+90);Serial.print(' ');Serial.println(u);
  float uNom = 30.0;
  float KL = 1.0;
  float KR = 1.0;
  *uL = max(0.0,min(127.0,(uNom - u)*KL));
  *uR = max(0.0,min(127.0,(uNom + u)*KR));
  Serial.print("angular displacement: "); Serial.println(yaw_des-yaw);
  Serial.print("x: "); Serial.println(x);
  Serial.print("y: "); Serial.println(y);

}

float angleDiff(float a)
{
  while (a<-PI)
    a += 2*PI;

  while (a>PI)
    a -= 2*PI;
    
  return a;
}

void DPControl(double* uV, float z_des, float z)
{
  float K_P = 60.0;
  float diff = z_des - z;
  *uV = max(-127.0, min(127.0, diff*K_P));
}

void DPIControl(double* uV, float z_des, float z, float e_accum)
{
  float K_P = 100.0;
  float K_I = 0.3;
  float diff = z_des - z;
  double uVP;
  double uVI;
  //uVP = max(-127.0, min(127.0, diff*K_P));
  //uVI = max(-127.0,min(127.0,e_accum*K_I));
  uVP = diff*K_P;
  uVI = e_accum*K_I;
  *uV = max(-127.0, min(127.0, uVP + uVI));
  //*uV = uVP + uVI;
  //Serial.print("uV is:              ");
  //Serial.println(*uV);
}


float v_to_depth(float v) 
{
  //TODO: fill in linear fit
  v = v*3.3/1023.0;
  return 0.7077*v -0.103;
}


// loop(): write the buffered data to the sd card
void loop() {
  digitalWrite(LOOP_LED,0);
  logger.write();
  if (digitalRead(USER_SWITCH==0)) {
    while(1) {digitalWrite(LOOP_LED,1); delay(500); digitalWrite(LOOP_LED,2); delay(500);}
  }
}
