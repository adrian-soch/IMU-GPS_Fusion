#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>
#include "Kalman.h"
using namespace BLA;

#define SerialPort SerialUSB
#define FREQ 100
//#define dt 0.01
//const double alpha = 1-0.8333;
MPU9250_DMP imu;
TinyGPSPlus gps;
//double calc_speed = 0;
double gps_speed = 0;
double accel;
const long interval = 100; //Print data at 10 Hz
unsigned long previousMillis = 0;

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3 // position, speed, acceleration
#define Nobs 2   // position, acceleration

// measurement std
#define n_p 0.3
#define n_a 5.0
// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8


BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // your Kalman filter

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

unsigned long T;
float DT;

//BLA::Matrix<Nstate> state; // true state vector for simulation
//BLA::Matrix<Nobs> noise;   // additive noise for simulation
//
//#define LOOP_DELAY 10  // add delay in the measurement loop
//#define SIMUL_PERIOD 0.3 // oscillating period [s]
//#define SIMUL_AMP 1.0

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {
  SerialPort.begin(115200);
  Serial1.begin(57600);

  // The model below is very simple since matrices are diagonal!
  
  // time evolution matrix
  K.F = {1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  // measurement matrix
  K.H = {1.0, 0.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_p*n_p,   0.0,
           0.0, n_a*n_a};
  // model covariance matrix
  K.Q = {m_p*m_p,     0.0,     0.0,
             0.0, m_s*m_s,     0.0,
			 0.0,     0.0, m_a*m_a};
  
  T = millis();

 if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration 
               100); 
}

void loop() {

 if ( imu.dmpUpdateFifo() )
  {
    imu.computeEulerAngles();
    double accelX = imu.calcAccel(imu.ax)*9.81;
    double accelY = imu.calcAccel(imu.ay)*9.81;
    //double accelZ = imu.calcAccel(imu.az)*9.81;
    accel = pow(accelX, 2) + pow(accelY, 2);
    accel = sqrt(accel);
    
    //SerialPort.print("Accel = " + String(accelZ)); // check that output is 9.81 ms/s^2
  }

  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
       if(gps.speed.isUpdated() && gps.speed.isValid()){
        //SerialPort.print("Speed = ");
        //SerialPort.println(gps.speed.mps());
        gps_speed = gps.speed.mps();
        // Perform
       }
    }
  }  
	
  // TIME COMPUTATION
  DT = (millis()-T)/1000.0;
  T = millis();

  K.F = {1.0,  DT,  DT*DT/2,
		 0.0, 1.0,       DT,
         0.0, 0.0,      1.0};

  // Perform Measurement
  SIMULATOR_UPDATE();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: measures, estimated state
  Serial << obs << ' ' << K.x << ' ' << K.P << '\n';
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

//void SIMULATOR_UPDATE(){
//	unsigned long tcur = millis();
//  state(0) = SIMUL_AMP*sin(tcur/1000.0/SIMUL_PERIOD);
//  state(1) = SIMUL_AMP/SIMUL_PERIOD*cos(tcur/1000.0/SIMUL_PERIOD);
//  state(2) = -SIMUL_AMP/SIMUL_PERIOD/SIMUL_PERIOD*sin(tcur/1000.0/SIMUL_PERIOD);
//  
//  noise(0) = n_p * SIMULATOR_GAUSS_NOISE();
//  noise(1) = n_a * SIMULATOR_GAUSS_NOISE();
//  obs = K.H * state + noise; // measure
//  
//  delay(LOOP_DELAY); //add a delay in measurement
//}

//double SIMULATOR_GAUSS_NOISE(){
//  // Generate centered reduced Gaussian random number with Box-Muller algorithm
//  double u1 = random(1,10000)/10000.0;
//  double u2 = random(1,10000)/10000.0;
//  return sqrt(-2*log(u1))*cos(2*M_PI*u2);
//}
