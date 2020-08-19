#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>

#define SerialPort SerialUSB
#define FREQ 100
#define dt 0.01
const double alpha = 1-0.8333;
MPU9250_DMP imu;
TinyGPSPlus gps;
double calc_speed = 0;
double gps_speed = 0;
double accel;
double speed_kph;
const long interval = 100; //Print data at 10 Hz
unsigned long previousMillis = 0;

const byte MEAS_RATE_1 = 6;
const byte MEAS_RATE_2 = 7;
const byte NAV_RATE_1  = 8;
const byte NAV_RATE_2  = 9;

const byte NMEA_LEN = 16;
const byte FREQ_LEN = 14;
const byte BAUD_LEN = 28;

const char CFG_RATE[FREQ_LEN] = {
  0xB5, // sync char 1
  0x62, // sync char 2
  0x06, // class
  0x08, // id
  0x06, // length LSB
  0x00, // length MSB
  0x64, // payload measRate (ms) 1
  0x00, // payload measRate (ms) 2
  0x00, // payload navRate (cycles) 1
  0x00, // payload navRate (cycles) 2
  0x01, // payload timeRef 1
  0x00, // payload timeRef 2
  0x00, // CK_A
  0x00  // CK_B
};


void insertChecksum(char packet[], const byte len)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  // exclude the first and last two bytes in packet
  for (byte i = 2; i < (len - 2); i++)
  {
    ck_a += packet[i];
    ck_b += ck_a;
  }

  packet[len - 2] = ck_a;
  packet[len - 1] = ck_b;
}

void sendPacket(const char packet[], const byte len)
{
//  if (usingUSB)
//    usb_port->write(packet, len);
//  else
    Serial1.write(packet, len);
}

void sendPacket(char packet[], const byte len)
{
//  if (usingUSB)
//    usb_port->write(packet, len);
//  else
    Serial1.write(packet, len);
}



void setup() 
{
  SerialPort.begin(115200);
  Serial1.begin(57600);
  int hertz = 5;
  uint16_t normHerz = hertz / (1000 / ((CFG_RATE[MEAS_RATE_2] << 8) | CFG_RATE[MEAS_RATE_1]));
  char configPacket[FREQ_LEN];
  memcpy(configPacket, CFG_RATE, FREQ_LEN);

  configPacket[NAV_RATE_1] = (char)(normHerz & 0xFF);
  configPacket[NAV_RATE_2] = (char)((normHerz >> 8) & 0xFF);

  insertChecksum(configPacket, FREQ_LEN);
  sendPacket(configPacket, FREQ_LEN);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
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

void loop() 
{
  long start = millis();
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
       }
    }
  }  
  calc_speed = alpha*(calc_speed + accel*dt) + (1-alpha)*(gps_speed);
  speed_kph = calc_speed*3.6;


  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    
    SerialPort.print("Filter Speed ");
    SerialPort.print(calc_speed);
    SerialPort.print(" m/s  ");
    SerialPort.print(speed_kph);
    SerialPort.print(" km/h   ");
  
    SerialPort.print("     GPS Speed ");
    SerialPort.print(gps_speed);
    SerialPort.print(" m/s   Loop time ");
    SerialPort.print(millis() - start);
    SerialPort.print(" ms"     );

    SerialPort.println("      R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  }
}
