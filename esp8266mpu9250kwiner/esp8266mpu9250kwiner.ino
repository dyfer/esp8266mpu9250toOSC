//TODO: calibration serial command to calibration - acc/gyro seprataely and mag separately
//write calibration data to EEPROM
//update check for NAN from readaccelbias to other read funcs

/*
Marcin Pączkowski

Based on Accelerometer WiFi sensors for various projects
started with Deep Decline (October 2015)

this is modified to work with MPU9250
code ported from Kris Winer repository

//-----------------
Hardware:
//----------------

ESP8266-based Adafruit Huzzzah board
MPU-9250  (noname)

//---------------
connections
//------------

ESP8266 | MPU-9250
V+     Vcc //is be connected directly to the battery, since the board has voltage regulator
GND      Gnd
GPIO 2   SDA
GPIO 4   SCL
GPIO 15   INT //NOTE: it MAY NOT be 16!


voltage divider for battery
100kOhm / 1MOhm


let's do flexible cofiguration as to what's being sent; also configure which pins are used for output
[counter, acc, gyro, accgyro, b1, b2, b3, b4, quaternion, euler, yawpitchroll, realaccel, worldaccel, o1, o2, o3, o4]

//------------
changelog/notes
//------------


//---------------
Setup:
//---------------

Serial communication OR:
Hold GPIO 0 button within 1s of powering on the board (but NOT before)
to enter access point mode

Connect to newly created wifi (ssid: _config_(partOfMacAddress))
Navigate to 192.168.4.1 or config.acc

set ssid, password and name

restart device

on the computer use
arp -i en0 -a (or enX) to find the device





*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Ticker.h>
#include "Wire.h"

//from kWiner
// Must immediately declare functions to avoid "Not declared in this scope" errors
void     myinthandler();
void     getMres();
void     getGres();
void     getAres();
void     I2Cscan();
void     readMPU9250Data(int16_t * destination);
void     readAccelData(int16_t * destination);
void     readGyroData(int16_t * destination);
void     readMagData(int16_t * destination);
int16_t  readTempData();
void     initAK8963(float * destination);
void     initMPU9250();
void     accelgyrocalMPU9250(float * dest1, float * dest2);
void     magcalMPU9250(float * dest1, float * dest2);
void     MPU9250SelfTest(float * destination);
void     MS5637Reset();
void     MS5637PromRead(uint16_t * destination);
uint32_t MS5637Read(uint8_t CMD, uint8_t OSR);
unsigned char MS5637checkCRC(uint16_t * n_prom);
void     writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t  readByte(uint8_t address, uint8_t subAddress);
void     readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void     MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void     MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	   0x03  // data
#define AK8963_XOUT_H	   0x04
#define AK8963_YOUT_L	   0x05
#define AK8963_YOUT_H	   0x06
#define AK8963_ZOUT_L	   0x07
#define AK8963_ZOUT_H	   0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define MS5637_ADDRESS 0x76   // Address of altimeter
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define MS5637_ADDRESS 0x76   // Address of altimeter
#endif

#define SerialDebug false  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 15;
bool newData = false;
bool newMagData = false;

int myLed = 13;

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias; read directly from AK8963 at init
float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
uint32_t filtNow = 0, filtLastSecond = 0, filtCount = 0;
float filtRate = 0;                       // used to calculate filter rate
uint32_t readNow = 0, readLastSecond = 0, readCount = 0;
float readRate = 0;                       // used to calculate sensor read rate
uint32_t sendNow = 0, sendLast = 0, sendLastSecond = 0, sendCount = 0;
float sendRate = 0;                       // used to calculate sensor send rate
unsigned long sendRateInverse = 4000; //time between sends in us; slow down traffic.... for now 4ms -> ~200Hz; this is not using a delay, just check what was the last time things were sent
uint32_t statusSendLast = 0; 							// for updating status
unsigned long statusSendRateInverse = 1000000; //time between status sends in us; set to 1000000 = 1s

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer
// int mpuInterruptPin = 15; //pin where INT is connected; it MAY NOT be 16!; changed on Feb 20, 2017
// bool useDMP = true;

// orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

String ssid = ""; //this is stored in the EEPROM anyway
String password = "";
String devName = "";

String ssidAp = "_config_";
//const char* passwordAp = "";
String devNameAP = "config";

// Create an instance of the webServer
// specify the port to listen on as an argument
ESP8266WebServer webServer ( 80 );
/* MDNSResponder mdns; */ //seems this is not needed
DNSServer dnsServer; //for AP mode
IPAddress myIP;
WiFiUDP Udp; //for sending
WiFiUDP UdpReceive; //for receiving
//2 separate becaouse of an issue/feature? https://github.com/esp8266/Arduino/issues/53
WiFiUDP UdpScsynth; //for sending directly to scsynth

OSCErrorCode error;

//int ledPin = 0;
int onboardButtonLedPin = 0; //LED uses button pin

boolean apMode = false;
boolean serialFallback = false; //this is for sending accelerometer data through serial instead of through OSC; should be switched using serial command
boolean serialStatus = false;
boolean isCalibratingAccGyro = false;
boolean isCalibratingMag = false;

uint8_t mac[6];
String macStr;

int outputPins[] = {13, 12, 14, 5}; //activate output on pins used for input; will be set from OSC message /o1 /o2 /o3 /o4

// int offsets[6]; //Ax, Ay, Az, Gx, Gy, Gz

IPAddress destIP;
long destPort;

int receivePort = 8000;
int sendPort = receivePort + 100; //different port for sending... workaround; works with dark (newer?) boards

//------------- SET VERSION NUMBER ------------
String version = "20forIMU9250";
//------------------------------------

boolean useMDNS = false; //for client mode, testing with off for performance reasons
const byte DNS_PORT = 53;

// EEPROM memory scheme:
// | ssid | password | name | destination IP | destination port | msgFormat | numUsedMessages | accBias | gyroBias | magBias | magScale |

//int eepromStringLengths = 64;
int ssidSize = 64;
int pwdSize = 64;
int nameSize = 64;
int destIpSize = 4; //4 bytes, one for each segment of the ip address
int destPortSize = 4; //must be 4 bytes to store long
int msgFormatEepromSize = 32; //needs to be the same as msgFormat size!
int numUsedMessagesSize = 4; //long
int accBiasSize = 12; //12xint (int on ESP is 4 bytes)
int gyroBiasSize = 12; //12xint (int on ESP is 4 bytes)
int magBiasSize = 12; //12xint (int on ESP is 4 bytes)
int magScaleSize = 12; //12xint (int on ESP is 4 bytes)

//ADC_MODE(ADC_VCC); //ability to measure input voltage - not useful, as the breakout board has a voltage regulator; using voltage divider to analog in now
//voltage measurement
float r1 = 100; //lower resistor value (kOhm)
float r2 = 1000; //higher resistor value (kOhm)
float v0 = 3.4; //voltage for 0%
float v100 = 4.2; //voltage for 100%

//all buttons
int allButtonPins[] = {13, 12, 14, 5}; //note 16 does not have internal pullup! we need to solder jumper cable to #5


//------------- IMPORTANT ------------
// float valueScaler = 2048; //for accelerometer data; 2048 for the board with green light, 1024 for the board with red light...
//------------------------------------

//for updates
boolean sendUpdateBool = false;
boolean wifiConnected = false; //additional bool to keep track when loosing connection

Ticker updateTrigger;

int msgCounter = 0;

String statusPath = "/status";
String sensorPath = "/acc/"; //added trailing slash, since we're concatenating this with device name

//serial stuff
String inputString = "";         // a string to hold incoming data
/* boolean stringComplete = false;  // whether the string is complete */
char* serialCommands[] = {
	"fallback", //0
	"status",
	"name", //2
	"ip", //3
	"port",
	"myIP", //5
	"myPort",
	"ssid", //7
	"pwd",
	"?", //9
	"??",
	"format", //11
	"formatInfo",
	"version", //13
	"calibrateAccGyro",
  "calibrateMag" //15
	/* "output" */
};
char* serialCommandsHints[] = { //careful to have these at proper indices in relation to serialCommands
	"on/off sending sensor data over serial; value: 0/1",
	"on/off sending status messages data over serial, also get status info; value: 0/1",
	"get/set name; value: name (string)",
	"get/set desintation IP; value: IP string x:x:x:x",
	"get/set desintation PORT; value: port number",
	"get this devices' IP address",
	"get this devices' PORT for listening to OSC messages",
	"get/set network name (ssid); value: network name string",
	"get/set network password; value: network password string",
	"print available commands (short)",
	"print available commands with descriptions",
	"get/set message format; space-delimited list; for more info send formatInfo",
	"list available format names",
	"print version string",
	"start calibration process for Accelerometer and Gyro; the accelerometer should be stable and perfectly flat",
  "start calibration process for Magnetometer; it should be rotated along all axis"
};

byte msgFormat[32];
int numUsedMessages = 0;
int lowestIndexToRequireDMP = 8;

char* possibleMessages[] = {
	"counter", //0
	"acc", //1
	"gyro", //2
	"accgyro", //3
	"b1", //4
	"b2", //5
	"b3",
	"b4", //7
	"quaternion",
	"euler", //9
	"yawpitchroll",
	"realaccel", //11
	"worldaccel",
};

//for calibration
const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower); defaults: fast 1000, slow 10000
// const int NFast =  100;    // fast for test
const int NSlow = 10000;    // used in "closing in" section of the calibration
// const int NSlow = 1000;    // fast for test
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

void readAllSensors() {
	// If intPin goes high, all data registers have new data
	if (newData == true) { // On interrupt, read data
		newData = false;  // reset newData flag
		readMPU9250Data(MPU9250Data); // INT cleared on any read
		//   readAccelData(accelCount);  // Read the x/y/z adc values

		// Now we'll calculate the accleration value into actual g's
		ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float)MPU9250Data[1] * aRes - accelBias[1];
		az = (float)MPU9250Data[2] * aRes - accelBias[2];

		//   readGyroData(gyroCount);  // Read the x/y/z adc values

		// Calculate the gyro value into actual degrees per second
		gx = (float)MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
		gy = (float)MPU9250Data[5] * gRes;
		gz = (float)MPU9250Data[6] * gRes;

		readMagData(magCount);  // Read the x/y/z adc values

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		if (newMagData == true) {
			newMagData = false; // reset newMagData flag
			mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
			mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
			mx *= magScale[0];
			my *= magScale[1];
			mz *= magScale[2];
		}

		readCount ++;
		if((millis() - readLastSecond) > 1000) { //update once per second
			readRate = readCount;
			readLastSecond = millis();
			readCount = 0;
		}
	}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	// sumCount++;

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
	// we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
	// positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
	// function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
	// This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
	// Pass gyro rate as rad/s
	MadgwickQuaternionUpdate(-ax, ay, az, gx * PI / 180.0f, -gy * PI / 180.0f, -gz * PI / 180.0f,  my,  -mx, mz);
	//MadgwickQuaternionUpdate(-ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  -mx, mz); //from another code... this seem to cause resetting yaw?
	//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);

	// Serial print and/or display at 0.5 s rate independent of data rates
	// delt_t = millis() - count;

	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth.
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	//Software AHRS:

	//        yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	//        pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
	//        roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	//        pitch *= 180.0f / PI;
	//        yaw   *= 180.0f / PI;
	// //       yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	//        if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	//        roll  *= 180.0f / PI;

	a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	pitch = -asinf(a32);
	roll  = atan2f(a31, a33);
	yaw   = atan2f(a12, a22);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	//    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	roll  *= 180.0f / PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;

	filtCount ++;
	if((millis() - filtLastSecond) > 1000) { //update once per second
		filtRate = filtCount;
		filtLastSecond = millis();
		filtCount = 0;
	}

	//  count = millis();
	//  sumCount = 0;
	//  sum = 0;
}

void myinthandler()
{
	newData = true;
}

void getMres() {
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
		mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
		break;
		case MFS_16BITS:
		mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void getGres() {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
		case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
		case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
		case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void getAres() {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
		case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
		case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
		case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}

void readMPU9250Data(int16_t * destination)
{
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	if (newMagData == true) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
		}
	}
}

void initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	delay(10);
}


void initMPU9250()
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x03; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear GFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear; THIS IS DIFFERENT THAN EXMAPLES for other microcontrollers! e.g. mpu9250basicAHRS uses the line above
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  Serial.println("Accelerometer/Gyro calibration: put device in a flat position!");
  Serial.println("In 4s...");
  delay(1000);
  Serial.println("In 3s...");
  delay(1000);
  Serial.println("In 2s...");
  delay(1000);
  Serial.println("In 1s...");
  delay(1000);
  Serial.println("Calibration started...");

	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
	}
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4)       & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4)       & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

  Serial.println("Calibration done.");

  Serial.println("accel biases (mg)"); Serial.println(1000.*dest1[0]); Serial.println(1000.*dest1[1]); Serial.println(1000.*dest1[2]);
  Serial.println("gyro biases (dps)"); Serial.println(dest2[0]); Serial.println(dest2[1]); Serial.println(dest2[2]);

  //write to EEPROM
  writeAccelBias(dest1);
  writeGyroBias(dest2);
}


void magcalMPU9250(float * dest1, float * dest2)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  Serial.println("In 4s...");
  delay(1000);
  Serial.println("In 3s...");
  delay(1000);
  Serial.println("In 2s...");
  delay(1000);
  Serial.println("In 1s...");
  delay(1000);
	Serial.println("Calibration started...");

	// shoot for ~fifteen seconds of mag data
	if (Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
	for (ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if (Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
		if (Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
	}

  Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
	dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad / ((float)mag_scale[0]);
	dest2[1] = avg_rad / ((float)mag_scale[1]);
	dest2[2] = avg_rad / ((float)mag_scale[2]);

	Serial.println("Mag Calibration done!");

  //write EEPROM
  writeMagBias(dest1);
  writeMagScale(dest2);
}



// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

	for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
		destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
	}

}

// I2C scan function

void I2Cscan()
{
	// scan for i2c devices
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
			Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address < 16)
			Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
	Serial.println("No I2C devices found\n");
	else
	Serial.println("done\n");

}


// I2C read/write functions for the BMP280 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);                  // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, 1);  // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}         // Put read results in the Rx buffer
}


/* serial things */
void readSerial() {
	char inChar = (char)Serial.read();
	if ((inChar == 10) || (inChar == 13)) { //filter out both new line and carriage return
		if(inputString.length() > 0) { //process only stirings that have some characters
			processSerialCommand();
			inputString = ""; //clear input string at the end
		}
	} else {
		inputString += inChar;
	}
	/* Serial.println(inputString); //echo */
}

void processSerialCommand() {
	String command, valueString = "";
	int spaceIndex = inputString.indexOf(' ');
	int commandIndex = -1; //default to no valid answer
	/* Serial.print("spaceIndex: ");
	/* Serial.println(spaceIndex); */
	command = inputString.substring(0, spaceIndex);
	/* Serial.print("command: "); */
	/* Serial.println(command); */
	/* Serial.print("sizeof(serialCommands): "); */
	/* Serial.println(sizeof(serialCommands)); */
	/* Serial.print("sizeof(serialCommands[0]): "); */
	/* Serial.println(sizeof(serialCommands[0])); */
	for (int i = 0; i < (sizeof(serialCommands) / sizeof(serialCommands[0])); i++) {
		if (command == serialCommands[i]) {
			commandIndex = i;
		}
	}
	/* Serial.print("commandIndex: "); */
	/* Serial.println(commandIndex); */
	if (spaceIndex > 0) {
		valueString = inputString.substring(spaceIndex + 1);
	}
	/* Serial.print("valueString.length(): "); */
	/* Serial.println(valueString.length()); */
	/* Serial.println(valueString); */

	switch (commandIndex) {
		case 0:
		if (valueString.length() > 0) {
			serialFallback = valueString.toInt();
		}
		Serial.print("fallback: ");
		Serial.println(serialFallback);
		break;
		case 1:
		if (valueString.length() > 0) {
			serialStatus = valueString.toInt();
		}
		Serial.print("status: ");
		Serial.println(serialStatus);
		printStatus();
		break;
		case 2: //name
		if (valueString.length() > 0) {
			Serial.println("Setting name");
			writeName(valueString);
		}
		Serial.print("Device name: ");
		Serial.println(devName);
		break;
		case 3: //ip
		if (valueString.length() > 0) {
			int ipInts[4];
			int indexOfPeriod; //so we start looking from 0 below?
			/* String remainingString; */
			Serial.println(valueString);
			Serial.print("Received new desination IP: ");
			for (int i = 0; i < 4; i++) {
				indexOfPeriod = valueString.indexOf('.');
				if (indexOfPeriod) {
					ipInts[i] = valueString.substring(0, indexOfPeriod).toInt();
					valueString = valueString.substring(indexOfPeriod + 1); //should it be indexOfPeriod + 1?
					Serial.print(ipInts[i]);
					Serial.print(".");
				} else {
					ipInts[i] = valueString.toInt();//assuming last number
					Serial.println(ipInts[i]);
				}
			};
			Serial.println("Setting IP");
			writeIP(ipInts[0], ipInts[1], ipInts[2], ipInts[3]);
		} else {
			Serial.print("Destination IP: ");
			for (int i = 0; i < 4; i++) {
				Serial.print(destIP[i]);
				if (i == 3) {
					Serial.println("");
				} else {
					Serial.print(".");
				}
			}
		}
		break;
		case 4: //port
		if (valueString.length() > 0) {
			Serial.println("Setting destination port");
			writePort(valueString.toInt());
		}
		Serial.print("Destination port: ");
		Serial.println(destPort);
		break;
		case 5: //myIP
		Serial.print("My IP: ");
		Serial.println(myIP);
		break;
		case 6: //myPort
		Serial.print("My receive port: ");
		Serial.println(receivePort);
		break;
		case 7: //ssid
		if (valueString.length() > 0) {
			Serial.println("Setting SSID");
			writeSSID(valueString);
		}
		Serial.print("SSID: ");
		Serial.println(ssid);
		break;
		case 8: //pwd
		if (valueString.length() > 0) {
			Serial.println("Setting password");
			writePwd(valueString);
		}
		Serial.print("Password: ");
		Serial.println(password);
		break;
		case 9: //?
		printHelp(0);
		break;
		case 10: //??
		printHelp(1);
		break;
		case 11: //format
		if (valueString.length() > 0) {
			setMessageFormat(valueString);
		}
		sendCurrentMsgFormat();
		break;
		case 12: //formatInfo
		sendMsgFormatInfo();
		break;
		case 13: //version
		Serial.print("Version: ");
		Serial.println(version);
		break;
		case 14: //calibrate
		Serial.println("Starting calibration for acc/gyro!");
		isCalibratingAccGyro = true;
		break;
		case 15: //calibrate
		Serial.println("Starting calibration for magnetometer!");
		isCalibratingMag = true;
		break;
		default:
		Serial.print("Received unknown command: ");
		Serial.println(command);
		printHelp(0); //print hints
		break;
	}
}

void printHelp(int hints) {
	Serial.println("List of possible commands:");
	/* Serial.print("sizeof(serialCommands): "); */
	/* Serial.println(sizeof(serialCommands)); */
	for (int i = 0; i < (sizeof(serialCommands) / sizeof(serialCommands[0])); i++) {
		Serial.print(serialCommands[i]);
		if (hints) {
			Serial.print(": ");
			Serial.println(serialCommandsHints[i]);
		} else {
			Serial.print(" ");
		}
	}
	Serial.println("");
}


void setSerialStatus(boolean val) {
	serialFallback = val;
	Serial.print("status: ");
	Serial.println(serialStatus);
}

void getSerialStatus() {
	Serial.print("status: ");
	Serial.println(serialStatus);
}


/* wifi setup */

void startAP() {
	apMode = true;
	//    devName = devNameAP;
	Serial.println("In start AP");
	ssidAp = ssidAp + String(mac[4], HEX) + "_" + String(mac[5], HEX);

	Serial.println("Starting in access point mode.");
	Serial.println("-----------------");
	Serial.print("Network name: ");
	Serial.println(ssidAp);
	Serial.print("Device name: ");
	Serial.println(devName);
	Serial.println("Configuring access point...");

	/* You can remove the password parameter if you want the AP to be open. */
	//  WiFi.softAP(ssid, password);
	WiFi.softAP(ssidAp.c_str());
	WiFi.mode(WIFI_AP);
	myIP = WiFi.softAPIP();

	Serial.print("My address: ");
	Serial.println(myIP);
	Serial.println("-----------------");
	Serial.print("Connect to WiFi network: ");
	Serial.println(ssidAp);
	Serial.print("Access configuration page at http://");
	Serial.println(myIP);
	Serial.print("or at http://");
	Serial.print(devNameAP);
	/* Serial.println(".local"); */
	Serial.println(".acc"); //this works with full DNS server... but might not work if other dns services run on the network
	Serial.println("-----------------");


	//start webServer
	webServer.on ( "/", handleRoot );
	webServer.on ( "/network", handleNetwork );
	webServer.on ( "/name", handleName );
	webServer.on ( "/ip", handleIP );
	webServer.on ( "/port", handlePort );
	webServer.on ( "/reboot", handleReboot );
	webServer.begin();
	Serial.println("HTTP server started");


	//start mDNS - seems not to work? use DNS below

	// if (useMDNS) {
	//  if ( MDNS.begin ( devNameAP.c_str(), myIP ) ) {
	//  Serial.println ( "MDNS responder started" );
	//}
	// MDNS.addService("http", "tcp", 80);
	//}

	//use DNS instead
	// modify TTL associated  with the domain name (in seconds)
	// default is 60 seconds
	dnsServer.setTTL(300);
	// set which return code will be used for all other domains (e.g. sending
	// ServerFailure instead of NonExistentDomain will reduce number of queries
	// sent by clients)
	// default is DNSReplyCode::NonExistentDomain
	dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);

	// start DNS server for a specific domain name
	dnsServer.start(DNS_PORT, devNameAP + ".acc", myIP);
	/* dnsServer.start(DNS_PORT, "www.example.com", myIP); */
}

void connectWiFi() {
	// Connect to WiFi network
	/* Serial.println("Starting in client mode."); */
	/* Serial.println("-----------------"); */
	if (wifiConnected) {
		Serial.println("WiFi connection lost.");
		Udp.stop();
		UdpReceive.stop();
		UdpScsynth.stop();
		WiFi.disconnect();
		wifiConnected = false;
	} else {
		WiFi.mode(WIFI_STA);
	}

	Serial.print("Connecting to ");
	Serial.println(ssid);

	WiFi.begin(ssid.c_str(), password.c_str());
	while ((WiFi.status() != WL_CONNECTED) && (!serialFallback)) {
		if (Serial.available() > 0) {
			//read all bytes?
			for (int i = 0; i < Serial.available(); i++) {
				readSerial();
			}
		}; //read serial here, too
		delay(500);
		Serial.print(".");
		//temp
		/* readVoltage(); */
		/* Serial.print("Buttons: "); */
		/* for (int i = 0; i < (sizeof(allButtonPins) / sizeof(allButtonPins[0])); i++ ) { */
		/* Serial.print(digitalRead(allButtonPins[i])); */
		/* Serial.print(" "); */
		/* } */
		/* Serial.println(""); */
		if (millis() < 2000) {
			if (digitalRead(onboardButtonLedPin) == 0) {
				Serial.println("");
				Serial.println("Connecting interrupted");
				detachInterrupt(intPin);//to avoid resets... not sure why, maybe has something to do with delay and interrupt...
				WiFi.disconnect();
				Serial.println("after wifi disconnect");
				delay(200);
				Serial.println("Before start AP");
				startAP();
				return;
			}
		}
	}
	if (WiFi.status() == WL_CONNECTED) {
		wifiConnected = true;

		myIP = WiFi.localIP();

		Udp.begin(sendPort);
		UdpReceive.begin(receivePort);

		Serial.println("");
		Serial.println("WiFi connected");
		// Print the IP address
		Serial.print("My address: ");
		Serial.print(myIP);
		Serial.print(":");
		Serial.println(receivePort);

		Serial.print("Sending OSC messages to ");
		Serial.print(destIP);
		Serial.print(":");
		Serial.println(destPort);

		//start mDNS
		if (useMDNS) {
			if ( MDNS.begin ( devName.c_str(), myIP ) ) {
				Serial.println ( "MDNS responder started" );
			}
			MDNS.addService("osc", "udp", receivePort);
		}
	}
	// attachInterrupt(mpuInterruptPin, dmpDataReady, RISING); // moved here because of restarts...
}


float readVoltage() {
	float rawVal, voltage;
	/* int percentage; */
	//  voltage = ESP.getVcc() / 1024.0;
	rawVal = analogRead(A0);
	voltage = rawVal / 1024.0 * (r1 + r2) / r1;
	return voltage;
}


void readVoltageAndPercentage(float &voltage, int &percentage) {
	voltage = readVoltage();
	percentage = int(mapF(voltage, v0, v100, 0.0, 100.0));//convert to percentage
	/* Serial.print("Input voltage: "); */
	/* Serial.print(voltage); */
	/* Serial.print(" ("); */
	/* Serial.print(percentage); */
	/* Serial.println("%)"); */
}

/* reading data to an OSCMessage */
void readAcc(OSCMessage &msg) {
	// int16_t ax, ay, az;
	// mpu.getAcceleration(&ax, &ay, &az);
	// msg.add(ax / valueScaler);
	// msg.add(ay / valueScaler);
	// msg.add(az / valueScaler);
	msg.add(ax);
	msg.add(ay);
	msg.add(az);
}

void readGyro(OSCMessage &msg) {
	// int16_t gx, gy, gz;
	// mpu.getRotation(&gx, &gy, &gz);
	// msg.add(gx / valueScaler);
	// msg.add(gy / valueScaler);
	// msg.add(gz / valueScaler);
	msg.add(gx);
	msg.add(gy);
	msg.add(gz);
}

void readAccGyro(OSCMessage &msg) {
	// int16_t ax, ay, az;
	// int16_t gx, gy, gz;
	// mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	// msg.add(ax / valueScaler);
	// msg.add(ay / valueScaler);
	// msg.add(az / valueScaler);
	// msg.add(gx / valueScaler);
	// msg.add(gy / valueScaler);
	// msg.add(gz / valueScaler);
	msg.add(ax);
	msg.add(ay);
	msg.add(az);
	msg.add(gx);
	msg.add(gy);
	msg.add(gz);
}

void readQuaternion(OSCMessage &msg) {
	/* if(readDmpFifo()) { */
	/* mpu.dmpGetQuaternion(&q, fifoBuffer); */ //moved to reaffifo function
	// msg.add(q.w);
	// msg.add(q.x);
	// msg.add(q.y);
	// msg.add(q.z);
	msg.add(q[0]);
	msg.add(q[1]);
	msg.add(q[2]);
	msg.add(q[3]);
	/* } */
}

void readEuler(OSCMessage &msg) {
	/* if(readDmpFifo()) { */
	/* mpu.dmpGetQuaternion(&q, fifoBuffer); */
	/* mpu.dmpGetEuler(euler, &q); */
	// msg.add(euler[0]);
	// msg.add(euler[1]);
	// msg.add(euler[2]);
	// msg.add(yaw);
	// msg.add(pitch);
	// msg.add(roll);
	/* } */
}

/* void readQuaternionEuler(OSCMessage &msg){ */
/*     if(readDmpFifo()) { */
/* 	mpu.dmpGetQuaternion(&q, fifoBuffer); */
/* 	mpu.dmpGetEuler(euler, &q); */
/* 	msg.add(q.w); */
/* 	msg.add(q.x); */
/* 	msg.add(q.y); */
/* 	msg.add(q.z); */
/* 	msg.add(euler[0]); */
/* 	msg.add(euler[1]); */
/* 	msg.add(euler[2]); */
/*     } */
/* } */

void readYawPitchRoll(OSCMessage &msg) {
	/* if(readDmpFifo()) { */
	/* mpu.dmpGetQuaternion(&q, fifoBuffer); */
	/* mpu.dmpGetGravity(&gravity, &q); */
	/* mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); */
	// msg.add(ypr[0]);
	// msg.add(ypr[1]);
	// msg.add(ypr[2]);
	msg.add(yaw);
	msg.add(pitch);
	msg.add(roll);
	/* } */
}

void readRealAccel(OSCMessage &msg) {
	/* if(readDmpFifo()) { */
	/* mpu.dmpGetQuaternion(&q, fifoBuffer); */
	/* mpu.dmpGetAccel(&aa, fifoBuffer); */
	/* mpu.dmpGetGravity(&gravity, &q); */
	/* mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); */
	// msg.add(aaReal.x);
	// msg.add(aaReal.y);
	// msg.add(aaReal.y);
	msg.add(lin_ax);
	msg.add(lin_ay);
	msg.add(lin_az);
	/* } */
}

void readWorldAccel(OSCMessage &msg) {
	/* if(readDmpFifo()) { */
	/* mpu.dmpGetQuaternion(&q, fifoBuffer); */
	/* mpu.dmpGetAccel(&aa, fifoBuffer); */
	/* mpu.dmpGetGravity(&gravity, &q); */
	/* mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); */
	// msg.add(aaReal.x);
	// msg.add(aaReal.y);
	// msg.add(aaReal.y);
	/* } */
}

void readCounter(OSCMessage &msg) {
	msg.add(msgCounter);
}

void readButton(OSCMessage &msg, int whichButton) { //button number is 0-based!
	int buttonVal;
	buttonVal = digitalRead(allButtonPins[whichButton]);
	buttonVal = buttonVal * -1 + 1; //flip the value
	msg.add(buttonVal);
}

/* more mpu reading here */
// void sendStatusUpdate() {
//   sendUpdateBool = true;
// }

void sendSensors() {
	/* mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); */
	//mpu.getAcceleration(&ax, &ay, &az);
	OSCMessage msg(sensorPath.c_str());
	//scaled:
	/* msgAcc.add(ax / valueScaler); */
	/* msgAcc.add(ay / valueScaler); */
	/* msgAcc.add(az / valueScaler); */
	//put all sensor readings here
	// if (useDMP) {
	//   readDmpFifo();
	// }
	//read sensors according to the array
	for (int i = 0; i < numUsedMessages; i++) {
		readSensorsByNumber(msgFormat[i], msg);
	}
	/* readAcc(msg); */
	/* readGyro(msg); */
	/* readAccGyro(msg); */
	/* readQuaternion(msg); */
	/* readButton(msg, 0); */
	/* readButton(msg, 1); */

	if (serialFallback) {
		Serial.print(sensorPath);
		for (int i = 0; i < msg.size(); i++) {
			Serial.print(" ");
			if (msg.isInt(i)) {
				Serial.print(msg.getInt(i));
			} else {
				Serial.print(msg.getFloat(i)); //assuming float
			} //possibly more types here
		}
		Serial.println("");
	} else {
		Udp.beginPacket(destIP, destPort);
		msg.send(Udp);
		Udp.endPacket();
	};
	msg.empty();
	msgCounter++;
}

void readSensorsByNumber(int msgNumer, OSCMessage &msg) { //these need to correspond with possibleMessages[]
	switch (msgNumer) {
		case 0:
		readCounter(msg);
		break;
		case 1:
		readAcc(msg);
		break;
		case 2:
		readGyro(msg);
		break;
		case 3:
		readAccGyro(msg);
		break;
		case 4:
		readButton(msg, 0);
		break;
		case 5:
		readButton(msg, 1);
		break;
		case 6:
		readButton(msg, 2);
		break;
		case 7:
		readButton(msg, 3);
		break;
		case 8:
		readQuaternion(msg);
		break;
		case 9:
		readEuler(msg);
		break;
		case 10:
		readYawPitchRoll(msg);
		break;
		case 11:
		readRealAccel(msg);
		break;
		case 12:
		readWorldAccel(msg);
		break;
		default:
		break;
	}
}

void setMessageFormat(String messageString) {
	int thisMsgCounter = 0;
	// useDMP = false; //update below
	Serial.println("Setting message format");
	while (messageString.length() > 0) {
		bool thisMessageNameIsValid = false;
		int spaceIndex = messageString.indexOf(' ');
		String thisString;
		if (spaceIndex > 0) {
			thisString = messageString.substring(0, spaceIndex);
		} else {
			thisString = messageString;
		};
		/* Serial.println(thisString); */
		for (int i = 0; i < (sizeof(possibleMessages) / sizeof(possibleMessages[0])); i++) {
			String msgString(possibleMessages[i]); //convert to String
			if (msgString == thisString) {
				/* Serial.println("yes"); */
				msgFormat[thisMsgCounter] = i;
				thisMessageNameIsValid = true;
				// if (i >= lowestIndexToRequireDMP) {
				// 	useDMP = true;
				// }
			}
		}
		if (spaceIndex > 0) {
			messageString = messageString.substring(spaceIndex + 1);
		} else {
			messageString = "";
		};
		if (!thisMessageNameIsValid) {
			Serial.print("Message name not found: ");
			Serial.println(thisString);
		} else {
			thisMsgCounter++;
		}
	}
	numUsedMessages = thisMsgCounter;
	writeMsgFormat(); //store to eeprom
}

// void checkUseDMP() { //call this after reading settings from memory
// 	useDMP = false; //update below
// 	/* Serial.println("Setting message format"); */
// 	for (int i = 0; i < numUsedMessages; i++) {
// 		if (msgFormat[i] >= lowestIndexToRequireDMP) {
// 			useDMP = true;
// 		}
// 	}
// }


void sendCurrentMsgFormat() {
	Serial.println("Current message format:");
	/* Serial.print("sizeof(serialCommands): "); */
	/* Serial.println(sizeof(serialCommands)); */
	for (int i = 0; i < numUsedMessages; i++) {
		Serial.print(possibleMessages[msgFormat[i]]);
		Serial.print(" ");
	}
	Serial.println("");
}

void sendMsgFormatInfo() {
	Serial.println("List of possible formats (send as space-delimited list, e.g. 'format acc b1 counter':");
	/* Serial.print("sizeof(serialCommands): "); */
	/* Serial.println(sizeof(serialCommands)); */
	for (int i = 0; i < (sizeof(possibleMessages) / sizeof(possibleMessages[0])); i++) {
		Serial.print(possibleMessages[i]);
		Serial.print(" ");
	}
	Serial.println("");
}

void updateStatus() {
	int percentage;
	float voltage;
	float temperature; //for later?
	readVoltageAndPercentage(voltage, percentage);
	//for now: voltage, percentage, msgCounter since last update
	OSCMessage msg(statusPath.c_str());
	msg.add(voltage);
	msg.add(percentage);
	// msg.add(msgCounter);
	msg.add(sendRate);
	msg.add(readRate);
	msg.add(filtRate);

	if (!serialFallback) {
		Udp.beginPacket(destIP, destPort);
		msg.send(Udp);
		Udp.endPacket();
	}
	msg.empty();

	/* Serial.print("Input voltage: "); */
	/* Serial.print(voltage); */
	/* Serial.print(" ("); */
	/* Serial.print(percentage); */
	/* Serial.println("%)"); */
	if (serialStatus) {
		printStatusNoRead(voltage, percentage);
    // printStatusNoRead();
	}

	msgCounter = 0; //resent msgCounter
}

void printStatus() {
	int percentage;
	float voltage;
	readVoltageAndPercentage(voltage, percentage);
	Serial.print(statusPath + " ");
	Serial.print(voltage);
	Serial.print("V (");
	Serial.print(percentage);
	Serial.print("%), ");
  // Serial.print(msgCounter);
	Serial.print(sendRate);
	// Serial.println(" messages");
	Serial.print(" messages; sensor read rate: ");
	Serial.print(readRate);
	Serial.print(", filter update rate: ");
	Serial.println(filtRate);
}

void printStatusNoRead(float voltage, int percentage) {
// void printStatusNoRead() {
	Serial.print(statusPath + " ");
	Serial.print(voltage);
	Serial.print("V (");
	Serial.print(percentage);
	Serial.print("%), ");
	// Serial.print(msgCounter);
	Serial.print(sendRate);
	// Serial.println(" messages");
	Serial.print(" messages; sensor read rate: ");
	Serial.print(readRate);
	Serial.print(", filter update rate: ");
	Serial.println(filtRate);
}

/* network handling */
void receiveOSC() {
	OSCMessage bundle; //message, not bundle
	int size = UdpReceive.parsePacket();
	//int size = Udp.parsePacket();
	//  Serial.println(Udp.read()); //not a good idea
	if (size > 0) {
		//    Serial.print(size);
		while (size--) {
			bundle.fill(UdpReceive.read());
			//      bundle.fill(Udp.read());
		}
		if (!bundle.hasError()) {
			Serial.println("received OSC");
			bundle.dispatch("/ip", setIP);
			bundle.dispatch("/port", setPort);
			bundle.dispatch("/name", sendName);
			bundle.dispatch("/vcc", sendVoltage);
		} else {
			error = bundle.getError();
			Serial.print("error: ");
			Serial.println(error);
		}
	}
}

void setIP(OSCMessage &msg) {
	Serial.print("Received ip message, size: ");
	Serial.println(msg.size());
	if (msg.size() > 0) {
		int addr0 = msg.getInt(0);
		int addr1 = msg.getInt(1);
		int addr2 = msg.getInt(2);
		int addr3 = msg.getInt(3);
		writeIP(addr0, addr1, addr2, addr3);
	} else {
		OSCMessage msgOut("/ip");
		msgOut.add(String(destIP).c_str());
		Udp.beginPacket(destIP, destPort);
		msgOut.send(Udp);
		Udp.endPacket();
		msgOut.empty();
	}

}

void setPort(OSCMessage &msg) {
	Serial.println("Received port message");
	if (msg.size() > 0) {
		long port = msg.getInt(0);
		writePort(port);
	} else {
		OSCMessage msgOut("/port");
		msgOut.add(String(destPort).c_str());
		Udp.beginPacket(destIP, destPort);
		msgOut.send(Udp);
		Udp.endPacket();
		msgOut.empty();
	}

}

void sendName(OSCMessage &msgArg) {
	OSCMessage msg("/name");
	msg.add(devName.c_str());
	Udp.beginPacket(destIP, destPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();
}


void sendVoltage(OSCMessage &msgArg) {
	OSCMessage msg("/vcc");
	msg.add(String(readVoltage()).c_str());
	Udp.beginPacket(destIP, destPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();
}




void handleRoot() {
	//  digitalWrite ( led, 1 );
	char temp[2000];

	snprintf ( temp, 2000,
		"<!DOCTYPE HTML PUBLIC '-//IETF//DTD HTML//EN'>\
		<html>\
		<head>\
		<title>%s - WiFi configuration</title>\
		</head>\
		<body>\
		<h1>WiFi accelerometer configuration</h1>\
		Device name: <b>%s</b><br>\
		Device MAC: <b>%s</b><br><br>\
		Set network configuration:<br>\
		<form>\
		SSID: <input type='text' name='ssid' value='%s'><br>\
		Password: <input type='text' name='pwd' value='%s'><br>\
		<input type='submit' value='Change network settings' formaction='network'>\
		</form>\
		Change device name (use no spaces):<br>\
		<form>\
		Device name: <input type='text' name='name' value='%s'>\
		<input type='submit' value='Change name'formaction='name'>\
		</form>\
		Set deistination IP (also available through OSC):<br>\
		<form>\
		<input type='text' name='ip0' value='%i'>.\
		<input type='text' name='ip1' value='%i'>.\
		<input type='text' name='ip2' value='%i'>.\
		<input type='text' name='ip3' value='%i'>\
		<input type='submit' value='Change IP'formaction='ip'>\
		</form>\
		Set destination port (also available through OSC):<br>\
		<form>\
		<input type='text' name='port' value='%i'>\
		<input type='submit' value='Change port'formaction='port'>\
		</form>\
		<br>\
		Input voltage: %sV<br>\
		<br>\
		<form>\
		<input type='submit' value='Reboot' formaction='reboot'>\
		</form>\
		</body>\
		</html>",
		devName.c_str(), devName.c_str(), macStr.c_str(), ssid.c_str(), password.c_str(), devName.c_str(),
		destIP[0], destIP[1], destIP[2], destIP[3], destPort, String(readVoltage()).c_str()
	);
	webServer.send ( 200, "text/html", temp );
}

void handleNetwork() {
	// get the value of request argument "state" and convert it to an int
	String newSSID;
	String newPassword;
	newSSID = webServer.arg("ssid");
	newSSID.replace("+", " "); //replace + with space... correct?
	newPassword = webServer.arg("pwd");
	newPassword.replace("+", " ");

	Serial.print("New SSID: ");
	Serial.println(newSSID);
	Serial.print("New password: ");
	Serial.println(newPassword);

	writeSSID(newSSID);
	writePwd(newPassword);
	/* EEPROM.commit(); */

	//  digitalWrite(led, state);
	webServer.send(200, "text/html", String("<p>SSID and password written to the internal memory successfully.</p>\
	<p>Reboot the device to connect or go back to change more settings.</p>"));
}

void handleName() {
	// get the value of request argument "state" and convert it to an int
	String newName;
	newName = webServer.arg("name");
	newName.replace("+", " "); //replace + with space... correct?

	Serial.print("New name: ");
	Serial.println(newName);

	writeName(newName);
	/* EEPROM.commit(); */

	//  digitalWrite(led, state);
	webServer.send(200, "text/html", String("<p>Name written to the internal memory successfully.</p>\
	<p>Reboot the device to connect or go back to change more settings.</p>"));
}

void handleIP() {
	writeIP(webServer.arg("ip0").toInt(), webServer.arg("ip1").toInt(), webServer.arg("ip2").toInt(), webServer.arg("ip3").toInt());
	webServer.send(200, "text/html", String("<p>Destination IP updated successfully.</p>\
	<p>Reboot the device to connect or go back to change more settings.</p>"));
}

void handlePort() {
	writePort(webServer.arg("port").toInt());
	webServer.send(200, "text/html", String("<p>Destination port updated successfully.</p>\
	<p>Reboot the device to connect or go back to change more settings.</p>"));
}


void handleReboot() {
	//  digitalWrite(led, state);
	webServer.send(200, "text/html", String("<p>Device will now reboot. Bye!</p>"));

	Serial.println("Rebooting!");
	ESP.restart();
}

// | ssid | password | name | destination IP | destination port |

String readSSID() {
	String result;
	result = readEEPROM(0, ssidSize);
	//  Serial.println("full ssid");
	//  Serial.println(result);
	result = result.substring(0, result.indexOf('\n')); //string, until newline character
	//  Serial.println("sub");
	//  Serial.println(result);
	return result;
}

String readPwd() {
	String result;
	result = readEEPROM(ssidSize, pwdSize);
	//  Serial.println("full pwd");
	//  Serial.println(result);
	result = result.substring(0, result.indexOf('\n')); //string, until newline character
	//  Serial.println("sub");
	//  Serial.println(result);
	return result;
}

String readName() {
	String result;
	result = readEEPROM(ssidSize + pwdSize, nameSize);
	result = result.substring(0, result.indexOf('\n')); //string, until newline character
	return result;
}

void readIP() {
	unsigned char result [4];
	for (int i = 0; i < destIpSize; ++i)
	{
		result[i] = EEPROM.read(i + ssidSize + pwdSize + nameSize);
	}
	//  result = readEEPROM(ssidSize + pwdSize + nameSize, destIpSize);
	destIP = IPAddress(result[0], result[1], result[2], result[3]);
	return;
}

long readPort() {
	long result;
	result = readEEPROMlong(ssidSize + pwdSize + nameSize + destIpSize);
	return result;
}

void readMsgFormat() {
	readEEPROMbyteArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize, msgFormatEepromSize, msgFormat);//msgFormat
	numUsedMessages = readEEPROMlong(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize);
	// checkUseDMP();
}

// void readOffsets() {
// 	// Serial.println("reading offsets");
// 	readEEPROMintArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize, sizeof(offsets) / sizeof(int), offsets);//offsets
// 	//and set
// 	Serial.print("Offset values read: ");
// 	for(int i = 0; i < (sizeof(offsets)/sizeof(int)); i++) {
// 		// Serial.printf("%d ", offsets[i]);
// 		Serial.print(offsets[i]);
// 		Serial.print(" ");
// 	};
// 	Serial.println("");
// 	// SetOffsets(offsets);
// }

void readAccelBias() {
	// Serial.println("reading offsets");
	readEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize, sizeof(accelBias) / sizeof(float), accelBias);//offsets
	if((accelBias[0] != accelBias[0]) && (accelBias[1] != accelBias[1]) && (accelBias[2] != accelBias[2])) {
		Serial.println("accelBias values seem to not have been previously written, initializing to {0,0,0}");
		for(int i = 0; i < (sizeof(accelBias)/sizeof(float)); i++) {
			accelBias[i] = 0;
		};
		writeAccelBias(accelBias);
	} else {
		//and set
		Serial.print("accelBias values read: ");
		for(int i = 0; i < (sizeof(accelBias)/sizeof(float)); i++) {
			// Serial.printf("%d ", soffsets[i]);
			Serial.print(accelBias[i]);
			Serial.print(" ");
		};
		Serial.println("");
	}
	// SetOffsets(offsets);
}

void readGyroBias() {
	// Serial.println("reading offsets");
	readEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize, sizeof(gyroBias) / sizeof(float), gyroBias);//offsets
	if((gyroBias[0] == NAN) && (gyroBias[0] == NAN) && (gyroBias[0] == NAN)) {
		Serial.println("gyroBias values seem to not have been previously written, initializing to {0,0,0}");
		for(int i = 0; i < (sizeof(gyroBias)/sizeof(float)); i++) {
			gyroBias[i] = 0;
		};
		writeAccelBias(gyroBias);
	} else {
		//and set
		Serial.print("gyroBias values read: ");
		for(int i = 0; i < (sizeof(gyroBias)/sizeof(float)); i++) {
			// Serial.printf("%d ", soffsets[i]);
			Serial.print(gyroBias[i]);
			Serial.print(" ");
		};
		Serial.println("");
	}
	// SetOffsets(offsets);
}

void readMagBias() {
	// Serial.println("reading offsets");
	readEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize + gyroBiasSize, sizeof(magBias) / sizeof(float), magBias);//offsets
	if((magBias[0] == NAN) && (magBias[0] == NAN) && (magBias[0] == NAN)) {
		Serial.println("magBias values seem to not have been previously written, initializing to {0,0,0}");
		for(int i = 0; i < (sizeof(magBias)/sizeof(float)); i++) {
			magBias[i] = 0;
		};
		writeAccelBias(magBias);
	} else {
		//and set
		Serial.print("magBias values read: ");
		for(int i = 0; i < (sizeof(magBias)/sizeof(float)); i++) {
			// Serial.printf("%d ", soffsets[i]);
			Serial.print(magBias[i]);
			Serial.print(" ");
		};
		Serial.println("");
	}
	// SetOffsets(offsets);
}

void readMagScale() {
	// Serial.println("reading offsets");
	readEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize + gyroBiasSize + magBiasSize, sizeof(magScale) / sizeof(float), magScale);//offsets
	if((magScale[0] == NAN) && (magScale[0] == NAN) && (magScale[0] == NAN)) {
		Serial.println("magScale values seem to not have been previously written, initializing to {0,0,0}");
		for(int i = 0; i < (sizeof(magScale)/sizeof(float)); i++) {
			magScale[i] = 0;
		};
		writeAccelBias(magScale);
	} else {
		//and set
		Serial.print("magScale values read: ");
		for(int i = 0; i < (sizeof(magScale)/sizeof(float)); i++) {
			// Serial.printf("%d ", soffsets[i]);
			Serial.print(magScale[i]);
			Serial.print(" ");
		};
		Serial.println("");
	}
	// SetOffsets(offsets);
}

void writeSSID(String thisSSID) {
	//  Serial.println(ssid);
	//  Serial.println(thisSSID);
	if (ssid != thisSSID) {
		writeEEPROM(0, ssidSize, thisSSID);
		ssid = thisSSID; //update var
		EEPROM.commit();
	} else {
		Serial.println("SSID the same, not changing");
	}
	return;
}

void writePwd(String thisPwd) {
	if (password != thisPwd) {
		writeEEPROM(ssidSize, pwdSize, thisPwd);
		password = thisPwd; //update var
		EEPROM.commit();
	} else {
		Serial.println("Password the same, not changing");
	}
	return;
}

void writeName(String thisName) {
	if (devName != thisName) {
		writeEEPROM(ssidSize + pwdSize, nameSize, thisName);
		devName = thisName; //update var
		EEPROM.commit();
	} else {
		Serial.println("Name the same, not changing");
	}
	return;
}

void writeIP(int val0, int val1, int val2, int val3) {
	int tempArr[] = {val0, val1, val2, val3};
	if (destIP[0] == val0 && destIP[1] == val1 && destIP[2] == val2 && destIP[3] == val3) {
		Serial.println("IP the same, not changing");
	} else {
		Serial.println("writing IP: ");
		for (int i = 0; i < destIpSize; ++i)
		{
			EEPROM.write(i + ssidSize + pwdSize + nameSize, tempArr[i]);
			Serial.print(tempArr[i]);
			Serial.print(".");
			//    destIP[i] = newIP[i]; //to avoid problems with assigning arrays???
		}
		Serial.println("");
		EEPROM.commit();
		destIP = IPAddress(val0, val1, val2, val3);
		//  destIP = newIP;
	}

	return;
}

void writePort(long newPort) {
	if (newPort != destPort) {
		writeEEPROMlong(ssidSize + pwdSize + nameSize + destIpSize, newPort);
		EEPROM.commit();
		destPort = newPort;
	} else {
		Serial.println("Port the same, not changing");
	}
	return;
}

void writeMsgFormat() {
	writeEEPROMbyteArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize, msgFormatEepromSize, msgFormat);//msgFormat
	writeEEPROMlong(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize, numUsedMessages);
	EEPROM.commit();
}

// void writeOffsets(int newOffsets[6]) {
// 	writeEEPROMintArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize, sizeof(offsets) / sizeof(int), newOffsets); //using global var for offsets to get proper size
// 	EEPROM.commit();
// }
void writeAccelBias(float newBias[3]) {
	writeEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize, sizeof(accelBias) / sizeof(int), newBias); //using global var for offsets to get proper size
	EEPROM.commit();
}

void writeGyroBias(float newBias[3]) {
	writeEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize, sizeof(gyroBias) / sizeof(int), newBias); //using global var for offsets to get proper size
	EEPROM.commit();
}

void writeMagBias(float newBias[3]) {
	writeEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize + gyroBiasSize, sizeof(magBias) / sizeof(int), newBias); //using global var for offsets to get proper size
	EEPROM.commit();
}

void writeMagScale(float newScale[3]) {
	writeEEPROMfloatArr(ssidSize + pwdSize + nameSize + destIpSize + destPortSize + msgFormatEepromSize + numUsedMessagesSize + accBiasSize + gyroBiasSize + magBiasSize, sizeof(magScale) / sizeof(int), newScale); //using global var for offsets to get proper size
	EEPROM.commit();
}

String readEEPROM(int start, int len) {
	Serial.println("Reading EEPROM");
	String result;
	for (int i = start; i < (start + len); ++i)
	{
		result += char(EEPROM.read(i));
	}
	return result;
}

void writeEEPROM(int start, int len, String str) {
	Serial.println("Writing EEPROM");
	for (int i = start; i < (start + len); ++i)
	{
		EEPROM.write(i, '\n'); //clear - set all to newline
	}
	for (int i = 0; i < str.length(); ++i)
	{
		EEPROM.write(i + start, str[i]);
	}
	return;
}

byte readEEPROMbyte(int start) { //as array to read to
	Serial.println("Reading EEPROM");
	byte result;
	result = EEPROM.read(start);
	return result;
	/* return true; */
}

void writeEEPROMbyte(int start, byte val) {
	EEPROM.write(start, val);
}

void readEEPROMbyteArr(int start, int len, byte arr[]) { //as array to read to
	// int len = sizeof(arr); //hope this works! NOPE, array passed as arg is just pointing to the first element
	Serial.println("Reading EEPROM");
	/* byte result; */
	for (int i = 0; i < len; ++i)
	{
		/* Serial.print("Reading from "); */
		/* Serial.print(i + start); */
		/* Serial.print(", value "); */
		arr[i] = EEPROM.read(start + i);
		/* Serial.println(arr[i]); */
	}
	/* return true; */
}

void writeEEPROMbyteArr(int start, int len, byte arr[]) {
	// int len = sizeof(arr);
	Serial.println("Writing EEPROM byte arr");
	for (int i = 0; i < len; ++i)
	{
		// Serial.print("Writing to ");
		// Serial.print(i + start);
		// Serial.print(", value ");
		// Serial.println(arr[i]);
		EEPROM.write(i + start, arr[i]);
	}
	/* return; */
}

void writeEEPROMintArr(int start, int len, int arr[]) {
	Serial.println("Writing EEPROM");
	for (int i = 0; i < len; ++i)
	{
		Serial.print("Writing to ");
		Serial.print(start + (i * sizeof(int)));
		Serial.print(", value ");
		Serial.println(arr[i]);
		writeEEPROMint(start + (i * sizeof(int)), arr[i]);
	}
	/* return; */
}

void writeEEPROMfloatArr(int start, int len, float arr[]) {
	Serial.println("Writing EEPROM");
	for (int i = 0; i < len; ++i)
	{
		Serial.print("Writing to ");
		Serial.print(start + (i * sizeof(float)));
		Serial.print(", value ");
		Serial.println(arr[i]);
		// writeEEPROMint(start + (i * sizeof(int)), arr[i]);
    // writeEEPROMfloat(start + (i * sizeof(float)), arr[i]);
    EEPROM.put(start + (i * sizeof(float)), arr[i]);
	}
  EEPROM.commit();
	/* return; */
}

void readEEPROMintArr(int start, int len, int arr[]) { //as array to read to

	// int len = sizeof(arr)/sizeof(int);
	Serial.println("Reading EEPROM int array");
	// Serial.println(len);
	// Serial.print("sizeof(arr): ");
	// Serial.println(sizeof(arr));
	// Serial.print("sizeof(int): ");
	// Serial.println(sizeof(int));
	for (int i = 0; i < len; ++i)
	{
		// Serial.print("Reading from ");
		// Serial.print(start + (i * sizeof(int)));
		// Serial.print(", value ");
		// arr[i] = readEEPROMint(start + (i * sizeof(int))); //<<<THIS DIDN'T WORK FOR SOME REASON
		arr[i] = (int)readEEPROMlong(start + (i * sizeof(int)));

		// Serial.println(arr[i]);
	}
	/* return true; */
}

void readEEPROMfloatArr(int start, int len, float arr[]) { //as array to read to

	// int len = sizeof(arr)/sizeof(int);
	Serial.println("Reading EEPROM float array");
	// Serial.println(len);
	// Serial.print("sizeof(arr): ");
	// Serial.println(sizeof(arr));
	// Serial.print("sizeof(int): ");
	// Serial.println(sizeof(int));
	for (int i = 0; i < len; ++i)
	{
		// Serial.print("Reading from ");
		// Serial.print(start + (i * sizeof(int)));
		// Serial.print(", value ");
		// arr[i] = readEEPROMint(start + (i * sizeof(int))); //<<<THIS DIDN'T WORK FOR SOME REASON
		// arr[i] = (float)readEEPROMlong(start + (i * sizeof(float)));
    EEPROM.get(start + (i * sizeof(float)), arr[i]);

		// Serial.println(arr[i]);
	}
	/* return true; */
}


byte readEEPROMint(int address) { //as array to read to
	// Serial.println("Reading EEPROM");

	int two, one, result;

	int four = EEPROM.read(address);
	int three = EEPROM.read(address + 1);

	Serial.print("four: ");
	Serial.println(four, BIN);
	Serial.print("three: ");
	Serial.println(three, BIN);

	if(sizeof(int) == 4) {
		Serial.println("reading 4 bytes total");
		two = EEPROM.read(address + 2);
		one = EEPROM.read(address + 3);
		Serial.print("two: ");
		Serial.println(two, BIN);
		Serial.print("one: ");
		Serial.println(one, BIN);
	}
	//Return the recomposed int by using bitshift.
	if(sizeof(int) == 4) {
		// Serial.println("reading 4 bytes total");
		result =  ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
	} else {
		// Serial.println("reading 2 bytes total");
		result =  ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF); //this assumes 2-byte int
	}
	return result;
}

void writeEEPROMint(int address, int value) {
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	// if(sizeof(int) == 4) {
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);
	// }

	// Serial.print("four: ");
	//   Serial.println(four, BIN);
	//   Serial.print("three: ");
	//     Serial.println(three, BIN);
	//     Serial.print("two: ");
	//       Serial.println(two, BIN);
	//       Serial.print("one: ");
	//         Serial.println(one, BIN);

	//Write the 2/4 bytes into the eeprom memory.
	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	if(sizeof(int) == 4) {
		// Serial.println("writing 4 bytes total");
		EEPROM.write(address + 2, two);
		EEPROM.write(address + 3, one);
	}
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to adress + 3.
void writeEEPROMlong(int address, long value)
{
	//Decomposition from a long to 4 bytes by using bitshift.
	//One = Most significant -> Four = Least significant byte
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);

	//Write the 4 bytes into the eeprom memory.
	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

long readEEPROMlong(long address)
{
	//Read the 4 bytes from the eeprom memory.
	long four = EEPROM.read(address);
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3);

	//Return the recomposed long by using bitshift.
	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

// void writeEEPROMfloat(int address, float value)
// {
// 	//Decomposition from a long to 4 bytes by using bitshift.
// 	//One = Most significant -> Four = Least significant byte
// 	byte four = (value & 0xFF);
// 	byte three = ((value >> 8) & 0xFF);
// 	byte two = ((value >> 16) & 0xFF);
// 	byte one = ((value >> 24) & 0xFF);
//
// 	//Write the 4 bytes into the eeprom memory.
// 	EEPROM.write(address, four);
// 	EEPROM.write(address + 1, three);
// 	EEPROM.write(address + 2, two);
// 	EEPROM.write(address + 3, one);
// }

// long readEEPROMfloat(int address)
// {
// 	//Read the 4 bytes from the eeprom memory.
// 	float four = EEPROM.read(address);
// 	float three = EEPROM.read(address + 1);
// 	float two = EEPROM.read(address + 2);
// 	float one = EEPROM.read(address + 3);
//
// 	//Return the recomposed long by using bitshift.
// 	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
// }

/* helper func */
float mapF(float val, float inMin, float inMax, float outMin, float outMax) {
	return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

String macToStr(const uint8_t* mac)
{
	String result;
	for (int i = 0; i < 6; ++i) {
		result += String(mac[i], 16);
		if (i < 5)
		result += ':';
	}
	return result;
}

//calibration, from IMU Zero example

void ForceHeader()
{ LinesOut = 99; }


void runCalibration(){
	// for (int i = iAx; i <= iGz; i++)
	// { // set targets and initial guesses
	//   Target[i] = 0; // must fix for ZAccel
	//   HighOffset[i] = 0;
	//   LowOffset[i] = 0;
	// } // set targets and initial guesses
	// Target[iAz] = 16384;
	// SetAveraging(NFast);
	//
	// PullBracketsOut();
	// PullBracketsIn(); //this should also write to eeprom
	//
	// Serial.println("------ calibration done ------");
	// isCalibrating = false; //exit calibration
}


void setup() {
	//setup
	Serial.begin(115200);
	EEPROM.begin(512);
	WiFi.macAddress(mac);    //get  MAC address
	macStr = macToStr(mac);  //convert to string

	delay(100);

	Serial.println("");
	Serial.println("");
	Serial.println("Starting up...");

	delay(100);

	Serial.println("Reading settings... ");
	ssid = readSSID();
	password = readPwd();
	devName = readName();
	//update sensor path
	sensorPath.concat(devName);

	readIP(); //this sets destIP
	destPort = readPort();
	readMsgFormat();
	// readOffsets();
	readAccelBias();
	readGyroBias();
	readMagBias();
	readMagScale();
	Serial.println("finished.");
	Serial.println("");

	delay(100);

	Serial.print("Device name: ");
	Serial.println(devName);
	Serial.print("Battery voltage: ");
	Serial.println(readVoltage());
  Serial.print("Message send rate: ");
	Serial.println(1000000L / sendRateInverse);

	/* setMessageFormat("accgyro b1 counter");//temp */
	sendCurrentMsgFormat();

	Serial.println("");

	/* delay(1500); */
	//  Serial.println(destIP[0]);

	// preapre button to temporarily turn on AP mode
	pinMode(onboardButtonLedPin, INPUT);
	/* Serial.println(sizeof(allButtonPins)); */
	/* delay(1500); */
	//set mode of button input pins
	for (int i = 0; i < (sizeof(allButtonPins) / sizeof(allButtonPins[0])); i++ ) {
		pinMode(allButtonPins[i], INPUT_PULLUP);
	}

	// join I2C bus (I2Cdev library doesn't do this automatically)
	/* #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE */

	/* #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE */
	/* Fastwire::setup(400, true); */
	/* #endif */

	// initialize accelerometer device
	Serial.println("Initializing I2C devices");
	Wire.setClock(400000);//speed up I2C
	Wire.begin(2, 4); //define which pins are used for I2C: Wire.begin(int sda, int scl)
	// mpu.initialize();

	// I2Cscan();// look for I2C devices on the bus

	// verify connection
	Serial.println(F("Testing device connections..."));
	// Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

	if (c == 0x73) // WHO_AM_I should always be 0x68 - Marcin: changed to 0x73
	{
		Serial.println("MPU9250 is online...");

		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
		Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
		Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
		Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
		Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
		Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
		// delay(1000);

		// get sensor resolutions, only need to do this once
		getAres();
		getGres();
		getMres();

		// Serial.println(" Calibrate gyro and accel");
		// accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		// Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
		// Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

		// delay(1000);

		initMPU9250();
		Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
		Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

		delay(100);

		// Get magnetometer calibration from AK8963 ROM
		initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

		// magcalMPU9250(magBias, magScale);
		// Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
		// Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
		// delay(2000); // add delay to see results before serial spew of data

		//    if (SerialDebug) {
		//  Serial.println("Calibration values: ");
		Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
		Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
		Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
		//    }

		attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

	}
	else
	{
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
		while (1) ; // Loop forever if communication doesn't happen CHANGEME
	}

	//trigger
	// updateTrigger.attach(1, sendStatusUpdate);

	Serial.println("");
	Serial.println("=== For list of possible commands send ? or ?? followed by Newline ===");
	Serial.println("");
	Serial.println("----- setup done -----");
	Serial.println("");
}

void loop() {
	if (Serial.available() > 0) {
		readSerial();
	};
	if(isCalibratingAccGyro){
		// runCalibration();
		accelgyrocalMPU9250(gyroBias, accelBias);
		isCalibratingAccGyro = false;
	} else if (isCalibratingMag) {
		magcalMPU9250(magBias, magScale);
		isCalibratingMag = false;
	} else {
		if (apMode) {
			dnsServer.processNextRequest();
			webServer.handleClient();
		} else {
			if ((WiFi.status() != WL_CONNECTED) && (!serialFallback)) {
				connectWiFi();
				return;
			} else {
				receiveOSC();
				readAllSensors(); //read data
				/* sendSensors(); */ //after processing mpu...

				sendNow = micros();
				if((sendNow - sendLast) > sendRateInverse) {
					sendSensors();
          sendCount ++;
          if((sendNow - sendLastSecond) > 10000000L) {
            readRate = sendCount;
            sendLastSecond = sendNow;
            sendCount = 0;
          }
					sendLast = sendNow;
				}
				if((sendNow - statusSendLast) > statusSendRateInverse) {
					updateStatus();
					statusSendLast = sendNow;
				}
			}
			// if(!useDMP) {
			// delay(delayTimeInMainLoop);
			// }
		}
	}
}
