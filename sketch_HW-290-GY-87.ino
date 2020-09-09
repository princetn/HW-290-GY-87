#include<Wire.h>
/// This library is written by Amir Gasmi in an effort to develop a complete and easy to use library for an autopilot 
/// using the 10 DOF HW-290 also known as GY-87 or GY-86 board which include the MPU 6050, BMP180 barometer 
/// and HMC5883L compass.
/// Rev. 05/16/2020.

typedef unsigned char T; // Primitive type unsigned byte (0-255).

#define MPU6050_I2C 0x68 // gyro-accelometer (6DOF)
#define BMP180_I2C 0x77 // Barometer (1DOF)
#define HMC5883L_I2C 0x1E // Compass (3DOF)

// MPU 6050 essential registers:
enum MPU6050_REG
{
  SELF_TEST_X = 13,
  SELF_TEST_Y = 14,
  SELF_TEST_Z = 15,
  SELF_TEST_A = 16,
  SMPRT_DIV_REG = 25, // sampling rate divider.
  CONFIG = 26, // configures FSYNC (Frame synchronization) & DLPF (Digital Low Pass Filter). FSYNC is of no use  for autopilot it is used for camera stabilization.
  GYRO_CONFIG = 27,
  ACCEL_CONFIG = 28,
  FIFO_EN = 35,
  ACCEL_XOUT_H = 59,
  TEMP_OUT_H = 65,
  GYRO_XOUT_H = 67,
  PWR_MGMT_1 = 0x6B,
  PWR_MGMT_2 = 108,
  FIFO_COUNT_H = 114, // counter of the FIFO buffer (how many sample data available 
  FIFO_COUNT_L = 115,
  FIFO_R_W = 116,
  WHO_AM_I = 117
  
};



enum GY_FS { GY_FS_250 = 0, GY_FS_500 = 1, GY_FS_1000 = 2, GY_FS_2000 = 3 }; // Gyro full scale Range in deg/s -> for GYRO_Config register

const struct {const float GY_SSF_250 = 131; const float GY_SSF_500 = 65.5; const float GY_SSF_1000 = 32.8; const float GY_SSF_2000 = 16.4; } GY_SSF ; // Accele. Sensitivity Scale Factor [LSB/G].

enum ACC_FS { ACC_FS_2G = 0, ACC_FS_4G = 1, ACC_FS_8G = 2, ACC_FS_16G = 3 }; // Accele. full scale Range in G. -> for ACC_Config register

enum ACC_SSF {ACC_SSF_2G = 16384, ACC_SSF_4G = 8192, ACC_SSF_8G = 4096, ACC_SSF_16G = 2048}; // Accele. Sensitivity Scale Factor [LSB/G].

enum DLPF_CFG_ { BW_260_256, BW_184_188, BW_94_98, BW_44_42, BW_21_20, BW_10, BW_5, BW_Disabled}; // Digital Low Pass filter in Hz for Gyro & Acc respectively.
// This register affect the sampling rate Fs ( BW_260 & BW_Disabled both provide Fs= 8kHz  and 1kHz in all other filtering. 
// accelerometer output rate is always 1kHz. 
// sampling rate = Fs/(1+SMPLRT_Div); unless sampling rate of the gyro is less than 1kHz accelerometer will continue to provide 1kHz sampling rate.


// I2C read and write from and to device:
T* ReadRegister(T device,T reg,T length)
{
  const T size = 10; // check this size you may need to increase if you want to buffer more data.
  static T buffer[size]; // allocates a buffer for the whole life of the program.
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();

  
  Wire.requestFrom(device, length);
    
  while(Wire.available()<length); // wait until data has been transfered to wire.
  
    if(Wire.available() == length) // copy data to buffer.
    {
        for(T i = 0; i < length; i++)
        {
            buffer[i] = Wire.read();
           
        }
    }
   

  return buffer;
  
}

void WriteRegister (T device, T reg, T data)
{
  
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

struct RawPt
{
  int X;
  int Y;
  int Z; 
  
};
struct ScaledPt
{
  float X;
  float Y;
  float Z; 
  
};

class MPU6050{
  public:
  MPU6050(T device = MPU6050_I2C);
  void Init();
  bool SelfTestAll(); // test all directions for gyro & acc.
  bool SelfTestGyX();
  bool SelfTestGyY();
  bool SelfTestGyZ();
  bool SelfTestAccX();
  bool SelfTestAccY();
  bool SelfTestAccZ();

  // get useful measurement data:
  RawPt getRawAcc();
  ScaledPt getScaledAcc();
  RawPt getRawGy();
  ScaledPt getScaledGy();

  // set registers:
  
  void setAccFS(ACC_FS acc_FS); // 3.
  void setGyFS(GY_FS gy_FS); // 2
  void setPWR_MGMT(T data); // 1
  void setDLPF_CFG(DLPF_CFG_ dlpf_cfg);

  // get the current state of registers: 

  void getSMPRT_DIV();
  void getaccFS();
  void getgyFS();
  void getDLPF_CFG();
  void getPWR_MGMT1();
  void getPWR_MGMT2();
  
  void Update_MPU();

  void CalibrateAccel();
  void CalibrateGyro();

  // print out all register data to serial monitor:
  void PrintParam();

  // helper functions for I2C communication  
  friend T* ReadRegister(T device,T reg,T length);
  friend void WriteRegister(T device, T reg, T data);

  protected:
  RawPt RawAcc;
  ScaledPt ScaledAcc;
  RawPt RawGy;
  ScaledPt ScaledGy;
  

  private:
  // configuration parameters:
  T accFS; // Acc. Full scale sensitivity
  T accBW; // Acc. Low pass Bandwidth
  T gyFS; // Gyro Full scale sensitivity
  T gyBW;// Gyro Low pass Bandwidth
  T DLPF_CFG;
  T gySMPLRT_DIV;
  T WhoAMI;
  bool TEMP_FIFO_EN;
  bool XG_FIFO_EN;
  bool YG_FIFO_EN;
  bool ZG_FIFO_EN;
  bool ACCEL_FIFO_EN;
  bool SLEEP;
  bool CYCLE;
  T CLKSEL;
  T SMPRT_DIV;
  
  
};





/// Implementation of Class members:

MPU6050::MPU6050(T device = MPU6050_I2C)
{
  WhoAMI = device;
  
 // RawAcc = (RawPt) malloc(sizeof(RawPt));
}
void MPU6050::Init()
{
  //Wire.begin();
  setPWR_MGMT(0);
  setGyFS(GY_FS_500);
  setAccFS(ACC_FS_8G); // 3.
  setDLPF_CFG(BW_Disabled);
  Update_MPU();
}

RawPt MPU6050::getRawAcc()
{
  
  T* var;
  

  var = ReadRegister(WhoAMI,ACCEL_XOUT_H,6);

  RawAcc.X = var[0]<<8 ;
  RawAcc.X |= var[1];
  RawAcc.Y = var[2]<<8 ;
  RawAcc.Y |= var[3];
  RawAcc.Z = var[4]<<8 ;
  RawAcc.Z |= var[5];


  

  return RawAcc; 
}

RawPt MPU6050::getRawGy()
{
  
  T* var;
  

  var = ReadRegister(WhoAMI,GYRO_XOUT_H,6);

  RawGy.X = var[0]<<8 ;
  RawGy.X |= var[1];
  RawGy.Y = var[2]<<8 ;
  RawGy.Y |= var[3];
  RawGy.Z = var[4]<<8 ;
  RawGy.Z |= var[5];


  

  return RawGy; 
}


void MPU6050::PrintParam()
{
  
  Serial.print("Who am I?: ");
  Serial.println(WhoAMI,HEX);
  Serial.print("accFS = ");
  Serial.println(accFS);
  Serial.print("gyFS = ");
  Serial.println(gyFS);
  Serial.print("DLPF_CFG = ");
  Serial.println(DLPF_CFG);

  
  Serial.print("CYCLE ");
  Serial.print(CYCLE);
  Serial.print(" SLEEP ");
  Serial.print(SLEEP);
  Serial.print(" CLKSEL ");
  Serial.println(CLKSEL);
  Serial.print(" SMPRT_DIV ");
  Serial.println(SMPRT_DIV);
  




  // Accel Raw data:
  Serial.print(" RawAcc: ");
  Serial.print(" X =");
  Serial.print(RawAcc.X);
  Serial.print(" Y =");
  Serial.print(RawAcc.Y);
  Serial.print(" Z =");
  Serial.println(RawAcc.Z);
   // Accel Raw data:
  Serial.print(" RawGy: ");
  Serial.print(" X =");
  Serial.print(RawGy.X);
  Serial.print(" Y =");
  Serial.print(RawGy.Y);
  Serial.print(" Z =");
  Serial.println(RawGy.Z);
  
  
}
void MPU6050::setAccFS(ACC_FS acc_fs)
{
  T var = *ReadRegister(WhoAMI,ACCEL_CONFIG,1);
  Serial.println("Setting AccFS...");
  Serial.println(var);
  var &= 0b11100111; // clearing the bites associated to Full scale sensitivity
  Serial.println(var);
  var |= acc_fs<<3; // assigning the desired full scale sensitivity.
  Serial.println(var);
  WriteRegister(WhoAMI, ACCEL_CONFIG, var); // writing the new value to the register.
  

}

void MPU6050::setGyFS(GY_FS gy_fs)
{
  T var = *ReadRegister(WhoAMI,GYRO_CONFIG,1);
  Serial.println("Setting GyFS...");
  Serial.println(var);
  var &= 0b11100111; // clearing the bites associated to Full scale sensitivity
  Serial.println(var);
  var |= gy_fs<<3; // assigning the desired full scale sensitivity.
  Serial.println(var);
  WriteRegister(WhoAMI, GYRO_CONFIG, var); // writing the new value to the register.
  
}

void MPU6050::setPWR_MGMT(T data)
{
  Serial.println("SetPWR_MGMT");
  WriteRegister(WhoAMI, PWR_MGMT_1, data);
  Serial.println(PWR_MGMT_1);
  Serial.println(data);
}
void MPU6050::setDLPF_CFG(DLPF_CFG_ dlpf_cfg)
{
  T var = *ReadRegister(WhoAMI,CONFIG,1);
  Serial.println("Setting dlpf_cfg...");
  Serial.println(var);
  var &= 0b11111000; // clearing the bites associated to Full scale sensitivity
  Serial.println(var);
  var |= dlpf_cfg; // assigning the desired full scale sensitivity.
  Serial.println(var);
  WriteRegister(WhoAMI, CONFIG, var); // writing the new value to the register.
  

}

void MPU6050::Update_MPU()
{
  // getaccFS
  getaccFS();
  // getgyFS
  getgyFS();

  getDLPF_CFG();


  // getPWR_MGMT_1:
  getPWR_MGMT1();

 // Get Sampling Rate Divider:
  getSMPRT_DIV();

  // Get Acc. Raw:
  getRawAcc();
  getRawGy();
  
  
}

void MPU6050::getSMPRT_DIV()
{
  T var = *ReadRegister(WhoAMI,SMPRT_DIV_REG,1);

  SMPRT_DIV = var;  
  
}
void MPU6050::getaccFS()
{
  T var = *ReadRegister(WhoAMI,ACCEL_CONFIG,1);
 
  accFS = var;
  accFS &= 0b11000;
  accFS = accFS>>3;
}

void MPU6050::getgyFS()
{
  T var = *ReadRegister(WhoAMI,GYRO_CONFIG,1);
  
  gyFS = var;
  gyFS &= 0b11000;
  gyFS = gyFS>>3;
  
}
void MPU6050::getPWR_MGMT1()
{
  T var = *ReadRegister(WhoAMI,PWR_MGMT_1,1);
  
  CYCLE = ((var &0b100000)>>5 == 1);
  SLEEP = ((var&0b1000000)>>6 == 1);
  CLKSEL = var &0b111;
}
void MPU6050::getPWR_MGMT2()
{
  
}
void MPU6050::getDLPF_CFG()
{
  T var = *ReadRegister(WhoAMI,CONFIG,1);
  DLPF_CFG = var & 0b111;
}


// Self test Section:

bool MPU6050::SelfTestGyX()
{
  int gyXoutEnabled,gyXoutDisabled,SelfTestResponse;
  //T var = *ReadRegister(WhoAMI,PWR_MGMT_1,1);
  //WriteRegister(WhoAMI, PWR_MGMT_1, data);// set pwr to non sleep.
  setPWR_MGMT(0); // wake up the sensor from sleep.
  setGyFS(GY_FS_250); // set gyro to full-scale range of +/-250dps
  T var = *ReadRegister(WhoAMI,GYRO_CONFIG,1);
  var |= 0b10000000; // we set Gyro x to self-test ON.
  Serial.println("**************************");
  Serial.println(var,BIN);
  Serial.println("**************************");
  WriteRegister(WhoAMI, GYRO_CONFIG, var);
  delay(5);
  T* GYSTOEnabled = ReadRegister(WhoAMI,GYRO_XOUT_H,2);
  T SLFT_GyX =  *ReadRegister(WhoAMI,SELF_TEST_X,1);
  SLFT_GyX &= 0b00011111;
  
  gyXoutEnabled = GYSTOEnabled[0]<<8|GYSTOEnabled[1];//SLFT_GyX;
  float FT = (SLFT_GyX != 0)?25 * 131 * pow(1.046,(SLFT_GyX-1)): 0.0;
  Serial.println("**************************");
  Serial.println(var,BIN);
  Serial.println("**************************");



  var &= 0b01111111; // we set Gyro x to self-test OFF.
  WriteRegister(WhoAMI, GYRO_CONFIG, var);
  delay(100);
  T* GYSTODisabled = ReadRegister(WhoAMI,GYRO_XOUT_H,2);
  SLFT_GyX =  *ReadRegister(WhoAMI,SELF_TEST_X,1);
  SLFT_GyX &= 0b00011111;
  gyXoutDisabled = GYSTODisabled[0]<<8|GYSTODisabled[1];//SLFT_GyX;
  
  //gyXoutEnabled = GYSTOEnabled[0]<<8|GYSTOEnabled[1];
  //gyXoutDisabled = GYSTODisabled[0]<<8|GYSTODisabled[1];
  Serial.println("**************************");
  Serial.println(gyXoutEnabled);
   Serial.println(gyXoutDisabled);
  Serial.println("**************************");

  SelfTestResponse = gyXoutEnabled - gyXoutDisabled;
  Serial.print("SelfTestResponse = ");
  Serial.println(SelfTestResponse);

  
  Serial.print("FT = ");
  Serial.println(FT);

  
  float changeFromFactoryTrim = ((float)SelfTestResponse - FT)/FT ;
  Serial.print("Change from Factory Trim = ");
  Serial.println(changeFromFactoryTrim);
  

  return true;
}
bool MPU6050::SelfTestGyY()
{

  return true;
}
bool MPU6050::SelfTestGyZ()
{

  return true;
}
bool MPU6050::SelfTestAccX()
{
  int gyXoutEnabled,gyXoutDisabled,SelfTestResponse;
  //T var = *ReadRegister(WhoAMI,PWR_MGMT_1,1);
  //WriteRegister(WhoAMI, PWR_MGMT_1, data);// set pwr to non sleep.
  setPWR_MGMT(0); // wake up the sensor from sleep.
  setAccFS(ACC_FS_8G); // set Accel. to full-scale range of +/-8G
  T var = *ReadRegister(WhoAMI,ACCEL_CONFIG,1);
  var |= 0b10000000; // we set Gyro x to self-test ON.
  Serial.println("**************************");
  Serial.println(var,BIN);
  Serial.println("**************************");
  WriteRegister(WhoAMI, ACCEL_CONFIG, var);
  delay(150);
  T* GYSTOEnabled = ReadRegister(WhoAMI,ACCEL_XOUT_H,2);
  T SLFT_AccX =  *ReadRegister(WhoAMI,SELF_TEST_X,1);
  SLFT_AccX &= 0b11100000;
  SLFT_AccX = SLFT_AccX>>3;
  T TMP =  *ReadRegister(WhoAMI,SELF_TEST_A,1); 
  TMP = TMP&0b00110000;
  SLFT_AccX |= TMP<<4;
  
  gyXoutEnabled = GYSTOEnabled[0]<<8|GYSTOEnabled[1];//SLFT_GyX;
  float FT = (SLFT_AccX != 0)?4096*0.34 * 131 * pow(0.92/0.34,(SLFT_AccX-1)/(pow(2,5)-2)): 0.0;
  Serial.println("**************************");
  Serial.println(var,BIN);
  Serial.println("**************************");



  var &= 0b01111111; // we set Gyro x to self-test OFF.
  WriteRegister(WhoAMI, GYRO_CONFIG, var);
  delay(100);
  T* GYSTODisabled = ReadRegister(WhoAMI,ACCEL_XOUT_H,2);
  //SLFT_GyX =  *ReadRegister(WhoAMI,SELF_TEST_X,1);
  //SLFT_GyX &= 0b00011111;
  gyXoutDisabled = GYSTODisabled[0]<<8|GYSTODisabled[1];//SLFT_GyX;
  
  //gyXoutEnabled = GYSTOEnabled[0]<<8|GYSTOEnabled[1];
  //gyXoutDisabled = GYSTODisabled[0]<<8|GYSTODisabled[1];
  Serial.println("**************************");
  Serial.println(gyXoutEnabled);
   Serial.println(gyXoutDisabled);
  Serial.println("**************************");

  SelfTestResponse = gyXoutEnabled - gyXoutDisabled;
  Serial.print("SelfTestResponse = ");
  Serial.println(SelfTestResponse);

  
  Serial.print("FT = ");
  Serial.println(FT);

  
  float changeFromFactoryTrim = ((float)SelfTestResponse - FT)/FT ;
  Serial.print("Change from Factory Trim = ");
  Serial.println(changeFromFactoryTrim);
  

  

  return true;
}
bool MPU6050::SelfTestAccY()
{

  return true;
}
bool MPU6050::SelfTestAccZ()
{

  return true;
}

















MPU6050 mpu6050;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
 
  //MPU6050 mpu6050(MPU6050_I2C);
  
  Serial.println("This begin");
  mpu6050.SelfTestAccX();
  mpu6050.SelfTestGyX();
  mpu6050.Update_MPU();
  mpu6050.PrintParam();
  
  delay(4000);
  
  mpu6050.Init();
  
  mpu6050.Update_MPU();
  mpu6050.PrintParam();

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //mpu6050.Update_MPU();

  //mpu6050.PrintParam();
  delay(5000);
  //delayMicroseconds(40000);

}
