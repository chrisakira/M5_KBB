#include <Arduino.h>
#include <PID_v1.h>
#include <M5Stack.h>
#include "I2Cdev.h"
#include "Elegant.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <WiFi.h>
#include "Wire.h"
#include <esp_now.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3
#define POS_X_GRAFICO 30
#define POS_Y_GRAFICO 3
#define ALTURA_GRAFICO 180
#define COMPRIMENTO_GRAFICO 270
#define POS_X_DADOS 30
#define POS_Y_DADOS 200

typedef struct struct_message {
    int x = 0;
    int y = 0;
} struct_message;
struct_message myData;

static double Setpoint, Input, Output;
double aggKp = 15, aggKi = 40, aggKd = 2;
double consKp = 12, consKi = 20, consKd = 1.2;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
MPU6050 mpu(0x69);
TaskHandle_t display;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;

float ypr[3];
float Y_angle;
float Z_angle;
int linhaExemplo = 20;
int leituraAtual = 1;
int fator = 1;
uint32_t Time_Ant;
uint32_t Time_Loop;

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {

  memcpy(&myData, incomingData, sizeof(myData));
}
 
void SetMotor(float Out)
{
  if (Out > 0)
  {
    ledcWrite(0,Out+myData.y);
    ledcWrite(1,Out-myData.y);

    digitalWrite(GPIO_NUM_17,LOW);
    digitalWrite(GPIO_NUM_2,HIGH);
    
    digitalWrite(GPIO_NUM_5,LOW);
    digitalWrite(GPIO_NUM_25,HIGH);
    
  }
  else if (Out < -0)
  {
    ledcWrite(0,-Out+myData.y);
    ledcWrite(1,-Out-myData.y);

    digitalWrite(GPIO_NUM_17,HIGH);
    digitalWrite(GPIO_NUM_2,LOW);
    
    digitalWrite(GPIO_NUM_5,HIGH);
    digitalWrite(GPIO_NUM_25,LOW);
  
  }
  else
  {
    ledcWrite(0,Out+myData.y);
    ledcWrite(1,Out-myData.y);

    digitalWrite(GPIO_NUM_17,HIGH);
    digitalWrite(GPIO_NUM_2,LOW);
    
    digitalWrite(GPIO_NUM_5,HIGH);
    digitalWrite(GPIO_NUM_25,LOW);
  
  }
}

void pinSetup()
{
  pinMode(GPIO_NUM_16,OUTPUT); // PWMA
  pinMode(GPIO_NUM_17,OUTPUT); // AIN1
  pinMode(GPIO_NUM_2,OUTPUT); // AIN2
  
  pinMode(GPIO_NUM_5,OUTPUT); // BIN1
  pinMode(GPIO_NUM_26,OUTPUT); // BIN2
  pinMode(GPIO_NUM_25,OUTPUT); // PWMB

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(GPIO_NUM_26,1);
  ledcAttachPin(GPIO_NUM_16,0);

}

void setup()
{
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  pinSetup();
  Wire.begin(GPIO_NUM_21, GPIO_NUM_22, 100000);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250, 250);
  myPID.SetSampleTime(50);
  M5.begin(true, false, false, false);
  M5.Speaker.mute();
  M5.Speaker.setVolume(0);
  M5.Speaker.end();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
//OFFSETS    -5014,    5208,    9498,     -31,     -42,     -80
  mpu.setXGyroOffset(-31);
  mpu.setYGyroOffset(-42);
  mpu.setZGyroOffset(-80);
  mpu.setXAccelOffset(-5014);
  mpu.setYAccelOffset(5208);
  mpu.setZAccelOffset(9498);

  if (devStatus == 0)
  {
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  if (!dmpReady)
    return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Time_Loop = esp_timer_get_time() - Time_Ant;
    Time_Ant = esp_timer_get_time();
    Y_angle = (ypr[2] * 180 / PI) - 0.1;
    Z_angle = ypr[1] * 180 / PI;
    Input = Y_angle;
    Setpoint = myData.x;
    
    double gap = abs(Setpoint - Input);
    if (gap < 10)
      myPID.SetTunings(consKp, consKi, consKd);
    else
      myPID.SetTunings(aggKp, aggKi, aggKd);
    
    while ((esp_timer_get_time() - Time_Ant) < 50)
    {
    }
    Time_Ant = esp_timer_get_time();
    myPID.Compute();
    if(Y_angle>45 || Y_angle<-45){
      SetMotor(0);  
    }else
      SetMotor(Output);
  }
}
