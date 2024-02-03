#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 定义引脚
// 电机驱动
#define DRV_IN1 18
#define DRV_IN2 10
#define DRV_IN3 1
#define DRV_IN4 0
// 陀螺仪
#define MPU_SDA 7
#define MPU_SLC 6
// 屏幕
#define PLAY_SDA 3
#define PLAY_SLC 2
// BLE uuid
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
String cmd = "";
String resStr = "";
int pwm = 0;
int leftMotorPwmOffset = 4, rightMotorPwmOffset = 4; //起转pwm
float Balance_Angle_raw = 0;    //静态机械平衡角度。
float integrate; // 偏差积累
float keepAngle;
float kp = 10,ki = 0.3,kd = 0.5;
int speed = -6;
int turnSpeed = 400;
// float kp,ki,kd;
float turnKp = 0.7, turnSpd, turnPwm;  //转向pwm比例向，转向速度设定

class MyCallbacks: public BLECharacteristicCallbacks {
  
    void onWrite(BLECharacteristic *pCharacteristic) {      //写方法
      
      std::string value = pCharacteristic->getValue();      //接收值
      if (value.length() > 0) {
        // Serial.println("*********");
        // Serial.print("收到新信息: ");
        for (int i = 0; i < value.length(); i++){
          // Serial.print(value[i]);
          resStr += value[i];
        }
        cmd = resStr;
      }
    }
};

// 实例化 MPU6050
MPU6050 mpu6050(Wire);

// 马达控制函数
void motor(int left_pwm, int right_pwm, int turnPwm){

  if (left_pwm >= 0)
  {
    left_pwm += leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (left_pwm < 0)
  {
    left_pwm -= leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (right_pwm >= 0)
  {
    right_pwm += rightMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (right_pwm < 0)
  {
    right_pwm -= rightMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }

  left_pwm -= turnPwm;
  right_pwm += turnPwm;

  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);  //限定PWM区间在-255~255
  if (left_pwm >= 0){
    analogWrite(DRV_IN1, 0);
    analogWrite(DRV_IN2, left_pwm);
  }
  if (left_pwm < 0){
    analogWrite(DRV_IN2, 0);
    analogWrite(DRV_IN1, 0 - left_pwm);
  }
  if (right_pwm >= 0){
    analogWrite(DRV_IN4, right_pwm);
    analogWrite(DRV_IN3, 0);
  }
  if (right_pwm < 0){
    analogWrite(DRV_IN4, 0);
    analogWrite(DRV_IN3, 0 - right_pwm);
  }
}

void BLEDebug(String input) { //控制程序
    Serial.println(input);
    if (input.equals("")){
    }
    if (input.equals("1")){
        pwm++;
    }
    if (input.equals("0")){
        pwm--;
    }
    int index = input.indexOf(":");
    Serial.println(index);
    if (index != 0){
        String sub_S = input.substring(0, index);
        if (sub_S.equals("kp")){
            kp = input.substring(index+1).toFloat();
        }
        if (sub_S.equals("ki")){
            ki = input.substring(index+1).toFloat();
        }
        if (sub_S.equals("kd")){
            kd = input.substring(index+1).toFloat();
        }
    }
    if (input.equals("F")){
      keepAngle += speed;
    }
    if (input.equals("B")){
      keepAngle -= speed;
    }
    if (input.equals("L")){
      turnSpd += turnSpeed;
    }
    if (input.equals("R")){
      turnSpd -= turnSpeed;
    }
    if (input.equals(" ")){
      keepAngle = Balance_Angle_raw;
      turnSpd = 0;
    }
    
}

// pwm值计算
float pwmCalculation(float angleY, float gyroY, float keepAngle) //直立PMW计算
{
  float diff = angleY - keepAngle; // 计算角度偏差。
  integrate += diff; //偏差的积分，integrate为全局变量，一直积累。
  integrate = constrain(integrate, -1000, 1000); //限定误差积分的最大和最小值
  return kp * diff + ki * integrate + kd * gyroY;
}

void bleStart() {
    BLEDevice::init("MyESP32Car");                      //设备初始化，名称MyESP32
  
  BLEServer *pServer = BLEDevice::createServer();            //BLEServer指针，创建Server

  BLEService *pService = pServer->createService(SERVICE_UUID);     //BLEService指针，创建Service

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(     //BLECharacteristic指针，创建Characteristic
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());            //设置回调函数

  pCharacteristic->setValue("Hello World");    //发送信息，hello world
  pService->start();                      //开启服务

  BLEAdvertising *pAdvertising = pServer->getAdvertising();        //初始化广播
  pAdvertising->start();                    //开始广播
}

void setup() {
    Serial.begin(115200);
    Wire.begin(MPU_SDA, MPU_SLC);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(false);
    bleStart();
    Serial.print("===========");
    keepAngle = Balance_Angle_raw;

}

void loop() {
    BLEDebug(cmd);
    cmd = "";
    resStr = "";
    mpu6050.update();
    // Serial.println(mpu6050.getAngleY());
    // Serial.println(mpu6050.getGyroY());
    pwm = pwmCalculation(mpu6050.getAngleY(),mpu6050.getGyroY(),keepAngle);

    turnPwm = -turnKp * (turnSpd - mpu6050.getGyroZ());
    // Serial.println(pwm);
    // Serial.println(kp);
    // Serial.println(ki);
    // Serial.println(kd);
    motor(pwm, pwm, turnPwm);
    // delay(500);
}