// 작성일자 2021.08.~                                                                                                                                                                                                                                                                                                                                                                           작성자 : HRRLAB_김민철 
// 작성자 : HRRLAB 김민철


//#include <TensorFlowLite.h>
//#include <tensorflow/lite/micro/all_ops_resolver.h>
//#include <tensorflow/lite/micro/micro_error_reporter.h>
//#include <tensorflow/lite/micro/micro_interpreter.h>
//#include <tensorflow/lite/schema/schema_generated.h>
//


//#include "model.h"

#include <Arduino_LSM9DS1.h>
#include <Wire.h>

// global variables used for TensorFlow Lite (Micro)

//tflite::MicroErrorReporter tflErrorReporter; 딥러닝 사용시 주석해제필요

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.

//tflite::AllOpsResolver tflOpsResolver; 딥러닝 사용시 주석해제필요
//
//const tflite::Model* tflModel = nullptr;
//tflite::MicroInterpreter* tflInterpreter = nullptr;
//TfLiteTensor* tflInputTensor = nullptr;
//TfLiteTensor* tflOutputTensor = nullptr;



unsigned long now, lastTime = 0;
float dt;  
float ax, ay, az, gx, gy, gz;             // Accelerometer gyroscope raw data
float rmsa = 0;
float v4 = 0;

float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    // Angle variable
int angle_velocity_decision = 0;
int impact_decision = 0;
int rmsg = 0;
int roll=0, pitch=0, yaw=0;
long axo = 0, ayo = 0, azo = 0;             // Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             // Gyro offset
float c = 0, s = 0;
float u = 0;
int v = 0;
float pi = 3.1415926;
float AcceRatio = 16384.0;                  // Accelerometer scale factor
float GyroRatio = 131.0;                    // Gyroscope scale factor
uint8_t n_sample = 4;                       // Accelerometer filter algorithm sampling number
float aaxs[4] = {0}, aays[4] = {0}, aazs[4] = {0};         // x,y-axis sampling queue
long aax_sum, aay_sum,aaz_sum;                      // x,y-axis sampling add

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; // Accelerometer covariance calculation queue
float Px=1, Rx, Kx, Sx, Vx, Qx;             // x-axis Calman variables
float Py=1, Ry, Ky, Sy, Vy, Qy;             // y-axis Calman variables
float Pz=1, Rz, Kz, Sz, Vz, Qz;             // z-axis Calman variables

char EMG_DSP[30] = {0,};

const int soundpin = 9;
int accident_cnn_value = 0;
int accident_sensor_value = 0;
int accident_smash_value = 0;
float final_value = 0;

void setup() { 
   Serial.begin(9600);
//    while (!Serial);
//    Serial.println("Started");
    pinMode(soundpin, OUTPUT);

    if (!IMU.begin()) { // IMU센서를 초기화합니다. 초기화중 문제가 발생하면 오류를 발생시킵니다.
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    unsigned short times = 200;  
    for(int i=0;i<times;i++)
    {
        accel();
        gyro();
   
        axo += ax; ayo += ay; azo += az;      // Sampling
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offset
    gxo /= times; gyo /= times; gzo /= times; // Calculate the gyro offset
    
}


void loop() {
 
    float now_time = millis();
    kalman();
    angle_velocity_decision == 0;


    int a = round(gx-2);
    int b = round(gy-1);
    int c = round(gz);
    rmsa = sqrt(pow(ax,2)+pow(ay,2)+pow(az,2));
    rmsg = sqrt(pow(a,2)+pow(b,2)+pow(c,2));
    int x1 = round(ax*10);
    int y1 = round(ay*10);
    int z1 = round(az*10);
    float x = x1/10.0;
    float y = y1/10.0;
    float z = z1/10.0;
    float rmsa1 = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
//    digitalWrite(soundpin,LOW);
//    Serial.print(yaw);Serial.print(",");
//    Serial.print(rmsa);Serial.print(",");
//    sprintf(EMG_DSP, "%04d,%04d", roll, pitch);
    Serial.println(yaw);

 //   impact();

//    finaljudgement();

}
void finaljudgement(){
    if((impact_decision ==1) || (angle_velocity_decision == 1)){
       
        digitalWrite(soundpin,HIGH);
        Serial.print("STATE3");Serial.print(",");
        sprintf(EMG_DSP, "%04d,%04d", roll, pitch);
        Serial.println(EMG_DSP);
        delay(5000);
        impact_decision =0;
        angle_velocity_decision = 0;
    }

}

void impact(){

    if((abs(gx) >= 100) || (abs(gy) >= 100) || (abs(gz) >= 100)){
        for(int i=0; i < 1000; i++){
            accel();
            gyro();
            kalman();
            finaljudgement();
            int x1 = round(ax*10);
            int y1 = round(ay*10);
            int z1 = round(az*10);
            float x = x1/10.0;
            float y = y1/10.0;
            float z = z1/10.0;
            int a = round(gx-2);
            int b = round(gy-1);
            int c = round(gz);
            rmsg = sqrt(pow(a,2)+pow(b,2)+pow(c,2));
            float rmsa1 = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
            if(rmsa1==0){
                impact_decision = 1;
                Serial.print("im");Serial.print(",");
            }
            if(((roll <= 60) || (pitch <= 80)) && ((abs(rmsg) >= 330))){
        
                angle_velocity_decision = 1;
                Serial.print("av");Serial.print(",");


            }
            else{
                impact_decision = 0;
                angle_velocity_decision = 0;
                digitalWrite(soundpin,LOW);
                Serial.print("STATE2");Serial.print(",");
                Serial.print(rmsa1);Serial.print(",");
                sprintf(EMG_DSP, "%04d,%04d", roll, pitch);
                Serial.println(EMG_DSP);
            }
            
        }

    }

    else{
        impact_decision = 0;
        digitalWrite(soundpin,LOW);
        Serial.print("STATE1");Serial.print(",");
        Serial.print(rmsa);Serial.print(",");
        sprintf(EMG_DSP, "%04d,%04d", roll, pitch);
        Serial.println(EMG_DSP);

    }
}
void accel(){
    if (IMU.accelerationAvailable()) {// 가속도 센서의 값을 출력합니다.
        IMU.readAcceleration(ax, ay, az); // x, y, z에 각 축별 데이터를 넣습니다.

        }
    
}
void gyro(){
    if (IMU.gyroscopeAvailable()) { // 자이로 센서의 값을 출력합니다.
        IMU.readGyroscope(gx, gy, gz);
////////////
        }
}
void kalman(){
    unsigned long now = millis();             // current time(ms)
    dt = (now - lastTime) / 1000.0;           // Differential time(s)
    lastTime = now;                           // Last sampling time(ms)
    accel();
    gyro();
    float accx = ax / AcceRatio;              // x-axis acceleration
    float accy = ay / AcceRatio;              // y-axis acceleration
    float accz = az / AcceRatio;              // z-axis acceleration

    aax = atan(accy / accx) * 180 / pi;    // The x-axis angle to the z-axis
    aay = atan(accz / accx) * 180 / pi;       // The y-axis angle to the z-axis
    aaz = atan(accz / accy) * 180 / pi;       // The z-axis angle to the y-axis

    aax_sum = 0;                              // Sliding weight filtering algorithm for accelerometer raw data
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 15.5 / 7.0; // Angle AM ​​to 0-90 °
    aays[n_sample-1] = aay;                        // Here we use the experimental method to obtain the appropriate coefficient
    aay_sum += aay * n_sample;                     // This example factor is 9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 15.5 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 15.5 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; // x-axis angular velocity
    float gyroy = - (gy-gyo) / GyroRatio * dt; // x-axis angular velocity
    float gyroz = - (gz-gzo) / GyroRatio * dt; // x-axis angular velocity
    agx += gyrox;                             // x-axis angular velocity integral
    agy += gyroy;                             // y-axis angular velocity integral
    agz += gyroz;                             // z-axis angular velocity integral
        /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //The average value of the calculation
        a_x[i-1] = a_x[i];                      // The acceleration average
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 // x-axis acceleration average
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 // y-axis acceleration average
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;                                 // z-axis acceleration average

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              // Get the variance
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         // 0.0025 in the following instructions ...
    Kx = Px / (Px + Rx);                      // Calculate the Kalman gain
    agx = agx + Kx * (aax - agx);             // Gyro angle and accelerometer speed superimposed
    Px = (1 - Kx) * Px;                       // Update p value

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    /* kalman end */
     roll = round(agx);
     pitch = round(agy);
     yaw = round(agz);
}
