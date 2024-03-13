# include "Robot.h"
# include "Simple_MPU6050.h"

// Variables Globales:
int angulo = 0;

// Clases:
Simple_MPU6050 mpu;
Motor motor_1(6, 7);
Motor motor_2(8, 9);
Motor motor_3(10, 11);
Motor motor_4(12, 13);
Robot Alpharius(&motor_1, &motor_2, &motor_3, &motor_4);

void updateCompass(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  
  angulo = xyz[0];
}

void setup() {
  Serial.begin(9600);

  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(10);
  mpu.CalibrateMPU(); 
  mpu.load_DMP_Image();
  mpu.on_FIFO(updateCompass);

  // Iniciar Motores:
  Alpharius.Iniciar();
}

void loop() {
  mpu.dmp_read_fifo(false);
  Alpharius.Estabilizar(angulo);
}
