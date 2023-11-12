#ifndef BRUJULA_H
#define BRUJULA_H
#include "MPU6050.h"

class MPU6050_J
{
private:
    volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
    unsigned long current_time, previous_time = 0, loop_time;
    int16_t accY, accZ, gyroX, gyroRate;
    MPU6050 mpu;

public:
    /**
     *  Prepara la Brujula.
     * 
     *  Aqui se hace la calibracion principal de la brujula.
     *  Es recomendable establecer unos valores pre-calculados para los offsets.
     *  En la carpeta de ejemplos se encuentra un archivo para calcular los offsets.
     */
    void Initialize()
    {
        mpu.initialize();
        mpu.setYAccelOffset(1593);
        mpu.setZAccelOffset(963);
        mpu.setXGyroOffset(40);
    }

    /**
     * Lecturas de la Brujula.
     * 
     * Aqui se realizan las Lecturas y Calculos de la brujula
     * que posteriormente pasaran a ser analizadas para un control PID
     * para el movimiento de los motores.
    */
    void Read()
    {
        // Lee los valores del MPU6050.
        accY = mpu.getAccelerationY();
        accZ = mpu.getAccelerationZ();
        gyroX = mpu.getRotationX();

        // Calcula el Angulo de Inclinacion.
        accAngle = atan2(accY, accZ) * RAD_TO_DEG;
        gyroRate = map(gyroX, -32768, 32767, -250, 250);
        gyroAngle = (float)gyroRate * sampleTime;
        currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);
        prevAngle = currentAngle;
    }
};

#endif