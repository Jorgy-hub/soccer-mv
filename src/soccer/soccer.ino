#include <Pixy2.h>
#include <Simple_MPU6050.h>
#include <Wire.h>

// Brujula:
Simple_MPU6050 mpu;
double offset = 0;
double mov_q = 100;
int mov_cor = 0;

/**
 *  Nuestro Algoritmo Principal.
 *  Ejecuta tan pronto recibe datos de actualizacion en el giroscopio.
 *  Esos datos son procesados en esta funcion para mover el Robot.
 *  @param
 */
void gyroValues(int16_t *gyro, int16_t *accel, int32_t *quat) {
    // Calcular angulos de Movimiento:
    Quaternion q;
    VectorFloat gravity;
    float ypr[3] = {0, 0, 0};
    float xyz[3] = {0, 0, 0};
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);

    // Velocidad Maxima de Movimiento.
    float angle = xyz[0] * -1;
    mov_cor = int(angle + offset) * -4;

    // Algoritmo Principal.
    int status = alpharius();
}

/**
 *  Incio de Arduino, configuraciones:
 *  - Motores
 *  - Comunicacion Serial con Camara
 *  - Giroscopio
 */
void setup() {
    // Declaracion de Motores:

    // Calibracion Basica de Arduino:
    Wire.begin();
    Serial3.begin(9600);
    Serial.begin(9600);

    // Calibracion del Giroscopio:
    mpu.begin();
    mpu.Set_DMP_Output_Rate_Hz(10);
    mpu.CalibrateMPU();
    mpu.load_DMP_Image();
    mpu.on_FIFO(gyroValues);
    //////////////////////////////
}

/**
 *  En el Loop solamente se mandara a hacer las lecturas
 *  del giroscopio para actualizar los valores.
 */
void loop() {
    static unsigned long FIFO_DelayTimer;
    if ((millis() - FIFO_DelayTimer) >= (99)) {
        if (mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis();
    }
}

void Alpharius() {
    switch (ball) {
        case 314:  // NO PELOTA
            freno();
            // Serial.println("Freno");
            break;
        case 300:  // PELOTA ADELANTE LEJOS
            if (L1 > LS1) {
                atras(255);
            } else {
                adelante(255);
            }
            // Serial.println("FRENTE LEJOS");
            break;
        case 301:  // PELOTA IZQUIERDA LEJOS
            if (L4 > LS4) {
                derecha(305);
            } else {
                izquierda(255);
            }
            // Serial.println("IZQ LEJOS");
            break;
        case 302:  // PELOTA DERECHA LEJOS
            if (L2 > LS2) {
                izquierda(255);
            } else {
                derecha(305);
            }
            // Serial.println("DER LEJOS");
            break;
        case 303:  // PELOTA ATRAS LEJOS
            if (L3 > LS3) {
                adelante(255);
            } else {
                atras(180);
            }
            // Serial.println("ATRS LEJOS");
            break;
        case 200:  // PELOTA EN POSESION
            if (L1 > LS1) {
                atras(255);
            } else {
                pixy2L();
            }
            // Serial.println("EN POSESION");
            break;
    }

    if (ball >= -180 and ball <= 180) {  // SI LA PELOTA ESTÁ CERCA
        if (ball <= 16 and ball >= -35) {
            // Serial.println("Adelante");
            if (L1 > LS1) {
                atras(255);
            } else {
                adelante(255);
            }
        } else if (ball >= 55 and ball < 120) {
            if (L3 > L3) {
                adelante(255);
            } else {
                atras(150);
            }
            // Serial.println("Atras");
        } else if (ball < -55 and ball > -120) {
            if (L3 > LS3) {
                adelante(255);
            } else {
                atras(150);
            }
            // Serial.println("Atras2");
        } else if (ball >= 21 and ball < 55) {
            if (L4 > LS4) {
                derecha(255);
            } else {
                izquierda(abs(ball * 3));
            }
            // Serial.println("izquierda");
        } else if (ball < -35 and ball > -55) {
            if (L2 > LS2) {
                izquierda(255);
            } else {
                derecha(abs(ball * 3));
            }
            // Serial.println("derecha");
        } else if (ball <= -120) {
            if (L4 > LS4) {
                derecha(255);
            } else {
                izquierda(potencia);
            }
            // Serial.println("izquierda VV");
        } else if (ball >= 120) {
            if (L2 > LS2) {
                izquierda(255);
            } else {
                derecha(potencia);
            }
            // Serial.println("derecha VV");
        }
    }
}

class Robot {
   private:
    int motor_pins[4][2] = {{13, 12}, {11, 10}, {8, 9}, {7, 6}};

   public:
    initialize() {
        for (int i = 6; i <= 13; i++) {
            pinMode(i, OUTPUT);
        }
    }

    moveMotor(int _motor, int _vel) {
        int _pin_a = motor_pins[_motor - 1][0];
        int _pin_b = motor_pins[_motor - 1][1];
        int vel = constrain(_vel, -255, 255);
        analogWrite(_pin_a, vel > 0 ? vel : 0);
        analogWrite(_pin_b, vel > 0 ? 0 : vel * -1);
    }

    void adelante(int vel) {
        moveMotor(1, -vel + correccion);
        moveMotor(2, -vel + correccion);
        moveMotor(3, vel + correccion);
        moveMotor(4, vel + correccion);
    }

    void atras(int vel) {
        motor01(vel + correccion);
        motor2(vel + correccion);
        motor3(-vel + correccion);
        motor4(-vel + correccion);
    }
    void izquierda(int vel) {
        motor01(vel + Q + correccion);
        motor2(-vel + correccion);
        motor3(-vel + correccion);
        motor4(vel + Q + correccion);
    }

    void derecha(int vel) {
        motor01(-vel + correccion);
        motor2(vel + correccion);
        motor3(vel + Q + correccion);
        motor4(-vel + Q + correccion);
    }
    void freno() {
        motor01(correccion);
        motor2(correccion);
        motor3(correccion);
        motor4(correccion);
    }
    void DiagonalI(int vel) {
        motor01(correccion);
        motor2(-vel + correccion);
        motor3(correccion);
        motor4(vel + correccion);
    }
    void DiagonalD(int vel) {
        motor01(-vel + correccion);
        motor2(correccion);
        motor3(vel + correccion);
        motor4(correccion);
    }
};