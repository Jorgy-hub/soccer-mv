# ifndef ROBOT_H
# define ROBOT_H

class Motor {
private:
  int pin_a;
  int pin_b;

public:
  Motor(int a, int b) {
    Motor::pin_a = a;
    Motor::pin_b = b;
  }
  void Iniciar() {
    pinMode(Motor::pin_a, OUTPUT);
    pinMode(Motor::pin_b, OUTPUT);
  }
  void Mover(int vel, bool inverted = false) {
    vel = vel * (inverted ? -1 : 1);
    vel = constrain(vel, -255, 255);
    analogWrite(Motor::pin_a, vel < 0 ? 0 : vel);
    analogWrite(Motor::pin_b, vel < 0 ? abs(vel) : 0);
  }
};

class Robot {
private:
  int kp = 5;
  Motor *ad_izq;
  Motor *ad_der;
  Motor *at_izq;
  Motor *at_der;
public:
  Robot(Motor *motor_01, Motor *motor_02, Motor *motor_03, Motor *motor_04) {
    Robot::ad_der = motor_01; 
    Robot::ad_izq = motor_02; 
    Robot::at_der = motor_03;
    Robot::at_izq = motor_04;  
  }

  void Iniciar() {
    Robot::ad_izq->Iniciar();
    Robot::ad_der->Iniciar();
    Robot::at_izq->Iniciar();
    Robot::at_der->Iniciar();
  }

  void Estabilizar(float angulo = 0) {
    int offset = (int)angulo * kp;
    Serial.println(offset);
    Robot::ad_izq->Mover(offset, true);
    Robot::ad_der->Mover(offset);
    Robot::at_izq->Mover(offset, true);
    Robot::at_der->Mover(offset);
  }
};

# endif
