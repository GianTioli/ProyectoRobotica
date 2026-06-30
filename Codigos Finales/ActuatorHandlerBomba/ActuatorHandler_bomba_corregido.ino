#include <AccelStepper.h>
#include <MultiStepper.h>

const int IN1 = 4;
const int IN2 = 5;
const int IN1P = 6;
const int IN2P = 7;
const int PWM_P = 10;

const int dirPinB = 3;
const int stepPinB = 2;
const int EnB = 11;
const int dirPinC = 9;
const int stepPinC = 8;
const int EnC = 12;

float t = 0.0;
float t_p = 0.0;
float targetCyl = 0.0;
float CylCurPos = 0.0;
float deltaCyl = 0.0;
float q1 = 0, q2 = 0, q3 = 0, q4 = 0;

bool Inicio = false;
int Cont = 0;

long GoToPos[2];

AccelStepper StepB(1, stepPinB, dirPinB);
AccelStepper StepC(1, stepPinC, dirPinC);
MultiStepper StepCtrl;

String inputString = "";
bool stringComplete = false;
bool DEBUG = false;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN1P, OUTPUT);
  pinMode(IN2P, OUTPUT);
  pinMode(PWM_P, OUTPUT);

  pinMode(EnB, OUTPUT);
  pinMode(EnC, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN1P, LOW);
  digitalWrite(IN2P, LOW);
  analogWrite(PWM_P, 160);

  digitalWrite(EnB, HIGH);
  digitalWrite(EnC, HIGH);

  StepB.setMinPulseWidth(3);
  StepC.setMinPulseWidth(3);
  StepB.setMaxSpeed(200);
  StepC.setMaxSpeed(200);

  StepCtrl.addStepper(StepB);
  StepCtrl.addStepper(StepC);

  inputString.reserve(50);

  // if (DEBUG) Serial.println("Iniciando Arduino (DEBUG ON)");
}

void loop() {
  if (!stringComplete){
    digitalWrite(EnB, HIGH);
    digitalWrite(EnC, HIGH);
    return;
  } 
    

  inputString.trim();
  digitalWrite(EnB, HIGH);
  digitalWrite(EnC, HIGH);

  if (inputString == "RESET") {
    Inicio = false;
    Cont == 0;
    digitalWrite(EnB, HIGH);
    digitalWrite(EnC, HIGH);
    q1 = 0;
    q2 = 0;
    q3 = 0;
    q4 = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN1P, LOW);
    digitalWrite(IN2P, LOW);
    inputString = "";
    stringComplete = false;
    Serial.println("OK");
    return;
  }

  int c1 = inputString.indexOf(',');
  int c2 = inputString.indexOf(',', c1 + 1);
  int c3 = inputString.indexOf(',', c2 + 1);

  if (c1 > 0 && c2 > 0) {
    q1 = inputString.substring(0, c1).toFloat();
    q2 = inputString.substring(c1 + 1, c2).toFloat();

    if (c3 > 0) {
      q3 = inputString.substring(c2 + 1, c3).toFloat();
      q4 = inputString.substring(c3 + 1).toFloat();
    } else {
      q3 = inputString.substring(c2 + 1).toFloat();
      q4 = 0.0;
    }
  }

  /*
  if (DEBUG) {
    Serial.print("q1: "); Serial.print(q1);
    Serial.print(" | q2: "); Serial.print(q2);
    Serial.print(" | q3: "); Serial.print(q3);
    Serial.print(" | q4: "); Serial.println(q4);
  }
  */

  // Movimiento articular: se mantiene la calibracion de la ultima version funcional.
  if(q3 > 0 && Cont == 0){
    Inicio = true;
    Cont == 1;
    digitalWrite(EnB, LOW);
    digitalWrite(EnC, LOW);
  } else{
    Cont == 0;
    Inicio = false;
    digitalWrite(EnB, HIGH);
    digitalWrite(EnC, HIGH);
  }
  if(Inicio){
    digitalWrite(EnB, LOW);
    digitalWrite(EnC, LOW);
    GoToPos[0] = -1 * q1 * (20.0 / 9.0) * 5.0;
    GoToPos[1] =  (q2) * (20.0 / 9.0) * 5.2;
    StepCtrl.moveTo(GoToPos);
    StepCtrl.runSpeedToPosition();
  } else{
    digitalWrite(EnB, HIGH);
    digitalWrite(EnC, HIGH);
  }
  
  
  MoveCylTo(q3);

  PumpM(q4);
  digitalWrite(IN1P, LOW);
  digitalWrite(IN2P, LOW);
  analogWrite(PWM_P, 150);

  // Bomba: el primer q4 no nulo despues de RESET succiona; los siguientes dispensan.
  inputString = "";
  stringComplete = false;

  Serial.println("OK");
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

/*
float CylMoveDist(float dist) {
  t = (20000.0 / 196977.0) * dist * 1000.0;
  return t;
}
*/

float CylMoveT(float dist){
  t = (20000.0/196977.0)*dist*1335;
  return t;
}

void CylMoveD(float delta){
    if(delta < 0.0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      delay((unsigned long)fabs(delta));
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    } else if(delta > 0.0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      delay((unsigned long)fabs(delta));
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    } else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
}

void MoveCylTo(float dist) {
  targetCyl = CylMoveT(dist);          // convierte distancia a tiempo equivalente
  CylMoveD(CylCurPos - targetCyl);             // calcula delta actualizado justo antes de mover
  CylCurPos = targetCyl;               // actualiza posición actual
}

/*
float Pump_mL(float mL) {
  t_p = mL * (5.0 / 6.0) * 1750.0;
  return t_p;
}
*/

float PumpT(float mL){
  t_p = abs(mL)*500;
  return t_p;
}

void PumpM(float mL){
  if (mL < 0.0) {
    digitalWrite(IN1P, HIGH);
    digitalWrite(IN2P, LOW);
    delay(PumpT(mL));
    digitalWrite(IN1P, LOW);
    digitalWrite(IN2P, LOW);
  } else if (mL > 0.0) {
    digitalWrite(IN1P, LOW);
    digitalWrite(IN2P, HIGH);
    delay(PumpT(mL));
    digitalWrite(IN1P, LOW);
    digitalWrite(IN2P, LOW);
  } else {
    digitalWrite(IN1P, LOW);
    digitalWrite(IN2P, LOW);
  }
}
