#include <AccelStepper.h>

#include <MultiStepper.h>

const int IN1 = 4;
const int IN2 = 5;

float t = 0.0;
float targetCyl = 0.0;
float CylCurPos = 0.0;
float deltaCyl = 0.0;

const int dirPinB = 3;
const int stepPinB = 2;
const int dirPinC = 9;
const int stepPinC = 8;

long GoToPos[2];

// Creates an instance
AccelStepper StepB(1, stepPinB, dirPinB);
AccelStepper StepC(1, stepPinC, dirPinC);

MultiStepper StepCtrl;

String inputString = "";
bool stringComplete = false;

//Parsear q1, q2, q3
float q1 = 0, q2 = 0, q3 = 0;

// ------------------------------
// ACTIVAR/DESACTIVAR PRINTS
// ------------------------------
bool DEBUG = true;    //Se cambia a true para activar serial prints en la consola de python (solo para Debug)

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  //Inicializar motores, cilindro y definir parametros
  StepB.setMinPulseWidth(3);
  StepC.setMinPulseWidth(3);

  StepB.setMaxSpeed(300);
  StepC.setMaxSpeed(300);
  
  //Unir ambos steppers para control multiple
  StepCtrl.addStepper(StepB);
  StepCtrl.addStepper(StepC);

  //Se asume que usuaría situa los motores en Home (Luego cambiar con FdC)
  StepB.setCurrentPosition(0.0);
  StepC.setCurrentPosition(0.0);

  inputString.reserve(40);

  if (DEBUG) Serial.println("Iniciando Arduino (DEBUG ON)");
}

void loop() {
  //Debug, imprime en el monitor serial (python) el valor de los ángulos recibidos
  //únicamente para debug y revisiones de comunicación efectiva, porque los serial prints
  //introducen pausas que podrían hacer que el robot tenga menor fluidez
  if (stringComplete) {
    
    if (DEBUG) {
      Serial.print("Recibido crudo: ");
      Serial.println(inputString);
    }

    int c1 = inputString.indexOf(',');
    int c2 = inputString.indexOf(',', c1 + 1);

    if (c1 > 0 && c2 > 0) {
      q1 = inputString.substring(0, c1).toFloat();
      q2 = inputString.substring(c1 + 1, c2).toFloat();
      q3 = inputString.substring(c2 + 1).toFloat();
    }

    if (DEBUG) {
      Serial.print("Parseado -> q1: "); Serial.print(q1);
      Serial.print(" | q2: "); Serial.print(q2);
      Serial.print(" | q3: "); Serial.println(q3);
    }


    // Mover los servos según los ángulos recibidos
    // servo1.write(q1);
    // servo2.write(q2);
    // servo3.write(q3);

    targetCyl = CylMoveDist(q3);
    deltaCyl = CylCurPos - targetCyl;
    GoToPos[0] = -1*q1*(20/9)*5; //Grados por reducción de microstepping entre steps/grado = 95*4/1.8, 4/1.8=20/9
    GoToPos[1] = q2*(20/9)*5.5;
    
    StepCtrl.moveTo(GoToPos);
    StepCtrl.runSpeedToPosition();
    
    if(deltaCyl < 0.0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      delay(abs(deltaCyl));
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
    else if(deltaCyl > 0.0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      delay(abs(deltaCyl));
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    } 
    else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    CylCurPos = targetCyl;

    //Limpiar buffer
    inputString = "";
    stringComplete = false;

    //Responder a Python
    Serial.println("OK");    //NO QUITAR, es parte del protocolo
  }
}

//Comunicación serial
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

float CylMoveDist(float dist){
  t = (20000.0/196977.0)*dist*1000;
  return t;
}
