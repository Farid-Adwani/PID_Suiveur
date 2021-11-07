#include <Arduino.h>

//il faut installer la librairies de QTR
#include <QTRSensors.h>


QTRSensors qtr;
// Nombre des capteurs
const uint8_t SensorCount = 8;

//tableau des valeurs des capteurs
uint16_t sensorValues[SensorCount];

//les coefficients de PID
float Kp = 0; 
float Ki = 0;
float Kd = 0;
//les variables relatives a l'erreur calculé
int P;
int I;
int D;

int lastError = 0;


//Maximum vitesse
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
// vitesse de base : initiale
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

//Déclaration des pins des moteurs

int moteurDroitePhase = 9;
int moteurDroitePWM = 6;
int moteurGauchePhase = 5;
int moteurGauchePWM = 3;


void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  //Configuration des pins de QTR
  qtr.setSensorPins((const uint8_t[]){10, 11, A0, A1, A2, A3, A4, A5}, SensorCount);
  //LEDON PIN
  qtr.setEmitterPin(7);

//Configuration des pins des moteurs
  pinMode(moteurDroitePhase, OUTPUT);
  pinMode(moteurDroitePWM, OUTPUT);
  pinMode(moteurGauchePhase, OUTPUT);
  pinMode(moteurGauchePWM, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

//Calibrage des capteurs
calibration(); 

forward(0, 0);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {


  PID_control();


}
void forward(int pwmD, int pwmG) {
  //if faut régler les valeur de phase des moteurs pour un forward parfaite
  analogWrite(moteurDroitePhase, 0 );
  analogWrite(moteurGauchePhase, 0 );
  analogWrite(moteurDroitePWM, pwmD);
  analogWrite(moteurGauchePWM, pwmG);
}


void PID_control() {
  //lecture de la position de ligne noire ( entre 0 et 7000 )
  uint16_t position = qtr.readLineBlack(sensorValues);
  //calcul d'erreur
  int error = 3500 - position;

  P = error;
  I = I + error; // Accumulatoin d'erreur
  D = error - lastError; // Dérivée => différence 
  // Mise à jour de dernier erreur
  lastError = error;
  // calcul de vitesse de sortie => application de PID
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  // Calcul des nouveau valeurs de pwm 
  int motorSpeedDroite = basespeeda + motorspeed;
  int motorSpeedGauche = basespeedb - motorspeed;
  
  // Saturation 

  if (motorSpeedDroite > maxspeeda) {
    motorSpeedDroite = maxspeeda;
  }
  if (motorSpeedGauche > maxspeedb) {
    motorSpeedGauche = maxspeedb;
  }
  if (motorSpeedDroite < 0) {
    motorSpeedDroite = 0;
  }
  if (motorSpeedGauche < 0) {
    motorSpeedGauche = 0;
  } 

  //Serial.print(motorSpeedDroite);Serial.print(" ");Serial.println(motorSpeedGauche);

  //Application de correction aux moteurs
  forward(motorSpeedDroite, motorSpeedGauche);
}
