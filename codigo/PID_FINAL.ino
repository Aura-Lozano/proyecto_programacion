#include <Wire.h>

const int pinMotorLeft = 4;   // ESC LANRC (La "perezosa")
const int pinMotorRight = 15; // ESC Genérica (La "sensible")

const int freq = 50;
const int resolution = 16; 

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Total_angle[2]; // [1] es Pitch

unsigned long timeCurr, timePrev; 
float elapsedTime;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0, pid_i=0, pid_d=0;

// ==========================================
// === ZONA DE AJUSTE PARA ESCs DIFERENTES ===
// ==========================================

// 1. GASOLINA EXTRA PARA LA LANRC (OFFSET)
// Si el motor 4 no reacciona bien, SÚBELE este número.
// Esto hace que su "cero" esté más alto (ej. 1230 en vez de 1150).
int offset_left = 150;  // Prueba con 60, 80, 100 hasta que suene igual al otro

// 2. DIRECCIÓN DE REACCIÓN (IMPORTANTE)
// Si bajas el lado izquierdo con la mano:
//  - Si el motor izquierdo SE CALLA -> Pon -1
//  - Si el motor izquierdo ACELERA -> Pon 1
int direction = 1; 

// 3. THROTTLE BASE
double throttle = 1150; 

// 4. CONSTANTES PID
double kp = 3.0; 
double ki = 0.02;
double kd = 1.0; 

float desired_angle = 0;

void writeESC(int pin, int microseconds) {
  int duty = (microseconds * 65535) / 20000; 
  ledcWrite(pin, duty); 
}

void setup() {
  Wire.begin(21, 22);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  
  Serial.begin(115200);

  ledcAttach(pinMotorLeft, freq, resolution);
  ledcAttach(pinMotorRight, freq, resolution);

  // --- ARMADO DE SEGURIDAD ---
  // IMPORTANTE: Ya no calibramos aquí para no confundir a las ESCs diferentes.
  // Solo armamos con 1000us.
  
  Serial.println("ARMANDO MOTORES (4 seg)...");
  writeESC(pinMotorLeft, 1000);
  writeESC(pinMotorRight, 1000);
  delay(4000); 
  
  Serial.println("¡PID ACTIVO!");
  timeCurr = millis();
}

void loop() {
  if (millis() - timeCurr < 10) return;
  
  timePrev = timeCurr;
  timeCurr = millis();
  elapsedTime = (timeCurr - timePrev) / 1000.0;
  if(elapsedTime == 0) elapsedTime = 0.001;

  // Lectura IMU
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  Acc_rawX=Wire.read()<<8|Wire.read();
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();
  
  // Calculo de Angulo
  float angle_pitch = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*57.296;

  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); 
  Gyr_rawX=Wire.read()<<8|Wire.read();
  Gyr_rawY=Wire.read()<<8|Wire.read();
  
  float Gyro_rate = Gyr_rawY/131.0; 
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_rate*elapsedTime) + 0.02*angle_pitch;
  
  // PID
  error = Total_angle[1] - desired_angle;
  pid_p = kp*error;
  if(abs(error) < 10) pid_i += ki*error; else pid_i = 0;
  pid_d = kd*((error - previous_error)/elapsedTime);
  PID = pid_p + pid_i + pid_d;
  
  if(PID < -400) PID=-400; if(PID > 400) PID=400;

  // ==========================================
  // === MEZCLA DE MOTORES CORREGIDA ===
  // ==========================================

  // 1. Calculamos la base con el PID
  // Usamos 'direction' para poder invertir la lógica fácil si hace falta
  pwmLeft = throttle + (PID * direction);
  pwmRight = throttle - (PID * direction);

  // 2. APLICAMOS LA CURA PARA LA ESC LANRC (Pin 4)
  // Le sumamos el offset SIEMPRE para sacarla de su "zona perezosa"
  pwmLeft = pwmLeft + offset_left;

  // 3. Límites de seguridad (Expandidos un poco)
  if(pwmRight < 1050) pwmRight=1050; if(pwmRight > 1950) pwmRight=1950;
  if(pwmLeft < 1050) pwmLeft=1050;   if(pwmLeft > 1950) pwmLeft=1950;

  // 4. Apagado si se vuelca
  if(abs(Total_angle[1]) > 55) { pwmLeft=1000; pwmRight=1000; }

  writeESC(pinMotorLeft, pwmLeft);
  writeESC(pinMotorRight, pwmRight);
  
  previous_error = error;

  // Debug
  static int printCount = 0;
  printCount++;
  if(printCount > 20) {
    Serial.print("Ang:"); Serial.print(Total_angle[1]);
    Serial.print(" | PID:"); Serial.print(PID);
    Serial.print(" | L(4):"); Serial.print(pwmLeft);
    Serial.print(" | R(15):"); Serial.println(pwmRight);
    printCount = 0;
  }
}