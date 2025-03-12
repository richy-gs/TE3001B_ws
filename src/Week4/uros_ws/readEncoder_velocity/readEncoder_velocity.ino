#define ENCODER_A 19
#define ENCODER_B 21

// Definir parámetros del encoder
#define PULSOS_POR_REV 240  // Cambiar según el encoder (ejemplo: 1000 pulsos/rev)

volatile long motorPosition = 0;
unsigned long lastTime = 0;
long lastPosition = 0;

void updateMotorPosition() {
  if (digitalRead(ENCODER_B) != digitalRead(ENCODER_A)) {
    motorPosition++;
  } else {
    motorPosition--;
  }
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime; // Tiempo en milisegundos

  if (deltaTime >= 10) {  // Medir cada 10 ms
    long deltaPosition = motorPosition - lastPosition;

    // Calcular la velocidad en pulsos por segundo
    float pulsePerSec = (deltaPosition * 1000.0) / deltaTime; 

    // Calcular velocidad en RPM
    float rpm = (pulsePerSec / PULSOS_POR_REV) * 60.0;
  
    // Calcular velocidad angular en rad/s (1 rev = 2π radianes)
    float radPerSec = rpm * (2.0 * PI) / 60.0;

    // Imprimir datos
    Serial.print("Pulsos: ");
    Serial.print(motorPosition);
    Serial.print(" | Pulsos/s: ");
    Serial.print(pulsePerSec);
    Serial.print(" | Velocidad: ");
    Serial.print(rpm);
    Serial.print(" RPM | ");
    Serial.print(radPerSec);
    Serial.println(" rad/s");

    // Actualizar valores previos
    lastPosition = motorPosition;
    lastTime = currentTime;
  }
}
