#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MeOrion.h>
#include <AccelStepper.h>
#include <math.h>
#include "GyverTimer.h"

#define LIMIT_SWITCH_X_A 3 // Дальние концевики от моторов
#define LIMIT_SWITCH_X_B 4 // Ближние концевики к моторам
#define LIMIT_SWITCH_Y_A 5 // Концевик для коретки со стороны  мотора X
#define LIMIT_SWITCH_Y_B 6 // Концевик для коретки со стороны мотора Y

#define SERVO_Z 1 // Серво для перемещения по Z инструмента
#define SERVO_Z2 2 // Дополнительный серво по Z у инструмента

#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2

#define STEPPERS_MAX_SPEED 1000
#define STEPPERS_ACCEL 20000
#define DEG_PER_STAPE 0.45

AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_DIR_PIN, STEPPER_X_STP_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_DIR_PIN, STEPPER_Y_STP_PIN);

GTimer_ms myTimer1(10);

const char *needSolve[3][3] = {
  {"RC", "BC", "GC"},
  {"RCC", "BCC", "GCC"},
  {"RB", "BB", "GB"}
}; // RC - красный куб, RCC - красный куб с выемкой, RB - красный шар

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

void setup() {
  Serial.begin(9600);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). Скорость по умолчанию очень низкая, так что её требуется переопределить. При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  stepperY.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperY.setAcceleration(STEPPERS_ACCEL);
}

void loop() {
  // Сдвинутся по X и Y в нулевую точку и по сигналу с концевика остановиться, обнулить позицию, зажечь свет
  // Получить данные с камеры
  if (myTimer1.isReady()) {
    //
  }
}

void moveCoreXY(int x, int y) {
  
}

void moveToolZ() {
  
}
