#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MeOrion.h>
#include <AccelStepper.h>
#include <math.h>
#include "GyverTimer.h"

#define LIMIT_SWITCH_X_START_PORT PORT_3 // Ближние концевики к моторам
#define LIMIT_SWITCH_X_START_SLOT SLOT_1 // Ближние концевики к моторам

#define LIMIT_SWITCH_X_END_PORT 0 // Дальние концевики от моторов
#define LIMIT_SWITCH_X_END_SLOT 0 // Дальние концевики от моторов

#define LIMIT_SWITCH_Y_START_PORT 0 // Концевик для коретки со стороны мотора X
#define LIMIT_SWITCH_Y_START_SLOT 0 // Концевик для коретки со стороны мотора X

#define LIMIT_SWITCH_Y_END_PORT 0 // Концевик для коретки со стороны мотора Y
#define LIMIT_SWITCH_Y_END_SLOT 0 // Концевик для коретки со стороны мотора Y

MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT); // Не работает концевик на 8, 7 в слоте 1
//MePort xStartlimitSwitch(LIMIT_SWITCH_X_START);

// Серво инструмента
#define SERVO_Z 0 // Серво для перемещения по Z инструмента
#define SERVO_Z2 0 // Дополнительный серво по Z у инструмента

// Шаговые двигатели X, Y
#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2

#define STEPPERS_MAX_SPEED 5000 // Максимальная скорость
#define STEPPERS_ACCEL 20000 // Ускорение
#define DEG_PER_STEP 0.9 // Градусы за шаг
#define STEP_TO_ROTATION 400 // Шагов за оборот

AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

GTimer_ms myTimer1(100);

// Как нужно скомплектовать коробку 
const String boxCompleteSolve[3][3] = {
  {"RC", "BC", "GC"},
  {"RCC", "BCC", "GCC"},
  {"RB", "BB", "GB"}
}; // RC - красный куб, RCC - красный куб с выемкой, RB - красный шар

// Переменые для хранения фигор на старте
String tStorage[3] = {"N", "N", "N"};
String bStorage[3] = {"N", "N", "N"};
String lStorage[3] = {"N", "N", "N"};
String rStorage[3] = {"N", "N", "N"};

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html

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
    //int flag1 = !xStartlimitSwitch.dpRead1(); //xStartlimitSwitch.dpRead2(); x2limitSwitchB.touched();
    //int flag = xStartlimitSwitch.touched();
    //Serial.print(flag);
  }
  while (!xStartlimitSwitch.touched()) {
    Serial.println("work");
    //stepperX.moveTo(800);
    //stepperY.moveTo(-800);
    //stepperX.run();
    //stepperY.run();
    stepperX.setSpeed(STEPPERS_MAX_SPEED);
    //stepperY.setSpeed(100);
    stepperX.runSpeed();
  }
  Serial.println();
}

// Управление из Serial
void manualСontrol() {
  while (true) {
    String command = Serial.readStringUntil('\n'); // Считываем из Serial строку до символа переноса на новую строку
    if (Serial.available() > 0) {  // Если есть доступные данные
      int ledR = command.substring(1).toInt();
    }
  }
}

void moveCoreXY(int x, int y) {
  
}

void moveToolZ() {
  
}
