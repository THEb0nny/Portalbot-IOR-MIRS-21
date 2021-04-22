#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MeOrion.h>
#include <AccelStepper.h>
//#include <Servo.h>
#include <math.h>
#include "GyverTimer.h"

#define LIMIT_SWITCH_X_START_PORT PORT_3 // Порт ближних концевиков к моторам
#define LIMIT_SWITCH_X_START_SLOT SLOT_1 // Слот ближних концевиков к моторам

#define LIMIT_SWITCH_X_END_PORT 0 // Порт дальних концевиков от моторов
#define LIMIT_SWITCH_X_END_SLOT 0 // Слот дальних концевиков от моторов

#define LIMIT_SWITCH_Y_START_PORT PORT_4 // Порт концевика для коретки со стороны мотора X
#define LIMIT_SWITCH_Y_START_SLOT SLOT_1 // Слот концевика для коретки со стороны мотора X

#define LIMIT_SWITCH_Y_END_PORT 0 // Порт концевика для коретки со стороны мотора Y
#define LIMIT_SWITCH_Y_END_SLOT 0 // Слот концевика для коретки со стороны мотора Y

// Не работает концевик на 8, 7 в слоте 1
MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT);
MeLimitSwitch yStartlimitSwitch(LIMIT_SWITCH_Y_START_PORT, LIMIT_SWITCH_Y_START_SLOT);
//MePort xStartlimitSwitch(LIMIT_SWITCH_X_START);

// Серво инструмента
#define SERVO_Z_PIN 0 // Серво для перемещения по Z инструмента
#define SERVO_Z2_PIN 0 // Дополнительный серво по Z у инструмента

//Servo servoZ;

// Шаговые двигатели X, Y
#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2

#define STEPPERS_MAX_SPEED 5000 // Максимальная скорость
#define STEPPERS_ACCEL 20000 // Ускорение
#define STEP_TO_ROTATION 400 // Шагов за оборот - 360 градусов
#define DEG_PER_STEP 360 / STEP_TO_ROTATION // Градусы за шаг - 0.9
#define DIST_MM_PER_STEP (1 / (PI * 4)) / STEP_TO_ROTATION // Дистанция в мм за прохождение 1 шага - 31.8471

AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

GTimer_ms myTimer1(10);

#define CAM_SERIAL_RX 0
#define CAM_SERIAL_TX 0

SoftwareSerial camSerial(CAM_SERIAL_RX, CAM_SERIAL_TX); // Сериал для общения с камерой 

// Как нужно скомплектовать коробку 
const String boxCompleteSolve[3][3] = {
  {"RC", "BC", "GC"},
  {"RCC", "BCC", "GCC"},
  {"RB", "BB", "GB"}
}; // RC - красный куб, RCC - красный куб с выемкой, RB - красный шар

// Переменые для хранения фигур после определения камерой
String tStorage[3] = {"N", "N", "N"};
String bStorage[3] = {"N", "N", "N"};
String lStorage[3] = {"N", "N", "N"};
String rStorage[3] = {"N", "N", "N"};

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
//https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/

void setup() {
  Serial.begin(9600);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). Скорость по умолчанию очень низкая, так что её требуется переопределить. При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  stepperY.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperY.setAcceleration(STEPPERS_ACCEL);
  //servoZ.attach(SERVO_Z_PIN);
}

void loop() {
  // Сдвинутся по X и Y в нулевую точку и по сигналу с концевика остановиться, обнулить позицию, зажечь свет
  // Получить данные с камеры
  if (myTimer1.isReady()) {
    //int flag1 = !xStartlimitSwitch.dpRead1(); //xStartlimitSwitch.dpRead2(); x2limitSwitchB.touched();
    //int flag = xStartlimitSwitch.touched();
    //Serial.print(flag);
  }
  searchStartPos(); // Вернуться на базу и установить 0-е позиции
  while(true) { delay(100); } // Конец выполнения
  Serial.println();
}

void searchStartPos() {
  //stepperX.moveTo(800);
  //stepperY.moveTo(-800);
  //stepperX.run();
  //stepperY.run();
  //stepperX.setSpeed(STEPPERS_MAX_SPEED);
  //stepperY.setSpeed(100);
  while (!yStartlimitSwitch.touched()) { // По y сместиться в крайнюю позицию
    stepperX.setSpeed(STEPPERS_MAX_SPEED);
    stepperY.setSpeed(STEPPERS_MAX_SPEED);
    stepperX.runSpeed();
    stepperY.runSpeed();
  }
  while (!xStartlimitSwitch.touched()) { // По x сместиться в крайнюю позицию
    stepperX.setSpeed(STEPPERS_MAX_SPEED);
    stepperY.setSpeed(-STEPPERS_MAX_SPEED);
    stepperX.runSpeed();
    stepperY.runSpeed();
  }
  // Установить позиции 0, 0
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  Serial.println(stepperX.currentPosition());
  Serial.println(stepperY.currentPosition());
}

float x, y, lx, ly;

void FK_CoreXY(float lx, float ly) { // void FK_CoreXY(long l1, long l2, float &x, float &y)
  lx *= DIST_MM_PER_STEP;
  ly *= DIST_MM_PER_STEP;
  //x = (float)(lx + ly) / 2.0;
  //y = x - (float)ly;
}

void IK_CoreXY(float x, float y) { // void IK_CoreXY(float x, float y, long &l1, long &l2)
  lx = floor((x + y) / DIST_MM_PER_STEP);
  ly = floor((x - y) / DIST_MM_PER_STEP);
}

void moveToolZ() {
  
}

// Управление из Serial
void manualControl() {
  while (true) {
    String command = Serial.readStringUntil('\n'); // Считываем из Serial строку до символа переноса на новую строку
    command.trim(); // Чистим символы
    if (Serial.available() > 0) { // Если есть доступные данные
      // Считываем x и y разделённых пробелом
      float xVal = getValue(command, " ", 0).toFloat();
      float yVal = getValue(command, " ", 1).toFloat();
    }
  }
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
