/** Arduino I2C blobs example.
 * Settings: Blob detector, I2C, addr 51, Dynamixel API, 5V.
 * Wiring:
 *       Camera         Arduino Camera
 * 1-VF >|O O|  2-+5      SCL  -  IC0
 * 3-Gnd |O O|  4-Gnd     SDA  -  ID1
 * 5-TX   O O|  6-RX      5V   -  +5
 * 7-SCK |O O|  8-SNS     Gnd  -  Gnd
 * 9-IC0 |O O| 10-ID1     
*/

#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MeOrion.h>
#include <AccelStepper.h>
#include "TrackingCamI2C.h"
#include "GyverTimer.h"

#define LIMIT_SWITCH_X_START_PORT PORT_3 // Порт ближних концевиков к моторам
#define LIMIT_SWITCH_X_START_SLOT SLOT_1 // Слот ближних концевиков к моторам

#define LIMIT_SWITCH_X_END_PORT 0 // Порт дальних концевиков от моторов
#define LIMIT_SWITCH_X_END_SLOT 0 // Слот дальних концевиков от моторов

#define LIMIT_SWITCH_Y_START_PORT PORT_3 // Порт концевика для коретки со стороны мотора X
#define LIMIT_SWITCH_Y_START_SLOT SLOT_2 // Слот концевика для коретки со стороны мотора X

#define LIMIT_SWITCH_Y_END_PORT 0 // Порт концевика для коретки со стороны мотора Y
#define LIMIT_SWITCH_Y_END_SLOT 0 // Слот концевика для коретки со стороны мотора Y

// Серво инструмента
#define SERVO_Z_PIN A2 // Порт серво для перемещения по Z инструмента
#define SERVO_TOOL_PIN A3 // Дополнительный серво по Z у инструмента

#define MAX_X_DIST_MM 140 // Максимальная дистанция по X для перемещения в мм
#define MAX_Y_DIST_MM 150 // Максимальная дистанция по Y для перемещения в мм
#define DIST_TO_CENTER_CARRIAGE 30 // Расстояние до центра корретки в мм

#define BUZZER_PORT PORT_4 // Порт пьезопищалки
#define BUZZER_SLOT SLOT_1 // Слот пьезопищалки, работает только во втором

#define RGB_PORT PORT_4 // Порт RGB ленты
#define RGB_SLOT SLOT_2 // Слот RGB ленты, работает только во втором
#define RGB_LED_NUM 4 // Количество светодиодов в ленте

// Шаговые двигатели X, Y
#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2

#define STEPPERS_MAX_SPEED 5000 // Максимальная скорость шагового двигателя
#define STEPPERS_ACCEL 15000 // Ускорение шагового двигателя
#define STEP_TO_ROTATION 400 // Шагов за оборот - 360 градусов
#define DIST_MM_PER_STEP_X 0.04 // Дистанция в мм за прохождение 1 шага мотора X
#define DIST_MM_PER_STEP_Y 0.04 // Дистанция в мм за прохождение 1 шага мотора X

MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT);
MeLimitSwitch yStartlimitSwitch(LIMIT_SWITCH_Y_START_PORT, LIMIT_SWITCH_Y_START_SLOT);
//MePort xStartlimitSwitch(LIMIT_SWITCH_X_START); // !Концевик не работает на 8, 7 в слоте 1, а только в слоте 2

MeBuzzer buzzer(BUZZER_PORT, BUZZER_SLOT); // Пьезопищалка

MeRGBLed led(RGB_PORT, RGB_SLOT); // RGB лента

// Шаговые двигатели
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

Servo servoZ, servoTool; // Серво инструмента

GTimer_ms myTimer1(10); // Таймер

TrackingCamI2C trackingCam; // Камера

float x, y, lx, ly;

// 10 - синий шар, 11 - зелёный, 12 - красный шар
// 20 - синий куб, 21 - зелёный, 22 - красный куб
// 30 - синий куб с выемкой, 31 - зелёный с выемкой, 32 - красный с выемкой

// Как нужно скомплектовать коробку
const short boxCompletataSolve[3][3] = {
  {10, 11, 12},
  {20, 21, 22},
  {30, 31, 32}
};

int storage1[3] = {-1, -1, -1};
int storage2[3] = {-1, -1, -1};
int storage3[3] = {-1, -1, -1};
int storage4[3] = {-1, -1, -1};

const int cellsPosX[5] = {10, 35, 70, 100, 135};
const int cellsPosY[5] = {140, 105, 75, 45, 10};
/*
/*const int cellsPosX[5][5] = { // Переменые для хранения фигур после определения камерой по X
  {-1, 35, 70, 100, -1},
  {10, 40, 75, 105, 135},
  {10, 40, 70, 105, 135},
  {10, 40, 70, 100, 130},
  {-1, 40, 70, 100, -1}
}; // Крайние пункты должны быть -1*/
/*
const int cellsPosY[5][5] = { // Переменые для хранения фигур после определения камерой по Y
  {-1, 140, 140, 140, -1},
  {105, 105, 105, 105, 105},
  {75, 75, 75, 75, 75},
  {45, 45, 45, 45, 45},
  {-1, 10, 10, 10, -1}
}; // Крайние пункты должны быть -1*/

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
//https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/
//http://wiki.neobot.ru/index.php?title=%D0%A1%D0%B2%D0%B5%D1%82%D0%BE%D0%B4%D0%B8%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BB%D0%B5%D0%BD%D1%82%D0%B0/LED_RGB_Strip-Addressable,_Sealed

void setup() {
  Serial.begin(115200);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); stepperY.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); stepperY.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  //servoZ.attach(SERVO_Z_PIN);
  //servoZ.write(139); // 130, 0
  
  buzzer.noTone();
  led.setNumber(RGB_LED_NUM); // Колпчество светодиодов в ленте
  for (int i = 0; i < RGB_LED_NUM; i++) indicator(i, false); // Выключаем все светодиодыs
  trackingCam.init(51, 100000); //cam_id - 1..127, default 51; speed - 100000/400000, cam enables auto detection of master clock
  //delay(5000);
}

void loop() {
  //camRead();
  searchStartPos(); // Вернуться на базу и установить 0-е позиции
  manualControl(2); // Ручное управление
  //mySolve();
  while(true) { delay(100); } // Конец выполнения
}

void mySolve() {
  // Получить данные с камеры
  if (myTimer1.isReady()) {
    
  }
  buzzer.tone(255, 2000); // Пищим о завершении
  Serial.println();
}

void indicator(short i, bool state) {
  if (state) led.setColorAt(i, 255, 0, 0); // Включаем красным выбранный
  else led.setColorAt(i, 0, 0, 1); // Ставим синим выбранный
  led.show();
}

void searchStartPos() { // Возвращение (поиск) на домашнюю позициию
  do {
    while (!yStartlimitSwitch.touched()) { // По y сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    if (yStartlimitSwitch.touched()) indicator(0, true); // Включаем светодиоды нулевого положения
    else indicator(0, false); // Иначе выключаем
    
    while (!xStartlimitSwitch.touched()) { // По x сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(-STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    if (xStartlimitSwitch.touched()) indicator(1, true); // Включаем светодиоды нулевого положения
    else indicator(1, false); // Иначе выключаем
  } while (!yStartlimitSwitch.touched() && !xStartlimitSwitch.touched()); // Пока концевики не сработали
  
  // Включаем светодиоды нулевого положения
  if (yStartlimitSwitch.touched()) indicator(0, true);
  if (xStartlimitSwitch.touched()) indicator(1, true);
  
  stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Установить позиции 0, 0
  Serial.println("x, y = 0, 0");
  //buzzer.tone(255, 500); // Пищим
}

// Прямая задача кинематики
int* FK_CoreXY(float lx, float ly) { // void FK_CoreXY(long l1, long l2, float &x, float &y)
  lx *= DIST_MM_PER_STEP_X;
  ly *= DIST_MM_PER_STEP_Y;
  x = (float)(lx + ly) / 2.0;
  y = x - (float)ly;
  int *return_array = new int[2];
  return_array[0] = x;
  return_array[1] = y;
  return return_array;
}

// Обратная задача кинематики
int* IK_CoreXY(float x, float y) {
  lx = floor((x + y) / DIST_MM_PER_STEP_X) * -1;
  ly = floor((x - y) / DIST_MM_PER_STEP_Y) * -1;
  int *return_array = new int[2];
  return_array[0] = lx;
  return_array[1] = ly;
  return return_array;
}

void controlZ(short pos) {
  //servoZ.write(pos);
}

void controlTool() {
  
}

// Управление из Serial
void manualControl(int type) {
  int* pos = IK_CoreXY(0, 0);
  while (true) {
    String command = Serial.readStringUntil('\n'); // Считываем из Serial строку до символа переноса на новую строку
    command.trim(); // Чистим символы
    if (command.length() > 0) { // Если есть доступные данные
      char strBuffer[11] = {};
      command.toCharArray(strBuffer, 11);

      // Считываем x и y разделённых пробелом
      int xVal = atoi(strtok(strBuffer, " "));
      int yVal = atoi(strtok(NULL, " "));
      Serial.print("xVal: "); Serial.print(xVal); Serial.print(", "); Serial.print("yVal: "); Serial.println(yVal);
      if (type == 1) {
        if (xVal <= MAX_X_DIST_MM && xVal >= 0 && yVal <= MAX_Y_DIST_MM && yVal >= 0) {
          pos = IK_CoreXY(xVal, yVal);
        }
      } else if (type == 2) {
        pos = IK_CoreXY(cellsPosX[xVal], cellsPosY[yVal]);
        Serial.print("cellsPosX: "); Serial.print(cellsPosX[xVal]); Serial.print(", "); Serial.print("cellsPosY: "); Serial.println(cellsPosY[yVal]);
      }
      Serial.print("pos0: "); Serial.print(pos[0]); Serial.print(", "); Serial.print("pos1: "); Serial.println(pos[1]);
      // Перемещаем
      while (true) { // Перемещаем моторы в позицию
        stepperX.moveTo(pos[0]); stepperY.moveTo(pos[1]);
        stepperX.run(); stepperY.run();
        // Включаем/выключаем светодиоды нулевого положения
        if (yStartlimitSwitch.touched()) indicator(0, true);
        else indicator(0, false);
        if (xStartlimitSwitch.touched()) indicator(1, true);
        else indicator(1, false);
        ////
        if (!stepperX.isRunning() && !stepperY.isRunning()) break; // Мотор остановился выполнив перемещение
      }
      if (xStartlimitSwitch.touched() && yStartlimitSwitch.touched()) { // Если позиция была указана 0, 0 то по окончанию обновить стартовую позицию
        stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0);
        indicator(0, true); indicator(1, true);
      }
    }
  }
}

unsigned long prevMillis = 0; // stores last time cam was updated

// Считываем данные с камеры
void camRead() {
  while (true) {
    uint8_t n = trackingCam.readBlobs(5); // read data about first 5 blobs
    Serial.println("All blobs");
    Serial.println(n); // print numbers of blobs
    for(int i = 0; i < n; i++) // print information about all blobs
    {
      Serial.print(trackingCam.blob[i].type, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].dummy, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].cx, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].cy, DEC);
      /*Serial.print(" ");
      Serial.print(trackingCam.blob[i].area, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].left, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].right, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].top, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].bottom, DEC);*/
      Serial.println(" ");
    }
  
    // wait for the next frame
    while(millis() - prevMillis < 33) {};
    prevMillis = millis();
  }
}
