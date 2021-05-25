#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MeOrion.h>
#include <AccelStepper.h>
#include <math.h>
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
#define SERVO_Z_PIN 8 // Порт серво для перемещения по Z инструмента
#define SERVO_Z2_PIN 0 // Дополнительный серво по Z у инструмента

#define MAX_X_DIST_MM 140 // Максимальная дистанция по X для перемещения в мм
#define MAX_Y_DIST_MM 150 // Максимальная дистанция по Y для перемещения в мм
#define DIST_TO_CENTER_CARRIAGE 30 // Расстояние до центра корретки в мм

#define BUZZER_PORT PORT_4 // Порт пьезопищалки
#define BUZZER_SLOT SLOT_2 // Слот пьезопищалки, работает только во втором

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

Servo servoZ, servoZ2; // Серво инструмента

GTimer_ms myTimer1(10); // Таймер

TrackingCamI2C trackingCam; // Камера

unsigned long previousMillis = 0; // stores last time cam was updated

// Как нужно скомплектовать коробку 
const String boxCompletateSolve[3][3] = {
  {"RC", "BC", "GC"},
  {"RCC", "BCC", "GCC"},
  {"RB", "BB", "GB"}
}; // RC - красный куб, RCC - красный куб с выемкой, RB - красный шар

String storage[4][3] = { // Хранилище
  {"N", "N", "N"}, // Выерхний
  {"N", "N", "N"}, // Левый
  {"N", "N", "N"}, // Правый
  {"N", "N", "N"} // Нижний
};

const int cellsPosX[5][5] = { // Переменые для хранения фигур после определения камерой по X
  {-1, 35, 70, 100, -1},
  {10, 40, 75, 105, 135},
  {10, 40, 70, 105, 135},
  {10, 40, 70, 100, 130},
  {-1, 40, 70, 100, -1}
}; // Крайние пункты должны быть -1

const int cellsPosY[5][5] = { // Переменые для хранения фигур после определения камерой по Y
  {-1, 140, 140, 140, -1},
  {105, 105, 105, 105, 105},
  {75, 75, 75, 75, 75},
  {45, 45, 45, 45, 45},
  {-1, 10, 10, 10, -1}
}; // Крайние пункты должны быть -1

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
//https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/
//http://wiki.neobot.ru/index.php?title=%D0%A1%D0%B2%D0%B5%D1%82%D0%BE%D0%B4%D0%B8%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BB%D0%B5%D0%BD%D1%82%D0%B0/LED_RGB_Strip-Addressable,_Sealed

void setup() {
  Serial.begin(9600);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); stepperY.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); stepperY.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  servoZ.attach(SERVO_Z_PIN);
  servoZ.write(0);
  /* TrackingCamI2C::init(uint8_t cam_id, uint32_t speed);
   *   cam_id - 1..127, default 51
   *   speed - 100000/400000, cam enables auto detection of master clock 
   */
  //trackingCam.init(51, 100000);
  //delay(5000);
  buzzer.noTone();
  led.setNumber(RGB_LED_NUM); // Колпчество светодиодов в ленте
  for (int i = 0; i < RGB_LED_NUM; i++) indicator(i, false); // Выключаем все светодиоды
}

void loop() {
  searchStartPos(); // Вернуться на базу и установить 0-е позиции
  manualControl(2); // Ручное управление
  //servoZ.write(180);
  //mySolve();
  while(true) { delay(100); } // Конец выполнения
}

void indicator(short i, bool state) {
  if (state) led.setColorAt(i, 255, 0, 0); // Включаем красным выбранный
  else led.setColorAt(i, 0, 0, 1); // Ставим синим выбранный
  led.show();
}

void mySolve() {
  // Получить данные с камеры
  if (myTimer1.isReady()) {
    
  }
  Serial.println();
}

void searchStartPos() {
  do {
    while (!yStartlimitSwitch.touched()) { // По y сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    if (yStartlimitSwitch.touched()) indicator(0, true); // Включаем светодиоды нулевого положения
    else indicator(0, false);
    
    while (!xStartlimitSwitch.touched()) { // По x сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(-STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    if (xStartlimitSwitch.touched()) indicator(1, true);
    else indicator(1, false);
  } while (!yStartlimitSwitch.touched() && !xStartlimitSwitch.touched()); // Пока концевики не сработали
  
  // Включаем светодиоды нулевого положения
  if (yStartlimitSwitch.touched()) indicator(0, true);
  if (xStartlimitSwitch.touched()) indicator(1, true);
  
  stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Установить позиции 0, 0
  Serial.println("x, y = 0, 0");
  //buzzer.tone(255, 500); // Пищим
}

float x, y, lx, ly;

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
int* IK_CoreXY(float x, float y) { // void IK_CoreXY(float x, float y, long &l1, long &l2)
  lx = floor((x + y) / DIST_MM_PER_STEP_X) * -1;
  ly = floor((x - y) / DIST_MM_PER_STEP_Y) * -1;
  int *return_array = new int[2];
  return_array[0] = lx;
  return_array[1] = ly;
  return return_array;
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
          Serial.print("x: "); Serial.print(pos[0]); Serial.print(", "); Serial.print("y: "); Serial.println(pos[1]);
        }
      } else if (type == 2) {  
        if (xVal >= 0 && xVal <= 4 && yVal >= 0 && yVal <= 4 || xVal != 0 && yVal != 0 || xVal != 4 && yVal != 0 || xVal != 0 && yVal != 4 || xVal != 4 && yVal != 4) {
          pos = IK_CoreXY(cellsPosX[xVal][yVal], cellsPosY[xVal][yVal]);
          Serial.print("iCell: "); Serial.print(pos[0]); Serial.print(", "); Serial.print("jCell: "); Serial.println(pos[1]);
        }
      }
      
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
      if (xVal == 0 && yVal == 0) { // Если позиция была указана 0, 0 то по окончанию обновить стартовую позицию
        stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0);
        indicator(0, true);
        indicator(1, true);
        buzzer.tone(255, 500); // Пищим
      }
    }
  }
}

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
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].area, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].left, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].right, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].top, DEC);
      Serial.print(" ");
      Serial.print(trackingCam.blob[i].bottom, DEC);
      Serial.println(" ");
    }
  
    // wait for the next frame
    while(millis() - previousMillis < 33) {};
    previousMillis = millis();
  }
}
