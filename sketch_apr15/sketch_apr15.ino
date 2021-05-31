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

#define LIMIT_SWITCH_X_START_PORT PORT_3 // Порт ближних концевиков к моторам по Y
#define LIMIT_SWITCH_X_START_SLOT SLOT_1 // Слот ближних концевиков к моторам по Y

#define LIMIT_SWITCH_Y_START_PORT PORT_3 // Порт концевика для коретки со стороны мотора X
#define LIMIT_SWITCH_Y_START_SLOT SLOT_2 // Слот концевика для коретки со стороны мотора X

// Серво Z и инструмента
#define SERVO_Z_PIN A2 // Порт серво для перемещения по Z инструмента
#define SERVO_TOOL_PIN A3 // Порт серво инструмента

#define MAX_X_DIST_MM 140 // Максимальная дистанция по X для перемещения в мм
#define MAX_Y_DIST_MM 150 // Максимальная дистанция по Y для перемещения в мм

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

// Номера типов фигур
#define R_BALL_TYPE 0
#define B_BALL_TYPE 1
#define G_BALL_TYPE 2
#define R_CUBE_TYPE -1
#define B_CUBE_TYPE -1
#define G_CUBE_TYPE -1
#define R_CUBE_WITH_RECESS_TYPE -1
#define B_CUBE_WITH_RECESS_TYPE -1
#define G_CUBE_WITH_RECESS_TYPE -1

#define R_POS 3 // Радиус координаты позиции, в котором можно найти фигуры

MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT);
MeLimitSwitch yStartlimitSwitch(LIMIT_SWITCH_Y_START_PORT, LIMIT_SWITCH_Y_START_SLOT);

MeBuzzer buzzer(BUZZER_PORT, BUZZER_SLOT); // Пьезопищалка

MeRGBLed led(RGB_PORT, RGB_SLOT); // RGB лента

// Шаговые двигатели
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

Servo servoZ, servoTool; // Серво инструмента

TrackingCamI2C trackingCam; // Камера

GTimer_ms myTimer1(10); // Таймер

// Как нужно скомплектовать коробку
const int boxCompletataSolve[3][3] = {
  {B_BALL_TYPE, G_BALL_TYPE, R_BALL_TYPE},
  {B_CUBE_TYPE, G_CUBE_TYPE, R_CUBE_TYPE},
  {B_CUBE_WITH_RECESS_TYPE, G_CUBE_WITH_RECESS_TYPE, R_CUBE_WITH_RECESS_TYPE}
};

int storage1[3] = {-1, -1, -1};
int storage2[3] = {-1, -1, -1};
int storage3[3] = {-1, -1, -1};
int storage4[3] = {-1, -1, -1};

const int cellsPosX[5] = {10, 35, 70, 100, 135};
const int cellsPosY[5] = {140, 105, 75, 45, 10};

// Координаты хранилищ
const int xCamStorage1[3] = {101, 136, 172}; // Верхнее хранилище
const int yCamStorage1[3] = {52, 51, 52};

const int xCamStorage2[3] = {67, 65, 66}; // Правое
const int yCamStorage2[3] = {87, 122, 156};

const int xCamStorage3[3] = {100, 135, 170}; // Нижнее
const int yCamStorage3[3] = {191, 192, 87};

const int xCamStorage4[3] = {206, 206, 206}; // Левое
const int yCamStorage4[3] = {87, 123, 158};

float x, y, lx, ly; // Глобальные переменные координат для работы с перемещением по X, Y

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
//https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/
//http://wiki.neobot.ru/index.php?title=%D0%A1%D0%B2%D0%B5%D1%82%D0%BE%D0%B4%D0%B8%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BB%D0%B5%D0%BD%D1%82%D0%B0/LED_RGB_Strip-Addressable,_Sealed

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); stepperY.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); stepperY.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  servoZ.attach(SERVO_Z_PIN); // Подключаем серво Z
  servoTool.attach(SERVO_TOOL_PIN); // Подключаем серво инструмента
  controlZ(139); // 130, 0
  buzzer.noTone();
  led.setNumber(RGB_LED_NUM); // Колпчество светодиодов в ленте
  for (int i = 0; i < RGB_LED_NUM; i++) indicator(i, false); // Выключаем все светодиодыs
  trackingCam.init(51, 100000); // cam_id - 1..127, default 51; speed - 100000/400000, cam enables auto detection of master clock
  //delay(5000);
}

void loop() {
  searchStartPos(); // Вернуться на базу и установить 0-е позиции
  //manualControl(2); // Ручное управление
  //searchFromCamObj();
  //mySolve();
  while(true) { delay(100); } // Конец выполнения
}

void mySolve() {
  // Получить данные с камеры
  if (myTimer1.isReady()) {
    
  }
  ////
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
  } while (!yStartlimitSwitch.touched() && !xStartlimitSwitch.touched()); // Пока 2 концевика не сработали
  
  // Включаем светодиоды нулевого положения
  if (yStartlimitSwitch.touched()) indicator(0, true);
  if (xStartlimitSwitch.touched()) indicator(1, true);
  
  stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Установить позиции 0, 0
  Serial.println("x, y = 0, 0");
  //buzzer.tone(255, 500); // Пищим
}

// Прямая задача кинематики для CoreXY
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

// Обратная задача кинематики для CoreXY
int* IK_CoreXY(float x, float y) {
  lx = floor((x + y) / DIST_MM_PER_STEP_X) * -1;
  ly = floor((x - y) / DIST_MM_PER_STEP_Y) * -1;
  int *return_array = new int[2];
  return_array[0] = lx;
  return_array[1] = ly;
  return return_array;
}

void controlZ(short pos) {
  servoZ.write(pos);
}

void controlTool(short pos) {
  servoTool.write(pos);
}

// Управление из Serial
void manualControl(int type) {
  int* motPos = new int[2]; //IK_CoreXY(0, 0);
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
          motPos = IK_CoreXY(xVal, yVal);
        }
      } else if (type == 2) {
        motPos = IK_CoreXY(cellsPosX[xVal], cellsPosY[yVal]);
        Serial.print("cellsPosX: "); Serial.print(cellsPosX[xVal]); Serial.print(", "); Serial.print("cellsPosY: "); Serial.println(cellsPosY[yVal]);
      }
      Serial.print("motPos0: "); Serial.print(motPos[0]); Serial.print(", "); Serial.print("motPos1: "); Serial.println(motPos[1]);
      // Перемещаем
      while (true) { // Перемещаем моторы в позицию
        stepperX.moveTo(motPos[0]); stepperY.moveTo(motPos[1]);
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

// Считываем данные с камеры и записываем
void searchFromCamObj() {
  while (true) {
    uint8_t n = trackingCam.readBlobs(3); // Считать первые 3 блобсы
    Serial.println("All blobs");
    Serial.println(n); // Сообщить о количестве найденных блобсах
    for(int i = 0; i < n; i++) {
      int objType = trackingCam.blob[i].type;
      int objCX = trackingCam.blob[i].cx;
      int objCY = trackingCam.blob[i].cy;
      Serial.print(objType, DEC);
      Serial.print(" ");
      Serial.print(objCX, DEC);
      Serial.print(" ");
      Serial.print(objCY, DEC);
      Serial.println(" ");

      if (pow(objCX - x0, 2) + pow(objCY - y0, 2) <= pow(R_POS, 2)) {
        // Записываем какой объект в координате
      }
    }
  
    // Ждем следующий кадр
    while(millis() - prevMillis < 33) {};
    prevMillis = millis();
  }
}
