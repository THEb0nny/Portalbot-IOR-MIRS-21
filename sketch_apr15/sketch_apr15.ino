/** Arduino I2C blobs example.
 * Settings: Blob detector, I2C, addr 51, Dynamixel API, 5V.
 * Wiring:
 *       Camera         Arduino Camera
 * 1-VF >|O O|  2-+5      SCL  -  IC0
 * 3-Gnd |O O|  4-Gnd     SDA  -  ID1
 * 5-TX   O O|  6-RX      5V   -  +5
 * 7-SCK |O O|  8-SNS     Gnd  -  Gnd
 * 9-IC0 |O O| 10-ID1     
**/

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
#define MAX_Y_DIST_MM 145 // Максимальная дистанция по Y для перемещения в мм

#define BUZZER_PORT PORT_4 // Порт пьезопищалки
#define BUZZER_SLOT SLOT_1 // Слот пьезопищалки, работает только во втором

// Порты подключнния шаговых двигателей
#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2

#define STEPPERS_MAX_SPEED 2000 // Максимальная скорость шагового двигателя
#define STEPPERS_ACCEL 15000 // Ускорение шагового двигателя
#define STEP_TO_ROTATION 400 // Шагов за оборот - 360 градусов
#define DIST_MM_PER_STEP_X 0.04 // Дистанция в мм за прохождение 1 шага мотора X
#define DIST_MM_PER_STEP_Y 0.04 // Дистанция в мм за прохождение 1 шага мотора X

// Номера типов фигур по настройкам камеры
#define R_BALL_TYPE 0
#define G_BALL_TYPE 1
#define B_BALL_TYPE 2
#define R_CUBE_TYPE -1
#define G_CUBE_TYPE -1
#define B_CUBE_TYPE -1
#define R_CUBE_WITH_RECESS_TYPE -1
#define G_CUBE_WITH_RECESS_TYPE -1
#define B_CUBE_WITH_RECESS_TYPE -1

#define R_ZONE_POS 5 // Радиус координаты позиции, в которой можно найти объект
#define XY_CELLS_ARR_LEN 5 // Размер для массивов координат ячеек по X и Y
#define TIME_TO_READ_FROM_CAM 5000 // Время для считываения с камеры

MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT);
MeLimitSwitch yStartlimitSwitch(LIMIT_SWITCH_Y_START_PORT, LIMIT_SWITCH_Y_START_SLOT);

MeBuzzer buzzer(BUZZER_PORT, BUZZER_SLOT); // Пьезопищалка

// Шаговые двигатели
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

Servo servoZ, servoTool; // Серво инструмента

TrackingCamI2C trackingCam; // Камера

GTimer_ms myTimer1(TIME_TO_READ_FROM_CAM); // Таймер

// Массив с правилами растановки объектов в коробку
int boxCompletateSolve[3][3] = {
  {-1, -1, -1},
  {-1, -1, -1},
  {-1, -1, -1}
};

int storages[4][3] = { // Склады
  {-1, -1, -1}, // Склад 1 сверху
  {-1, -1, -1}, // Склад 2 справа
  {-1, -1, -1}, // Склад 3 снизу
  {-1, -1, -1} // Склад 4 слева
};

const int cellsPosX[XY_CELLS_ARR_LEN] = {3, 35, 70, 100, 130}; // Координаты рядов ячеек
const int cellsPosY[XY_CELLS_ARR_LEN] = {145, 110, 70, 40, 10}; // Координаты строк ячеек

// Координаты хранилищ в камере
const int storagesCellsCamPosX[XY_CELLS_ARR_LEN] = {66, 97, 132, 171, 203};
const int storagesCellsCamPosY[XY_CELLS_ARR_LEN] = {27, 62, 96, 132, 166};

float x, y, lx, ly; // Глобальные переменные координат для работы с перемещением по X, Y

// ИНФА
//http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
//http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
//https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/
//http://wiki.neobot.ru/index.php?title=%D0%A1%D0%B2%D0%B5%D1%82%D0%BE%D0%B4%D0%B8%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BB%D0%B5%D0%BD%D1%82%D0%B0/LED_RGB_Strip-Addressable,_Sealed
//https://community.alexgyver.ru/resources/biblioteka-gyvertimer.11/

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); stepperY.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); stepperY.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  servoZ.attach(SERVO_Z_PIN); // Подключаем серво Z
  servoTool.attach(SERVO_TOOL_PIN); // Подключаем серво инструмента
  buzzer.noTone();
  controlZ(180); // 180 - поднято, 0 - опущено
  controlTool(180); // 180 - внутри, 0 - выпущено
  trackingCam.init(51, 100000); // cam_id - 1..127, default 51; speed - 100000/400000, cam enables auto detection of master clock
  //delay(1000);
}

void loop() {
  searchStartPos(); // Вернуться на базу и установить 0-е позиции
  //manualControl(2); // Ручное управление
  moveCoreXY("IK", MAX_X_DIST_MM, MAX_Y_DIST_MM);
  searchFromCamObj(); // Ищем с камеры объекты
  setBoxCompletate(); // Установить массив с итоговой комплектацией
  mySolve(); // Решаем задачу
  searchStartPos(); // Возвращаемся в нулевую точку после выполнения
  buzzer.tone(255, 2000); // Пищим о завершении
  while(true) { delay(100); } // Конец выполнения
}

// Моё решение
void mySolve() {
  for (int i = 0; i < 3; i++) { // Идём по массиву необходимой комплетации по столбцам
    for (int j = 0; j < 3; j++) { // Идём по массиву необходимой комплетации по строкам
      for (int n = 0; n < 4; n++) { //
        for (int m = 0; m < 3; m++) { //
          if (boxCompletateSolve[i][j] == storages[n][m]) { // Совпадаение
            int moveCellPosX, moveCellPosY;
            if (n == 0) {
              Serial.print("С координат "); Serial.print(cellsPosX[m + 1]); Serial.print(", "); Serial.print(cellsPosY[0]); Serial.print(" взять "); Serial.print(storages[n][m]);
              moveCellPosX = cellsPosX[m + 1];
              moveCellPosY = cellsPosY[0];
            } else if (n == 1) {
              Serial.print("С координат "); Serial.print(cellsPosX[4]); Serial.print(", "); Serial.print(cellsPosY[m + 1]); Serial.print(" взять "); Serial.print(storages[n][m]);
              moveCellPosX = cellsPosX[4];
              moveCellPosY = cellsPosY[m + 1];
            } else if (n == 2) {
              Serial.print("С координат "); Serial.print(cellsPosX[m + 1]); Serial.print(", "); Serial.print(cellsPosY[4]); Serial.print(" взять "); Serial.print(storages[n][m]);
              moveCellPosX = cellsPosX[m + 1];
              moveCellPosY = cellsPosY[4];
            } else if (n == 3) {
              Serial.print("С координат "); Serial.print(cellsPosX[0]); Serial.print(", "); Serial.print(cellsPosY[m + 1]); Serial.print(" взять "); Serial.print(storages[n][m]);
              moveCellPosX = cellsPosX[0];
              moveCellPosY = cellsPosY[m + 1];
            }
            moveCoreXY("IK", moveCellPosX, moveCellPosY); // Перемещаемся к найденой
            //
            controlTool(0);
            delay(500);
            controlZ(0);
            delay(1000);
            controlZ(180);
            storages[n][m] = -1; // Удаляем в storages, ставим там пустоту
            //
            delay(2000);
            Serial.print(" и перенести в "); Serial.print(cellsPosX[j + 1]); Serial.print(", "); Serial.println(cellsPosY[i + 1]);
            moveCoreXY("IK", cellsPosX[j + 1], cellsPosY[i + 1]);
            //
            controlZ(0);
            delay(500);
            controlTool(180);
            delay(500);
            controlZ(180);
            //
            delay(2000);
          }
        }
      }
    }
  }
}

unsigned long prevMillis = 0; // stores last time cam was updated

// Считываем данные с камеры и записываем
void searchFromCamObj() {
  myTimer1.reset(); // Сбросить таймер
  do {
    uint8_t n = trackingCam.readBlobs(10); // Считать первые n блобсов
    Serial.print("All blobs ");
    Serial.println(n); // Сообщить о количестве найденных блобсах
    for(int k = 0; k < n; k++) { // Проходимся по всем блобсам с камеры
      int objType = trackingCam.blob[k].type;
      int objCX = trackingCam.blob[k].cx;
      int objCY = trackingCam.blob[k].cy;
      //Serial.print(objType, DEC); Serial.print(" "); Serial.print(objCX, DEC); Serial.print(" "); Serial.print(objCY, DEC); Serial.println(" ");
      for (int i = 0; i < XY_CELLS_ARR_LEN; i++) { // Проходимся по столбцам хранилищ
        int cellCamX = storagesCellsCamPosX[i]; // Координаты хранилищ для камеры по X
        for (int j = 0; j < XY_CELLS_ARR_LEN; j++) { // Проходимся по строкам хранилищ
          // Если смотрим координаты не хранилищ, то пропускаем шаг
          if ((i >= 1 && j >= 1) && (i <= 3 && j <= 3) || (i == 0 && j == 0) || (i == 0 && j == 4) || (i == 4 && j == 0) || (i == 4 && j == 4)) continue;
          int cellCamY = storagesCellsCamPosY[j]; // Координаты хранилищ для камеры по Y
          if (pow(objCX - cellCamX, 2) + pow(objCY - cellCamY, 2) <= pow(R_ZONE_POS, 2)) { // Если объект с координатами центра попадает в область позиций
            // Записываем какой объект в координате в массив для хранилищ, но, если в ячейку склада уже не было записано значение
            Serial.print("Found "); Serial.print(objType, DEC); Serial.print(" "); Serial.print(objCX, DEC); Serial.print(" "); Serial.print(objCY, DEC); Serial.println(); 
            if (j == 0 && storages[0][i - 1] == -1) storages[0][i - 1] = objType; // Если строка первая, то склад 1
            else if (j == 4 && storages[2][i - 1] == -1) storages[2][i - 1] = objType; // Если строка последняя, то склад 3
            else { // Иначе остальные - 1 или 3
              if (i == 0 && storages[3][j - 1] == -1) storages[3][j - 1] = objType; // Если ряд первый - 0, то склад 4
              else if (i == 4 && storages[1][j - 1] == -1) storages[1][j - 1] = objType; // Если ряд последний - 4, то склад 2
            }
          }
        }
      }
    }
    // Ждем следующий кадр
    while(millis() - prevMillis < 33) {};
    prevMillis = millis();
  } while (!myTimer1.isReady()); // Ждём пока закончится время для считывания
  myTimer1.stop(); // Останавливаем таймер
  // Выводим с массива то, что считали
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(storages[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void setBoxCompletate() {
  int* columnColor[3] = {-1, -1, -1}; // Красный - 1, Синий - 2, Залёный - 3
  int* rowForm[3] = {-1, -1, -1}; // Шар - 1, Куб - 2, Куб с отверстием - 3
  
  // Определяем цвет по колонкам первого склада
  for (int j = 0; j < 3; j++) {
    if (storages[0][j] == R_BALL_TYPE || storages[0][j] == R_CUBE_TYPE || storages[0][j] == R_CUBE_WITH_RECESS_TYPE) columnColor[j] = 1; // Красный
    else if (storages[0][j] == B_BALL_TYPE || storages[0][j] == B_CUBE_TYPE || storages[0][j] == B_CUBE_WITH_RECESS_TYPE) columnColor[j] = 2; // Синий
    else if (storages[0][j] == G_BALL_TYPE || storages[0][j] == G_CUBE_TYPE || storages[0][j] == G_CUBE_WITH_RECESS_TYPE) columnColor[j] = 3; // Зелёный
  }

  // Проверяем все ли заполнены ячейки с цветами
  int objSum = 0;
  for (int i = 0; i < 3; i++) { // Узнаём сумму элементов, которые не равны -1
    if (columnColor[i] != -1) objSum += columnColor[i];
  }
  // Если сумма фигур больше 2-х, то можно добавить нехватающий элемент
  if (objSum > 2) { // Значит, что 2 элемента известно
    for (int i = 0; i < 3; i++) {
      if (columnColor[i] != -1) {
        // Правила по сумме
        if (objSum == 3) columnColor[i] = 3;
        else if (objSum == 4) columnColor[i] = 2;
        else if (objSum == 5) columnColor[i] = 1;
        break; // Выходим
      }
    }
  }
  /////////
  // Определяем формы конфет по 4 складу
  for (int j = 0; j < 3; j++) {
    if (storages[3][j] == R_BALL_TYPE || storages[3][j] == B_BALL_TYPE || storages[3][j] == G_BALL_TYPE) rowForm[j] = 1; // Тип шар
    else if (storages[3][j] == R_CUBE_TYPE || storages[3][j] == B_CUBE_TYPE || storages[3][j] == G_CUBE_TYPE) rowForm[j] = 2; // Куб
    else if (storages[3][j] == R_CUBE_WITH_RECESS_TYPE || storages[3][j] == B_CUBE_WITH_RECESS_TYPE || storages[3][j] == G_CUBE_WITH_RECESS_TYPE) rowForm[j] = 3; // Шар с выемкой
  }
  
  // Проверяем все ли заполнены ячейки с формами
  objSum = 0;
  for (int j = 0; j < 3; j++) { // Узнаём сумму элементов, которые не равны -1
    if (rowForm[j] != -1) objSum += rowForm[j];
  }
  // Если сумма фигур больше 2-х, то можно добавить нехватающий элемент
  if (objSum > 2) { // Значит, что 2 элемента известно
    for (int j = 0; j < 3; j++) {
      if (rowForm[j] != -1) {
        // Правила по сумме
        if (objSum == 3) rowForm[j] = 3;
        else if (objSum == 4) rowForm[j] = 2;
        else if (objSum == 5) rowForm[j] = 1;
        break; // Выходим
      }
    }
  }
}

// Возвращение (поиск) на домашнюю позициию
void searchStartPos() {
  do {
    while (!yStartlimitSwitch.touched()) { // По y сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    ////
    while (!xStartlimitSwitch.touched()) { // По x сместиться в крайнюю позицию
      stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(-STEPPERS_MAX_SPEED);
      stepperX.runSpeed(); stepperY.runSpeed();
    }
  } while (!yStartlimitSwitch.touched() && !xStartlimitSwitch.touched()); // Пока 2 концевика не сработали
  
  stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Установить позиции 0, 0
  Serial.println("x, y = 0, 0");
}

// Перемещение на позицию каретки
void moveCoreXY(String kinematic, int x, int y) {
  int* motPos = new int[2];
  if (kinematic == "IK") motPos = IK_CoreXY(x, y);
  else if (kinematic == "FK") motPos = FK_CoreXY(x, y);
  // Перемещаем
  while (true) { // Перемещаем моторы в позицию
    stepperX.moveTo(motPos[0]); stepperY.moveTo(motPos[1]);
    stepperX.run(); stepperY.run();
    if (!stepperX.isRunning() && !stepperY.isRunning()) break; // Мотор остановился выполнив перемещение
  }
  if (xStartlimitSwitch.touched() && yStartlimitSwitch.touched()) { // Если позиция была указана 0, 0 то по окончанию обновить стартовую позицию
    stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0);
  }
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

// Управление по Z
void controlZ(short pos) {
  servoZ.write(pos);
  delay(100);
}

// Управление инструментом
void controlTool(short pos) {
  servoTool.write(pos);
  delay(100);
}

// Управление из Serial
void manualControl(int type) {
  int* motPos = new int[2];
  while (true) {
    String command = Serial.readStringUntil('\n'); // Считываем из Serial строку до символа переноса на новую строку
    command.trim(); // Чистим символы
    if (command.length() > 0) { // Если есть доступные данные
      char strBuffer[11] = {};
      command.toCharArray(strBuffer, 11);
      // Считываем x и y разделённых пробелом
      int val1 = atoi(strtok(strBuffer, " "));
      int val2 = atoi(strtok(NULL, " "));
      if (type == 1) {
        if (val2 <= MAX_X_DIST_MM && val2 >= 0 && val2 <= MAX_Y_DIST_MM && val2 >= 0) {
          Serial.print("xVal: "); Serial.print(val1); Serial.print(", "); Serial.print("yVal: "); Serial.println(val2);
          Serial.print("motPos0: "); Serial.print(motPos[0]); Serial.print(", "); Serial.print("motPos1: "); Serial.println(motPos[1]);
          moveCoreXY("IK", val1, val2);
        }
      } else if (type == 2) {
        Serial.print("val1: "); Serial.print(val1); Serial.print(", "); Serial.print("val2: "); Serial.println(val2);
        Serial.print("motPos0: "); Serial.print(motPos[0]); Serial.print(", "); Serial.print("motPos1: "); Serial.println(motPos[1]);
        Serial.print("cellsPosX: "); Serial.print(cellsPosX[val1]); Serial.print(", "); Serial.print("cellsPosY: "); Serial.println(cellsPosY[val2]);
        moveCoreXY("IK", cellsPosX[val2], cellsPosY[val1]);
      }
    }
  }
}
