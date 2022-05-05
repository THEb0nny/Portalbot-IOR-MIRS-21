// http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/#post-132258
// http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// http://learn.makeblock.com/Makeblock-library-for-Arduino/class_me_port.html
// http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_limit_switch.html
// https://www.marginallyclever.com/2015/01/adapting-makelangelo-corexy-kinematics/
// http://wiki.neobot.ru/index.php?title=%D0%A1%D0%B2%D0%B5%D1%82%D0%BE%D0%B4%D0%B8%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BB%D0%B5%D0%BD%D1%82%D0%B0/LED_RGB_Strip-Addressable,_Sealed
// https://community.alexgyver.ru/resources/biblioteka-gyvertimer.11/

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
#include "TrackingCamI2C.h" // Обрезанная версия, чтобы хватало динамической памяти
#include "GyverTimer.h"

#define LIMIT_SWITCH_X_START_PORT PORT_3 // Порт концевика по X
#define LIMIT_SWITCH_X_START_SLOT SLOT_1 // Слот концевика по X

#define LIMIT_SWITCH_Y_START_PORT PORT_3 // Порт концевика по Y
#define LIMIT_SWITCH_Y_START_SLOT SLOT_2 // Слот концевика по Y

#define SERVO_Z_PIN A2 // Пин серво Z
#define SERVO_TOOL_PIN A3 // Пин серво инструмента

#define MAX_X_DIST_MM 140 // Максимальная дистанция по X для перемещения в мм
#define MAX_Y_DIST_MM 145 // Максимальная дистанция по Y для перемещения в мм

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

// Номера фигур по настройкам камеры
#define R_BALL_TYPE 0
#define G_BALL_TYPE 1
#define B_BALL_TYPE 2
#define R_CUBE_TYPE 3
#define G_CUBE_TYPE 4
#define B_CUBE_TYPE 5
#define R_CUBE_WITH_RECESS_TYPE 6
#define G_CUBE_WITH_RECESS_TYPE 7
#define B_CUBE_WITH_RECESS_TYPE 8

// Номера цветов
#define RED_OBJ 1
#define GREEN_OBJ 2
#define BLUE_OBJ 3

// Номера форм
#define BALL_OBJ 1
#define CUBE_OBJ 2
#define CUBE_WITH_RECESS_OBJ 3

#define STORAGE_COLOR_RULES 0 // Склад, с которого считываем цвета для правила сборки boxCompletateSolve
#define STORAGE_FORM_RULES 3 // Склад, с которого считываем формы для правила сборки boxCompletateSolve

#define R_ZONE_POS 5 // Радиус координаты позиции, в которой можно найти объект
#define XY_CELLS_ARR_LEN 5 // Размер для массивов координат ячеек по X и Y
#define TIME_TO_READ_FROM_CAM 5000 // Время для считываения с камеры
#define MAX_CAM_BLOBS_READ 10 // Максимальное смчитывание блобсов за один раз

#define SERVO_Z_UP 45 // Значение, когда Z поднято
#define SERVO_Z_DOWN 160 // Значение, когда Z опущено

#define SERVO_TOOL_UP 140 // Инструмент поднят
#define SERVO_TOOL_DOWN 20 // Инструмент выпущен

// Концевики
MeLimitSwitch xStartlimitSwitch(LIMIT_SWITCH_X_START_PORT, LIMIT_SWITCH_X_START_SLOT);
MeLimitSwitch yStartlimitSwitch(LIMIT_SWITCH_Y_START_PORT, LIMIT_SWITCH_Y_START_SLOT);

// Шаговые двигатели
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

Servo servoZ, servoTool; // Серво инструмента

TrackingCamI2C trackingCam; // Камера

GTimer_ms camTimer(TIME_TO_READ_FROM_CAM); // Таймер

int storages[4][3] = { // Склады
  {5, -1, 3}, // Склад 1 сверху
  {7, -1, 4}, // Склад 2 справа
  {0, 6, 1}, // Склад 3 снизу
  {-1, 2, 8} // Склад 4 слева
};

// Массив с правилами растановки в коробку
int boxCompletateSolve[3][3] = {
  {-1, -1, -1},
  {-1, -1, -1},
  {-1, -1, -1}
};

const int cellsPosX[XY_CELLS_ARR_LEN] = {3, 36, 70, 100, 135}; // Координаты рядов ячеек
const int cellsPosY[XY_CELLS_ARR_LEN] = {135, 103, 72, 38, 8}; // Координаты строк ячеек

// Координаты хранилищ в камере
const int storagesCellsCamPosX[XY_CELLS_ARR_LEN] = {70, 104, 139, 175, 209};
const int storagesCellsCamPosY[XY_CELLS_ARR_LEN] = {23, 56, 92, 128, 162};

float x, y, lx, ly; // Глобальные переменные координат по X, Y кинематики CoreXY

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  Serial.println();
  stepperX.setMaxSpeed(STEPPERS_MAX_SPEED); stepperY.setMaxSpeed(STEPPERS_MAX_SPEED); // Установка максимальной скорости (оборотов в минуту). При движении шаговый двигатель будет ускоряться до этой максимальной скорости и замедляться при подходе к концу движения
  stepperX.setAcceleration(STEPPERS_ACCEL); stepperY.setAcceleration(STEPPERS_ACCEL); // Установка ускорения, в шагах в секунду за секунду
  servoZ.attach(SERVO_Z_PIN); // Подключаем серво Z
  servoTool.attach(SERVO_TOOL_PIN); // Подключаем серво инструмента
  ControlZ(SERVO_Z_UP, 0); // Поднимаем Z
  ControlTool(SERVO_TOOL_UP, 0); // Поднимаем инструмент
  trackingCam.init(51, 100000); // cam_id - 1..127, default 51; speed - 100000/400000
}

void loop() {
  SearchStartPos(); // Вернуться на базу и установить нулевую позицию
  ManualControl(2); // Ручное управление
  /*
  MoveToPosCoreXY(cellsPosX[0], cellsPosY[0]); // Чтобы не сбить столбик из жёлтых фигур перемещаемся не сразу по диагонали
  MoveToPosCoreXY(MAX_X_DIST_MM, MAX_Y_DIST_MM); // Перемещаемся в крайнюю точку, чтобы считывать с камеры
  SearchFromCamObj(); // Ищем с камеры объекты
  SetBoxCompletate(); // Установить массив с итоговой комплектацией
  Solve(); // Решаем задачу
  SearchStartPos(); // Возвращаемся в нулевую точку после выполнения
  buzzer.tone(255, 5000); // Пищим о завершении
  while(true); // Конец выполнения
  */
}

// Моё решение
void Solve() {
  for (int i = 0; i < 3; i++) { // Идём по массиву необходимой комплетации по столбцам
    for (int j = 0; j < 3; j++) { // Идём по массиву необходимой комплетации по строкам
      for (int n = 0; n < 4; n++) { // Идём по столбцам хранилищ (разным хранилищам)
        for (int m = 0; m < 3; m++) { // Идём по строкам хранилища
          if (boxCompletateSolve[i][j] > 2) continue; // Делаем только шары
          if (boxCompletateSolve[i][j] == storages[n][m] && storages[n][m] != -1) { // Если совпадаение
            int moveCellPosX, moveCellPosY; // Временные переменные для перемещения
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
            //ControlTool() - 180 - поднято, 30 - опущено максимально
            //ControlZ() - 40 - поднятно, 160 - опущено

            MoveToPosCoreXY(moveCellPosX, moveCellPosY); // Перемещаемся к найденой
            ControlTool(30, 500); // Опускаем инструмент
            if (0 <= storages[n][m] && storages[n][m] < 3) ControlZ(170, 500); // Опускаем Z, если это шар, то отпускаем Z так
            else ControlZ(160, 500); // Иначе отпускаем так
            ControlZ(40, 500); // Поднимаем Z
            storages[n][m] = -1; // Удаляем в storages, ставим там пустоту
            Serial.print(" и перенести в "); Serial.print(cellsPosX[j + 1]); Serial.print(", "); Serial.println(cellsPosY[i + 1]);
            MoveToPosCoreXY(cellsPosX[j + 1], cellsPosY[i + 1]); // Перемещаемся в нужную ячейку, чтобы поставить
            // НОВОЕ
            //int tmpCell = 0, tmpRow = 0;
            //if (n < 2) tmpCell = 0;
            //else tmpCell = 3;
            //if (m < 2) tmpRow = 0;
            //else tmpRow = 3;
            //MoveToPosCoreXY(tmpCell, tmpRow); // Перемещаемся в нужную ячейку, чтобы поставить
            //MoveToPosCoreXY(cellsPosX[j + 1], cellsPosY[i + 1]); // Перемещаемся в нужную ячейку, чтобы поставить
            if (0 <= storages[n][m] && storages[n][m] < 3) ControlZ(170, 500); // Опускаем Z так, если это шар
            else ControlZ(160, 500); // Иначе отпускаем так
            ControlTool(180, 500); // Поднимаем инструмент (отлепляем)
            ControlZ(40, 500); // Поднимаем Z
          }
        }
      }
    }
  }
}

// Считываем данные с камеры и записываем
void SearchFromCamObj() {
  unsigned long prevMillis = 0;
  camTimer.reset(); // Сбросить таймер
  do {
    uint8_t n = trackingCam.readBlobs(MAX_CAM_BLOBS_READ); // Считать первые n блобсов
    Serial.print("All blobs ");
    Serial.println(n); // Сообщить о количестве найденных блобсах
    for(int k = 0; k < n; k++) { // Проходимся по всем блобсам с камеры
      int objType = trackingCam.blob[k].type;
      int objCX = trackingCam.blob[k].cx;
      int objCY = trackingCam.blob[k].cy;
      Serial.print(objType, DEC); Serial.print(" "); Serial.print(objCX, DEC); Serial.print(" "); Serial.print(objCY, DEC); Serial.println(" "); // Выводим инфу с камеры
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
    while(millis() - prevMillis < 33);
    prevMillis = millis();
  } while (!camTimer.isReady()); // Ждём пока закончится время для считывания
  camTimer.stop(); // Останавливаем таймер
  Serial.println("Storages:"); // Выводим с массива то, что считали
  for (int i = 0; i < 4; i++) { // Проходимся по строкам хранилища
    for (int j = 0; j < 3; j++) { // Проходимся по столбцам хранилища
      Serial.print(storages[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Расчитать правила растановки по найденным объектам
void SetBoxCompletate() {
  int columnColor[3] = {-1, -1, -1}; // Красный - 1, Синий - 2, Залёный - 3
  int rowForm[3] = {-1, -1, -1}; // Шар - 1, Куб - 2, Куб с отверстием - 3
  //// Определяем ЦВЕТА по колонкам первого склада
  bool redExists = false, greenExists = false, blueExists = false;
  for (int j = 0; j < 3; j++) {
    if (storages[STORAGE_COLOR_RULES][j] == R_BALL_TYPE || storages[STORAGE_COLOR_RULES][j] == R_CUBE_TYPE || storages[STORAGE_COLOR_RULES][j] == R_CUBE_WITH_RECESS_TYPE) {
      redExists = true;
      columnColor[j] = RED_OBJ; // Красный
    } else if (storages[STORAGE_COLOR_RULES][j] == G_BALL_TYPE || storages[STORAGE_COLOR_RULES][j] == G_CUBE_TYPE || storages[STORAGE_COLOR_RULES][j] == G_CUBE_WITH_RECESS_TYPE) {
      greenExists = true;
      columnColor[j] = GREEN_OBJ; // Зелёный
    } else if (storages[STORAGE_COLOR_RULES][j] == B_BALL_TYPE || storages[STORAGE_COLOR_RULES][j] == B_CUBE_TYPE || storages[STORAGE_COLOR_RULES][j] == B_CUBE_WITH_RECESS_TYPE) {
      blueExists = true;
      columnColor[j] = BLUE_OBJ; // Синий
    }
  }

  for (int j = 0; j < 3; j++) { // Заполняем у одного не пустого
    if (columnColor[j] == -1) {
      if (redExists && greenExists) columnColor[j] = BLUE_OBJ;
      else if (redExists && blueExists) columnColor[j] = GREEN_OBJ;
      else if (blueExists && greenExists) columnColor[j] = RED_OBJ;
    }
  }
 
  //// Определяем ФОРМЫ конфет по 4 складу
  bool ballExists = false, cubeExists = false, cubeWithRessExists = false;
  for (int j = 0; j < 3; j++) {
    if (storages[STORAGE_FORM_RULES][j] == R_BALL_TYPE || storages[STORAGE_FORM_RULES][j] == B_BALL_TYPE || storages[STORAGE_FORM_RULES][j] == G_BALL_TYPE) {
      ballExists = true;
      rowForm[j] = BALL_OBJ; // Тип шар
    } else if (storages[STORAGE_FORM_RULES][j] == R_CUBE_TYPE || storages[STORAGE_FORM_RULES][j] == B_CUBE_TYPE || storages[STORAGE_FORM_RULES][j] == G_CUBE_TYPE) {
      cubeExists = true;
      rowForm[j] = CUBE_OBJ; // Куб
    } else if (storages[STORAGE_FORM_RULES][j] == R_CUBE_WITH_RECESS_TYPE || storages[STORAGE_FORM_RULES][j] == B_CUBE_WITH_RECESS_TYPE || storages[STORAGE_FORM_RULES][j] == G_CUBE_WITH_RECESS_TYPE) {
      cubeWithRessExists = true;
      rowForm[j] = CUBE_WITH_RECESS_OBJ; // Куб с выемкой
    }
  }
  
  for (int i = 0; i < 3; i++) { // Заполняем у одного не пустого
    if (rowForm[i] == -1) {
      if (ballExists && cubeExists) rowForm[i] = CUBE_WITH_RECESS_OBJ;
      else if (ballExists && cubeWithRessExists) rowForm[i] = CUBE_OBJ;
      else if (cubeExists && cubeWithRessExists) rowForm[i] = BALL_OBJ;
    }
  }
 
  // Генерируем необходимое решение для сборки - boxCompletateSolve
  for (int i = 0; i < 3; i++) { // Проходимся по строкам
    for (int j = 0; j < 3; j++) { // Проходимся по столбцам
      if (i == 2 && j == 2) continue; // Жёлтая башня в центре
      if (columnColor[j] == RED_OBJ && rowForm[i] == BALL_OBJ) boxCompletateSolve[i][j] = R_BALL_TYPE; // Если цвет 1 и форма 1, то это красный шар
      else if (columnColor[j] == GREEN_OBJ && rowForm[i] == BALL_OBJ) boxCompletateSolve[i][j] = G_BALL_TYPE; // Если цвет 2 и форма 1, то это зелёный шар
      else if (columnColor[j] == BLUE_OBJ && rowForm[i] == BALL_OBJ) boxCompletateSolve[i][j] = B_BALL_TYPE; // Если цвет 3 и форма 1, то это синий шар
      else if (columnColor[j] == RED_OBJ && rowForm[i] == CUBE_OBJ) boxCompletateSolve[i][j] = R_CUBE_TYPE; // Если цвет 1 и форма 2, то это красный куб
      else if (columnColor[j] == GREEN_OBJ && rowForm[i] == CUBE_OBJ) boxCompletateSolve[i][j] = G_CUBE_TYPE; // Если цвет 1 и форма 2, то это зелёный куб
      else if (columnColor[j] == BLUE_OBJ && rowForm[i] == CUBE_OBJ) boxCompletateSolve[i][j] = B_CUBE_TYPE; // Если цвет 1 и форма 2, то это зелёный куб
      else if (columnColor[j] == RED_OBJ && rowForm[i] == CUBE_WITH_RECESS_OBJ) boxCompletateSolve[i][j] = R_CUBE_WITH_RECESS_TYPE; // Если цвет 1 и форма 3, то это красный куб с выемкой
      else if (columnColor[j] == GREEN_OBJ && rowForm[i] == CUBE_WITH_RECESS_OBJ) boxCompletateSolve[i][j] = G_CUBE_WITH_RECESS_TYPE; // Если цвет 2 и форма 3, то это зелёный куб с выемкой
      else if (columnColor[j] == BLUE_OBJ && rowForm[i] == CUBE_WITH_RECESS_OBJ) boxCompletateSolve[i][j] = B_CUBE_WITH_RECESS_TYPE; // Если цвет 3 и форма 3, то это синий куб с выемкой
    }
  }
  Serial.println("Box Completate Solve:");
  for (int i = 0; i < 3; i++) { // Проходимся по строкам хранилища
    for (int j = 0; j < 3; j++) { // Проходимся по столбцам хранилища
      Serial.print(boxCompletateSolve[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Возвращение на домашнюю позициию
void SearchStartPos() {
  do {
    stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(-STEPPERS_MAX_SPEED); // Установить скорости, а почему только тут?
    while (!yStartlimitSwitch.touched()) { // По x сместиться в крайнюю позицию
      stepperX.runSpeed(); stepperY.runSpeed();
    }
    stepperX.setSpeed(STEPPERS_MAX_SPEED); stepperY.setSpeed(STEPPERS_MAX_SPEED); // Установить скорости, а почему только тут?
    while (!xStartlimitSwitch.touched()) { // По y сместиться в крайнюю позицию
      stepperX.runSpeed(); stepperY.runSpeed();
    }
  } while (!xStartlimitSwitch.touched() && !yStartlimitSwitch.touched()); // Пока 2 концевика не сработали
  stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Установить позиции 0, 0
  Serial.println("x, y = 0, 0");
}

// Перемещение на позицию каретки
void MoveToPosCoreXY(int x, int y) {
  int* motPos = new int[2];
  motPos = IK_CoreXY(x, y); // Считаем обратную кинематику
  while (true) { // Перемещаем моторы в позицию
    stepperX.moveTo(motPos[0]); stepperY.moveTo(motPos[1]);
    stepperX.run(); stepperY.run();
    if (!stepperX.isRunning() && !stepperY.isRunning()) break; // Мотор остановился выполнив перемещение
  }
  //stepperX.stop(); stepperY.stop(); // Остановить моторы
  stepperX.disableOutputs(); stepperY.disableOutputs();
  if (xStartlimitSwitch.touched() && yStartlimitSwitch.touched()) { // Сработали концевики, значит мы на нулевой позии
    stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0); // Обнулить
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
void ControlZ(short pos, int delayTime) {
  servoZ.write(pos);
  delay(delayTime);
}

// Управление инструментом
void ControlTool(short pos, int delayTime) {
  servoTool.write(pos);
  delay(delayTime);
}

// Управление из Serial
void ManualControl(int type) {
  int x = 0, y = 0, row, col, z, tool;
  while (true) {
    String command = Serial.readStringUntil('\n'); // Считываем из Serial строку до символа переноса на новую строку
    command.trim(); // Чистим символы
    if (command.length() > 0) { // Если есть доступные данные
      char strBuffer[11]; // Создаём пустой массив символов
      command.toCharArray(strBuffer, 11); // Перевести строку в массив символов
      // Считываем x и y разделённых пробелом, а также Z и инструментом
      int value1 = atoi(strtok(strBuffer, " "));
      int value2 = atoi(strtok(NULL, " "));
      String value3 = String(strtok(NULL, " "));
      String value4 = String(strtok(NULL, " "));
      Serial.print(value1); Serial.print(" "); Serial.print(value2); Serial.print(" "); Serial.print(value3);  Serial.print(" "); Serial.println(value4);
      if (type == 1) { // Тип работы по координатам X и Y
        if (value1 != 0) x = constrain(value1, 0, MAX_X_DIST_MM); // Записываем X и ограничиваем её
        if (value2 != 0) y = constrain(value2, 0, MAX_Y_DIST_MM); // Записываем Y и ограничиваем её
        Serial.print("xVal: "); Serial.print(x); Serial.print(", "); Serial.print("yVal: "); Serial.println(y);
        MoveToPosCoreXY(x, y);
      } else if (type == 2) { // Тип работы по ячейкам
        row = constrain(value1, 0, 4); // Считываем номер строки и ограничиваем
        col = constrain(value2, 0, 4); // Считываем номер столбца и ограничиваем
        Serial.print("row: "); Serial.print(row); Serial.print(", "); Serial.print("col: "); Serial.println(col);
        Serial.print("cellsPosX: "); Serial.print(cellsPosX[row]); Serial.print(", "); Serial.print("cellsPosY: "); Serial.println(cellsPosY[col]);
        MoveToPosCoreXY(cellsPosX[col], cellsPosY[row]);
      }
      // Управляем Z
      if (value3 == "zu") ControlZ(SERVO_Z_UP, 500); // Если команда zu
      else if (value3 == "zd") ControlZ(SERVO_Z_DOWN, 500); // Если команда zd
      else if (value3 != "") { // Если не пустота, то выставляем позицию
        z = constrain(value3.toInt(), 0, 270);
        ControlZ(z, 1000);
      }
      // Управляем Tool
      if (value4 == "tu") ControlTool(SERVO_TOOL_UP, 500); // Если команда tu
      else if (value4 == "td") ControlTool(SERVO_TOOL_DOWN, 500); // Если команда td
      else if (value3 != "") { // Если не пустота, то выставляем позицию
        tool = constrain(value4.toInt(), 0, 270);
        ControlTool(tool, 1000);
      }
    }
  }
}
