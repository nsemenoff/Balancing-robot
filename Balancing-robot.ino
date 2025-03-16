#include <MsTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "Adeept_Distance.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050                 mpu; //Работа с гироскопом и акселерометром
BalanceCar              balancecar; //Управление двигателей
KalmanFilter            kalmanfilter; //Фильтр Калмана для углов гироскопа
Adeept_Distance         Dist; //Измерение расстояния до препятствия

//Определение функции на ножках платы////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIG            3 //Посылка сигнала
#define ECHO            5 //Прием сигнала
#define RLED            A0 //Красный светодиод
#define GLED            A1 //Зеленый светодиод
#define BLED            A2 //Синий светодиод
#define BUZZER          11 //Звук
#define TB6612_STBY     8 //Остановка двигателей
#define TB6612_PWMA     10 //Скорость левого двигателя
#define TB6612_PWMB     9 //Скорость правого двигателя
#define TB6612_AIN1     12 //Энкодер левого +
#define TB6612_AIN2     13 //Энкодер левого -
#define TB6612_BIN1     7 //Энкодер правого +
#define TB6612_BIN2     6 //Энкодер правого -
#define MOTOR1          2 //Питание левого экодера
#define MOTOR2          4 //Питание правого энкодера
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Определение переменных///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char x_axis = 0;          //Пройденный путь колеса по энкодеру (ось х)
char y_axis = 0;          //Пройденный путь колеса по энкодеру (ось у)
byte klaxon = 0;          //Включение звука                           

int UT_distance = 0;      //Дистанция в см
int detTime = 0;          //Время в мс

int lz = 0;               //Разница счетчика левого колеса
int rz = 0;               //Разница счетчика правого колеса
int lpluse = 0;           //Счетчик импульсов энкодера левого колеса
int rpluse = 0;           //Счетчик импульсов энкодера правого колеса
int sumam;                //Средний пройденный путь двумя колесами (в импульсах энкодера)

//Фильтр Калмана
float Q_angle = 0.001, Q_gyro = 0.005;   //Коэффициенты накопления вертикального угла акселерометра и гироскопа
float R_angle = 0.5 , C_0 = 1;           //Коэффициент накопления угла поворота
float timeChange = 5;                    //Интервал времеи между измерениями в мс
float dt = timeChange * 0.001;           //Интервал времеи между измерениями в с

//Определение угла ориентации
float Q;
float Angle_ax;                          //Текущий накопленный угол наклона по направлению движения вдоль оси х
float Angle_ay;                          //Текущий накопленный угол наклона поперек направления движения вдоль оси у
float K1 = 0.05;                         //Весовой множитель акселерометра
float angle0 = -0.17;                    //Порог отключения двигателей по наклону в рад

double kp = 23, ki = 0.0, kd = 0.48;                        //Параметры ПИД регулятора по углу наклона
double kp_speed = 5.52, ki_speed = 0.1098, kd_speed = 0.0;  //Параметры ПИД регулятора по скорости движения
double kp_turn = 10, ki_turn = 0, kd_turn = 0.09;           //Параметры ПИД регулятора по углу поворота

int16_t ax, ay, az, gx, gy, gz;     //Ускорения и угловые скорости по осям, вычисленные из MPU

int front = 0;                      //Команда "вперед"
int back  = 0;                      //Команда "назад"
int turnl = 0;                      //Команда "налево"
int turnr = 0;                      //Команда "направо"
int spinl = 0;                      //Команда "вокруг оси против часовой стрелки"
int spinr = 0;                      //Команда "вокруг оси по часовой стрелке"

double setp0 = 0, dpwm = 0, dl = 0; //Внутренние переменные ПИД регулятора

double Setpoint;                    //Счетчик по энкодеру для возврата в точку старта
double Setpoints, Outputs = 0;      //Счетчик пройденного пути

int speedcc = 0;                    //Делитель частоты вызова прерываний для движения прямо
volatile long count_left = 0;       //Счетчик импульсов левого колеса
volatile long count_right = 0;      //Счетчик импульсов правого колеса

int turncount = 0;                  //Делитель частоты вызова прерываний для поворота
float turnoutput = 0;               //Счетчик поворота в градусах
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Функция, вызываемая по таймеру каждые 5 мс/////////////////////////////////////////////////////////////////////////////////////////////////////////
void Timer2Isr()
{
  sei();                                                  //Разрешение прерываний
  countpluse();                                           //Вызов функции расчета по энкодерам
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);           //Вычитывание углов ускорения из MPU
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);  //Расчет ориентации фильтром Калмана
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x; //Фильтрация комплементарным фильтром
  speedcc++;                                              //Увеличение значения делителя частоты прерываний
  if (speedcc >= 8)                                       //Каждые 40 мс (по документации)
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0); //Управление двигателями для движения прямо
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4) //Каждые 20 мс
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z); //Управление двигателями для поворота 
    turncount = 0;
  }
  balancecar.posture++; //Счетчик времени в интервалах по 5 мс
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB); //Физическое управление двигателями
  detTime++; //Счетчик делителя измерения расстояния
  if(detTime >= 4) //Каждые 20 мс
  {
    detTime = 0;
    UT_distance = Dist.getDistanceCentimeter();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Обработчики прерываний////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Code_left() 
{
  count_left ++; //Для левого колеса
}

ISR(PCINT2_vect)
{
  count_right ++; //Для правого колеса
}   

void attachPinChangeInterrupt(int pin)
{
  pinMode(pin, INPUT_PULLUP); //Для MPU
  cli();
  PCMSK2 |= bit(PCINT20);
  PCIFR |= bit(PCIF2);
  PCICR |= bit(PCIE2); 
  sei();
}

//Инициализация программы///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);                  //Битовая скорость последовательного интерфейса (!!снять BT перед прошивкой платы!!)     
  Pin_Config();                          //Инициализация ножек микросхемы
  Pin_Init();                            //Запись начальных значений
  Dist.begin(ECHO, TRIG);                //Инициализация модуля измерения расстояния
  Wire.begin();                          //Инициализация I2C    
  mpu.initialize();                      //Инициализация MPU
  delay(1500);                           //Ожидание калибровки MPU
  balancecar.pwm1 = 0;                   //Начальное значения ШИМ на левом двигателе
  balancecar.pwm2 = 0;                   //Начальное значения ШИМ на правом двигателе
    
  MsTimer2::set(5, Timer2Isr);           //Настройка таймера 5 мс
  MsTimer2::start();
}

//Основной цикл программы//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  attachInterrupt(0, Code_left, CHANGE); //Включение обработчика прерываний левого колеса         
  attachPinChangeInterrupt(MOTOR2);      //Включение обработчика прерываний правого колеса
  TX_Information(UT_distance);           //Вывод отладочной информации
  RX_Information();                      //Чтение команд
  
  if(UT_distance<20)
  {
    digitalWrite(RLED, LOW);
    digitalWrite(GLED, HIGH);
    digitalWrite(BLED, HIGH);
    ResetCarState();   
  }
  else
  {
    digitalWrite(RLED, HIGH);
    digitalWrite(GLED, HIGH);
    digitalWrite(BLED, HIGH);    
  }
/*  Serial.print(kalmanfilter.angle);
  Serial.print(", ");
  Serial.print(turnoutput);
  Serial.print(", ");
  Serial.println(Outputs);
  delay(10);*/
}

//Функция счетчика импульсов энкодера с учетом направления движения/////////////////////////////////////////////////////////////////////////////// 
void countpluse()
{
  lz = count_left; 
  rz = count_right;

  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))       //Движение вперед
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))  //Движение назад
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))  //Поворот налево
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))  //Поворот направо
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  balancecar.stopl += lpluse;        //Сохранение значение энкодера для движения прямо левым колесом
  balancecar.stopr += rpluse;        //Сохранение значение энкодера для движения прямо правым колесом
  balancecar.pulseleft += lpluse;    //Сохранение значение энкодера для поворота левым колесом
  balancecar.pulseright += rpluse;   //Сохранение значение энкодера для поворота правым колесом
  sumam = (balancecar.pulseright + balancecar.pulseleft) * 4; //Пройденный путь двумя колесами
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Команда остановки робота//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ResetCarState()
{
  turnl = 0; 
  turnr = 0;  
  front = 0; 
  back = 0; 
  spinl = 0; 
  spinr = 0; 
  turnoutput = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Обработка команд////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RX_Information(void)
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();

    if (cmd == '0')
    {
      Serial.println("Start");
      ResetCarState();
      back = -30;
    }
    else if (cmd == 'X')
    {
      Serial.println("Stop");
      ResetCarState();
    }
    else if (cmd == 'F')
    {
      Serial.println("Вперёд");
      ResetCarState();
      back = -30;
    }
    else if (cmd == 'B')
    {
      Serial.println("Назад");
      ResetCarState();
      front = -30;
    }
    else if (cmd == 'L')
    {
      Serial.println("Налево");
      turnl = 1;
    }
    else if (cmd == 'R')
    {
      Serial.println("Направо");
      turnr = 1;
    }
    else if (cmd == 'W')
    {
      Serial.println("Квадрат");
      spinr = 1;
    }
    else if (cmd == 'Z')
    {
      Serial.println("Ручное управление");
      spinl = 1;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Вывод отладочных данных/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TX_Information(byte dat)
{
//  Serial.print("Distance = "); //Вывод расстояния до препятствия
//  Serial.println(dat);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Инициализация ножек микросхемы/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pin_Init()
{   
  digitalWrite(BUZZER, LOW);
  digitalWrite(TB6612_STBY, HIGH);                        
  digitalWrite(TB6612_PWMA, LOW);                         
  digitalWrite(TB6612_PWMB, LOW);                          
  digitalWrite(TB6612_AIN1, LOW);                          
  digitalWrite(TB6612_AIN2, HIGH);                         
  digitalWrite(TB6612_BIN1, HIGH);                         
  digitalWrite(TB6612_BIN2, LOW);                         
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Настройка параметров ножек микросхемы//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pin_Config()
{
  pinMode(TRIG, OUTPUT);                                  
  pinMode(ECHO, INPUT);                                   

  pinMode(RLED, OUTPUT);                                  
  pinMode(GLED, OUTPUT);                                  
  pinMode(BLED, OUTPUT);                                  
  
  pinMode(BUZZER, OUTPUT);                                
  
  pinMode(TB6612_STBY, OUTPUT);                           
  pinMode(TB6612_PWMA, OUTPUT);                           
  pinMode(TB6612_PWMB, OUTPUT);                           
  pinMode(TB6612_AIN1, OUTPUT);                            
  pinMode(TB6612_AIN2, OUTPUT);                            
  pinMode(TB6612_BIN1, OUTPUT);                            
  pinMode(TB6612_BIN2, OUTPUT);                            
  
  pinMode(MOTOR1, INPUT);                                 
  pinMode(MOTOR2, INPUT);                                 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
