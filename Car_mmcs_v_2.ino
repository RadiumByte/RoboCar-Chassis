#include <SoftwareSerial.h>            
#include <DFPlayer_Mini_Mp3.h>

#define MOTOR_PWM_1            5     // порты(ШИМ - широтно-импульсная модуляция) управления скоростью
#define MOTOR_PWM_2            6
#define STEERING_POS           3     // аналоговый порт для резистора актуатора
#define STEERING_CONTROL_1     12    // порты управления актуатором
#define STEERING_CONTROL_2     4

#define CENTER                 659   // аналоговые показания для центра рулевой балки, default = 659

#define MAX_BT_COMMAND_LENGTH  64    // максимальный размер команды по bluetooth 
#define WATCHDOG_TIMEOUT       10000  // лимит на ожидание отклика
#define ASK_TIMEOUT            200   // интервал посыла запросов от сторожевого таймера
#define VERSION     "CAR-MMCS v4.0"  // название и версия программы для передачи по bluetooth

#define TABLE_SIZE    33
const int steering_table[TABLE_SIZE] = {164, 182, 200, 218, 235, 271, 306, 341, 376, 412, 447, 483, 518, 553, 588, 624, 659, 667, 675, 683, 690, 698, 706, 714, 721, 729, 737, 745, 752, 760, 768, 776, 783};

SoftwareSerial BTSerial (10, 11);   // RX, TX - цифровые порты для передачи данных
SoftwareSerial Player   (8, 7);

char command_buffer[MAX_BT_COMMAND_LENGTH];  // массив для получения команды по bluetooth

int current_speed;
int target_angle = CENTER;

bool safe_mode = false;       // флаг watchdog-а
bool safe_mode_prev = false;
bool manual_control = false;  // флаг ручного управления
bool direction = false;       // флаг направления поворота

unsigned long watchdog_interval;   // временные интервалы для watchdog
unsigned long ask_interval;

int current_analog = CENTER;

void InitializeCOM()
{
  BTSerial.begin(9600);         
  Serial.begin(38400);
}

void InitializeCar()
{
  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);

  pinMode(STEERING_CONTROL_1, OUTPUT);
  pinMode(STEERING_CONTROL_2, OUTPUT);
  pinMode(A3, INPUT);

  current_speed = 0;

  Player.begin(9600);
  
  mp3_set_serial(Player);
  delay(10);
  mp3_set_volume (50);
  delay(10);
}

//Функция движения вперед
void MotorForward(int spd)
{
  analogWrite(MOTOR_PWM_1, spd);
  analogWrite(MOTOR_PWM_2, 0);
}

//Функция движения назад
void MotorBackward(int spd)
{
  analogWrite(MOTOR_PWM_1, 0);
  analogWrite(MOTOR_PWM_2, spd);
}

void Stop()
{
  for (int i = current_speed; i >= 0; i -= 5)
  {
    MotorForward(i);
    delay(15);
  }
  current_speed = 0;
}

void Left()
{
  digitalWrite(STEERING_CONTROL_1, HIGH);
  digitalWrite(STEERING_CONTROL_2, LOW);
}

void Right()
{
  digitalWrite(STEERING_CONTROL_1, LOW);
  digitalWrite(STEERING_CONTROL_2, HIGH);
}

void StopSteering()
{
  digitalWrite(STEERING_CONTROL_1, LOW);
  digitalWrite(STEERING_CONTROL_2, LOW);
}

// Универсальная функция поворота колес. По-умолчанию передаем угол для центровки.
void Center(int position = CENTER)
{
  int current_pos = analogRead(STEERING_POS);
  if (current_pos > position)
  {
    Right();
    direction = false;
  }
  else if (current_pos < position)
  {
    Left();
    direction = true;
  }
  target_angle = position;
}

void SteerTo(int percent)
{
  int target_index = 0;
  int half_count = ((TABLE_SIZE - 1) / 2);                   // center's index
  int array_shift = abs(50 - percent) / (50 / half_count);   // shift between target index and center's index

  if (percent >= 50)
    target_index = half_count - array_shift;
  else
    target_index = half_count + array_shift;

  int resistor_border = steering_table[target_index]; // target resistor value
  Center(resistor_border);
}

String GetCommand() 
{
  int inputCount = BTSerial.available();   // получаем число доступных байтов, лежащих в буфере приема
  if (inputCount > 0)
  {
    if (manual_control == false)
    {
      // Фраза:
      // Управление перехвачено
      
      mp3_play(17);
    }

    safe_mode_prev = safe_mode;
    safe_mode = false;
    
    manual_control = true;

    int totalByte = BTSerial.readBytesUntil('\n', command_buffer, MAX_BT_COMMAND_LENGTH);  // читаем данные из буфера
    String command = command_buffer;            // создали строку - "сырую команду"
    command = command.substring(0, totalByte);  // обрезали пустые места
    command.replace("\r", "");                  // удаляем символы табуляции
    command.replace("\n", "");

    return command;
  }

  if (!manual_control)
  {
    int inputCount = Serial.available();
    if (inputCount > 0)
    {
      int totalByte = Serial.readBytesUntil('\n', command_buffer, MAX_BT_COMMAND_LENGTH);
      String command = command_buffer;
      command = command.substring(0, totalByte);
      command.replace("\r", "");
      command.replace("\n", "");
      return command;
    }
  }
  return "";
}

void CheckCommand()      // парсинг команды и принятие решения
{
  String command_name = GetCommand();   // получили команду
  if (command_name.length() > 0)        // если команда не пуста
  {
    BTSerial.flush();  // очистка буферов
    Serial.flush();
    
    if (command_name == "STATUS")      // текущая версия прошивки машины
    {
      String ver = VERSION;
      BTSerial.println("Software version: " + ver);
      Serial.println("Software version: " + ver);
    }
    else if (command_name == "START")   // запуск сторожевого таймера
    {
      // Фраза:
      // Сторожевой таймер активирован
      mp3_play(4);
      //delay(5000);
      
      safe_mode = true;
      ask_interval = millis();
      watchdog_interval = millis();
    }
    else if (command_name == "RUN")
    {
      Stop();
      manual_control = false;
      
      safe_mode = safe_mode_prev;
      if (safe_mode)
        watchdog_interval = millis();

      Serial.flush();
      BTSerial.flush();
      
      // Фраза:
      // Смена канала приема

      mp3_play(6);
      //delay(4000);
    }
    else if (command_name == "LEFT")   // поворот влево
    {
      Left();
      direction = true;
      target_angle = 1000;
    }
    else if (command_name == "ANSWER") // пришел ответ от ПК
    {
      watchdog_interval = millis();    // сброс сторожевого таймера
    }
    else if (command_name == "RIGHT")  // поворот вправо
    {
      Right();
      direction = false;
      target_angle = 0;
    }
    else if (command_name == "CENTER")  // центровка колес
    {
      Center();
    }
    else if (command_name == "HALT")    
    {
      Stop();
    }
    else if (command_name == "FAILOBST")
    {
      // Фраза:
      // Внимание: угроза столкновения

      mp3_play(8);
      delay(5000);
    }
    else if (command_name == "FAILSEN")
    {
      // Фраза:
      // Внимание: отказ блока сенсоров

      mp3_play(9);
      delay(6000);
    }
    else if (command_name == "FAILLAS")
    {
      // Фраза:
      // Внимание: отказ лазерного дальномера

      mp3_play(10);
      delay(6000);
    }
    else if (command_name == "FAILCOMM")
    {
      // Фраза:
      // Внимание: отказ внешней связи

      mp3_play(11);
      delay(5000);
    }
    else if (command_name == "CONNKUB")
    {
      // Фраза:
      // Подключено устройство с идентификатором Кубус

      mp3_play(13);
      delay(3000);
    }
    else if (command_name == "CONNSERV")
    {
      // Фраза:
      // Подключен удаленный сервер

      mp3_play(14);
      delay(5000);
    }
    else if (command_name == "CONNRAD")
    {
      // Фраза:
      // Подключено устройство с идентификатором Радий

      mp3_play(15);
      delay(3000);
    }
    else if (command_name == "CONNTRIT")
    {
      // Фраза:
      // Подключено устройство с идентификатором Тритий

      mp3_play(16);
      delay(3000);
    }
    else if (command_name == "AIMODE")
    {
      Serial.println("STOP");
      safe_mode = false;
      safe_mode_prev = false;
    }
    else                                // если пришла команда установки скорости
    {
      if (!manual_control)
      {
        String motor_select = command_name.substring(0, 1);
        String postfix = command_name.substring(command_name.length() - 1, command_name.length());
        if (postfix == "A")
        {
          String motor_speed = command_name.substring(1, command_name.length());
          if (motor_speed.length() >= 0)
          {
            if (motor_select == "F")        // ведущая ось
            {
              int throttle = motor_speed.toInt();
              int ToPWM = map(throttle, 0, 100, 0, 255); 
              MotorForward(ToPWM);
            }
            else if (motor_select == "B")
            {
              int throttle = motor_speed.toInt();
              int ToPWM = map(throttle, 0, 100, 0, 255);
              MotorBackward(ToPWM);
            }
            else if (motor_select == "S")   // рулевой двигатель
            {
              int current_angle = motor_speed.toInt();
              SteerTo(motor_speed.toInt());
            }
          }
        }
        BTSerial.flush();
        Serial.flush();
      }
      else
      {
        String motor_select = command_name.substring(0, 1);
        String motor_speed = command_name.substring(1, command_name.length());
        if (motor_speed.length() >= 0)
        {
          if (motor_select == "L")        // ведущая ось
          {
            int throttle = motor_speed.toInt();

            if (throttle > 50)
              MotorForward((throttle - 50) * 5.1f);   
            else
              MotorBackward((50 - throttle) * 5.1f);

            current_speed = (throttle - 50) * 5.1f;
          }
          else if (motor_select == "S")   // рулевой двигатель
          {
            int current_angle = motor_speed.toInt();
            if (current_angle > 100)
              current_angle = 100;
            if (current_angle < 0)
              current_angle = 0;   
            SteerTo(motor_speed.toInt());
          }
        }
        BTSerial.flush();
        Serial.flush();
      }
    }
  }
}

void Watchdog()
{
  if ((millis() - ask_interval) > ASK_TIMEOUT)  // пеленг системы управления с интервалом ASK_TIMEOUT
  {
    ask_interval = millis();
    Serial.println("ASK");
  }

  if ((millis() - watchdog_interval) > WATCHDOG_TIMEOUT)  // проверка сработки сторожевого таймера
  {
    Stop();
    Center();
    
    // Фраза:
    // Внимание: отказ блока управления
    
    mp3_play(5);
    delay(5000);
  }
}

void CheckAngle()
{
  if (direction)
  {
    if (analogRead(3) >= target_angle)
      StopSteering();
  }
  else
  {
    if (analogRead(3) <= target_angle)
      StopSteering();
  }
}

void setup()
{
  InitializeCar();
  InitializeCOM();

  // Фраза:
  // Инициализация ходовой части

  mp3_play(1);
  delay(4000);

  // Фраза:
  // Системы связи активированы

  mp3_play(2);
  delay(4000);

  // Фраза:
  // Проверка бортовых систем завершена

  mp3_play(12);
  delay(5000);

  // Фраза:
  // Автоматика готова к работе

  mp3_play(3);
  delay(4000);

  Center();
  Stop();
}

void loop()
{
  if (safe_mode)
  {
    Watchdog();  // проверка валидности соединения c системой управления
    delay(10);
  }

  CheckCommand(); // проверка команд
  CheckAngle();   // проверка поворота
  delay(10);
}
