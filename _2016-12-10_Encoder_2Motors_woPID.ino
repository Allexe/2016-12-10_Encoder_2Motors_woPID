//The sample code for driving one way motor encoder
#include <PID_v1.h>

//************************************************************************************************************
//Энкодер на правом моторе:
//пин A правого энкодера подключается к цифр.пину 2 - используется для обработки прерываний (interrupt) int.1:
const byte Encoder_Right_Pin_A = 2;

//пин B правого энкодера подключается к цифр.пину 8:
const byte Encoder_Right_Pin_B = 8;

//переменная для сохранения последнего состояния на пине А правого энкодера:
byte Encoder_Right_Pin_A_Last;

//************************************************************************************************************
//Энкодер на левом моторе
//пин A левого энкодера подключается к цифр.пину 3 - используется для обработки прерываний (interrupt) int.0:
const byte Encoder_Left_Pin_A = 3;

//пин B левого энкодера подключается к цифр.пину 9:
const byte Encoder_Left_Pin_B = 9;
//переменная для сохранения последнего состояния на пине А левого энкодера:

byte Encoder_Left_Pin_A_Last;

//************************************************************************************************************
//Переменные для управления мотор-редукторами:

//Левый мотор M_Left:
int M_Left = 4;    //Направление вращения, вперёд-назад
int E_Left = 5;    //Скорость вращения, ШИМ 0-255

//Правый мотор M_Right:
int M_Right = 7;    //Направление вращения, вперёд-назад
int E_Right = 6;    //Скорость вращения, ШИМ 0-255

//************************************************************************************************************
//Переменные количества импульсов и направления вращения:

//количество импульсов на правом энкодере:
volatile int Impulses_Right;

//количество импульсов на левом энкодере:
volatile int Impulses_Left;

//модули количества импульсов, т.к. может быть отрицательным - в зависимости от напр.вращения
double abs_Impulses_Right;
double abs_Impulses_Left;

//направление вращения правого колеса:
volatile boolean Direction_Right;

//направление вращения левого колеса:
volatile boolean Direction_Left;

//************************************************************************************************************

boolean result_Right, result_Left;

double val_output_Right, val_output_Left;//Power supplied to the motor PWM value.

double Setpoint;

double Kp = 0.6, Ki = 5, Kd = 0; // 0.6, 5, 0

PID myPID_Right(&abs_Impulses_Right, &val_output_Right, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_Left(&abs_Impulses_Left, &val_output_Left, &Setpoint, Kp, Ki, Kd, DIRECT);

//************************************************************************************************************
//Функции движения:

//вперёд
void advance()
{
  
  digitalWrite(M_Left, HIGH);
  analogWrite(E_Left, val_output_Left);
  
  digitalWrite(M_Right, HIGH);
  analogWrite (E_Right, val_output_Right);
}

//назад
void back()//Motor reverse
{
  digitalWrite(M_Left, LOW);
  analogWrite(E_Left, val_output_Left);

  digitalWrite(M_Left, LOW);
  analogWrite(E_Left, val_output_Left);
}

//стоп
void Stop()//Motor stops
{
  digitalWrite(E_Right, LOW);
  digitalWrite(E_Left, LOW);
}

//************************************************************************************************************
void setup()
{
  delay(4000);

  int i;
  for (i = 4; i <= 7; i++) pinMode(i, OUTPUT);
  
  Serial.begin(9600);//Initialize the serial port
  
  Setpoint = 12;  //Set the output value of the PID

  myPID_Right.SetMode(AUTOMATIC);//PID is set to automatic mode
  myPID_Right.SetSampleTime(100);//Set PID sampling frequency is 100ms
  myPID_Left.SetMode(AUTOMATIC);//PID is set to automatic mode
  myPID_Left.SetSampleTime(100);//Set PID sampling frequency is 100ms
  
  EncoderInit();//Initialize the module
}

//************************************************************************************************************
void loop()
{
  advance();//Motor Forward
  
  abs_Impulses_Right = abs(Impulses_Right);
  abs_Impulses_Left = abs(Impulses_Left);
  
  result_Right = myPID_Right.Compute(); //PID conversion is complete and returns 1
  result_Left = myPID_Left.Compute();
  
  if (result_Left)
  {
    Serial.print("Impulses_Left: "); Serial.print(Impulses_Left); Serial.print("\t");
    Serial.print("val_output_Left: "); Serial.print(val_output_Left); Serial.print("\t");
    //Serial.println();
    Impulses_Left = 0; //Count clear, wait for the next count
  }

  if (result_Right)
  {
    Serial.print("Impulses_Right: "); Serial.print(Impulses_Right); Serial.print("\t");
    Serial.print("val_output_Right: "); Serial.print(val_output_Right); Serial.print("\t");
    Serial.println();
    Impulses_Right = 0; //Count clear, wait for the next count
  }
}

//************************************************************************************************************
void EncoderInit()
{
  Direction_Right = true;//default -> Forward
  Direction_Left = true;
  pinMode(Encoder_Right_Pin_B, INPUT);
  pinMode(Encoder_Left_Pin_B, INPUT);
  //attachInterrupt(0, wheelSpeed, CHANGE);
  attachInterrupt(1, wheelSpeed_Right, CHANGE);
  attachInterrupt(0, wheelSpeed_Left, CHANGE);
}

//************************************************************************************************************

void wheelSpeed_Right() {
  int LstatE_Left = digitalRead(Encoder_Right_Pin_A);
  if ((Encoder_Right_Pin_A_Last == LOW) && LstatE_Left == HIGH) {
    int val = digitalRead(Encoder_Right_Pin_B);
    if (val == LOW && Direction_Right) {
      Direction_Right = false;                       //Reverse
    }
    else if (val == HIGH && !Direction_Right) {
      Direction_Right = true;                        //Forward
    }
  }
  Encoder_Right_Pin_A_Last = LstatE_Left;
  if (!Direction_Right)  Impulses_Right++;
  else  Impulses_Right--;
}

//************************************************************************************************************

void wheelSpeed_Left() {
  int LstatE_Right = digitalRead(Encoder_Left_Pin_A);
  if ((Encoder_Left_Pin_A_Last == LOW) && LstatE_Right == HIGH) {
    int val = digitalRead(Encoder_Left_Pin_B);
    if (val == LOW && Direction_Left) {
      Direction_Left = false;                       //Reverse
    }
    else if (val == HIGH && !Direction_Left) {
      Direction_Left = true;                        //Forward
    }
  }
  Encoder_Left_Pin_A_Last = LstatE_Right;
  if (!Direction_Left)  Impulses_Left++;
  else  Impulses_Left--;
}

// the end
//************************************************************************************************************

//это кусок первоначального кода:
/*
void wheelSpeed_Right()
{
  int Lstate = digitalRead(Encoder_Right_Pin_A);
  if ((Encoder_Right_Pin_A_Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder_Right_Pin_B);
    if (val == LOW && Direction_Right)
    {
      Direction_Right = false; //Reverse
    }
    else if (val == HIGH && !Direction_Right)
    {
      Direction_Right = true;  //Forward
    }
  }
  Encoder_Right_Pin_A_Last = Lstate;

  if (!Direction_Right)  Impulses_Right++;
  else  Impulses_Right--;

}
*/
