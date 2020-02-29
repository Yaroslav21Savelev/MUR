#define SPEED 255
#define M_1 7
#define M_2 6
#define H_1 3
#define H_2 4
//#define H_1 12
//#define H_2 13
#define M_OFF 1000
#define H_TIME 11000
#define O_b 11
#define C_b 10
volatile bool state = 0;
volatile bool pos = 0;
volatile unsigned long timer = millis(), h_timer = millis();
int dir = 0;
volatile int s_circle = 0;
void setup()
{
  //Serial.begin(2000000);
  pinMode(M_1, OUTPUT);
  pinMode(M_2, OUTPUT);
  pinMode(H_1, OUTPUT);
  pinMode(H_2, OUTPUT);
  pinMode(O_b, INPUT_PULLUP);
  pinMode(C_b, INPUT_PULLUP);
  timerInit();
  delay(2000);
}
void loop()
{
  
  
  motor(-1);
  delay(14000);
  holder(1);
  while(!state)
  {
    ;
  }
  delay(4000);
  motor(1);
  delay(20000);
  holder(0);
  while(!state)
  {
    ;
  }
  delay(5000);
  /*
  motor(-1);
  while(!state)
  {
    ;
  }
  holder(1);
  delay(3000);
  motor(1);
  while(!state)
  {
    ;
  }
  holder(0);
  delay(3000);*/
}

void backend()
{
  static int motor_dir = -1;
  
  if(pos == 0)
  {
    if(millis() - h_timer <= H_TIME)
    {
      digitalWrite(H_1, HIGH);
      digitalWrite(H_2, LOW);
      state = 0;
    }
    else
    {
      digitalWrite(H_1, LOW);
      digitalWrite(H_2, LOW);
      state = 1;
    }
    
  }
  else if(pos == 1)
  {
    if(millis() - h_timer <= H_TIME)
    {
      digitalWrite(H_1, LOW);
      digitalWrite(H_2, HIGH);
      state = 0;
    }
    else
    {
      digitalWrite(H_1, LOW);
      digitalWrite(H_2, LOW);
      state = 1;
    }
  }
    
    //Serial.println(millis() - timer);
  if(millis() - timer >= M_OFF)
  {
    if(dir == 1)
    {
      
      digitalWrite(M_1, LOW);
      analogWrite(M_2, SPEED);
      //Serial.println("dn");
    }
    else if(dir == -1)
    {
      
      analogWrite(M_1, SPEED);
      digitalWrite(M_2, LOW);
      //Serial.println("up");
    }
    else
    {
      
      digitalWrite(M_1, LOW);
      digitalWrite(M_2,LOW);
      //Serial.println("st");
    }
  }
  
}
void motor(int p)
{
  //Serial.println(dir);
  if(p != dir)
  {
    digitalWrite(M_1, LOW);
    digitalWrite(M_2, LOW);
    timer = millis();
    dir = p;    
    Serial.println("Stop");
  }
}
void holder(bool p)
{
  if(p != pos)
  {
    pos = p;
    state = 0;
    h_timer = millis();
  }
}
void timerInit()
{
  TCCR0A |= (1 << WGM01);
  OCR0A = 0xF9;
  TIMSK0 |= (1 << OCIE0A);
  TCCR0B |= (1 << CS01) | (1 << CS00);
  sei();
}

ISR (TIMER0_COMPA_vect)
{
  backend();
}
