/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

#define pin_key_fw      7
#define pin_key_left    8
#define pin_key_right   5
#define pin_key_enter   6
#define pin_indicator   A2
#define pin_motor_left  9
#define pin_motor_right 10

#define MAX_CMD     100

enum ENM_KEY
{
  KEY_LEFT,
  KEY_RIGHT,
  KEY_FW,
  KEY_ENTER,
  KEY_CNT 
  };

uint8_t pin_keys[KEY_CNT] = {
  pin_key_left, 
  pin_key_right,
  pin_key_fw,
  pin_key_enter
};

enum ENM_AP_STATE{
  AP_INIT,
  AP_INPUT_CMD,
  AP_OPER_DELAY,
  AP_WRITE_CMD,
  AP_RUN,
  AP_STATE_CNT
  };

enum ENM_KEY_EVENT
{
  KEVENT_NONE,
  KEVENT_CLICKED,
  KEVENT_PRESSED,
  KEVENT_CNT
  };

typedef struct structCommands_list{
  //counts of commands
  uint16_t cnt;
  
  //high : key code, low : counts
  uint16_t cmds[MAX_CMD];
} CommandList;

uint8_t ap_state = 0;
uint8_t clicked_key = 0;
uint8_t command_key = 0;
uint8_t clicked_cnt = 0;

CommandList g_command_list;
CommandList g_command_list_rd;

uint8_t key_proc();
uint8_t event_proc(uint8_t key_event);
void app_proc(uint8_t key_event);
void Motor_direction_change(uint8_t dir);
void Motor_stop();

#define LED_OFF 0
#define LED_ON 1

void led_onoff(bool bOn)
{
  if(bOn)
    digitalWrite(pin_indicator, LOW);
  else
    digitalWrite(pin_indicator, HIGH);
}

void setup() {
//  Serial.begin(115200);
//  Serial.println("Start");
  // put your setup code here, to run once:
  for(uint8_t i = 0; i < KEY_CNT; i++)
  {
    pinMode(pin_keys[i], INPUT_PULLUP);
  }
  pinMode(pin_motor_left, OUTPUT);
  pinMode(pin_motor_right, OUTPUT);
  pinMode(pin_indicator, OUTPUT);
  
  digitalWrite(pin_motor_right, HIGH);
  digitalWrite(pin_motor_left, HIGH);
  
  led_onoff(LED_ON);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t _k_event = 0;
  static uint32_t tm = 0;
  tm = micros();
  
  _k_event = key_proc();
  event_proc(_k_event);
  app_proc(_k_event);
  
  _k_event = 0;
  delayMicroseconds(1000 + tm - micros());
}

uint8_t key_proc()
{
  static uint8_t key_state[KEY_CNT] = {1,1,1,1}, prev_key_state[KEY_CNT] = {1,1,1,1};
  static uint32_t pressing_cnt = 0;
  static bool _enter_pressed = false;
  
  uint8_t ret = 0;
  static uint8_t tm_cnt = 0;
  if(tm_cnt++ < 10)
  {
    return ret;
  }else
  {
    tm_cnt = 0;
  }
  for(uint8_t i = 0; i < KEY_CNT; i++)
  {
    key_state[i] = digitalRead(pin_keys[i]);
    if(i == KEY_ENTER && key_state[i] == LOW && !_enter_pressed)
    {
      if(pressing_cnt++ > 800) //3 seconds
      {
        pressing_cnt = 0;
        ret = KEVENT_PRESSED;
        _enter_pressed = true;
        break;
      }
    }else if(prev_key_state[i] != key_state[i] && key_state[i] == HIGH)
    {
      if(i != KEY_ENTER || !_enter_pressed)
      {
        clicked_key = i;
        ret = KEVENT_CLICKED;
        if(i != KEY_ENTER)
          command_key = i;
        break;
      }
      _enter_pressed = false;
      pressing_cnt = 0;
    }
  }
  
  memcpy(prev_key_state, key_state, sizeof(key_state));
  return ret;
}

uint8_t event_proc(uint8_t key_event)
{
  static uint8_t prev_clicked_key = 0;
  uint8_t __event = 0;
  
  if(key_event == KEVENT_PRESSED)
  {
    Serial.println("key pressed");
    clicked_cnt = 0;
  }else if(key_event == KEVENT_CLICKED && clicked_key != KEY_ENTER)
  {
    switch(clicked_key)
    {
      case KEY_LEFT:
        Serial.println("left key");
      break;
      case KEY_RIGHT:
        Serial.println("right key");
      break;
      case KEY_FW:
        Serial.println("fw key");
      break;
      case KEY_ENTER:
        Serial.println("enter key");
      break;
    }
    if(prev_clicked_key == clicked_key)
    {
      clicked_cnt++;
    }else
      clicked_cnt = 1;
    prev_clicked_key = clicked_key;
  }
}

void app_proc(uint8_t key_event)
{
  static uint8_t nBlink = 0;
  static uint32_t tm_blink = 0;
  
  switch(ap_state)  
  {
  case AP_INIT:
    ap_state = AP_INPUT_CMD;
    memset(&g_command_list, 0, sizeof(CommandList));
    memset(&g_command_list_rd, 0, sizeof(CommandList));
    Serial.println("enter command mode");    
  break;
  case AP_INPUT_CMD:
    if(nBlink > 0)
    {
      if(millis() - tm_blink > 500)
      {
        nBlink--;
        tm_blink = millis();
        led_onoff(LED_ON);
      }else
      {
        led_onoff(LED_OFF);
      }
    }
    if(key_event == KEVENT_CLICKED && clicked_key == KEY_ENTER)
    {
      Serial.println("add command");
      if(g_command_list_rd.cnt < MAX_CMD && clicked_cnt > 0)
      {
        g_command_list_rd.cmds[g_command_list_rd.cnt] = (command_key << 8) + clicked_cnt;
        clicked_cnt = 0;
        g_command_list_rd.cnt++;
        nBlink = 1;
        tm_blink = millis();
      }
    }else if(key_event == KEVENT_PRESSED && g_command_list_rd.cnt > 0)
    {
      ap_state = AP_OPER_DELAY;
      Serial.println("storing commands...");
      Serial.print(g_command_list_rd.cnt);
      Serial.println(" commands are ready");
      for(int i = 0; i < g_command_list_rd.cnt; i++)
      {
        Serial.print(g_command_list_rd.cmds[i], HEX);
        Serial.print(", ");
      }
      
      nBlink = 8;
    }
  break;
  case AP_OPER_DELAY:
	led_onoff(LED_ON);
	delay(500);
	ap_state = AP_WRITE_CMD;
	
  break;
  case AP_WRITE_CMD:
	if(key_event == KEVENT_CLICKED && clicked_key == KEY_ENTER)
	{
		memcpy(&g_command_list, &g_command_list_rd, sizeof(CommandList));
		led_onoff(LED_OFF);
		ap_state = AP_RUN;
	}
    if(millis() - tm_blink > 500)
    {
	    tm_blink = millis();
	    digitalWrite(pin_indicator, (digitalRead(pin_indicator) == HIGH)?LOW:HIGH);
    }
  break;
  case AP_RUN:
	delay(1000);
    
	uint16_t cmd = g_command_list.cmds[0];
    uint8_t sec = cmd & 0xff;
    uint8_t dir = (cmd >> 8) & 0xff;
	
	for(int i = 0; i < g_command_list.cnt; i++)
	{
        cmd = g_command_list.cmds[i];
        dir = (cmd >> 8) & 0xff;
		sec = cmd & 0xff;
        Motor_direction_change(dir);
		delay(320*sec);
	}
	Motor_stop();
	ap_state = AP_WRITE_CMD;
  break;
  }
}

void Motor_stop()
{
  digitalWrite(pin_motor_left, HIGH);
  digitalWrite(pin_motor_right, HIGH);  
//  delay(10);
}
void Motor_direction_change(uint8_t dir)
{
  static uint8_t prev_dir = 0xff;
/*  if(dir != prev_dir)
  {
    Motor_stop();
    delay(2);
  }*/
  switch(dir)
  {
    case KEY_FW:
    digitalWrite(pin_motor_right, LOW);
    digitalWrite(pin_motor_left, LOW);
//    Serial.println("one step foward");
    break;
    case KEY_LEFT:
    digitalWrite(pin_motor_right, LOW);
    digitalWrite(pin_motor_left, HIGH);
//    Serial.println("one step left");
    break;
    case KEY_RIGHT:
    digitalWrite(pin_motor_right, HIGH);
    digitalWrite(pin_motor_left, LOW);
//    Serial.println("one step right");
    break;
  }
  prev_dir = dir;
}
