/*********************************************************************************\
 *                                                                               *
 * This program controls water pump working cycle                                *
 * It is possible to set manually time for pump run and pump brake cycle         *
 *                                                                               *
\*********************************************************************************/

/*********************************************************************************
*                               Includes                                         *
**********************************************************************************/
 #include <EEPROM.h>
 #include <Wire.h> 
 #include <LiquidCrystal_I2C.h>

/*********************************************************************************
*                               Defines                                          *
**********************************************************************************/

/* Inut Pin defines */
#define INPUT_CLK                  4
#define INPUT_DT                   5
#define INPUT_SET_BUTTON           6
#define INPUT_START_BUTTON         2
#define INPUT_STOP_BUTTON          3
#define INPUT_SWITCH_UP            7

/* Output Pin defines */
#define OUTPUT_PUMP                10
#define OUTPUT_LED_PUMP_ON         8
#define OUTPUT_LED_PUMP_OFF        9

/* Timer Pump On defines */
#define TIMER_PUMP_ON_MAX          500 /* time in seconds */
#define TIMER_PUMP_ON_MIN          5   /* time in seconds */
#define TIMER_PUMP_ON_STEP         5   /* time in seconds */

/* Timer Pump brake defines */
#define TIMER_PUMP_BRAKE_MAX       60 /* time in minutes */
#define TIMER_PUMP_BRAKE_MIN       1  /* time in minutes */
#define TIMER_PUMP_BRAKE_STEP      1  /* time in minutes */

/* Upper limit switch debounce defines */
#define TIMER_UP_SW_DEBOUNCE       10 /* time in seconds */

/* EEPROM defines */
#define ADDR_EEPROM_VALID          0
#define ADDR_TIM_PUMP_ON_BYTE_0    1
#define ADDR_TIM_PUMP_ON_BYTE_1    2
#define ADDR_TIM_PUMP_BR_BYTE_0    3
#define ADDR_TIM_PUMP_BR_BYTE_1    4
#define EEPROM_VALID               1

/* LCD config */
#define LCD_ADDR                   0x27
#define LCD_SIZE_ROW               2
#define LCD_SIZE_COL               16

#define COUNTER_MIN 1

/* Macros for time base change */
#define Change_s_to_ms(_time_sec) ((_time_sec) * 1000)
#define Change_min_to_ms(_time_sec) ((_time_sec) * 60000)

/*********************************************************************************
*                       Structures definitions                                   *
**********************************************************************************/
typedef enum Config_Modes_Tag
{
  CONFIG_OFF,
  CONFIG_TIM_ON,
  CONFIG_TIM_BRAKE
} Config_Modes_T;

typedef struct Input_Signals_Tag
{
  bool enc_clk;
  bool enc_dt;
  bool button_mode;
  bool button_start;
  bool button_stop;
  bool limit_switch_up;
} Input_Signals_T;

typedef struct Output_Signals_Tag
{
  bool led_start;
  bool led_stop;
  bool pump_run;
} Output_Signals_T;

typedef enum Std_Return_Type_Tag
{
  RET_OK,
  RET_NOK,
  RET_PENDING
} Std_Return_Type_T;

typedef enum Pump_Cycle_Tag
{
  PUMP_CYCLE_OFF,
  PUMP_CYCLE_RUN,
  PUMP_CYCLE_BRAKE
} Pump_Cycle_T;

typedef enum Up_Limit_Switch_State_Tag
{
  UP_SWITCH_OFF,
  UP_SWITCH_ON
} Up_Limit_Switch_State_T;

typedef struct Timer_Eeprom_Byte_Type
{
  byte timer_byte_0;
  byte timer_byte_1;
} Timer_Eeprom_Byte_T;

typedef union Timer_Pump_Tag
{
  unsigned int        timer_value_int;
  Timer_Eeprom_Byte_T timer_value_byte;
} Timer_Pump_T;

/*********************************************************************************
*                         Global variables                                       *
**********************************************************************************/
static Timer_Pump_T              timer_value_pump_on;
static Timer_Pump_T              timer_value_pump_brake;
static unsigned long             timer_value_start          = 0;
static int                       current_value_CLK          = 0;
static int                       previous_value_CLK         = 0;
String                           encDir                     = "";
unsigned long                    time_now                   = 0;
unsigned long                    time_saved                 = 0;
unsigned long                    time_dif                   = 0;
const unsigned long              but_debounce_time_set      = 50;
unsigned long                    but_debounce_time_mode     = 0;
unsigned long                    but_debounce_time_pump     = 0;
unsigned long                    up_sw_debounce_time        = 0;
static Up_Limit_Switch_State_T   up_limit_switch_state      = UP_SWITCH_OFF;
static Input_Signals_T           inputs;
static Output_Signals_T          outputs;
LiquidCrystal_I2C                lcd(LCD_ADDR, LCD_SIZE_COL, LCD_SIZE_ROW);


/*********************************************************************************
*                       Function prototypes                                      *
**********************************************************************************/

static void Set_Timers(unsigned int *timer, int timer_min, int timer_max, int timer_step);
static void Serial_Print_Every_Mili(unsigned long time_set, bool button_state);
static Std_Return_Type_T Read_Inputs(Input_Signals_T *signals);
static Std_Return_Type_T Write_Outputs(Output_Signals_T signals);
static Std_Return_Type_T Check_For_Config_Mode_Change(bool *previous_config_button_state, const bool button_state, Config_Modes_T *config_mode, 
                                                      const unsigned long *but_debounce_time_set, unsigned long *but_debounce_time_mode);
static Std_Return_Type_T Timers_Setup_Handler(bool *previous_config_button_state, Config_Modes_T *config_mode, Pump_Cycle_T *pump_cycle);
static Std_Return_Type_T Pump_Control_Logic(const Input_Signals_T inputs, Output_Signals_T *outputs, Pump_Cycle_T *pump_cycle, 
                                            unsigned long *timer_value_start, unsigned int timer_value_pump_on, unsigned int timer_value_pump_brake);
static Std_Return_Type_T Pump_Turn_On(Up_Limit_Switch_State_T up_limit_switch_state, Output_Signals_T *outputs);
static Up_Limit_Switch_State_T Check_Up_Limit_Switch_Status(bool input_limit_switch_up, Up_Limit_Switch_State_T *up_limit_switch_state, 
                                                            unsigned long *up_sw_debounce_time);
static void Clean_Eeprom();
static void Prepare_Values_From_Eeprom(Timer_Pump_T *p_timer_value_pump_on, Timer_Pump_T *p_timer_value_pump_brake);
static void Write_Timer_Values_To_Eeprom(Timer_Pump_T *p_timer_value_pump_on, Timer_Pump_T *p_timer_value_pump_brake);
static void Lcd_Handling(Pump_Cycle_T pump_cycle, Config_Modes_T config_mode, Timer_Pump_T timer_value_pump_on, 
                         Timer_Pump_T timer_value_pump_brake, Up_Limit_Switch_State_T up_limit_switch_state);

/*********************************************************************************
*                          Setup Runnable                                        *
**********************************************************************************/
void setup() {
  /* Encoder pins config */
  pinMode(INPUT_CLK          , INPUT_PULLUP);
  pinMode(INPUT_DT           , INPUT_PULLUP);

  /* Buttons pins config */
  pinMode(INPUT_SET_BUTTON   , INPUT_PULLUP);
  pinMode(INPUT_START_BUTTON , INPUT_PULLUP);
  pinMode(INPUT_STOP_BUTTON  , INPUT_PULLUP);
  pinMode(INPUT_SWITCH_UP    , INPUT_PULLUP);

  /* OUTPUT pins config */
  pinMode(OUTPUT_LED_PUMP_ON , OUTPUT);
  pinMode(OUTPUT_LED_PUMP_OFF, OUTPUT);
  pinMode(OUTPUT_PUMP        , OUTPUT);
  
  /* Serial monitor config*/
  Serial.begin(9600);
  
  /* Initial setup for CLK variable */
  previous_value_CLK = digitalRead(INPUT_CLK);

  /* Initialise input pins variables */
  inputs.enc_clk         = HIGH;
  inputs.enc_dt          = HIGH;
  inputs.button_mode     = HIGH;
  inputs.button_start    = HIGH;
  inputs.button_stop     = LOW;
  inputs.limit_switch_up = LOW;

  /* Initialise output pins variables */
  outputs.led_start  = LOW;
  outputs.led_stop   = HIGH;
  outputs.pump_run   = LOW;

  /* EEPROM handling */
  Prepare_Values_From_Eeprom(&timer_value_pump_on, &timer_value_pump_brake);

  /* LCD prepare */
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  
}


/*********************************************************************************
*                           Main Runnable                                        *
**********************************************************************************/
void loop()
{
  static Config_Modes_T config_mode = CONFIG_OFF;
  static bool previous_config_button_state = HIGH;
  static Pump_Cycle_T pump_cycle = PUMP_CYCLE_OFF;

  Read_Inputs(&inputs);

  Check_Up_Limit_Switch_Status(inputs.limit_switch_up, &up_limit_switch_state, &up_sw_debounce_time);

  Timers_Setup_Handler(&previous_config_button_state, &config_mode, pump_cycle);

  Pump_Control_Logic(inputs, &outputs, &pump_cycle, &timer_value_start, timer_value_pump_on.timer_value_int, timer_value_pump_brake.timer_value_int);

  Write_Outputs(outputs);

  Lcd_Handling(pump_cycle, config_mode, timer_value_pump_on, timer_value_pump_brake, up_limit_switch_state);
  
}


/*********************************************************************************
*                         Function definitions                                   *
**********************************************************************************/

static void Lcd_Handling(Pump_Cycle_T pump_cycle, Config_Modes_T config_mode, Timer_Pump_T timer_value_pump_on, 
                         Timer_Pump_T timer_value_pump_brake, Up_Limit_Switch_State_T up_limit_switch_state)
{
  static Up_Limit_Switch_State_T previous_up_limit_switch_state = up_limit_switch_state;
  static unsigned int previous_timer_value_pump_brake           = timer_value_pump_brake.timer_value_int;
  static unsigned int previous_timer_value_pump_on              = timer_value_pump_on.timer_value_int;
  static Config_Modes_T previous_config_mode                    = config_mode;
  static Pump_Cycle_T previous_pump_cycle                       = pump_cycle;
  String state;

  if((previous_timer_value_pump_on != timer_value_pump_on.timer_value_int) ||
     (previous_timer_value_pump_brake != timer_value_pump_brake.timer_value_int) ||
     (pump_cycle != previous_pump_cycle) || (config_mode != previous_config_mode) ||
     (previous_up_limit_switch_state != up_limit_switch_state))
  {
    lcd.clear();
  }

  switch(config_mode)
  {
    case CONFIG_OFF:
      lcd.setCursor(0, 1);
      lcd.print("Zal=");
      lcd.print(timer_value_pump_on.timer_value_int);
      lcd.print("s");
      lcd.setCursor(9, 1);
      lcd.print("Wyl=");
      lcd.print(timer_value_pump_brake.timer_value_int);
      lcd.print("m");
      lcd.setCursor(0, 0);
      lcd.print("Stan: ");
      lcd.setCursor(6, 0);

      if(UP_SWITCH_ON == up_limit_switch_state)
      {
        state = "ZB. PELNY";
      }
      else if(PUMP_CYCLE_OFF == pump_cycle)
      {
        state = "STOP";
      }
      else if(PUMP_CYCLE_RUN == pump_cycle)
      {
        state = "POMPA ZAL";
      }
      else
      {
        state = "PRZERWA";
      }
  
      lcd.print(state);
      break;

    case CONFIG_TIM_ON:
      lcd.setCursor(0, 0);
      lcd.print("Czas zalaczenia");
      lcd.setCursor(7, 1);
      lcd.print(timer_value_pump_on.timer_value_int);
      lcd.print("s");
      break;

    case CONFIG_TIM_BRAKE:
      lcd.setCursor(2, 0);
      lcd.print("Czas przerwy");
      lcd.setCursor(6, 1);
      lcd.print(timer_value_pump_brake.timer_value_int);
      lcd.print("min");
      break;
    default:
      /* do nothing */
      break;
  }


  previous_pump_cycle = pump_cycle;
  previous_config_mode = config_mode;
  previous_timer_value_pump_on = timer_value_pump_on.timer_value_int;
  previous_timer_value_pump_brake = timer_value_pump_brake.timer_value_int;
  previous_up_limit_switch_state = up_limit_switch_state;

}

/*------------------------------------------------------------------------------------------*/

static void Write_Timer_Values_To_Eeprom(Timer_Pump_T *p_timer_value_pump_on, Timer_Pump_T *p_timer_value_pump_brake)
{
  EEPROM.write(ADDR_TIM_PUMP_ON_BYTE_0, p_timer_value_pump_on->timer_value_byte.timer_byte_0);
  EEPROM.write(ADDR_TIM_PUMP_ON_BYTE_1, p_timer_value_pump_on->timer_value_byte.timer_byte_1);
  EEPROM.write(ADDR_TIM_PUMP_BR_BYTE_0, p_timer_value_pump_brake->timer_value_byte.timer_byte_0);
  EEPROM.write(ADDR_TIM_PUMP_BR_BYTE_1, p_timer_value_pump_brake->timer_value_byte.timer_byte_1);
  Serial.println("Timer values written to EEPROM");
}

/*------------------------------------------------------------------------------------------*/

static void Prepare_Values_From_Eeprom(Timer_Pump_T *p_timer_value_pump_on, Timer_Pump_T *p_timer_value_pump_brake)
{
  if(EEPROM_VALID != EEPROM.read(ADDR_EEPROM_VALID))
  {
    Clean_Eeprom();
    EEPROM.write(ADDR_TIM_PUMP_ON_BYTE_0, TIMER_PUMP_ON_MIN);
    EEPROM.write(ADDR_TIM_PUMP_ON_BYTE_1, 0);
    EEPROM.write(ADDR_TIM_PUMP_BR_BYTE_0, TIMER_PUMP_BRAKE_MIN);
    EEPROM.write(ADDR_TIM_PUMP_BR_BYTE_1, 0);
    EEPROM.write(ADDR_EEPROM_VALID, EEPROM_VALID);
    Serial.println("EEPROM cleaned and validated");

    p_timer_value_pump_on->timer_value_byte.timer_byte_0 = EEPROM.read(ADDR_TIM_PUMP_ON_BYTE_0);
    p_timer_value_pump_on->timer_value_byte.timer_byte_1 = EEPROM.read(ADDR_TIM_PUMP_ON_BYTE_1);
    p_timer_value_pump_brake->timer_value_byte.timer_byte_0 = EEPROM.read(ADDR_TIM_PUMP_BR_BYTE_0);
    p_timer_value_pump_brake->timer_value_byte.timer_byte_1 = EEPROM.read(ADDR_TIM_PUMP_BR_BYTE_1);
  }
  else
  {
    p_timer_value_pump_on->timer_value_byte.timer_byte_0 = EEPROM.read(ADDR_TIM_PUMP_ON_BYTE_0);
    p_timer_value_pump_on->timer_value_byte.timer_byte_1 = EEPROM.read(ADDR_TIM_PUMP_ON_BYTE_1);
    p_timer_value_pump_brake->timer_value_byte.timer_byte_0 = EEPROM.read(ADDR_TIM_PUMP_BR_BYTE_0);
    p_timer_value_pump_brake->timer_value_byte.timer_byte_1 = EEPROM.read(ADDR_TIM_PUMP_BR_BYTE_1);
  }

  Serial.print("Values loaded to timers: timer_value_pump_on = ");
  Serial.print(p_timer_value_pump_on->timer_value_int);
  Serial.print(" timer_value_pump_brake = ");
  Serial.println(p_timer_value_pump_brake->timer_value_int);
}

/*------------------------------------------------------------------------------------------*/

static void Clean_Eeprom()
{
  unsigned int idx = 0;

  for(idx = 0; idx < EEPROM.length(); idx++)
  {
    EEPROM.write(idx, 0);
  }
}

/*------------------------------------------------------------------------------------------*/

static Up_Limit_Switch_State_T Check_Up_Limit_Switch_Status(bool input_limit_switch_up, Up_Limit_Switch_State_T *up_limit_switch_state, 
                                                            unsigned long *up_sw_debounce_time)
{
  if(UP_SWITCH_OFF == *up_limit_switch_state)
  {
    if(HIGH == input_limit_switch_up)
    {
      if((millis() - *up_sw_debounce_time) > Change_s_to_ms(TIMER_UP_SW_DEBOUNCE))
      {
        *up_limit_switch_state = UP_SWITCH_ON;
        Serial.println("Limit switch activated");
      }
      else
      {
        /* do nothig */
      }
    }
    else
    {
      *up_sw_debounce_time = millis();
    }
  }
  else
  {
    if(LOW == input_limit_switch_up)
    {
      *up_limit_switch_state = UP_SWITCH_OFF;
      *up_sw_debounce_time = millis();
      Serial.println("Limit switch deactivated");
    }
    else
    {
      /* do nothing */
    }
  }

  return *up_limit_switch_state;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Pump_Control_Logic(const Input_Signals_T inputs, Output_Signals_T *outputs, Pump_Cycle_T *pump_cycle, 
                                            unsigned long *timer_value_start, unsigned int timer_value_pump_on, unsigned int timer_value_pump_brake)
{
  if((LOW == inputs.button_start) && (LOW == inputs.button_stop) && (PUMP_CYCLE_OFF == *pump_cycle))
  {
    outputs->led_stop  = LOW;
    outputs->led_start = HIGH;
    Pump_Turn_On(up_limit_switch_state, outputs);
    *pump_cycle = PUMP_CYCLE_RUN;
    *timer_value_start = millis();
    Serial.println("Pump cycle PUMP_CYCLE_RUN!");
  }
  else if((HIGH == inputs.button_stop) && ((PUMP_CYCLE_RUN == *pump_cycle) || (PUMP_CYCLE_BRAKE == *pump_cycle)))
  {
    outputs->led_stop  = HIGH;
    outputs->led_start = LOW;
    outputs->pump_run  = LOW;
    *pump_cycle = PUMP_CYCLE_OFF;
    Serial.println("Pump cycle PUMP_CYCLE_OFF");
  }
  else
  {
    if(PUMP_CYCLE_RUN == *pump_cycle)
    {
      if((millis() - *timer_value_start) > Change_s_to_ms(timer_value_pump_on))
      {
        outputs->pump_run = LOW;
        *pump_cycle = PUMP_CYCLE_BRAKE;
        *timer_value_start = millis();
        Serial.println("Change from PUMP_CYCLE_RUN to PUMP_CYCLE_BRAKE");
      }
      else
      {
        Pump_Turn_On(up_limit_switch_state, outputs);
      }
    }
    else if(PUMP_CYCLE_BRAKE == *pump_cycle)
    {
      if((millis() - *timer_value_start) > Change_min_to_ms(timer_value_pump_brake))
      {
        Pump_Turn_On(up_limit_switch_state, outputs);
        *pump_cycle = PUMP_CYCLE_RUN;
        *timer_value_start = millis();
        Serial.println("Change from PUMP_CYCLE_BRAKE to PUMP_CYCLE_RUN");
      }
      else
      {
        outputs->pump_run = LOW;
      }
    }
    else
    {
      /* do nothing */
    }
  }

  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Pump_Turn_On(Up_Limit_Switch_State_T up_limit_switch_state,  Output_Signals_T *outputs)
{
  if(UP_SWITCH_OFF == up_limit_switch_state)
  {
    outputs->pump_run = HIGH;
  }
  else
  {
    outputs->pump_run = LOW;
  }

  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Timers_Setup_Handler(bool *previous_config_button_state, Config_Modes_T *config_mode, Pump_Cycle_T pump_cycle)
{
  static bool timer_values_saved_eeprom = true;

  if(PUMP_CYCLE_OFF == pump_cycle)
  {
    Check_For_Config_Mode_Change(previous_config_button_state, inputs.button_mode, config_mode, 
                                 &but_debounce_time_set, &but_debounce_time_mode);
  }
  else
  {
    /* do nothing */
  }
  

  if(CONFIG_TIM_ON == *config_mode)
  {
    timer_values_saved_eeprom = false;
    Set_Timers(&timer_value_pump_on.timer_value_int, TIMER_PUMP_ON_MIN, TIMER_PUMP_ON_MAX, TIMER_PUMP_ON_STEP);
  }
  else if(CONFIG_TIM_BRAKE == *config_mode)
  {
    Set_Timers(&timer_value_pump_brake.timer_value_int, TIMER_PUMP_BRAKE_MIN, TIMER_PUMP_BRAKE_MAX, TIMER_PUMP_BRAKE_STEP);
  }
  else
  {
    if(!timer_values_saved_eeprom)
    {
      Write_Timer_Values_To_Eeprom(&timer_value_pump_on, &timer_value_pump_brake);
      timer_values_saved_eeprom = true;
    }
  }

  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Check_For_Config_Mode_Change(bool *previous_config_button_state, const bool button_state, Config_Modes_T *config_mode, 
                                                      const unsigned long *but_debounce_time_set, unsigned long *but_debounce_time_mode)
{
  if((LOW == button_state) && (button_state != *previous_config_button_state) && 
    ((millis() - *but_debounce_time_mode) > *but_debounce_time_set))
  {
    switch(*config_mode)
    {
      case CONFIG_OFF:
        *config_mode = CONFIG_TIM_ON;
        break;
      
      case CONFIG_TIM_ON:
        *config_mode = CONFIG_TIM_BRAKE;
        break;
      
      case CONFIG_TIM_BRAKE:
        *config_mode = CONFIG_OFF;
        break;
      
      default:
        /*do nothing */
        break;
    }
    
    *but_debounce_time_mode = millis();
  }
  
  *previous_config_button_state = button_state;

  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Read_Inputs(Input_Signals_T *signals)
{
  signals->enc_clk         = digitalRead(INPUT_CLK);
  signals->enc_dt          = digitalRead(INPUT_DT);
  signals->button_mode     = digitalRead(INPUT_SET_BUTTON);
  signals->button_start    = digitalRead(INPUT_START_BUTTON);
  signals->button_stop     = digitalRead(INPUT_STOP_BUTTON);
  signals->limit_switch_up = digitalRead(INPUT_SWITCH_UP);

  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static Std_Return_Type_T Write_Outputs(Output_Signals_T signals)
{
  digitalWrite(OUTPUT_LED_PUMP_ON, signals.led_start);
  digitalWrite(OUTPUT_LED_PUMP_OFF, signals.led_stop);
  digitalWrite(OUTPUT_PUMP, signals.pump_run);
  
  return RET_OK;
}

/*------------------------------------------------------------------------------------------*/

static void Set_Timers(unsigned int *timer, int timer_min, int timer_max, int timer_step)
{
  current_value_CLK = digitalRead(INPUT_CLK);

  if(current_value_CLK != previous_value_CLK)
  {
    if(digitalRead(INPUT_DT) != current_value_CLK)
    {
      if(*timer >= timer_min + timer_step)
      {
        *timer -= timer_step; 
      }
      encDir = "CCW";
    }
    else
    {
      if(*timer <= timer_max + timer_step)
      {
        *timer += timer_step; 
      }
      encDir = "CW";
    }

    Serial.print("Direction = ");
    Serial.print(encDir);
    Serial.print("    timer = ");
    Serial.println(*timer);
  }

  previous_value_CLK = current_value_CLK;
}

/*------------------------------------------------------------------------------------------*/

static void Serial_Print_Every_Mili(unsigned long time_set, bool button_state)
{
  time_now = millis();
  time_dif = time_now - time_saved;
  if(time_dif >= time_set)
  {
    Serial.print("but_state = ");
    Serial.println(button_state);
    time_saved = time_now;
  }
}
