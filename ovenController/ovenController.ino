// Libraries
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <OneWire.h> // https://github.com/PaulStoffregen/OneWire
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal

// Pins
#define TEMP_SENS_PIN 2 
#define RELAY_PIN 12

// Key easy to adjust values
float kP = 8;
int estOvershoot = 18;
float fallGradient = 0.6;
float minRelayPower = 1;
float maxRelayPower = 30;


// Definitions for screen state
#define SET_SCREEN 1
#define CURRENT_TEMP_SCREEN 0
#define RIGHT 0
#define UP 1
#define DOWN 2
#define SELECT 3

// Definitions for control
#define IDLING 0
#define BEANS 1
#define COAST 2
#define PROP 3

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10;

// Screen and state stuff
bool screenState = CURRENT_TEMP_SCREEN; // sets the screen state to display temperature
int butCheckPeriod = 12;
int screenRefreshPeriod = 1000;
unsigned long lastScreenRefresh = 450;
unsigned long lastButCheck = 200;
unsigned long lastTempCheck = 300;
unsigned long lastRelayUpdate = 0;
int relayUpdatePeriod = 500;
float fallDetectThresh = 0.5;

int tempCheckPeriod = 1000;
unsigned long relayStartTime = 200;
unsigned long relayPeriod = 25000;
unsigned long relayOnTime = 0;
float slowFall = 4;

// Control stuff
int controlState = IDLING;
bool relayState = 0;
float relayPower = 0;

//Temperature stuff
float currentTemp = 420;
int setTemp  = 60;
int temporarySetTemp = 60;
int setTempStep = 1;
float maxTemp = 0;

// Button stuff
bool prevButState[4] = {0,0,0,0};
bool currButState[4] = {0,0,0,0};

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(TEMP_SENS_PIN);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);


void setup() {
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Oven Controller");
  lcd.setCursor(0,1);
  lcd.print("By Caleb Jackson");
  Serial.begin(9600);
  sensors.begin();
  pinMode(RELAY_PIN, OUTPUT);
  updateTempAndControl();
  delay(2000);
  if(currentTemp < 0) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temp sens bad");
    while(1) {
      delay(1000);
    }
  }
}


void loop()
{  
  if((millis() - lastScreenRefresh) > screenRefreshPeriod) {
    updateScreen();
    lastScreenRefresh = millis();
  }

  if(((millis()-lastButCheck) > butCheckPeriod)) {
    checkButtons();
    updateState();
    lastButCheck = millis();
  }

  if((millis() - lastTempCheck) > tempCheckPeriod) {
    updateTempAndControl();
    lastTempCheck = millis();
  }

  if((millis() - lastRelayUpdate) > 10) {
    lastRelayUpdate = millis();
    updateRelay();
  }
} 


void updateScreen()
{
  // If screen is in display current temperature state
  if(screenState == CURRENT_TEMP_SCREEN) {
    temporarySetTemp = setTemp;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Current Temp");
    lcd.setCursor(0,1);
    lcd.print(currentTemp);
  }

  // If screen is in display set temperature state
  if(screenState == SET_SCREEN) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Temp");
    lcd.setCursor(0,1);
    lcd.print(temporarySetTemp);
  }
}


void checkButtons() 
{
  int butVoltage;
  butVoltage = analogRead(0);
  if (butVoltage < 60) {
    //RIGHT
    currButState[RIGHT] = true; 
  } else if ((butVoltage < 200)) {
    // UP
    currButState[UP] = true;
  } else if (butVoltage < 400){
    // DOWN
    currButState[DOWN] = true;
  } else if ((butVoltage < 800) && ( butVoltage > 600)) {
    // SELECT
    currButState[SELECT] = true;
  }
}


void buttonsUsed()
{
  for(int i = 0; i <= SELECT; i++) {
    prevButState[i] = currButState[i];
    currButState[i] = 0;
  }
}


void updateState()
{
  if((currButState[RIGHT] == true) && (prevButState[RIGHT] == false)) {
    screenState = !screenState;
    updateScreen();
  }
  else if ((currButState[UP] == true) && (prevButState[UP] == false) && (screenState == SET_SCREEN)) {
    if(temporarySetTemp <= 145) {
      temporarySetTemp = temporarySetTemp + setTempStep;
      updateScreen();
    }
  }
  else if ((currButState[DOWN] == true) && (prevButState[DOWN] == false) && (screenState == SET_SCREEN)) {
    if(temporarySetTemp >= 41) {
      temporarySetTemp = temporarySetTemp - setTempStep;
      updateScreen();         
    }
  }
  else if((currButState[SELECT] == true) && (prevButState[SELECT] == false) && (screenState == SET_SCREEN)) {
    setTemp = temporarySetTemp;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Starting...");
    lcd.setCursor(0,1);
    lcd.print("Target: ");
    lcd.setCursor(9,1);
    lcd.print(setTemp);
    Serial.println("Starting");
    if((currentTemp + 25) > setTemp) {
      //No point waiting for beans and coasting:(, might as well do PROP immediatley
      controlState = PROP;
    } else {
      controlState = BEANS;
    }
    slowFall = (setTemp * 0.1);
    relayStartTime = millis();
    delay(1000);
    screenState = CURRENT_TEMP_SCREEN;
  }
  buttonsUsed();
}


void updateTempAndControl() {
  // Read the temperature that the sensor prepared
  currentTemp = sensors.getTempCByIndex(0);
  
  if(currentTemp > 0) {
    // Stops bad readings from being used
    
    // If in beans control
    if(controlState == BEANS) {
      if((setTemp - currentTemp) > estOvershoot) {
        relayPower = 100;
      }else {
        Serial.println("Coasting Now");
        relayPower = 0;
        controlState = COAST;
      }
    }
  
    // If we are coasting after beans control
    if(controlState == COAST) {
      if(currentTemp > maxTemp) {
        maxTemp = currentTemp;
      }
      if((maxTemp - currentTemp) > fallDetectThresh) {
        controlState = PROP;
      }
    }

    // If we are in proportional control
    if(controlState == PROP) {
      if(currentTemp > setTemp) {
        relayPower = slowFall - (currentTemp - setTemp) * fallGradient;
        if(relayPower < minRelayPower) {
          relayPower = 0;
        }
      } else {
        relayPower = (setTemp - currentTemp)*kP + slowFall;
        if(relayPower > maxRelayPower) {
          relayPower = maxRelayPower;
        }
      }
    }
  
    if(currentTemp > (setTemp + 30)) {
      //kill
      digitalWrite(RELAY_PIN, LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Over Temp Error");
      while(1) {
        Serial.println("Saftey threshold hit, Oven OFF");
        delay(1000);
      }
    }
    
    // Ask the temperature sensor to prepare another reading for next time
    sensors.setWaitForConversion(false);  // makes it async
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
  
    Serial.print(currentTemp);
    Serial.print(",");
    Serial.print(controlState);
    Serial.print(",");
    Serial.println(relayPower);
  } else {
    // If the temp sensor gives a bad reading, then power is cut to the
    // element until it sorts itself out.
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Bad temp read");
    relayPower = 0;
  }
}


// Controls the state of the relay depending on the relayPower
void updateRelay()
{
  if((relayStartTime + relayPeriod) < millis()) {
    relayStartTime = millis();
    relayOnTime = (relayPeriod*relayPower)/100;
  }
  if((relayStartTime + relayOnTime) > millis()) {
    //We are in the high part of the pwm cycle
    relayState = true;
  } else {
    // In the low part of the pwm cycle
    relayState = false;
  }
  
  digitalWrite(RELAY_PIN, relayState);
}
