#include <DallasTemperature.h>
#include <OneWire.h>
#include <LiquidCrystal.h>

#define ONE_WIRE_BUS 2 
#define RELAY_PIN 13

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
#define HYSTER 3
#define TIME_FOR_1C 6792


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
int tempCheckPeriod = 1000;

// Control stuff
int controlState = IDLING;
int upperThresh = 1;
int lowerThresh = 5;
bool relayState = 0;
int estOvershoot = 18;



//Temperature stuff
int currentTemp = 420;
int setTemp  = 60;
int temporarySetTemp = 60;
int setTempStep = 5;
int maxTemp = 0;


// Button stuff
bool prevButState[4] = {0,0,0,0};
bool currButState[4] = {0,0,0,0};


// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS);

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
  updateTemp();
  delay(2000);
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
    updateTemp();
    lastTempCheck = millis();
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
      temporarySetTemp = temporarySetTemp + 5;
      updateScreen();
    }
  }
  else if ((currButState[DOWN] == true) && (prevButState[DOWN] == false) && (screenState == SET_SCREEN)) {
    if(temporarySetTemp >= 45) {
      temporarySetTemp = temporarySetTemp - 5;
      updateScreen();         
    }
  }
  else if((currButState[SELECT] == true) && (prevButState[SELECT] == false) && (screenState == SET_SCREEN) && (controlState == IDLING)) {
    setTemp = temporarySetTemp;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Starting...");
    lcd.setCursor(0,1);
    lcd.print("Target: ");
    lcd.setCursor(9,1);
    lcd.print(setTemp);
    Serial.println("Starting");
    controlState = BEANS;
    delay(1000);
    screenState = CURRENT_TEMP_SCREEN;
  }
  buttonsUsed();
}

void updateTemp() {
  // Read the temperature that the sensor prepared
  currentTemp = sensors.getTempCByIndex(0);

  // If in beans control
  if(controlState == BEANS) {
    if((setTemp - currentTemp) > estOvershoot) {
      relayState = true;
    }else {
      Serial.println("Coasting Now");
      relayState = false;
      controlState = COAST;
    }
  }

  // If we are coasting after beans control
  if(controlState == COAST) {
    if(currentTemp > maxTemp) {
      maxTemp = currentTemp;
    }
    if((maxTemp - currentTemp) > 2) {
      Serial.print("Max Temp = ");
      Serial.print(maxTemp);
      Serial.println("Starting Hysteresis");
      controlState = HYSTER;
    }
  }

  // We have stopped coasting, activate hysteresis
  if(controlState == HYSTER) {
    if((setTemp - currentTemp) > lowerThresh) {
      relayState = true;
    }
    else if((setTemp - currentTemp) < -upperThresh) {
      relayState = false;
    }
  }
  
  // Ask the temperature sensor to prepare another reading for next time
  sensors.setWaitForConversion(false);  // makes it async
  sensors.requestTemperatures();
  sensors.setWaitForConversion(true);
  
  Serial.print(currentTemp);
  Serial.print(",");
  Serial.print(relayState);
  Serial.print(",");
  Serial.println(controlState);
  
  digitalWrite(RELAY_PIN, relayState);
}
