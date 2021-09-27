#ifdef ESP32
#include "HX711.h"
#include "LiquidCrystal_I2C.h" //allows interfacing with LCD screens
#include "PointXY.h"
#include "RunningAverage.h"
#include "TimeFunction.h"
#include "Wire.h" //allows communication over i2c devices
#include <Arduino.h>
#include <CircularBuffer.h>
#include <PID_v1.h>
#include <Wire.h>
#include <TB6612_ESP32.h>
#else
#include <SparkFun_TB6612.h>
#endif

// ========== physical dimentions ==========
#define plungerDiameter 44
double plungerArea = PI * pow((plungerDiameter/2.0),2); 

// ========== debugging variables ==========
// #define LCDAVAILABLE
#define SERIALDATA
// #define PREINFUSIONBYPRESSURE
// #define DATAFUCTIONS

// ========== Pins configuration ==========
#ifdef ESP32
// motor Pins
#define AIN1 26
#define AIN2 25
#define PWMA 14
#define STBY 15
// caliper pins
#define caliperClkPin 19
#define caliperDataPin 18
// load cell pins
#define loadcellDOutPin 32
#define loadcellClkPin 27
// switches pin
#define modePin 39
#define directionPin 34
#define movePin 35
#define tarePin 36
// pot pin
#define potPin 12
// pressure pin
#define pressurePin 4
#else
#define AIN1 9
#define AIN2 8
#define PWMA 11
const int loadcellDOutPin = 6;
const int loadcellClkPin = 10;
#define potPin A1
#define modePin 3
#define directionPin 4
#define movePin 5
#define tarePin 2
#endif

// ========== Condition variables ==========
double maxWeightPreinfusion = 1.0; // in grams
double maxPressurePreinfusion = 4.0; // in bars
double preinfusionFlow = 4.0; // in mililiters/s
double stopByWeight = 100;

// ========== caliper variables ==========
volatile double caliperPosition = 0.0;
volatile double caliperSpeed = 0.0;
volatile bool mm_in;
volatile bool newData = false;

// ========== Pressure Sensor Configuration ==========
const int pressureZero = 72; //analog reading of pressure transducer at 0psi
const int pressureMax = 921.6; //analog reading of pressure transducer at 100psi
const int pressuretransducermaxPSI = 300; //psi value of transducer being used
const int sensorreadDelay = 250; //constant integer to set the sensor read delay in milliseconds
float pressureValue = 0; //variable to store the value coming from the pressure transducer
float pressureBar;

// ========== PID Configuration ==========
#define PWMDefault 1024
double kp = 2550, ki = 0, kd = 0;
double inputPidActuator = 0, outputPidActuator = 0, setpointPidActuator = 0;
double inputPidFlow = 0, setpointPidFlow = 0;
PID pidActuator(&inputPidActuator, &outputPidActuator, &setpointPidActuator, kp, ki, kd, DIRECT);

// ========== LCD configuration ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ========== constants ==========
#define baudRate 115200
#define DOWNDIRECTION false
#define UPDIRECTION true

// ========== Global varibles ==========
float currentWeight = 0;
// profile curves
TimeFunction pressureFunction;
TimeFunction flowFunction;
PointXY pressureProfile[MAXPOINTS], flowProfile[MAXPOINTS];
int profilePoints = 2;
// moving average
RunningAverage weightMovingAvrg(15);
RunningAverage pressureMovingAvrg(15);
// serial reading
CircularBuffer<String, MAXPOINTS + 10> commands; // uses 538 bytes
String commandRead;
// load cell
HX711 balanza;
// actuator
const int offsetA = 1;
Motor actuator = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 32000, 10, 0);
// serial information for UI
bool sendDataFlag = false;
// switches
int mode;

// ==== function prototypes ====
double  readCurrentPressure(int times = 5);
double  readCurrentWeight(int numberOfSamples = 1);
void    sendSensorValuesSerial(int mode);
void    tareFunction();
bool    reachPressure(double pressureTarget, bool direction);
void    updatePressure(double );
void    updateFlow(double );
void    printLCD();
void    setPWMPrescaler(uint8_t pin, uint16_t prescale);
double  readCurrentFlow();
int     readFromSerial();

void IRAM_ATTR caliper();
void IRAM_ATTR process_data(bool data[], double time);

void setup() //setup routine, runs once when system turned on or reset
{
    Serial.begin(baudRate); //initializes serial communication at set baud rate bits per second
    Serial.println("ON");
#ifdef LCDAVAILABLE
    lcd.init(); //initializes the LCD screen
    lcd.backlight();
#endif
#ifndef ESP32
    setPWMPrescaler(11,1);
#endif
    // hardwasre config
    pinMode(modePin, INPUT_PULLUP);
    pinMode(directionPin, INPUT_PULLUP);
    pinMode(movePin, INPUT_PULLUP);
    pinMode(tarePin, INPUT_PULLUP);
    pinMode(caliperClkPin, INPUT);
    pinMode(caliperDataPin, INPUT);

    // Scale configuration
    balanza.begin(loadcellDOutPin, loadcellClkPin);
    balanza.set_scale(4931.93); // this have to be confugre depending on the load cell
    balanza.tare(20);
    
    Serial.println("load cell was initialized");
    // tare interrup
    attachInterrupt(digitalPinToInterrupt(tarePin), tareFunction, RISING);

    // Interruption configuration
    attachInterrupt(digitalPinToInterrupt(caliperClkPin), caliper, FALLING);
    

    pidActuator.SetMode(AUTOMATIC); //set PID in Auto mode
    pidActuator.SetSampleTime(1); // refresh rate of PID controller
    pidActuator.SetOutputLimits(30, PWMDefault);


}

void loop() //loop routine runs over and over again forever
{
    mode = digitalRead(modePin);
    static bool plungerPhase = false,
                preInfusionPhase = false,
                profileCurvePhase = false;
    static bool holdStartFlag = false;
    static bool startFlag = false;
    bool direction = digitalRead(directionPin);

    // here we scan serial for possible messages
    int countCommands = readFromSerial();
    // Serial.print("currentPressure: ");
    // Serial.println(readCurrentPressure());


    if (countCommands > 0){
        String serialCommand = commands.shift();
        serialCommand.toLowerCase();
        // Serial.print("countCommands: ");
        // Serial.println(countCommands);
        Serial.print("commando: ");
        Serial.println(serialCommand);
        if (serialCommand.equals("pressure")){
            countCommands -= 1;
            // Serial.print("countCommands: ");
            // Serial.println(countCommands);
            for (int i =0; i < countCommands; i ++){
                String point = commands.shift();
                // Serial.print("point: ");
                // Serial.println(point);
                int dashIndex = point.indexOf(",");
                double firstPosition = point.substring(0, dashIndex).toDouble() * 1000.0;
                double secondPosition = point.substring(dashIndex + 1, point.length()).toDouble();
                pressureProfile[i].x = firstPosition;
                pressureProfile[i].y = secondPosition;
            }
            profilePoints = countCommands;
            pressureFunction.setPoints(pressureProfile, profilePoints);
        }
        else if (serialCommand.equals("flow")) {
            countCommands -= 1;
            for (int i = 0; i < countCommands; i++) {
                String point = commands.shift();
                int dashIndex = point.indexOf(",");
                double firstPosition = point.substring(0, dashIndex).toDouble() * 1000.0;
                double secondPosition = point.substring(dashIndex + 1, point.length()).toDouble();
                flowProfile[i].x = firstPosition;
                flowProfile[i].y = secondPosition;
            }
            profilePoints = countCommands;
            flowFunction.setPoints(flowProfile, profilePoints);
        }
        else if (serialCommand.equals("start")){
            startFlag = true;
            Serial.println("starting...");
        }
        else if (serialCommand.equals("stop")) {
            holdStartFlag = false;
            startFlag = false;
            sendDataFlag = false;
            outputPidActuator = 0;
            Serial.println("stopss...");
        }
        else if (serialCommand.equals("weight")) {
            stopByWeight = commands.shift().toDouble();
            Serial.print("max Weight: ");
            Serial.println(stopByWeight);
        } else if (serialCommand.equals("tare")){
            tareFunction();
            Serial.println("scale tared");
        } else if (serialCommand.equals("preinfusionweight")) {
            maxWeightPreinfusion = commands.shift().toDouble();
            Serial.print("max preinfusion weight= ");
            Serial.println(maxWeightPreinfusion);
        } else if (serialCommand.equals("preinfusionpressure")) {
            maxPressurePreinfusion = commands.shift().toDouble();
            Serial.print("max preinfusion pressure= ");
            Serial.println(maxPressurePreinfusion);
        } else if (serialCommand.equals("preinfusionflow")) {
            preinfusionFlow = commands.shift().toDouble();
            Serial.print("max preinfusion flow= ");
            Serial.println(preinfusionFlow);
        }
        commands.clear();
    }

    if (mode == MANUAL && !startFlag){ // Manual
        int desirePwm = 1024 - analogRead(potPin) / 4;
        if (desirePwm > 1024){
            desirePwm = 1024;
        }
        if (desirePwm < 0)
        {
            desirePwm = 0;
        }
        
        // bool direction = digitalRead(directionPin);
        

        bool moveActuator = digitalRead(movePin);
        if (moveActuator){
            outputPidActuator = desirePwm;
            #ifdef LCDAVAILABLE
            lcd.setCursor(8,0);
            lcd.print("moving...   ");
            #endif
        } else {
            actuator.brake();
            lcd.setCursor(8, 0);
            lcd.print("            ");
            outputPidActuator = 0;
        }

    }
    else { // Automatic
        static double startPressure;
        //wait for the start signal

        // bool start = digitalRead(movePin);

        // if one the following conditions are true, it starts the espresso procces until a stop signal is triggered or the process finishes
        if (startFlag || holdStartFlag){
            if (!holdStartFlag) {
                // here we program all we want when the espresso process begins and it is only executed once
                Serial.println("Espresso process begins...");
                plungerPhase = true;
                preInfusionPhase = false;
                profileCurvePhase = false;
                startPressure = readCurrentPressure();
                startPressure = 1.1 * readCurrentPressure();

                holdStartFlag = true;
            }

            if (plungerPhase){
                // piston touches plunger phase
                if ( reachPressure(startPressure, DOWNDIRECTION) ){
                    plungerPhase = false;
                    preInfusionPhase = true;
                    profileCurvePhase = false;
                    flowFunction.resetTime();
                    Serial.println("PreInfusionCurve process begins...");
                }
            }
            else if (preInfusionPhase){
                // pre-infusion phase
                sendDataFlag = true;
#ifdef PREINFUSIONBYPRESSURE
                // updatePressure(flowFunction.getValue());
                // if (flowFunction.getValue() >= 4.0) {
                    plungerPhase = false;
                    preInfusionPhase = false;
                    profileCurvePhase = true;
                    pressureFunction.resetTime();
                    Serial.println("profileCurve process begins...");
                // }
#else
                updateFlow(preinfusionFlow);
                double currentWeight = readCurrentWeight();
                double currentPressure = readCurrentPressure();
                bool condition1 = currentPressure >= maxPressurePreinfusion;
                bool condition2 = currentWeight > maxWeightPreinfusion;


                if ( condition1 || condition2){ // conditions for stop preinfusion
                
                Serial.print("condition1 ");
                Serial.println(condition1);
                Serial.print("condition2 ");
                Serial.println(condition2);
                Serial.print("currentWeight: ");
                Serial.println(currentWeight);
                Serial.print("current pressure");
                Serial.println(currentPressure);
                    plungerPhase = false;
                    preInfusionPhase = false;
                    profileCurvePhase = true;
                    pressureFunction.resetTime();
                    Serial.println("ProfileCurve process begins...");
                }
#endif
#ifdef DATAFUCTIONS
                Serial.print("FlowFunction: ");
                Serial.println(flowFunction.getValue());
#endif

            }
            else if (profileCurvePhase){
                // profile phase
                sendDataFlag = true;
                updatePressure(pressureFunction.getValue());
#ifdef DATAFUCTIONS
                Serial.print("PressureFuncion: ");
                Serial.println(pressureFunction.getValue());
#endif
                if (pressureFunction.isFinished() || (currentWeight >= stopByWeight) ){
                    plungerPhase = false;
                    preInfusionPhase = false;
                    profileCurvePhase = false;
                    Serial.println("Espresso process is done...");
                }

            }
            else{
                // here the espresso process has finished so we reset all the necessary variables
                holdStartFlag = false;
                startFlag = false;
                sendDataFlag = false;
                outputPidActuator = 0;

            }
        }
    }

    //in this part of the code, we use the values updated in the previous stage

    if (direction == UPDIRECTION) {
        actuator.drive(outputPidActuator);
    } else {
        actuator.drive(-outputPidActuator);
    }

    // we send the infomration trough Serial
    printLCD();
    // update the scale
    if (balanza.is_ready()) {
        weightMovingAvrg.add(readCurrentWeight());
        currentWeight = weightMovingAvrg.getAverage();
    }
#ifdef SERIALDATA
    if (sendDataFlag){
        sendSensorValuesSerial(mode);
    }
#endif
// if (newData){
// Serial.print("mm: ");
// Serial.println(caliperPosition);
// newData = false;
// Serial.print("flow: ");
// Serial.println(readCurrentFlow());
// }

}

// INterruption Function
void IRAM_ATTR caliper(){
    static long last_received = 0;
    static int received_bit_location = 0;
    static bool data[24] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //24 bit boolean array for storing the received data
    static unsigned long lastTime = millis();
    unsigned long time;


    if (millis() - last_received > 2)
    {
        received_bit_location = 0;
    }

    if (received_bit_location <= 23) {
        data[received_bit_location] = !digitalRead(caliperDataPin); //inverting the received data due to the hardware circuit level shifting
        received_bit_location++;
    }

    if (received_bit_location == 24) {
        time = millis() - lastTime;
        // Serial.print("millis: ");
        // Serial.println(millis());
        lastTime = millis();
        received_bit_location = 0;
        process_data(data, time);
        newData = true;
    }

    last_received = millis();
    //Serial.println(received_bit_location);

}

void IRAM_ATTR process_data(bool data[], double time){
    static double lastPosition = 0;
    caliperPosition = 0.0;

    if (data[23] == 0) { //if it's in the mm mode
        mm_in = 0;
        //converting binary to decimal value
        for (int i = 0; i <= 15; i++) {
            caliperPosition += data[i] * pow(2, i) / 100.0;
        }
        if (data[20] == 1) {
            caliperPosition = caliperPosition * -1; //add the negative sign if it exists
        }
    }

    if (data[23] == 1) { //if it's in the inch mode
        mm_in = 1;
        //converting binary to decimal value
        for (int i = 1; i <= 16; i++) {
            caliperPosition += data[i] * pow(2, (i - 1)) / 1000.0;
        }
        if (data[0] == 1) {
            caliperPosition += 0.0005; //add the 0.5 mil sign if it exists
        }
        if (data[20] == 1) {
            caliperPosition = caliperPosition * -1; //add the negative sign if it exists
        }
    }

    caliperSpeed = (lastPosition - caliperPosition) /  time;
    // Serial.print("t: ");
    // Serial.println(time);
    lastPosition = caliperPosition;
}

void updatePressure(double desirePressure){
    inputPidActuator = readCurrentPressure();
    setpointPidActuator = desirePressure;
    pidActuator.Compute();
}

void updateFlow(double desireFlow){
    inputPidActuator = readCurrentFlow();
    setpointPidActuator = desireFlow;
    pidActuator.Compute();
    Serial.print("pidOutput ");
    Serial.println(outputPidActuator);
}

void printLCD(){
    static unsigned long t = millis();
    bool direction = digitalRead(directionPin);
    
#ifdef LCDAVAILABLE
    if (mode == AUTOMATIC){
        // Here we print to the LCD for AutomaticMode
        if (millis() - t > 500) {
            // lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("P_c=");
            lcd.print(readCurrentPressure(), 1);
            lcd.print(" P_s=");
            lcd.print(setpointPidActuator, 1);

            lcd.setCursor(0, 1);
            lcd.print("Pwm_c=");
            lcd.print(outputPidActuator, 0);
            lcd.print(" dir=");
            if (direction == UPDIRECTION) {
                lcd.print("up");
            } else {
                lcd.print("down");
            }
            t = millis();
        }
    }
    else if (mode == MANUAL){
        if (millis() - t > 500) {
            // lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("bar=");
            lcd.print(readCurrentPressure(), 1);
            lcd.print(" Manual   ");

            lcd.setCursor(0, 1);
            lcd.print("Pwm_d=");
            lcd.print(outputPidActuator);
            lcd.print(" dir=");
            if (direction == UPDIRECTION) {
                lcd.print("up");
            } else {
                lcd.print("down");
            }
            t = millis();
        }
    }
#endif
}

double readCurrentFlow(){
    // Serial.print("readCF");
    return -1 * caliperSpeed * plungerArea;
}

double readCurrentPressure(int times){
    double currentPressure = 0; //reads value from input pin and assigns to variable
    for (int i = 0; i<times; i++){
        currentPressure += analogRead(pressurePin);
    }
    currentPressure = (currentPressure/double(times))/4.0;
    currentPressure = ((currentPressure - pressureZero) * pressuretransducermaxPSI) / (pressureMax - pressureZero); //conversion equation to convert analog reading to psi
    currentPressure = currentPressure / 14.504;

    return currentPressure;
}

//Atmega 328p (Arduino Uno, Nano)
// Frecuencias
// 5, 6    Timer0   62500 Hz
// 9, 10    Timer1   31250 Hz
// 3, 11    Timer2   31250 Hz
// Prescalers
// 5, 6    Timer0   1 8 64 256 1024
// 9, 10    Timer1   1 8 64 256 1024
// 3, 11    Timer2   1 8 32 64 128 256 1024
// Valores por defecto
// 5, 6    Timer0 64   977Hz
// 9, 10    Timer1 64   490Hz
// 3, 11    Timer2 64   490Hz
// Consecuencias
// 5, 6    Timer0   delay() y millis()
// 9, 10    Timer1   Librería servo
// 3, 11    Timer2 
#ifndef ESP32
void setPWMPrescaler(uint8_t pin, uint16_t prescale) {
  
  byte mode;
  
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(prescale) {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   64: mode = 0b011; break; 
      case  256: mode = 0b100; break;
      case 1024: mode = 0b101; break;
      default: return;
    }
    
  } else if(pin == 3 || pin == 11) {
    switch(prescale) {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   32: mode = 0b011; break; 
      case   64: mode = 0b100; break; 
      case  128: mode = 0b101; break;
      case  256: mode = 0b110; break;
      case 1024: mode = 0b111; break;
      default: return;
    }
  }
  
  if ((pin==5 || pin==6)) {
    TCCR0B = TCCR0B & 0b11111000 | mode;
  } else if (pin==9 || pin==10) {
    TCCR1B = TCCR1B & 0b11111000 | mode;
  } else if (pin==3 || pin==11) {
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
#endif

void tareFunction(){
    balanza.tare(20);
}

double readCurrentWeight(int numberOfSamples){
    return balanza.get_units(numberOfSamples);
}

void sendSensorValuesSerial(int mode)
{
    static unsigned long tOld = millis();
    if (millis() - tOld > 25) {
        // pressureMovingAvrg.add()
        Serial.print(readCurrentPressure());
        Serial.print(",");
        // Serial.print(readCurrentWeight());
        Serial.print(currentWeight);
        Serial.print(",");
        if (mode == MANUAL) {
            Serial.print(analogRead(potPin) / 4);
        } else {
            Serial.print(readCurrentFlow());
        }
        Serial.print("\n");
        tOld = millis();
    }
}

double inputPidOnOff, setpointPidOnOff;
PID onOffPID(&inputPidOnOff, &outputPidActuator, &setpointPidOnOff, 25500, 0, 0, DIRECT); //25500 means an error of 0.01 returns 255 value as output
// @brief this function calculate a pwm value for the actuator in order to reach a determine pressure
bool reachPressure(double pressureTarget, bool direction){
    onOffPID.SetMode(AUTOMATIC); //set PID in Auto mode
    onOffPID.SetSampleTime(1); // refresh rate of PID controller
    onOffPID.SetOutputLimits(0, PWMDefault);

    setpointPidOnOff = pressureTarget;
    inputPidOnOff = readCurrentPressure(5);
    onOffPID.Compute();;
    
    return (outputPidActuator <= 0 ? true: false);
}

// function to read from Serial port
int readFromSerial()
{
    static int numData = 0;
    while (Serial.available()) {
        char uartValue = Serial.read();
        if (uartValue == ' ') {
            //limitar commandRead a ser un commando valido para poder ser pusheado
            commands.push(commandRead);
            Serial.print("Commando ingresado al Buffer: ");
            Serial.println(commandRead);
            commandRead = "";
            numData += 1;
        } if (uartValue == '\n'){
            commands.push(commandRead);
            Serial.print("Commando ingresado al Buffer: ");
            Serial.println(commandRead);
            commandRead = "";
            numData += 1;
            int numDataReturn = numData;
            numData = 0;
            return numDataReturn;
        }else {
            commandRead.concat(uartValue);
        }
        // delay(3);
    }

    return 0;
}