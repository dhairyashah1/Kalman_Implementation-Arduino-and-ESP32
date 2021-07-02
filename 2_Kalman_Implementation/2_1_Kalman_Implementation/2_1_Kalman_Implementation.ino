// Version 1 - EKF Implementation

//current_error_fixer() written but, not used right now

#include <BasicLinearAlgebra.h>
using namespace BLA;

// Ro, R1, tau, ocv(lookup in array), capacity, 
//////////////////////// GLOBAL VARIABLES //////////////////////////////////////////////////////
const float e = 2.718281828459045; 

int outputPin = 9;     // Output signal wire connected to digital pin 9 (Charging pin to battery)
int outputValue = 150;     //value of PWM output signal 
float outputReadValue = 0;

int batteryCapacity = 12000;         //capacity rating of battery in mAh 
float resistance = 10.0;             //measured resistance of the power resistor (connected to battery)
int cutoffVoltage = 4800;            //maximum battery voltage (in mV) that should not be exceeded = 4.8V (for 5V Li battery) //29.5 or 30 v (charge)// lowe cutoff 22 -23v (discharge)
float cutoffTemperatureC = 75;       //maximum battery temperature that should not be exceeded (in degrees C)
//float cutoffTemperatureF = 167;    //maximum battery temperature that should not be exceeded (in degrees F) (F = 9*C/5 + 32)
long cutoffTime = 60*1000;           //maximum charge time of 1 min that should not be exceeded (in seconds)

// voltage reading probes across battery
int analogPinOne = 0;                //first voltage probe connected to analog pin 1
float valueProbeOne = 0;             //variable to store the anlog value of analogPinOne
float voltageProbeOne = 0;           //calculated voltage at analogPinOne

int analogPinTwo = 1;                //second voltage probe connected to analog pin 2
float valueProbeTwo = 0;             //variable to store the analog value of analogPinTwo
float voltageProbeTwo = 0;           //calculated voltage at analogPinTwo

// pin to measure temperature
int analogPinThree = 2;              //third voltage probe connected to analog pin 2
float valueProbeThree = 0;           //variable to store the value of analogPinThree
float tmp36Voltage = 0;              //calculated voltage at analogPinThree
float temperatureC = 20;             //calculated temperature of probe in degrees C
float prev_temperatureC = 0;         //variable to store the previous temperature
//float temperatureF = 0;            //calculated temperature of probe in degrees F
    
// Assumed batteryVoltage = voltageDifference between analogPins 1 and 2
float voltageDifference = 0;         //difference in voltage between analogPinOne and analogPinTwo
float batteryVoltage = 0;            //calculated voltage of battery

float current = 0;                   //calculated current through the load (in mA)
float targetCurrent = batteryCapacity / 10;     //target output current (in mA) set at C/10 or 1/10 of the battery capacity per hour // remove
float currentError = 0;              //difference between target current and actual current (in mA)
float slope = 15;                    //(v_final - v_init)/(I_final - I_init) {assumed}  // rate of change of voltage wrt soc  ==> if prec- soc =0 conditon
float prev_current = 0;
float t = 0;

//calculated calues of constants
float R0 = 0.107;
float R1 = 0.053;
float time_constant = 476*1000;     //time constant in milli seconds
float time_diff = 3000;             //3 seconds(delay also os 3 sec)
float del_T = 476000;                 // tau - 476 sec // time constant

// Kalman Variables Initialisation
BLA::Matrix<2,1> X = {0.0, 1.0};    //state variables - [Vcap, SOC] 
BLA::Matrix<2,1> U = {0.0, 25.0};   //Input variables : [current,temp diff]
BLA::Matrix<2,2> P = {1.0 ,0.0 ,
                      0.0 ,1.0}; 
BLA::Matrix<1,2> K = {0.0, 0.0};
BLA::Matrix<2,2> ak = {0.9757691089747373,0,
                       0, 1};
BLA::Matrix<2,2> bk = {-0.006784883474301327,0,
                      0,0};
BLA::Matrix<1,2> ck = {-1, slope};
BLA::Matrix<1,2> dk = {-R0, 0.05};   // 0.05 is assumed 
BLA::Matrix<1,1> voltageError = {0};
BLA::Matrix<2,2> Qk = {1,0,
                      0,1};
BLA::Matrix<1,1> Rk = {0.5};

/*
float X[2] = {0.0, 1.0};
float U[2] = {0.0, 25.0}; 
int P[2][2] = {{1,0},{0,1}}; 
float K[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
float ak[2][2] = {{0.9757691089747373,0},{0,1}};
float bk[2][2] = {{-0.006784883474301327,0},{0,0}};
float ck[2] = {-1, slope};
float dk[2] = {-resistance, 0.05};
float error = 0;
float Qk[2][2] = {{1,0},{0,1}};
float Rk[1] = {0.5};
*/
BLA::Matrix<1,1> voltage_difference = {voltageDifference};
BLA::Matrix<1,1> battery_voltage = {batteryVoltage};
BLA::Matrix<1,1> v_pred = {4.25};   // Voltage Predicted (initial assumed)
BLA::Matrix<1,1> soc = {100.0};     // % soc
BLA::Matrix<2,2> M = {1.0, 0,
                      0, 1.0};

BLA::Matrix<1,1> prev_soc = {90};   // % previous soc (ssumed)
BLA::Matrix<1,1> temp = {0};

/////////////////////////////  SETUP //////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);             // serial setup
  pinMode(outputPin, OUTPUT);
  pinMode(analogPinOne, INPUT);
  pinMode(analogPinTwo, INPUT);
  pinMode(analogPinThree, INPUT);

} 


/////////////////////////////  LOOP //////////////////////////////////////////////////////
void loop() {
  ///////////////////////////////////////////////////////////////////////////////////////
  analogWrite(outputPin, outputValue);

  Serial.print("Output: ");        //display output values for monitoring with a computer
  Serial.println(outputValue);
  
  ////////////////////////////////////////////////////////////////////////////////////////
  measure_current();
  measure_voltage();
  measure_temperature();

  // Extended Kalman Filter Implementation
  U(0,0) = prev_current;
  U(1,0) = prev_temperatureC - 25;
  ak(0,0) = pow(e, - time_diff/del_T);
  bk(0,0) = (1 - pow(e, -time_diff/del_T))*R1;
  bk(1,0) = - time_diff/(batteryCapacity*3600);

  //prediction step
  X = ak*X + bk*U;
  t = X(1,0);
  P = ak*(P*(~ak)) + Qk;
  v_pred = battery_voltage - X(0,0) + dk*U;  // battery_voltage ==> lookup table

  //update step 
  voltageError = voltage_difference - v_pred;
  temp = ck*(P*(~ck)) + Rk;
  K = P*(~ck)/temp(0,0);
  BLA::Matrix<2,1> K = {K(0,0), K(0,1)};  
  X = X + K*voltageError;

  P = (M - K*ck)*P;

  calculate_soc();
  limit_checker();
  update_variables();

  Serial.println();                   //extra spaces after one cycle to make debugging data easier to read
  Serial.println(); 
  delay(3000); // 3 sec delay
}

/* Updates the current and previous variables for electric current, voltage, soc and other variables after each iteration*/
void update_variables(){
  prev_current = current;
  //  measure_current();

  prev_soc(0,0) = soc(0,0);

  prev_temperatureC = temperatureC; 
  
  
  
}

/* Calculates soc based on other parameters measured  and prints it*/
void calculate_soc(){ //rounding in lookup table for soc values (indexing)
  soc(0,0) = X(1,0)*100;
  Serial.println("==================================================");
  Serial.print("State of Charge (SOC): ");
  Serial.println(soc(0,0));
  Serial.println("==================================================");
}

/* Reads the cvoltage across probes. calculates voltage across battery  and displays it */
void measure_voltage(){
  // voltage across probes
  valueProbeOne = analogRead(analogPinOne);     // read the input value at probe one
  voltageProbeOne = (valueProbeOne*5000)/1023;  //calculate voltage at probe one in milliVolts => 10 bit ADC
  Serial.print("Voltage Probe One (mV): ");     //display voltage at probe one
  Serial.println(voltageProbeOne);  
    
  valueProbeTwo = analogRead(analogPinTwo);     // read the input value at probe two                   // resistor values (voltage divider)
  voltageProbeTwo = (valueProbeTwo*5000)/1023;  //calculate voltage at probe two in milliVolts
  Serial.print("Voltage Probe Two (mV): ");     //display voltage at probe two
  Serial.println(voltageProbeTwo);  
  
  batteryVoltage = voltageProbeOne - voltageProbeTwo;     //calculate battery voltage // -IR
  Serial.print("Battery Voltage (mV): ");       //display battery voltage
  Serial.println(batteryVoltage);
}

/* Reads the current as per voltage across probes and displays it */
void measure_current(){
  // temporary 
  current = (voltageProbeOne - voltageProbeTwo)/resistance;    // sensor
  Serial.print("Target Current (mA): ");       //display target current 
  Serial.println(targetCurrent);  
  Serial.print("Battery Current (mA): ");      //display actual current
  Serial.println(current);  
}

/* Fixes the error in current as per measured and desired current(constant current) and displays it */
void current_error_fixer(){
  // current error calculation and displaying it
  currentError = targetCurrent - current;      //difference between target current and measured current
  Serial.print("Current Error  (mA): ");       //display current error 
  Serial.println(currentError);   

  // bounding range of current - can change (current varies -> temperature varies)
  if(abs(currentError) > 50)                  //if output error is large enough more than diff of 50mA, adjust output 
   { 
    outputValue = outputValue + currentError/2;
    if(outputValue < 1)                       //output can never go below 0
      outputValue = 0;
    else if(outputValue > 254)                //output can never go above 255
      outputValue = 255;
    analogWrite(outputPin, outputValue);      //write the new output value
   }
}


/* Reads the temperature of the battery and displays it */
void measure_temperature(){
  valueProbeThree = analogRead(analogPinThree);  // read the input value at probe three  
  tmp36Voltage = valueProbeThree * 5.0/1023;     // converting that reading to voltage

  // currently not in use - formula depends on sensor used (temporary)
  temperatureC = (tmp36Voltage - 0.05) * 100 ;   //converting from 10 mv per degree wit 50 mV offset to degrees ((voltage - 50mV) times 100)
  Serial.print("Temperature (degrees C) ");      //display the temperature in degrees C
  Serial.println(temperatureC); 
 
 /*
  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;     //convert to Fahrenheit
  Serial.print("Temperature (degrees F) ");
  Serial.println(temperatureF); 
 */
}


/* Checks, bounds various electrical parameters of the battery and changes as per need displays it */
void limit_checker(){
 
  if(temperatureC > cutoffTemperatureC)        //stop charging if the battery temperature exceeds the safety threshold
   {
    outputValue = 0;
    Serial.println("Max Temperature Exceeded: ");
   }
   
  /*
  if(temperatureF > cutoffTemperatureF)        //stop charging if the battery temperature exceeds the safety threshold
   {
    outputValue = 0;
   }
   */
   
   if(batteryVoltage > cutoffVoltage)          //stop charging if the battery voltage exceeds the safety threshold
   {
    outputValue = 0;
    Serial.println("Max Voltage Exceeded: ");
   }  
 
   if(millis() > cutoffTime)                  //stop charging if the charge time exceeds threshold charging time
   {
    outputValue = 0;
    Serial.println("Max Charge Time Exceeded");
   }  
}
