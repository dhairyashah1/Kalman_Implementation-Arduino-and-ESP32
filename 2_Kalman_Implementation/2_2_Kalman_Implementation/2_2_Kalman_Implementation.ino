// v.2 : Extended kalman Filter Implementation (Compiles for both Arduino and ESP32 Dev Boards)

// including library for matrix operations
#include "BasicLinearAlgebra.h"

// Library for I2C communication with OLED
#include <Wire.h>
// including OLED libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// including libraries for sensors(current, coltage and temperature)

#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <WebServer.h>
 
Adafruit_ADS1115 ads;

using namespace BLA;

//////////////////////// OLED CONFIGURATION  //////////////////////////////////////////////////////
#define SCREEN_WIDTH 128                   // OLED display width, in pixels
#define SCREEN_HEIGHT 64                   // OLED display height, in pixels
#define OLED_RESET -1                      // OLED RESET Pin # (-1 -> same as chip reset)
#define SCREEN_ADDRESS 0x3D                // Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//////////////////////// GLOBAL CONSTANTS //////////////////////////////////////////////////////
const float e = 2.718281828459045;  

// battery and circuit parameters
int batteryCapacity = 12;                   //capacity rating of battery in Ah (12 Ah)
float resistance = 10.0;                    //measured resistance of the power resistor (connected to battery)
float internal_resistance = 5.0;            //internal resistance of the battery

//calculated values of Kalman Equivalent circuit constants
float R0 = 0.107;
float R1 = 0.053;
float time_diff = 3000;                     // assumed 3 seconds (Cycle repetition time)
float time_constant = 476000;               // tau - 476 sec time constant tau = 5*R1*C1

// open circuit voltage lookup table : ocv[soc] represents the practically observed open circuit voltage for a soc of integer index "soc"
// i.e ocv[0] = 23.3319 v (0 % SOC) ; ocv[100] = 26.92084718 v (100 % SOC)
BLA::Matrix<101,1> ocv = {23.3319, 23.45, 23.58304852 , 23.80765487 ,24.00768307 ,24.25500805 ,24.4414175, 24.5786138,  24.69821588,24.80176113,
                          24.83070727, 24.86643428, 24.91024622, 24.9333732,  24.96697321, 24.98213404, 24.99987516, 25.01114964, 25.01684596, 25.02779002, 
                          25.0347469,  25.05842286, 25.06946717, 25.10847401, 25.11598437, 25.12248794, 25.13842499,25.14418827, 25.15012488, 25.1565382,  
                          25.16368975,25.16780108, 25.17105568, 25.17560083, 25.17954955, 25.18298245, 25.1839496,25.1844725,  25.18654586, 25.18913961, 25.19120067, 
                          25.19565495, 25.20840915,25.21435272, 25.2243597,  25.24129064, 25.25899448, 25.27731043, 25.2960699,25.30509832, 25.31421711, 25.32324551, 
                          25.33200249, 25.35030866, 25.35798813, 25.36487042, 25.37079233, 25.37559986, 25.38515007, 25.39131299, 25.4019735,25.41103324, 25.41841246,
                          25.42405195, 25.42791492, 25.42998887, 25.4302875, 25.43885261, 25.44575596, 25.45110117, 25.45502565, 25.45770242, 25.45934205,25.46019455, 
                          25.46455123, 25.47014661, 25.47116031, 25.48221895, 25.48439802,25.48822377, 25.49027512, 25.49318553, 25.49564491, 25.49640149, 25.49726374,
                          25.50210221, 25.50685147, 25.51951197, 25.53115195, 25.56290933, 25.58599357,25.60168758, 25.64134965, 25.68641525, 25.7818399,  25.90889655, 
                          26.06958641,26.25223192, 26.45868309, 26.69087849, 26.92084718};

//////////////////////// GLOBAL VARIABLES //////////////////////////////////////////////////////

// circuit cutoff variables
int upperCutoffVoltage = 30.0;                //maximum battery voltage (in V) that should not be exceeded = upper cutoff voltage = 29.5 - 30v (charge)
int lowerCutoffVoltage = 23.0;                //minimum battery voltage (in V) that should not be preceeded = lower cutoff voltage =  22 - 23v (discharge)
float cutoffTemperatureC = 75;                //maximum battery temperature that should not be exceeded (in degrees C)
//float cutoffTemperatureF = 167;             //maximum battery temperature that should not be exceeded (in degrees F) (F = 9*C/5 + 32)
long cutoffTime = 3600*1000;                  //maximum charge time of 1 hour that should not be exceeded (in seconds)
    
// voltage reading probe across battery (other terminal is GND)
int16_t analogPinMeasureBatteryVoltage = 0;                         //first voltage probe connected to analog pin 1 = A0
float valueProbeMeasureBatteryVoltage = 0;                      //variable to store the analog value of analogPinMeasureBatteryVoltage
float voltageProbeMeasureBatteryVoltage = 0;                    //calculated voltage at analogPinMeasureBatteryVoltage as per ADC resolution and maximum range capacity

// analog pin to measure temperature
int16_t analogPinTemp = 1;                    //second voltage probe connected to analog pin 2 = A1
float valueProbeTemp = 0;                     //variable to store the value of analogPinTemp
float temperatureC = 0;                       //calculated voltage at analogPinTemp
float prev_temperatureC = 0;                  //variable to store the previous temperature
//float temperatureF = 0;                     //calculated temperature of probe in degrees F

// pin to measure current from current sensor
int16_t analogPinCurrent = 0;                     //first voltage probe connected to analog pin 3 = A2
float valueProbeCurrent = 0;                  //variable to store the analog value of analogPinCurrent
float voltageProbeCurrent = 0;                //calculated voltage at analogPinCurrent

float voltageDifference = 0;                  //difference in voltage between analogPinMeasureBatteryVoltage and GND
float batteryVoltage = 0;                     //calculated voltage of battery (considering voltage drop across power resistor)

float current = 0;                            //calculated current through the load (in mA)
float slope = (ocv(100,1) - ocv(0,1))/101;    //(v_final - v_init)/(I_final - I_init) --> rate of change of voltage wrt soc  ==> if prev_soc - soc = 0 (rate becomes infinte )conditon
float prev_current = 0;                       //variable to store value of previous current in A
float t = 0;

// Kalman Variables Initialisation
BLA::Matrix<2,1> X = {0.0, 1.0};              //state variables - [Vcap, SOC] 
BLA::Matrix<2,1> U = {0.0, 25.0};             //Input variables : [current,temp diff]
BLA::Matrix<2,2> P = {1.0 ,0.0 ,
                      0.0 ,1.0}; 
BLA::Matrix<2,1> K = {0.0, 0.0};
BLA::Matrix<2,2> ak = {0.9757691089747373,0,
                       0, 1};
BLA::Matrix<2,2> bk = {-0.006784883474301327,0,
                      0,0};
BLA::Matrix<1,2> ck = {-1, slope};
BLA::Matrix<1,2> dk = {-R0, 0.05};            // 0.05 is assumed 
BLA::Matrix<1> voltageError = {0};
BLA::Matrix<2,2> Qk = {1,0,0,1};
BLA::Matrix<1> Rk = {0.5};

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

BLA::Matrix<1> voltage_difference = {voltageDifference};
BLA::Matrix<1> battery_voltage = {batteryVoltage};
BLA::Matrix<1> v_pred = {24.25};             // Voltage Predicted (initial assumed)
float soc = 0.0;                             // % Current SOC
BLA::Matrix<2,2> M = {1.0, 0,
                      0, 1.0};
float prev_soc = 10.0;                       // % previous SOC (ssumed)
BLA::Matrix<1> temp = {0};


/////////////////////////////  SETUP //////////////////////////////////////////////////////
void setup() {
  //PIN setup
  Serial.begin(115200);                        // serial setup
  pinMode(analogPinMeasureBatteryVoltage, INPUT);              // voltage probe
  pinMode(analogPinTemp, INPUT);               // temperature
  pinMode(analogPinCurrent, INPUT);            // current sensing pin

  //OLED setup
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);

  //ADS sensor setup
  // ads.setGain(GAIN_TWOTHIRDS);  +/- 6.144V  1 bit = 0.1875mV (default)
  ads.setGain(GAIN_ONE);       // +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_TWO);        +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    +/- 0.256V  1 bit = 0.0078125mV 
  ads.begin();
} 


/////////////////////////////  LOOP //////////////////////////////////////////////////////
void loop() { 
  ////////////////////////////////////////////////////////////////////////////////////////
  measure_temperature();
  measure_voltage();
  delay(10);
  measure_current();

  // Extended Kalman Filter Implementation
  U(0,0) = prev_current;
  U(1,0) = prev_temperatureC - 25;
  ak(0,0) = pow(e, - time_diff/time_constant);
  bk(0,0) = (1 - pow(e, -time_diff/time_constant))*R1;
  bk(1,0) = - time_diff/(batteryCapacity*3600);

  // prediction step
  X = ak*X + bk*U;
  t = X(1,0);
  P = ak*(P*(~ak)) + Qk;
  v_pred = ocv(int(X(1,0)*100),1) - X(0,0),  dk*U;  
  //update step 
  voltageError = voltage_difference - v_pred;
  temp = ck*(P*(~ck)) + Rk;
  K = P*(~ck)/temp(0,0);
  X = X + K*voltageError;
  P = (M - K*ck)*P;

  calculate_soc();
  limit_checker();
  update_variables();

  display.clearDisplay();
  display.setTextSize(1);
  display.display();
  
  Serial.println();                            //extra spaces after one cycle to make debugging data easier to read
  Serial.println(); 
  delay(time_diff);                            //3 sec delay
}

/* Updates the current and previous variables for electric current, voltage, soc and other variables after each iteration*/
void update_variables(){
  prev_current = current;
  prev_soc = soc;
  prev_temperatureC = temperatureC; 

}

/* Calculates soc based on other parameters measured  and prints it*/
void calculate_soc(){ //rounding in lookup table for soc values (indexing)
  soc = X(1,0)*100;
  Serial.println("==================================================");
  Serial.print("State of Charge (SOC): ");
  Serial.println(soc);
  Serial.println("==================================================");
  
  display.print("State of Charge (SOC):");
  display.setTextSize(3);
  display.println(soc);
}

/* Reads the cvoltage across probes. calculates voltage across battery  and displays it */
void measure_voltage(){
  // voltage across probes ==> R1 = 39k & R2 = 5.1k
  valueProbeMeasureBatteryVoltage = ads.readADC_SingleEnded(1);                         // read the input value at voltage probe
  voltageProbeMeasureBatteryVoltage = (valueProbeMeasureBatteryVoltage)*(30.125/1000)* 8.64705;        //calculate voltage at probe one in Volts (Max value = 30 V) => 16 bit ADC
  Serial.print("Voltage Probe One (V): ");          //display voltage at probe one
  Serial.println(voltageProbeMeasureBatteryVoltage);  
    
  voltageDifference = voltageProbeMeasureBatteryVoltage;
  batteryVoltage = voltageDifference - current*resistance;     //calculated battery voltage = voltage difference across probes - voltage drop across power resistor
  Serial.print("Battery Voltage (V): ");            //display battery voltage
  Serial.println(batteryVoltage);
}

/* Reads the current as per analog reading and displays it */
void measure_current(){
  valueProbeCurrent = ads.readADC_SingleEnded(2);     // read the input value at probe three
  voltageProbeCurrent = valueProbeCurrent*0.125/1000; //calculate voltage at probe one in Amperes (Full Max. value = 50 A) => 16 bit ADC
  current = (voltageProbeCurrent-1.5959)/0.015;       // actual current
  Serial.println("Current Voltage: ");
  Serial.println(voltageProbeCurrent);
  
  Serial.print("Battery Current (A): ");              //display actual current
  Serial.println(current);  
}


/* Reads the temperature of the battery and displays it */
void measure_temperature(){
  valueProbeTemp = ads.readADC_SingleEnded(0);      //Read temperature ADC value
  temperatureC = valueProbeTemp*0.125/10;           // converting that reading to voltage
  Serial.print("Temperature (degrees C) ");         //display the temperature in degrees C
  Serial.println(temperatureC); 
 
 /*
  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0; //convert to Fahrenheit
  Serial.print("Temperature (degrees F) ");
  Serial.println(temperatureF); 
 */
}


/* Checks, bounds various electrical parameters of the battery and changes as per need displays it */
void limit_checker(){
 
  if(temperatureC > cutoffTemperatureC)            //stop charging if the battery temperature exceeds the safety threshold
   {
    Serial.println("Max Temperature Exceeded: ");
   }
   
  /*
  if(temperatureF > cutoffTemperatureF)            //stop charging if the battery temperature exceeds the safety threshold
   {
    outputValue = 0;
   }
   */
   if(soc == prev_soc)
   {
    Serial.println("Previous SOC is same as Current SOC: docv/dsoc = inf");
   }
   
   if(batteryVoltage > upperCutoffVoltage)        //stop charging if the battery voltage exceeds the safety threshold
   {
    Serial.println("Max Cutoff Voltage Exceeded: ");
   }  

   if(batteryVoltage < lowerCutoffVoltage)        //stop discharging if the battery voltage preceeds the minimum safety threshold
   {
    Serial.println("Min Cutoff Voltage Preceeded: ");
   }
   
   if(millis() > cutoffTime)                      //stop charging if the charge time exceeds threshold charging time
   {
    Serial.println("Max Charge Time Exceeded");
   }  

   if(current > 0){
    Serial.println("Battery is charging");        //battery charging condition
   }

   if(current < 0){
    Serial.println("Battery is discharging");     //battery discharging condition
   }
}
