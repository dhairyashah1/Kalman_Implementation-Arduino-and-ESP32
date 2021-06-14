int batteryCapacity = 2500;     //capacity rating of battery in mAh (eg)
float resistance = 10.0;     //measured resistance of the power resistor (connected to battery)
int cutoffVoltage = 4800;     //maximum battery voltage (in mV) that should not be exceeded = 4.8V (for 5V Li battery)
float cutoffTemperatureC = 75;     //maximum battery temperature that should not be exceeded (in degrees C)
//float cutoffTemperatureF = 167;     //maximum battery temperature that should not be exceeded (in degrees F) (F = 9*C/5 + 32)
long cutoffTime = 60*1000;     //maximum charge time of 1 min that should not be exceeded (in seconds)

int outputPin = 9;     // Output signal wire connected to digital pin 9 (Charging pin to battery)
int outputValue = 150;     //value of PWM output signal 
int outputFeedbackPin = 5; // feedback to read output
float outputReadValue = 0;

int analogPinOne = 0;     //first voltage probe connected to analog pin 1
float valueProbeOne = 0;     //variable to store the value of analogPinOne
float voltageProbeOne = 0;     //calculated voltage at analogPinOne

int analogPinTwo = 1;     //second voltage probe connected to analog pin 2
float valueProbeTwo = 0;     //variable to store the value of analogPinTwo
float voltageProbeTwo = 0;     //calculated voltage at analogPinTwo

int analogPinThree = 2;     //third voltage probe connected to analog pin 2
float valueProbeThree = 0;     //variable to store the value of analogPinThree
float tmp36Voltage = 0;     //calculated voltage at analogPinThree
float temperatureC = 0;     //calculated temperature of probe in degrees C
//float temperatureF = 0;     //calculated temperature of probe in degrees F

float voltageDifference = 0;     //difference in voltage between analogPinOne and analogPinTwo
float batteryVoltage = 0;     //calculated voltage of battery
float current = 0;     //calculated current through the load (in mA)
float targetCurrent = batteryCapacity / 10;     //target output current (in mA) set at C/10 or 1/10 of the battery capacity per hour
float currentError = 0;     //difference between target current and actual current (in mA)
float SocPercentage = 0;


void setup()
{
  Serial.begin(9600);     //  setup serial
  pinMode(outputPin, OUTPUT);     // sets the pin as output
}



void loop()
{   
  analogWrite(outputPin, outputValue);  //Write output value to output pin

  Serial.print("Output: ");     //display output values for monitoring with a computer
  Serial.println(outputValue); 

  valueProbeOne = analogRead(analogPinOne);    // read the input value at probe one
  voltageProbeOne = (valueProbeOne*5000)/1023;     //calculate voltage at probe one in milliVolts => 10 bit ADC
  Serial.print("Voltage Probe One (mV): ");     //display voltage at probe one
  Serial.println(voltageProbeOne);  
  
  valueProbeTwo = analogRead(analogPinTwo);    // read the input value at probe two
  voltageProbeTwo = (valueProbeTwo*5000)/1023;     //calculate voltage at probe two in milliVolts
  Serial.print("Voltage Probe Two (mV): ");     //display voltage at probe two
  Serial.println(voltageProbeTwo);  
  
  batteryVoltage = voltageProbeOne - voltageProbeTwo;     //calculate battery voltage 
  Serial.print("Battery Voltage (mV): ");     //display battery voltage
  Serial.println(batteryVoltage); 

  current = (voltageProbeOne - voltageProbeTwo) / resistance;     //calculate charge current
  Serial.print("Target Current (mA): ");     //display target current 
  Serial.println(targetCurrent);  
  Serial.print("Battery Current (mA): ");     //display actual current
  Serial.println(current);  
      
  currentError = targetCurrent - current;     //difference between target current and measured current
  Serial.print("Current Error  (mA): ");     //display current error 
  Serial.println(currentError);     

  valueProbeThree = analogRead(analogPinThree);    // read the input value at probe three  
  tmp36Voltage = valueProbeThree * 5.0/1023;     // converting that reading to voltage

  // currently not in use - formula depends on sensor used (temporary)
  temperatureC = (tmp36Voltage - 0.05) * 100 ;     //converting from 10 mv per degree wit 50 mV offset to degrees ((voltage - 50mV) times 100)
  Serial.print("Temperature (degrees C) ");     //display the temperature in degrees C
  Serial.println(temperatureC); 
 
 /*
  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;     //convert to Fahrenheit
  Serial.print("Temperature (degrees F) ");
  Serial.println(temperatureF); 
 */
 
  Serial.println();     //extra spaces to make debugging data easier to read
  Serial.println();  


// bounding range of current - can change (current varies -> temperature varies)
  if(abs(currentError) > 50)     //if output error is large enough more than diff of 50mA, adjust output 
   {
    outputValue = outputValue + currentError/2;

    if(outputValue < 1)    //output can never go below 0
     {
      outputValue = 0;
     }

    if(outputValue > 254)     //output can never go above 255
     {
      outputValue = 255;
     }
    
    analogWrite(outputPin, outputValue);     //write the new output value
   }
 
 
  if(temperatureC > cutoffTemperatureC)     //stop charging if the battery temperature exceeds the safety threshold
   {
    outputValue = 0;
    Serial.println("Max Temperature Exceeded: ");
   }
   
  /*
  if(temperatureF > cutoffTemperatureF)     //stop charging if the battery temperature exceeds the safety threshold
   {
    outputValue = 0;
   }
   */
   
   if(batteryVoltage > cutoffVoltage)     //stop charging if the battery voltage exceeds the safety threshold
   {
    outputValue = 0;
    Serial.println("Max Voltage Exceeded: ");
   }  
 
   if(millis() > cutoffTime)     //stop charging if the charge time exceeds threshold charging time
   {
    outputValue = 0;
    Serial.println("Max Charge Time Exceeded");
   }  

   delay(3000);     //delay 3 seconds before next iteration
}
