void setup() {
  // put your setup code here, to run once:
  int batteryCapacity = 2500;     //capacity rating of battery in mAh (eg)
  float resistance = 10.0;     //measured resistance of the power resistor (connected to battery)
  int cutoffVoltage = 4800;     //maximum battery voltage (in mV) that should not be exceeded = 4.8V (for 5V Li battery)
  float cutoffTemperatureC = 75;     //maximum battery temperature that should not be exceeded (in degrees C)
  //float cutoffTemperatureF = 167;     //maximum battery temperature that should not be exceeded (in degrees F) (F = 9*C/5 + 32)
  long cutoffTime = 60*1000;     //maximum charge time of 1 min that should not be exceeded (in seconds)

  // voltage reading probes across battery
  int analogPinOne = 0;     //first voltage probe connected to analog pin 1
  float valueProbeOne = 0;     //variable to store the anlog value of analogPinOne
  float voltageProbeOne = 0;     //calculated voltage at analogPinOne
  
  int analogPinTwo = 1;     //second voltage probe connected to analog pin 2
  float valueProbeTwo = 0;     //variable to store the analog value of analogPinTwo
  float voltageProbeTwo = 0;     //calculated voltage at analogPinTwo

  // pin to measure temperature
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
  float slope = 15; //(v_final - v_init)/(I_final - I_init) {assumed}

  // state variables - [Vcap, SOC] 
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
  v_pred = np.zeros(time_vector.size) # Voltage Predicted
  soc = np.zeros(time_vector.size)
  v_pred[0] = 4.25
  soc[0] = 100
}

void loop() {
  // put your main code here, to run repeatedly:

}
