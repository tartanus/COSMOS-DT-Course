/*
  ESP32 DAC - Voltage Output Experiment
  espdac-voltage.ino
  ESP32 DAC Voltage Output Demo
  Steps through preset output voltages
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Define DAC pins
#define DAC_CH1 25
#define DAC_CH2 26

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);
}

void loop() {

  // Step through voltages, delay between levels
  dacWrite(DAC_CH2, 0);
  Serial.println("DAC Value 0");
  delay(500);

  dacWrite(DAC_CH2, 64);
  Serial.println("DAC Value 64");
  delay(500);

  dacWrite(DAC_CH2, 128);
  Serial.println("DAC Value 128");
  delay(500);

  dacWrite(DAC_CH2, 192);
  Serial.println("DAC Value 192");
  delay(500);

  dacWrite(DAC_CH2, 255);
  Serial.println("DAC Value 255");
  delay(500);
}







