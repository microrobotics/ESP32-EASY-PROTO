#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int button1Pin = 32;
const int button2Pin = 33;
const int irPin = 36;
const int potPin = 34;
const int ldrPin = 35;
const int redLEDPin = 2;
const int blueLEDPin = 15;
const int rgbLEDPin = 4;
const int buzzerPin = 27;

Adafruit_NeoPixel rgbLED(1, rgbLEDPin, NEO_GRB + NEO_KHZ800);

int currentScreen = 0;
unsigned long pulseLow, pulseHigh;
int potValue, ldrValue;
bool button1State, button2State;
bool buzzerActive = false;

// Define the I2C addresses for the sensors
#define LSM6DS3TR_C_ADDRESS 0x6A
#define HDC1000_ADDRESS 0x40
#define LPS22HB_ADDRESS 0x5C

// Define register addresses for the LSM6DS3TR-C
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

float temp = 0;
float hum = 0;
float pressure = 0;
float altitude = 0;
int16_t accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Gyroscope offsets
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// Known sea level pressure for Centurion, South Africa in hPa
const float knownSeaLevelPressure = 1026.0;

void setup() {
  Serial.begin(115200);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(irPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  Wire.begin(21, 22);  // Initialize I2C with SDA on GPIO 21 and SCL on GPIO 22

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  // Clear the display to remove the Adafruit logo
  display.clearDisplay();
  display.display();
  delay(2000);

  // Initialize the IMU
  writeRegister(LSM6DS3TR_C_ADDRESS, CTRL1_XL, 0x60);
  writeRegister(LSM6DS3TR_C_ADDRESS, CTRL2_G, 0x60);

  Serial.println("LSM6DS3TR-C initialization complete");

  // Initialize RGB LED
  rgbLED.begin();
  rgbLED.show();

  // Calibrate the gyroscope
  calibrateGyroscope();

  // Create FreeRTOS tasks
  xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
  xTaskCreate(displayTask, "Display Task", 2048, NULL, 1, NULL);
}

void loop() {
  // Main loop is not used in this FreeRTOS implementation
}

void calibrateGyroscope() {
  const int samples = 500;
  int32_t sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    int16_t gx = read16(LSM6DS3TR_C_ADDRESS, OUTX_L_G, OUTX_H_G);
    int16_t gy = read16(LSM6DS3TR_C_ADDRESS, OUTY_L_G, OUTY_H_G);
    int16_t gz = read16(LSM6DS3TR_C_ADDRESS, OUTZ_L_G, OUTZ_H_G);

    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gyroXOffset = sumX / (float)samples;
  gyroYOffset = sumY / (float)samples;
  gyroZOffset = sumZ / (float)samples;

  Serial.print("Gyro Offsets - X: ");
  Serial.print(gyroXOffset);
  Serial.print(", Y: ");
  Serial.print(gyroYOffset);
  Serial.print(", Z: ");
  Serial.println(gyroZOffset);
}

void sensorTask(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    button1State = digitalRead(button1Pin) == LOW;
    button2State = digitalRead(button2Pin) == LOW;

    if (button1State) {
      currentScreen++;
      if (currentScreen > 10) currentScreen = 0;
      delay(300); // Debounce delay
    }

    if (button2State) {
      currentScreen--;
      if (currentScreen < 0) currentScreen = 10;
      delay(300); // Debounce delay
    }

    // Read sensor data
    switch (currentScreen) {
      case 0:
        if (digitalRead(irPin) == LOW) {
          pulseLow = pulseIn(irPin, LOW, 100000);  // Timeout of 100 ms
          pulseHigh = pulseIn(irPin, HIGH, 100000); // Timeout of 100 ms
        }
        break;
      case 1:
        accelX = averageRead16(LSM6DS3TR_C_ADDRESS, OUTX_L_XL, OUTX_H_XL);
        accelY = averageRead16(LSM6DS3TR_C_ADDRESS, OUTY_L_XL, OUTY_H_XL);
        accelZ = averageRead16(LSM6DS3TR_C_ADDRESS, OUTZ_L_XL, OUTZ_H_XL);
        break;
      case 2:
        gyroX = averageGyroRead(LSM6DS3TR_C_ADDRESS, OUTX_L_G, OUTX_H_G) - gyroXOffset;
        gyroY = averageGyroRead(LSM6DS3TR_C_ADDRESS, OUTY_L_G, OUTY_H_G) - gyroYOffset;
        gyroZ = averageGyroRead(LSM6DS3TR_C_ADDRESS, OUTZ_L_G, OUTZ_H_G) - gyroZOffset;
        break;
      case 3:
        temp = averageTemperature();
        hum = averageHumidity();
        Serial.print("Temp: ");
        Serial.print(temp);
        Serial.print(" C, Humidity: ");
        Serial.print(hum);
        Serial.println(" %");
        break;
      case 4:
        pressure = averagePressure();
        altitude = calculateAltitude(pressure, knownSeaLevelPressure);
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" hPa");
        Serial.print("Altitude: ");
        Serial.print(altitude);
        Serial.println(" m");
        break;
      case 5:
        potValue = averageAnalogRead(potPin, 20); // Increase averaging samples to 20
        break;
      case 6:
        ldrValue = analogRead(ldrPin);
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for task
  }
}

void displayTask(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Turn off RGB LED before switching screens
    rgbLED.setPixelColor(0, rgbLED.Color(0, 0, 0));
    rgbLED.show();

    // Turn off LEDs before switching screens
    digitalWrite(redLEDPin, LOW);
    digitalWrite(blueLEDPin, LOW);

    // Turn off buzzer before switching screens
    controlBuzzer(false);

    switch (currentScreen) {
      case 0:
        display.println("IR Receiver Data");
        char lowPulseHex[10];
        char highPulseHex[10];
        sprintf(lowPulseHex, "0x%lX", pulseLow);
        sprintf(highPulseHex, "0x%lX", pulseHigh);
        display.print("Low: ");
        display.print(pulseLow);
        display.print(" (");
        display.print(lowPulseHex);
        display.println(")");
        display.print("High: ");
        display.print(pulseHigh);
        display.print(" (");
        display.print(highPulseHex);
        display.println(")");
        break;
      case 1:
        display.println("Accelerometer Data");
        display.print("X: ");
        display.println(round(accelX * 0.061 / 1000, 2));
        display.print("Y: ");
        display.println(round(accelY * 0.061 / 1000, 2));
        display.print("Z: ");
        display.println(round(accelZ * 0.061 / 1000, 2));
        break;
      case 2:
        display.println("Gyroscope Data");
        display.print("X: ");
        display.println(round(gyroX * 8.75 / 1000, 2));
        display.print("Y: ");
        display.println(round(gyroY * 8.75 / 1000, 2));
        display.print("Z: ");
        display.println(round(gyroZ * 8.75 / 1000, 2));
        break;
      case 3:
        display.println("Temp & Humidity");
        display.print("Temp: ");
        display.print(round(temp, 2));
        display.println(" C");
        display.print("Hum: ");
        display.print(round(hum, 2));
        display.println(" %");
        break;
      case 4:
        display.println("Pressure Data");
        display.print("Pressure: ");
        display.print(round(pressure, 2));
        display.println(" hPa");
        display.print("Altitude: ");
        display.print(round(altitude, 2));
        display.println(" m");
        break;
      case 5:
        display.println("Potentiometer Data");
        display.print("Value: ");
        display.println(potValue);
        break;
      case 6:
        display.println("LDR Data");
        display.print("Value: ");
        display.println(ldrValue);
        break;
      case 7:
        display.println("Red LED ON");
        digitalWrite(redLEDPin, HIGH);
        break;
      case 8:
        display.println("Blue LED ON");
        digitalWrite(blueLEDPin, HIGH);
        break;
      case 9:
        display.println("RGB LED Control");
        rgbLED.setPixelColor(0, rgbLED.Color(255, 0, 0)); // Red
        rgbLED.show();
        vTaskDelay(pdMS_TO_TICKS(500));
        rgbLED.setPixelColor(0, rgbLED.Color(0, 255, 0)); // Green
        rgbLED.show();
        vTaskDelay(pdMS_TO_TICKS(500));
        rgbLED.setPixelColor(0, rgbLED.Color(0, 0, 255)); // Blue
        rgbLED.show();
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      case 10:
        display.println("Buzzer Control");
        controlBuzzer(true);
        break;
    }

    display.display();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for task
  }
}

void controlBuzzer(bool state) {
  if (state && !buzzerActive) {
    tone(buzzerPin, 1000); // Play a 1000 Hz tone
    buzzerActive = true;
  } else if (!state && buzzerActive) {
    noTone(buzzerPin); // Stop the tone
    buzzerActive = false;
  }
}

void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

int16_t read16(uint8_t deviceAddress, uint8_t lowAddress, uint8_t highAddress) {
  uint8_t lowByte = readRegister(deviceAddress, lowAddress);
  uint8_t highByte = readRegister(deviceAddress, highAddress);
  return (int16_t)(highByte << 8 | lowByte);
}

float readTemperature() {
  Wire.beginTransmission(HDC1000_ADDRESS);
  Wire.write(0x00); // Temperature register
  Wire.endTransmission();
  delay(20); // Add a small delay
  Wire.requestFrom(HDC1000_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
    return (rawTemp / 65536.0) * 165.0 - 40.0;
  }
  return 0; // Return 0 if no data is available
}

float readHumidity() {
  Wire.beginTransmission(HDC1000_ADDRESS);
  Wire.write(0x01); // Humidity register
  Wire.endTransmission();
  delay(20); // Add a small delay
  Wire.requestFrom(HDC1000_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint16_t rawHum = (Wire.read() << 8) | Wire.read();
    return (rawHum / 65536.0) * 100.0;
  }
  return 0; // Return 0 if no data is available
}

float readPressure() {
  Wire.beginTransmission(LPS22HB_ADDRESS);
  Wire.write(0x28 | 0x80); // Pressure register (auto increment)
  Wire.endTransmission();
  Wire.requestFrom(LPS22HB_ADDRESS, 3);
  if (Wire.available() == 3) {
    uint32_t rawPress = Wire.read();
    rawPress |= (Wire.read() << 8);
    rawPress |= (Wire.read() << 16);
    return rawPress / 4096.0;
  }
  return 0; // Return 0 if no data is available
}

float calculateAltitude(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

int16_t averageRead16(uint8_t deviceAddress, uint8_t lowAddress, uint8_t highAddress) {
  const int samples = 3;
  int32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += read16(deviceAddress, lowAddress, highAddress);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

float averageGyroRead(uint8_t deviceAddress, uint8_t lowAddress, uint8_t highAddress) {
  const int samples = 10; // Increase samples for better averaging
  int32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += read16(deviceAddress, lowAddress, highAddress);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

float averageTemperature() {
  const int samples = 3;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readTemperature();
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

float averageHumidity() {
  const int samples = 3;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readHumidity();
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

float averagePressure() {
  const int samples = 3;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readPressure();
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

int averageAnalogRead(int pin, int samples) {
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }
  return sum / samples;
}

float round(float value, int decimalPlaces) {
  float scale = pow(10, decimalPlaces);
  return round(value * scale) / scale;
}
