#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

enum InputDevice : uint8_t
{
  Sensor1 = 0,
  Sensor2 = 2,
  Sensor3 = 4,
  Sensor4 = 6,
  Sensor5 = 8,
  Sensor6 = 10,
  PushButton = 11,
};

constexpr float DISTANCESCM[] = {0, 14.3, 21.8, 29.3, 35, 39.8};
constexpr uint16_t SENSOR_COUNT =
    sizeof(DISTANCESCM) / sizeof(DISTANCESCM[0]);
constexpr int Sensors[] = {InputDevice::Sensor1, InputDevice::Sensor2, InputDevice::Sensor3,
                           InputDevice::Sensor4, InputDevice::Sensor5, InputDevice::Sensor6};

unsigned long timeStamps[SENSOR_COUNT];
float velocities[SENSOR_COUNT - 1];
bool triggered[SENSOR_COUNT] = {};

constexpr float r_bola = 0.005;
constexpr float rho_fluida = 1260.0;
constexpr float rho_bola = 7850.0;
constexpr float gravity = 9.8;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void ResetExperiment()
{
  memset(triggered, 0, sizeof(triggered));
  memset(velocities, 0, sizeof(velocities));
  memset(timeStamps, 0, sizeof(timeStamps));
}

void PrintResults()
{
  Serial.println("START_DATA");
  for (size_t i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print("t");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(timeStamps[i]);
  }
  float sum = 0;
  for (size_t i = 0; i < SENSOR_COUNT - 1; i++)
  {
    Serial.print("v");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(velocities[i], 6);
    sum += velocities[i];
  }
  Serial.println("END_DATA");

  float v_avg = 0.0;
  if (SENSOR_COUNT > 1)
    v_avg = sum / (SENSOR_COUNT - 1);

  float viscosity = 0.0;
  if (v_avg > 0.0001)
    viscosity = (2.0 / 9.0) * gravity * (r_bola * r_bola) * (rho_bola - rho_fluida) / v_avg;

  Serial.print("V rata-rata: ");
  Serial.println(v_avg, 6);
  Serial.print("Viscositas: ");
  Serial.println(viscosity, 6);
  Serial.print("Formula components: ");
  Serial.print("2/9=");
  Serial.print(2.0 / 9.0, 4);
  Serial.print(" g=");
  Serial.print(gravity, 4);
  Serial.print(" r²=");
  Serial.print(r_bola * r_bola, 6);
  Serial.print(" Δρ=");
  Serial.println(rho_bola - rho_fluida, 1);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(velocities[SENSOR_COUNT - 2], 6);
  lcd.setCursor(0, 1);
  lcd.print("Vis: ");
  lcd.print(viscosity, 6);
}

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.backlight();

  for (auto i : Sensors)
    pinMode(i, INPUT);
  pinMode(InputDevice::PushButton, INPUT_PULLUP);

  ResetExperiment();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Alat siap pakai");
}

void loop()
{
  static bool isExperimentRunning = false;
  static unsigned long startTime = 0;
  static unsigned long elapsed = 0;

  if (!isExperimentRunning && !digitalRead(InputDevice::PushButton))
  {
    delay(50);
    if (digitalRead(InputDevice::PushButton))
      return;
    isExperimentRunning = true;
    startTime = micros();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Eksperimen mulai");
  }

  if (!isExperimentRunning)
    return;

  bool states[SENSOR_COUNT];
  for (size_t i = 0; i < SENSOR_COUNT; i++)
  {
    bool input = digitalRead(Sensors[i]);
    if (i < 3)
      states[i] = !input;
    else
      states[i] = input;
  }

  elapsed = micros() - startTime;

  for (size_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (states[i] && !triggered[i])
    {
      timeStamps[i] = (i == 0 ? 0 : elapsed);
      triggered[i] = true;
      if (i > 0)
      {
        size_t idx = i - 1;
        float distance = DISTANCESCM[i] - DISTANCESCM[i - 1];     // cm
        float time_s = (timeStamps[i] - timeStamps[i - 1]) / 1e6; // μs to s

        if (time_s > 0.0001)
        {
          velocities[idx] = (distance / 100.0) / time_s; // cm to m

          Serial.print("Distance: ");
          Serial.print(distance);
          Serial.print(" cm, Time: ");
          Serial.print(time_s, 6);
          Serial.print(" s, Velocity: ");
          Serial.println(velocities[idx], 6);
        }
        else
        {
          velocities[idx] = 0.0;
          Serial.println("Warning: Time difference too small!");
        }
      }
      if (i == SENSOR_COUNT - 1)
      {
        PrintResults();
        ResetExperiment();
        isExperimentRunning = false;
      }
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" triggered at ");
      Serial.print(timeStamps[i]);
      Serial.print(" us, velocity: ");
      if (i > 0)
        Serial.println(velocities[i - 1], 2);
      else
        Serial.println("N/A");
      break;
    }
  }
}
