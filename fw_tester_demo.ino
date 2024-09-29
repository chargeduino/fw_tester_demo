#include <Adafruit_RGBLCDShield.h>

#define USE_SERIAL 0
#define USE_RGBLCD 1

// simulation settings
#define TIME_CONNECTED_MS   5000
#define TIME_CHARGING_MS    15000

// fixed parameters
// micro-controller
#define ANALOG_VREF         5
#define ADC_MAX_VALUE_F     1023.0
// charging system
#define DIODE_VOLTAGE_F     0.6
#define PILOT_DIVISION_K    3
// LCD: refresh rate
#define RGB_LCD_REFRESH_MS  500
// LCD: backlight colors
#define RGB_LCD_RED         0x1
#define RGB_LCD_YELLOW      0x3
#define RGB_LCD_GREEN       0x2
#define RGB_LCD_TEAL        0x6
#define RGB_LCD_BLUE        0x4
#define RGB_LCD_VIOLET      0x5
#define RGB_LCD_WHITE       0x7

// pin definitions
#define PIN_CHARGE_START    5
#define PIN_PROXIMITY       (A2)
#define PIN_CTRL_PILOT      (A3)

// type: state (for the state-machine)
typedef enum {
  IDLE,
  CONNECTED,
  CHARGING,
  FINISHED
} t_state;

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {
#if USE_SERIAL
  while(!Serial){;}
  Serial.begin(115200); // UART debug
#endif

#if USE_RGBLCD
  lcd.begin(16, 2); // LCD 16x2
  lcd.clear(); // also sets position to zero
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_CHARGE_START, OUTPUT);
  digitalWrite(PIN_CHARGE_START, LOW);

  analogReference(DEFAULT); // https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
}

void loop() {
  static unsigned long prevTime = 0;
  static unsigned long prevIdleTime = 0;
  static unsigned long prevConnTime = 0;
  static t_state status;
  static bool readyToCharge;
  float pilotVoltage;
  float fullscaleVoltage;
  float dutyCycle;

  // rescale what's read on the ADC input to the real (outside) voltage
  pilotVoltage = (analogRead(PIN_CTRL_PILOT) * ANALOG_VREF / ADC_MAX_VALUE_F) * PILOT_DIVISION_K;
  fullscaleVoltage = ((readyToCharge)? 6.0 : 9.0) - DIODE_VOLTAGE_F;
  dutyCycle = 100 * pilotVoltage / fullscaleVoltage;

  /*
  Serial.println(pilotVoltage);
  Serial.println(dutyCycle);
  delay(100);
  */

  if (millis() - prevTime > RGB_LCD_REFRESH_MS) {
    prevTime = millis();
    
    switch (status) {
      default:
        status = IDLE;
        readyToCharge = false;
      case IDLE:
#if USE_SERIAL
        Serial.println("No EV");
#endif
#if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_VIOLET);
        lcd.print("No EV");
#endif
        if (pilotVoltage >= 0.1) {
          status = CONNECTED;
        }
        prevIdleTime = millis();
        readyToCharge = false;
        break;
        
      case CONNECTED:
#if USE_SERIAL
        Serial.println("EV Connected");
#endif
#if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_YELLOW);
        lcd.print("EV Connected");
        lcd.setCursor(0,1); // second row
        lcd.print("PWM: ");
        lcd.print(dutyCycle);
#endif
        if (pilotVoltage < 0.1) {
          status = IDLE;
        }
        else if (millis() - prevIdleTime > TIME_CONNECTED_MS) {
          status = CHARGING;
        }
        prevConnTime = millis();
        readyToCharge = false;
        break;
        
      case CHARGING:
#if USE_SERIAL
        Serial.println("EV Charging...");
        Serial.println(dutyCycle);
#endif
#if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_RED);
        lcd.print("EV Charging...");
        lcd.setCursor(0,1); // second row
        lcd.print("PWM: ");
        lcd.print(dutyCycle);
#endif
        if (pilotVoltage < 0.1) {
          status = IDLE;
        }
        else if (millis() - prevConnTime > TIME_CHARGING_MS) {
          status = FINISHED;
        }
        readyToCharge = true;
        break;
        
      case FINISHED:
#if USE_SERIAL
        Serial.println("Charge finished");
#endif
#if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_GREEN);
        lcd.print("Charge finished!");
#endif
        if (pilotVoltage < 0.1) {
          status = IDLE;
        }
        readyToCharge = false;
        break;
    }
  }

  digitalWrite( PIN_CHARGE_START, (readyToCharge)? HIGH : LOW );
}
