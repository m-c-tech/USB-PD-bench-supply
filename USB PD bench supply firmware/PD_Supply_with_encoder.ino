#include <Bounce2.h>
#include <Arduino.h>
#include <AP33772.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <neotimer.h>

#define ENCODER_PIN1 3
#define ENCODER_PIN2 2
#define BUTTON_PIN 4
#define LED_PIN 25
#define LOAD_SWITCH_PIN 25

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// 0X3C+SA0 - 0x3C or 0x3D
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AP33772 usbpd;                          // Automatically wire to Wire0,
Neotimer mytimer_100ms = Neotimer(100); // Set timer's preset to 1s
Neotimer mytimer_500ms = Neotimer(500); // Set timer's preset to 1s

Bounce debouncer = Bounce();

volatile int encoderValue = 2;
volatile bool buttonState = false;
bool buttonLongPressed = false;
unsigned long buttonPressStartTime = 0;
const unsigned long longPressThreshold = 200; // 200 milliseconds (0.2 second) threshold for a long press

bool state = 0;
int targetVoltage;
int targetCurrent;
int oldVoltage;
int oldCurrent;

int lastEncoded = 0;
long encoderValueRaw = 0;

void handleEncoder()
{
  int MSB = digitalRead(ENCODER_PIN1);
  int LSB = digitalRead(ENCODER_PIN2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValueRaw++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValueRaw--;

  lastEncoded = encoded;
}

void handleButton()
{
  // Read button state
  bool currentButtonState = digitalRead(BUTTON_PIN);
  //Serial.println(currentButtonState);
  
  if (currentButtonState == 0)
  {
    buttonPressStartTime = millis();
  }
  else
  {
    if (millis() - buttonPressStartTime >= longPressThreshold)
    {
      Serial.println("Long press");
    }
    else
    {
      Serial.println("short press");
    }
    buttonPressStartTime = millis();
  }
}

void setup()
{
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  Serial.begin(9600);
  pinMode(ENCODER_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN2), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, CHANGE);

  usbpd.begin(); // Start pulling the PDOs from power supply
  usbpd.printPDO();

  if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    while (1)
    {
      Serial.println("Oled init fail");
    }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);

  oled.println(F("USB C - Power Supply"));
  oled.display();

  pinMode(LED_PIN, OUTPUT); // Built in LED
  pinMode(LOAD_SWITCH_PIN, OUTPUT); // Load Switch
  digitalWrite(23, HIGH);
  delay(2000);
  oled.clearDisplay();
}

void loop()
{
  // Read encoder
  if (encoderValueRaw != 0)
  {
    encoderValue += encoderValueRaw;
    encoderValueRaw = 0;

    Serial.print("Total Rotation: ");
    Serial.println(encoderValue/4);
  }
  if (mytimer_500ms.repeat()) //Reduce spam call over CC1/CC2
  {
    usbpd.setVoltage(int(myRA_V.getAverage()));    // Casting Float -> Int
    usbpd.setMaxCurrent(int(myRA_I.getAverage())); // Casting Float -> Int
  }

  if (mytimer_100ms.repeat()) //Reduce refresh rate of OLED
  {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.print("Voltage (V): ");
    oled.println(usbpd.readVoltage() / 1000.0);

    oled.write("Current (mA): ");
    oled.println(usbpd.readCurrent());

    oled.write("Target current: ");
    oled.println(targetCurrent);

    oled.display();
  }

  // Your non-blocking code can go here

  delay(50); // Adjust the delay as needed
}
