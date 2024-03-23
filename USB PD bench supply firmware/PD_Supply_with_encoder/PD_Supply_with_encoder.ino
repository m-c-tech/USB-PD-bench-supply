#include <Bounce2.h>
#include <Arduino.h>
#include <AP33772.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <neotimer.h>

#define ENCODER_PIN1 2
#define ENCODER_PIN2 3
#define BUTTON_PIN 4
#define LED_PIN 25
#define LOAD_SWITCH_PIN 23

#define LOGO_HEIGHT 32
#define LOGO_WIDTH 32

#define textsize 1

static const unsigned char PROGMEM logo_bmp[] = {
  // 'OIG, 32x32px
  0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x02, 0x30, 0x00, 0x00, 0x04, 0x60, 0x00, 0x00, 0x08, 0xc0,
  0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0xf8, 0x3c, 0x01, 0xe0, 0x00, 0x7e, 0x03, 0xff, 0xc0,
  0x67, 0x07, 0x30, 0x30, 0x63, 0x0e, 0x3f, 0xc8, 0x73, 0x9c, 0x7d, 0xf4, 0x31, 0xfc, 0x7d, 0xfa,
  0x31, 0xf8, 0x78, 0xfd, 0x39, 0xf8, 0xf8, 0xfd, 0x18, 0xf1, 0xf2, 0x7e, 0x1c, 0xf1, 0xf2, 0x7e,
  0x0c, 0x63, 0xe7, 0x3e, 0x0e, 0x23, 0xe7, 0x3e, 0x06, 0x07, 0xe0, 0x3e, 0x07, 0x07, 0xcf, 0x9e,
  0x03, 0x0f, 0xcf, 0x9c, 0x03, 0x9f, 0x9f, 0xcd, 0x01, 0xff, 0x9f, 0xcd, 0x01, 0xff, 0xff, 0xfa,
  0x00, 0x07, 0xff, 0xf4, 0x00, 0x00, 0xff, 0xc8, 0x0f, 0xe0, 0x60, 0x30, 0x00, 0xc0, 0x1f, 0xc0,
  0x01, 0x88, 0x00, 0x00, 0x03, 0x10, 0x00, 0x00, 0x06, 0x20, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00
};

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// 0X3C+SA0 - 0x3C or 0x3D
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AP33772 usbpd;                              // Automatically wire to Wire0,
Neotimer mytimer_50ms = Neotimer(50);       // Set timer's preset to 100mS
Neotimer mytimer_500ms = Neotimer(500);     // Set timer's preset to 1s
Neotimer mytimer_1000ms = Neotimer(1000);   // Set timer's preset to 1s
Neotimer encoderSpeedTimer = Neotimer(10);  // Timer for encoder speed calculation, set to 10ms
Neotimer buttonTimer = Neotimer(10000);

Bounce debouncer = Bounce();

volatile int encoderVoltageValue = 5000;
volatile int encoderCurrentValue = 500;
volatile bool buttonState = false;
bool buttonLongPressed = false;
bool buttonReleased = true;  // Added state variable
unsigned long buttonPressStartTime = 0;
const unsigned long lockPressThreshold = 200;      // 200 milliseconds (0.2 second) threshold
const unsigned long restartPressThreshold = 2000;  // 1000 milliseconds (1 second) threshold
int lastModeChangeTime = 0;
int debounceTime = 1000;


// Constants for speed control
const int lowSpeedThreshold = 8;      // Number of encoder steps per time window for low speed
const int highSpeedThreshold = 20;    // Number of encoder steps per time window for high speed
const unsigned long timeWindow = 10;  // Time window in milliseconds
int encoderValue = 0;
int lastEncoderValue = 0;
int tempEncoderValueRaw = 0;
int encoderSpeedValue = 0;
int lastEncoderSpeedValue = 0;
int step = 5;

enum MODE {
  VOLTAGE = 0,
  CURRENT = 1,
  LOCKED = 2
};
int mode = VOLTAGE;  // mode 0 = voltage control, mode 1 = current control, mode 2 = locked
bool state = 0;
int targetCurrent = 500;
int oldVoltage;
int oldCurrent;

int lastEncoded = 0;
long encoderValueRaw = 0;

// create a buffer for the text
char l0[20];
char l1[20];
char l2[20];
char l3[20];

void handleEncoder() {
  int MSB = digitalRead(ENCODER_PIN1);
  int LSB = digitalRead(ENCODER_PIN2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValueRaw++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValueRaw--;
  }
  lastEncoded = encoded;
}

void handleButton()  //now unsused function
{
  // Read button state
  bool currentButtonState = digitalRead(BUTTON_PIN);
  //Serial.println(currentButtonState);

  if (currentButtonState == 0) {
    buttonTimer.reset();
    buttonTimer.start();
  } else {
    if (buttonTimer.stop() < lockPressThreshold) {
      Serial.print("short press m=");
      Serial.print(mode);
      if (mode != LOCKED) {
        if (mode == VOLTAGE) {
          mode = CURRENT;
        } else {
          mode = VOLTAGE;
        }
        Serial.print(" -> ");
        Serial.println(mode);
      }
    } else if (buttonTimer.stop() < restartPressThreshold) {
      Serial.print("Long press m=");
      Serial.print(mode);
      if (mode == LOCKED) {
        mode = VOLTAGE;
      } else {
        mode = LOCKED;
      }
      Serial.print(" -> ");
      Serial.println(mode);
    } else {
      //Serial.print("Resetting...");
      delay(100);
      usbpd.reset();
    }
    buttonTimer.reset();
  }
}

void setup() {
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  Serial.begin(9600);
  delay(500);
  pinMode(ENCODER_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // BOUNCE SETUP
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10);


  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN2), handleEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, CHANGE);


  while (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Oled init fail");
  }

  oled.clearDisplay();
  oled.setTextSize(textsize);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  //Display the image with a 90-degree rotation
  oled.setRotation(2);


  // invert the display
  // oled.invertDisplay(false);
  oled.drawBitmap(0, 0, logo_bmp, LOGO_HEIGHT, LOGO_WIDTH, SSD1306_WHITE);
  // oled.invertDisplay(true);
  oled.setCursor(40, 12);
  oled.println(F("USB PD"));
  oled.setCursor(40, 20);
  oled.println(F("Bench Supply"));
  oled.display();

  usbpd.begin();  // Start pulling the PDOs from power supply


  pinMode(LED_PIN, OUTPUT);          // Built in LED
  pinMode(LOAD_SWITCH_PIN, OUTPUT);  // Load Switch
  digitalWrite(LOAD_SWITCH_PIN, HIGH);
  delay(2000);
  usbpd.printPDO();
  oled.clearDisplay();

  sprintf(l0, "                ");
  sprintf(l1, "                ");
  sprintf(l2, "                ");
}

void loop() {
  debouncer.update();

  // Read the debounced state of the button
  int buttonState = debouncer.read();

  // Handle button press logic
  if (buttonState == LOW) {
    unsigned long currentTime = millis();
    // Check if enough time has passed since the last mode change
    if (currentTime - lastModeChangeTime > debounceTime) {
      // Button is pressed, handle short/long press logic
      if (debouncer.duration() < lockPressThreshold) {
        // Short press logic
        Serial.print("short press m=");
        Serial.print(mode);
        if (mode != LOCKED) {
          if (mode == VOLTAGE) {
            mode = CURRENT;
          } else {
            mode = VOLTAGE;
          }
          Serial.print(" -> ");
          Serial.println(mode);
        }
      } else if (debouncer.duration() < restartPressThreshold) {
        // Long press logic
        Serial.print("Long press m=");
        Serial.print(mode);
        if (mode == LOCKED) {
          mode = VOLTAGE;
        } else {
          mode = LOCKED;
        }
        Serial.print(" -> ");
        Serial.println(mode);
      } else {
        // Resetting logic
        Serial.println("Resetting...");
        delay(100);
        usbpd.reset();
        usbpd.begin();
        usbpd.printPDO();
      }

      // Update the last mode change timestamp
      lastModeChangeTime = currentTime;
    }
  }


  // Read encoder for speed calculation
  //int tempEncoderValueRaw = encoderValueRaw;  // Store the raw value temporarily
  if (encoderValueRaw != 0) {
    // Encoder speed sample
    encoderSpeedValue += encoderValueRaw * 10;
  }

  // Check encoder speed every timeWindow milliseconds
  if (encoderSpeedTimer.repeat()) {
    // Calculate encoder speed
    int encoderSpeed = encoderSpeedValue - lastEncoderSpeedValue;
    // Adjust encoder speed based on the calculated speed
    if (abs(encoderSpeed) >= highSpeedThreshold) {
      step = 40;
      //Serial.println("high speed bby");
    } else {
      step = 5;
    }

    // Update lastEncoderSpeedValue
    lastEncoderSpeedValue = encoderSpeedValue;

    // Reset encoder speed values
    encoderSpeedValue = 0;
  }

  // Read encoder
  if (mode == VOLTAGE)  // voltage control
  {
    if (encoderValueRaw != 0) {
      //int tempEncoderValueRaw = encoderValueRaw;  // Store the raw value temporarily
      // Serial.print(encoderValueRaw);
      encoderVoltageValue += encoderValueRaw * step;
      encoderValueRaw = 0;

      if (encoderVoltageValue < 3300) {
        encoderVoltageValue = 3300;
      }
      if (encoderVoltageValue > 21000) {
        encoderVoltageValue = 21000;
      }
    }
  } else if (mode == CURRENT)  // current control
  {
    if (encoderValueRaw != 0) {
      //int tempEncoderValueRaw = encoderValueRaw;  // Store the raw value temporarily
      // Serial.print(encoderValueRaw);
      encoderCurrentValue += encoderValueRaw * step;
      encoderValueRaw = 0;

      if (encoderCurrentValue <= 500) {
        encoderCurrentValue = 500;
      }
      if (encoderCurrentValue > usbpd.getMaxCurrent()) {
        encoderCurrentValue = usbpd.getMaxCurrent();
      }
    }
  } else if (encoderValueRaw != 0) {
    encoderValueRaw = 0;
  }

  if (mytimer_500ms.repeat())  //Reduce spam call over CC1/CC2
  {
    usbpd.setVoltage(encoderVoltageValue);  // Casting Float -> Int
    if (usbpd.getMaxCurrent() < encoderCurrentValue) {
      encoderCurrentValue = usbpd.getMaxCurrent();
    }
    usbpd.setMaxCurrent(encoderCurrentValue);  // Casting Float -> Int
  }


  if (mytimer_50ms.repeat())  //Reduce refresh rate of OLED
  {
    oled.clearDisplay();

    sprintf(l0, " %4.1f %3.2f", encoderVoltageValue / 1000.0, encoderCurrentValue / 1000.0);
    sprintf(l1, " %4.1f %3.2f", usbpd.readVoltage() / 1000.0, usbpd.readCurrent() / 1000.0);
    sprintf(l3, "    Volts | Amps");


    oled.setTextSize(1);

    oled.setCursor(0, 8);

    oled.println(l2);

    oled.println("Set");
    oled.setTextSize(2);

    oled.println(l0);
    oled.setTextSize(1);

    oled.println("Actual");

    oled.setTextSize(2);

    oled.println(l1);


    oled.setCursor(0, 2);

    oled.setTextSize(1);


    oled.println(l3);




    if (mode == VOLTAGE) {
      l1[1] = '>';
      oled.setTextSize(1);

      oled.setCursor(0 * 6, 50);
      oled.print(l1[1]);

      // oled.print(l1[4]);
      // //oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      // for (int i = 0; i < 6; i++) {
      //   oled.print(l1[5 + i]);
      // }
      //oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    } else {
      l1[1] = ' ';
    }

    if (mode == CURRENT) {
      // oled.clearDisplay();

      l1[7] = '>';
      oled.setTextSize(1);

      oled.setCursor(11 * 6 - 3, 50);
      oled.print(l1[7]);
      // oled.print(l1[8]);
      // //oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      // for (int i = 0; i < 6; i++) {
      //   oled.print(l1[9 + i]);
      // }
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    } else {
      l1[7] = ' ';
    }

    oled.display();
  }

  if (mytimer_1000ms.repeat()) {
    Serial.println(l0);
    Serial.println(l1);
    Serial.println(l2);
    Serial.println(l3);
  }

  // Your non-blocking code can go here

  delay(50);  // Adjust the delay as needed
}
