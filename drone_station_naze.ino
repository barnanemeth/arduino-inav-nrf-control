#include <XBOXRECV.h>
#include "RF24.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define INDEX_THROTTLE          1
#define INDEX_YAW               0
#define INDEX_PITCH             2
#define INDEX_ROLL              3

#define INDEX_AUX1              0
#define INDEX_AUX2              1
#define INDEX_AUX3              2
#define INDEX_AUX4              3
#define INDEX_AUX5              4
#define INDEX_AUX6              5

#define CONTROL_FRAME_SIZE      20
#define TELEM_FRAME_SIZE        10
#define SEND_PACKET_DELAY_MS    3
#define RUMBLE_TIME             500

#define STICK_DEADBAND          7000
#define TRIGGER_DEADBAND        3

#define CONTROL_PIPE            0x8F0F0E1
#define TELEM_PIPE              0x8F0F0E2
#define NRF_CHANNEL             115

#define CRITICAL_BATTERY_LEVEL  99
#define RUMBLE_STRENGTH         70

#define CONVERT_TRIGGER(a)      ((3.9215 * a) + 1000)
#define CONVERT_STICK(a)        ((0.0152 * a) + 1500)

RF24 radio(8, 9);
USB Usb;
XBOXRECV Xbox(&Usb);
Adafruit_SSD1306 display(4);

typedef struct nrf24Payload {
  int16_t control_values[4] = {1000, 1500, 1500, 1500};
  int16_t aux_values[6] = {1000, 1000, 1000, 1000, 1000, 1500};
} nrf24Payload;

typedef struct nrf24Telem {
  uint16_t vbat;
  int32_t latitude;
  int32_t longitude;
} nrf24Telem;

nrf24Payload data;
nrf24Telem telem;

uint16_t vbat = 200;

bool controllerConnected = true;

int rawThrottle = 0;
int rawPitch = 0;
int rawRoll = 0;
int rawYaw = 0;

int actualThrottle = 0;
int actualPitch = 0;
int actualRoll = 0;
int actualYaw = 0;

bool throttleInZero = true;
bool pitchInCenter = true;
bool rollInCenter = true;
bool yawInCenter = true;

bool headlock = false;
bool althold = false;
int altholdThrottle = 0;
bool poshold = false;
bool rth = false;
bool surface = false;

bool adjustment = false;
uint8_t adjustmentNumber = 0;
int16_t adjustmentValues[9] = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900};

bool criticalBatteryLevel = false;
bool rumbling = false;
unsigned long rumblerTime = 0;

unsigned long pollingTime = 0;
unsigned long adjTime = 0;

String message;

void setup() {
  Serial.begin(9600);
  if (Usb.Init() == -1) {
    while (1); //halt
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.display();
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();

  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.setPayloadSize(CONTROL_FRAME_SIZE);
  radio.openWritingPipe(CONTROL_PIPE);
  radio.openReadingPipe(1, TELEM_PIPE);
  radio.startListening();

  message = "Ready";
  writeToDisplay();

  delay(1000);
}

void writeToDisplay() {
  Serial.println(message);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(message);
  display.display();
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected && Xbox.Xbox360Connected[0]) {
    if (!controllerConnected) {
      message = "Controller connected";
      writeToDisplay();
      controllerConnected = true;
    }

    rawThrottle = Xbox.getButtonPress(R2, 0);
    rawPitch = Xbox.getAnalogHat(LeftHatY, 0);
    rawRoll = Xbox.getAnalogHat(LeftHatX, 0);
    rawYaw = Xbox.getAnalogHat(RightHatX, 0);

    //Throttle
    if (rawThrottle > TRIGGER_DEADBAND && !althold) {
      if (actualThrottle != rawThrottle) {
        data.control_values[INDEX_THROTTLE] = CONVERT_TRIGGER(rawThrottle);
        throttleInZero = false;
        actualThrottle = rawThrottle;
      }
    } else if (!throttleInZero && !althold) {
      throttleInZero = true;
      data.control_values[INDEX_THROTTLE] = 1000;
    }

    //Roll
    if (rawRoll > STICK_DEADBAND || rawRoll < -STICK_DEADBAND) {
      if (actualRoll != rawRoll) {
        data.control_values[INDEX_ROLL] = CONVERT_STICK(rawRoll);
        rollInCenter = false;
        actualRoll = rawRoll;
      }
    } else if (!rollInCenter) {
      rollInCenter = true;
      data.control_values[INDEX_ROLL] = 1500;
    }

    //Pitch
    if (rawPitch > STICK_DEADBAND || rawPitch < -STICK_DEADBAND) {
      if (actualPitch != rawPitch) {
        data.control_values[INDEX_PITCH] = CONVERT_STICK(rawPitch);
        pitchInCenter = false;
        actualPitch = rawPitch;
      }
    } else if (!pitchInCenter) {
      pitchInCenter = true;
      data.control_values[INDEX_PITCH] = 1500;
    }

    //Yaw
    if (rawYaw > STICK_DEADBAND || rawYaw < -STICK_DEADBAND) {
      if (actualRoll != rawYaw) {
        data.control_values[INDEX_YAW] = CONVERT_STICK(rawYaw);
        yawInCenter = false;
        actualRoll = rawYaw;
      }
    } else if (!yawInCenter) {
      yawInCenter = true;
      data.control_values[INDEX_YAW] = 1500;
    }

    //Arming
    if (Xbox.getButtonClick(START, 0)) {
      data.aux_values[INDEX_AUX1] = 1600;
      message = "ARMED";
      writeToDisplay();
    }
    if (Xbox.getButtonClick(BACK, 0)) {
      data.aux_values[INDEX_AUX1] = 1000;
      message = "DISARMED";
      writeToDisplay();
    }

    //Primary flight modes: Rate/Acro, Angle, Horizon
    //Secondary flight modes: (ARM), Baro, Mag, HeadFree, HeadAdj, Sonar
    //Need GPS and Compass: GPS Home, GPS Hold
    //Other modes: Beeper, LEDLOW, OSD SW, G-Tune

    //Angle - Horizon - Rate
    if (Xbox.getButtonClick(A, 0)) {
      data.aux_values[INDEX_AUX2] = 1100;
      message = "Angle";
      writeToDisplay();
    }
    if (Xbox.getButtonClick(B, 0)) {
      data.aux_values[INDEX_AUX2] = 1200;
      message = "Horizon";
      writeToDisplay();
    }
    if (Xbox.getButtonClick(X, 0)) {
      data.aux_values[INDEX_AUX2] = 1000;
      message = "Rate";
      writeToDisplay();
    }

    //Heading Lock ON/OFF
    if (Xbox.getButtonClick(R1, 0)) {
      if (headlock) {
        headlock = false;
        data.aux_values[INDEX_AUX3] = 1000;
        message = "HeadLock OFF";
        writeToDisplay();
      } else {
        headlock = true;
        data.aux_values[INDEX_AUX3] = 1100;
        message = "HeadLock OFF";
        writeToDisplay();
      }
    }

    //Altitude Hold ON/OFF
    if (Xbox.getButtonClick(L1, 0)) {
      if (althold) {
        althold = false;
        data.aux_values[INDEX_AUX4] = 1000;
        message = "AltHold OFF";
        writeToDisplay();
      } else {
        althold = true;
        altholdThrottle = rawThrottle;
        data.aux_values[INDEX_AUX4] = 1100;
        message = "AltHold ON";
        writeToDisplay();
      }
    }


    //Adjustment ON/OFF
    if (Xbox.getButtonClick(Y, 0)) {
      if (adjustment) {
        adjustment = false;
        data.aux_values[INDEX_AUX5] = 1000;
        message = "Adj OFF";
        writeToDisplay();
      } else {
        adjustment = true;
        adjustmentNumber = 0;
        data.aux_values[INDEX_AUX5] = adjustmentValues[adjustmentNumber];
        message = "Adj ON";
        writeToDisplay();
      }
    }

    //Adjustment mode UP/DOWN
    if (Xbox.getButtonClick(UP, 0)) {
      if (adjustmentNumber < 8 && adjustmentNumber >= 0 && adjustment) {
        adjustmentNumber++;
        data.aux_values[INDEX_AUX5] = adjustmentValues[adjustmentNumber];
        message = "Adj.: " + String(adjustmentNumber);
        writeToDisplay();
      }
    }
    if (Xbox.getButtonClick(DOWN, 0)) {
      if (adjustmentNumber <= 8 && adjustmentNumber > 0 && adjustment) {
        adjustmentNumber--;
        data.aux_values[INDEX_AUX5] = adjustmentValues[adjustmentNumber];
        message = "Adj.: " + String(adjustmentNumber);
        writeToDisplay();
      } else if (!surface && !adjustment) {
        surface = true;
        data.aux_values[INDEX_AUX6] = 1300;
        message = "Surface ON";
        writeToDisplay();
      } else if (surface && !adjustment) {
        surface = false;
        data.aux_values[INDEX_AUX6] = 1500;
        message = "Surface OFF";
        writeToDisplay();
      }
    }

    //Adjustment increment/decrement / GPS Position Hold / GPS Return To Home
    if (Xbox.getButtonClick(LEFT, 0)) {
      if (adjustment) {
        data.aux_values[INDEX_AUX6] = 1100;
        adjTime = millis();
      } else if (!poshold) {
        adjustment = false;
        data.aux_values[INDEX_AUX5] = 1000;
        adjustmentNumber = 0;
        poshold = true;
        data.aux_values[INDEX_AUX6] = 1100;
        message = "PosHold ON";
        writeToDisplay();
      } else if (poshold) {
        poshold = false;
        data.aux_values[INDEX_AUX6] = 1500;
        message = "PosHold OFF";
        writeToDisplay();
      }
    }
    if (Xbox.getButtonClick(RIGHT, 0)) {
      if (adjustment) {
        data.aux_values[INDEX_AUX6] = 1900;
        adjTime = millis();
      } else if (!rth) {
        adjustment = false;
        data.aux_values[INDEX_AUX5] = 1000;
        adjustmentNumber = 0;
        rth = true;
        data.aux_values[INDEX_AUX6] = 1200;
        message = "RTH ON";
        writeToDisplay();
      } else if (rth) {
        rth = false;
        data.aux_values[INDEX_AUX6] = 1500;
        message = "RTH OFF";
        writeToDisplay();
      }
    }

    //Adjustment back to center
    if (adjustment && millis() - adjTime > 350 && data.aux_values[INDEX_AUX6] != 1500) {
      data.aux_values[INDEX_AUX6] = 1500;
      adjTime = millis();
    }

    //Send packet
    if (millis() - pollingTime > SEND_PACKET_DELAY_MS) {
      radio.stopListening();
      radio.startWrite(&data, CONTROL_FRAME_SIZE, 1);
      delay(2);
      radio.startListening();
      pollingTime = millis();
    }

    if (criticalBatteryLevel && millis() - rumblerTime > RUMBLE_TIME) {
      if (rumbling) {
        Xbox.setRumbleOff();
      } else {
        Xbox.setRumbleOn(RUMBLE_STRENGTH, RUMBLE_STRENGTH, 0);
      }

      rumbling = !rumbling;
      rumblerTime = millis();
    }
  } else {
    //reset control values if controller is disconnected
    data.control_values[INDEX_THROTTLE] = 1000;
    data.control_values[INDEX_PITCH] = 1500;
    data.control_values[INDEX_ROLL] = 1500;
    data.control_values[INDEX_YAW] = 1500;

    data.aux_values[INDEX_AUX1] = 1000; //disarm

    if (controllerConnected) {
      message = "Controller not found";
      writeToDisplay();
      controllerConnected = false;
    }
  }

  if (radio.available()) {
    radio.read(&telem, TELEM_FRAME_SIZE);

    message = "Battery: " + String(telem.vbat / 10.0) + " V";
    writeToDisplay();

    if (telem.vbat <= CRITICAL_BATTERY_LEVEL) {
      criticalBatteryLevel = true;
    } else {
      criticalBatteryLevel = false;
    }
  }
}
