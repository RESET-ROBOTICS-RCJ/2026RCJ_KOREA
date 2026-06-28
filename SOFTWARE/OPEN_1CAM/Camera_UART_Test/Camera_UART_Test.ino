/*
  Simple camera UART receive test for Teensy 4.1.

  Wiring:
    Camera TX -> Teensy UART RX pin
    Camera RX -> Teensy UART TX pin, optional for this receive-only test
    GND shared

  Open Serial Monitor at 115200 baud.
  Expected camera packet:
    A5 ballFlags ballSign ballAngle ballDist yellowValid yellowSign yellowAngle yellowDist blueValid blueSign blueAngle blueDist 55
*/

const uint32_t USB_BAUD = 115200;
const uint32_t CAM_BAUD = 115200;

struct CamPort {
  const char *name;
  HardwareSerial *serial;
  uint8_t rxBuf[14];
  uint8_t rxIdx;
  unsigned long lastByteTime;
  unsigned long lastPacketTime;
  unsigned long byteCount;
  unsigned long packetCount;
  unsigned long badPacketCount;
};

CamPort ports[] = {
  {"Serial1", &Serial1, {0}, 0, 0, 0, 0, 0, 0},
  {"Serial3", &Serial3, {0}, 0, 0, 0, 0, 0, 0},
  {"Serial5", &Serial5, {0}, 0, 0, 0, 0, 0, 0},
  {"Serial7", &Serial7, {0}, 0, 0, 0, 0, 0, 0},
};

const int PORT_COUNT = sizeof(ports) / sizeof(ports[0]);

int signedAngle(uint8_t signByte, uint8_t magnitude)
{
  return (signByte == 1) ? magnitude : -magnitude;
}

void printPacket(CamPort &port)
{
  uint8_t ballFlags = port.rxBuf[1];
  bool ballValid = (ballFlags & 0x01) != 0;
  bool ballIn = (ballFlags & 0x02) != 0;
  int ballAngle = signedAngle(port.rxBuf[2], port.rxBuf[3]);
  int ballDist = port.rxBuf[4];

  bool yellowValid = port.rxBuf[5] != 0;
  int yellowAngle = signedAngle(port.rxBuf[6], port.rxBuf[7]);
  int yellowDist = port.rxBuf[8];

  bool blueValid = port.rxBuf[9] != 0;
  int blueAngle = signedAngle(port.rxBuf[10], port.rxBuf[11]);
  int blueDist = port.rxBuf[12];

  Serial.print(port.name);
  Serial.print(" OK #");
  Serial.print(port.packetCount);
  Serial.print("  Ball: ");
  if (ballValid) {
    Serial.print(ballAngle);
    Serial.print(" deg, dist ");
    Serial.print(ballDist);
    if (ballIn) {
      Serial.print(" IN");
    }
  } else {
    Serial.print("none");
  }

  Serial.print("  Yellow: ");
  if (yellowValid) {
    Serial.print(yellowAngle);
    Serial.print(" deg, dist ");
    Serial.print(yellowDist);
  } else {
    Serial.print("none");
  }

  Serial.print("  Blue: ");
  if (blueValid) {
    Serial.print(blueAngle);
    Serial.print(" deg, dist ");
    Serial.print(blueDist);
  } else {
    Serial.print("none");
  }

  Serial.print("  raw:");
  for (int i = 0; i < 14; i++) {
    Serial.print(' ');
    if (port.rxBuf[i] < 0x10) Serial.print('0');
    Serial.print(port.rxBuf[i], HEX);
  }
  Serial.println();
}

void readCameraPort(CamPort &port)
{
  while (port.serial->available()) {
    uint8_t b = port.serial->read();
    port.byteCount++;
    port.lastByteTime = millis();

    if (port.rxIdx == 0) {
      if (b == 0xA5) {
        port.rxBuf[port.rxIdx++] = b;
      }
      continue;
    }

    port.rxBuf[port.rxIdx++] = b;

    if (port.rxIdx == 14) {
      port.rxIdx = 0;

      if (port.rxBuf[13] != 0x55) {
        port.badPacketCount++;
        Serial.print(port.name);
        Serial.print(" bad packet end byte. Count: ");
        Serial.println(port.badPacketCount);
        continue;
      }

      port.packetCount++;
      port.lastPacketTime = millis();
      printPacket(port);
    }
  }
}

void setup()
{
  Serial.begin(USB_BAUD);
  for (int i = 0; i < PORT_COUNT; i++) {
    ports[i].serial->begin(CAM_BAUD);
  }

  while (!Serial && millis() < 3000) {
    delay(10);
  }

  Serial.println("Camera UART test started");
  Serial.println("Scanning Serial1, Serial3, Serial5, Serial7 at 115200 baud...");
}

void loop()
{
  for (int i = 0; i < PORT_COUNT; i++) {
    readCameraPort(ports[i]);
  }

  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime >= 1000) {
    lastStatusTime = millis();

    Serial.println("Status:");
    for (int i = 0; i < PORT_COUNT; i++) {
      Serial.print("  ");
      Serial.print(ports[i].name);
      Serial.print(" bytes=");
      Serial.print(ports[i].byteCount);
      Serial.print(" packets=");
      Serial.print(ports[i].packetCount);
      Serial.print(" bad=");
      Serial.print(ports[i].badPacketCount);
      Serial.print(" lastByte=");
      if (ports[i].lastByteTime == 0) {
        Serial.print("never");
      } else {
        Serial.print(millis() - ports[i].lastByteTime);
        Serial.print("ms ago");
      }
      Serial.print(" lastPacket=");
      if (ports[i].lastPacketTime == 0) {
        Serial.println("never");
      } else {
        Serial.print(millis() - ports[i].lastPacketTime);
        Serial.println("ms ago");
      }
    }
  }
}
