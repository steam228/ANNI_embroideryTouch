import processing.serial.*;

/* Frame sent by src/v2/main.cpp (firmware v2):
   0xF0, START, COUNT, OVERSAMPLE, COUNT x (hi7, lo7), checksum
   value = (hi7 << 7 | lo7) / OVERSAMPLE  ->  0..1023 ADC average
   step d = START + i  ->  frequency = 16 MHz / (d + 1)
   Only the 0xF0 byte has its high bit set, so a lost byte resyncs at the next frame. */

final int BAUD = 500000; // must match Serial.begin() in src/v2/main.cpp
// Leave empty to auto-detect the Arduino, or set it, e.g. "/dev/cu.usbmodem1401" (Mac) or "COM4" (Windows)
final String PORT = "";

Serial port;
String portName = "searching for port...";
int packets = 0, errors = 0, sweeps = 0;
int fwStart, fwCount, fwOversample; // sweep settings reported by the firmware

int[] frame = new int[3 + 2 * 127 + 1];
int count = -1; // -1 = waiting for 0xF0
int expected = 0;

float[] latestX, latestY; // last complete sweep: frequency (kHz), amplitude (0..1023)
ArrayList<float[]> newSweeps = new ArrayList<float[]>(); // every sweep since last frame, oldest first

void connect() {
  String[] ports = Serial.list();
  printArray(ports);
  String pick = PORT.length() > 0 ? PORT : null;
  for (int i = 0; i < ports.length && pick == null; i++) {
    String p = ports[i];
    if (p.contains("usbmodem") || p.contains("usbserial") || p.contains("ttyACM") || p.contains("ttyUSB")) {
      pick = p;
      break;
    }
  }
  if (pick == null) {
    portName = "Arduino not found: plug it in and press r";
    return;
  }
  try {
    if (port != null) port.stop();
    port = new Serial(this, pick, BAUD);
    port.clear();
    portName = pick;
  } catch (Exception e) {
    portName = "could not open " + pick;
  }
}

// Called from draw(): no serial thread, no races with drawing
void readSerial() {
  if (port == null) return;
  while (port.available() > 0) {
    int b = port.read();
    if (b == 0xF0) {
      count = 0;
      continue;
    }
    if (count < 0) continue;
    if (b >= 0x80) { // corrupted, wait for the next frame
      count = -1;
      continue;
    }
    frame[count++] = b;
    if (count == 3) {
      expected = 3 + 2 * frame[1] + 1;
      if (frame[1] == 0) count = -1;
    } else if (count > 3 && count == expected) {
      handleFrame();
      count = -1;
    }
  }
}

void handleFrame() {
  packets++;
  int n = frame[1];
  int sum = 0;
  for (int i = 3; i < 3 + 2 * n; i++) sum += frame[i];
  if ((sum & 0x7F) != frame[3 + 2 * n]) {
    errors++;
    return;
  }
  fwStart = frame[0];
  fwCount = n;
  fwOversample = max(1, frame[2]);
  float[] xs = new float[n], ys = new float[n];
  for (int i = 0; i < n; i++) {
    xs[i] = 16000.0 / (fwStart + i + 1);
    ys[i] = (frame[3 + 2 * i] << 7 | frame[4 + 2 * i]) / (float) fwOversample;
  }
  latestX = xs;
  latestY = ys;
  newSweeps.add(ys);
  if (newSweeps.size() > 200) newSweeps.remove(0); // sketches that never read it
  sweeps++;
}

void exit() {
  if (port != null) port.stop();
  super.exit();
}
