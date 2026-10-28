import processing.serial.*;

// Palette, highlight -> bg
color C1 = #32213A;
color C2 = #383B53;
color C3 = #66717E;
color C4 = #D4D6B9;
color C5 = #D1CAA1;

final int MAX_VAL = 17;   // readCapacitivePin returns 0..17
final int THRESHOLD = 4;  // same as the Arduino sketch
final int HIST = 300;

Serial port;
float[] hist = new float[HIST];
float raw = 0;
float smoothed = 0;
float pulse = 0;
boolean touched = false;
String status = "searching for port...";

void setup() {
  size(900, 600);
  smooth(8);
  connect();
}

void connect() {
  String[] ports = Serial.list();
  printArray(ports);
  String pick = null;
  for (String p : ports) {
    if (p.contains("usbmodem") || p.contains("usbserial") || p.contains("ttyACM") || p.contains("ttyUSB")) {
      pick = p;
      break;
    }
  }
  if (pick == null && ports.length > 0) pick = ports[ports.length - 1];
  if (pick == null) {
    status = "no serial port found";
    return;
  }
  try {
    if (port != null) port.stop();
    port = new Serial(this, pick, 9600);
    status = pick;
  } catch (Exception e) {
    status = "could not open " + pick;
  }
}

// Drain everything the Arduino sent since last frame, keep only the newest value
void readSerial() {
  if (port == null) return;
  String s = null;
  String line;
  while (port.available() > 0 && (line = port.readStringUntil('\n')) != null) {
    s = line;
  }
  if (s == null) return;
  s = trim(s);
  if (s.length() == 0) return;
  try {
    raw = constrain(Integer.parseInt(s), 0, MAX_VAL);
  } catch (NumberFormatException e) {
  }
}

void draw() {
  background(C5);
  readSerial();

  smoothed = lerp(smoothed, raw, 0.15);
  boolean now = raw > THRESHOLD;
  if (now && !touched) pulse = 1;
  touched = now;
  pulse *= 0.93;

  System.arraycopy(hist, 1, hist, 0, HIST - 1);
  hist[HIST - 1] = raw;

  drawOrb();
  drawGraph();
  drawHud();
}

void drawOrb() {
  float cx = width / 2.0, cy = height * 0.36;
  float t = smoothed / MAX_VAL;
  float base = 60 + t * 130;

  // concentric rings, softly breathing
  noFill();
  for (int i = 5; i >= 1; i--) {
    float r = base + i * 26 + sin(frameCount * 0.04 + i) * 4 + pulse * i * 18;
    stroke(lerpColor(C5, C3, 0.25 + i * 0.1), 255 - i * 35);
    strokeWeight(1.5);
    ellipse(cx, cy, r * 2, r * 2);
  }

  // wobbly core blob
  noStroke();
  fill(lerpColor(C3, C1, constrain(t * 1.4, 0, 1)));
  beginShape();
  for (int a = 0; a < 360; a += 4) {
    float ang = radians(a);
    float n = noise(cos(ang) + 1, sin(ang) + 1, frameCount * 0.015);
    float r = base + (n - 0.5) * (20 + t * 70) + pulse * 20;
    vertex(cx + cos(ang) * r, cy + sin(ang) * r);
  }
  endShape(CLOSE);

  // inner glow when touched
  fill(C4, touched ? 230 : 70);
  float ir = base * 0.38 + pulse * 25;
  ellipse(cx, cy, ir * 2, ir * 2);
}

void drawGraph() {
  float x0 = 60, x1 = width - 60;
  float y0 = height - 60, y1 = height - 190;

  // panel
  noStroke();
  fill(C4, 160);
  rect(x0 - 20, y1 - 20, x1 - x0 + 40, y0 - y1 + 40, 14);

  // threshold line
  float ty = map(THRESHOLD, 0, MAX_VAL, y0, y1);
  stroke(C3);
  strokeWeight(1);
  for (float x = x0; x < x1; x += 10) line(x, ty, x + 5, ty);

  // filled area
  noStroke();
  fill(C2, 90);
  beginShape();
  vertex(x0, y0);
  for (int i = 0; i < HIST; i++) {
    vertex(map(i, 0, HIST - 1, x0, x1), map(hist[i], 0, MAX_VAL, y0, y1));
  }
  vertex(x1, y0);
  endShape(CLOSE);

  // line
  noFill();
  stroke(C1);
  strokeWeight(2.5);
  beginShape();
  for (int i = 0; i < HIST; i++) {
    vertex(map(i, 0, HIST - 1, x0, x1), map(hist[i], 0, MAX_VAL, y0, y1));
  }
  endShape();
}

void drawHud() {
  fill(C1);
  textAlign(LEFT, TOP);
  textSize(14);
  text(status, 24, 20);

  textAlign(RIGHT, TOP);
  textSize(48);
  text(nf(raw, 1, 0), width - 24, 12);
  textSize(14);
  fill(C3);
  text(touched ? "TOUCH" : "idle", width - 24, 66);
}

void keyPressed() {
  if (key == 'r') connect();
}
