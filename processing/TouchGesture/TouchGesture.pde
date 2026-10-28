// Dynamic gesture recorder / recogniser for the Touché sweep sensor
// Arduino side: src/main.cpp + src/SendData.cpp (500000 baud)
//
// Hold REC on a card (or hold keys 1-6) while performing a gesture to record one example.
// Record a few examples per gesture. Right-click a card removes its last example.
// Click a gesture name to rename it. Drag the slider to set the match threshold.
// b = recalibrate baseline (hands off the sensor), r = reconnect.
// Examples, names and threshold are saved to data/gestures.json.

// Palette, highlight -> bg
final color C1 = #32213A;
final color C2 = #383B53;
final color C3 = #66717E;
final color C4 = #D4D6B9;
final color C5 = #D1CAA1;

final int SLOTS = 6;
final int HISTORY = 240;    // frames kept for the time view
final int CALIB_FRAMES = 20;
final int WARMUP_FRAMES = 30; // Arduino filter starts at 0 after reset: let it settle first

class Frame {
  float[] f;
  int rec = -1;  // slot being recorded on this frame
  int det = -1;  // slot detected on this frame
  Frame(float[] f) {
    this.f = f;
  }
}

Graph graph;
float[] shown;
ArrayList<Frame> history = new ArrayList<Frame>();

float[] baseline;
float noise = 2;
int calibrating = 0;
int warmup = 0;
ArrayList<float[]> calibFrames = new ArrayList<float[]>();

String[] names = new String[SLOTS];
ArrayList<Template> templates = new ArrayList<Template>();
float[] sim = new float[SLOTS];
float[] shownSim = new float[SLOTS];
float[] peakSim = new float[SLOTS];
float threshold = 0.6;
int quiet = 0; // frames to wait after a detection

int recording = -1;
char recKey = 0;
ArrayList<float[]> recFrames = new ArrayList<float[]>();
int recStartMs;

int detected = -1, detectedAt = -100000;
float detectedSim;
int editing = -1;
boolean dragging = false;

String toastMsg = "";
int toastAt = -100000;
int rateCount = 0, rateAt = 0;
float sweepRate = 0;
float heatScale = 20;
PImage heat;

// Layout
final float LX = 50, LW = 820;                 // left column
final float RX = 890, RW = 340;                // right column
final float HM_X = 110, HM_Y = 440, HM_W = 730, HM_H = 210;
final float CARD_Y = 60, CARD_H = 84, CARD_GAP = 10;
final float SL_Y = 630, SL_H = 145;

void setup() {
  size(1280, 820);
  pixelDensity(displayDensity());
  smooth(8);
  textFont(createFont("SansSerif", 14));
  graph = new Graph(110, 90, 730, 250);
  heat = createImage(HISTORY, BINS, RGB);
  for (int i = 0; i < SLOTS; i++) names[i] = "Gesture " + char('A' + i);
  loadGestures();
  connect();
  startCalibration();
}

void draw() {
  readSerial();
  for (float[] s : newSweeps) onSweep(s);
  newSweeps.clear();

  if (millis() - rateAt > 1000) {
    sweepRate = rateCount * 1000.0 / (millis() - rateAt);
    rateCount = 0;
    rateAt = millis();
  }
  for (int i = 0; i < SLOTS; i++) shownSim[i] = lerp(shownSim[i], sim[i], 0.3);

  background(C5);
  drawHeader();
  drawSweep();
  drawHeat();
  drawBanner();
  drawCards();
  drawSlider();
  drawHint();
}

// ---------------------------------------------------------------- data

void onSweep(float[] ys) {
  rateCount++;
  float[] d = downsample(ys);

  if (warmup > 0) {
    warmup--;
    return;
  }
  if (calibrating > 0) {
    calibFrames.add(d);
    if (--calibrating == 0) finishCalibration();
    return;
  }
  if (baseline == null) return;

  float[] f = new float[BINS];
  for (int k = 0; k < BINS; k++) f[k] = d[k] - baseline[k];
  Frame fr = new Frame(f);
  history.add(fr);
  if (history.size() > HISTORY) history.remove(0);

  if (recording >= 0) {
    recFrames.add(f);
    fr.rec = recording;
  } else {
    recognise(fr);
  }
}

// Also gives you a moment to take your hands off when recalibrating
void startCalibration() {
  calibFrames.clear();
  warmup = WARMUP_FRAMES;
  calibrating = CALIB_FRAMES;
}

void finishCalibration() {
  baseline = new float[BINS];
  for (float[] d : calibFrames) for (int k = 0; k < BINS; k++) baseline[k] += d[k] / calibFrames.size();
  float s = 0;
  for (float[] d : calibFrames) for (int k = 0; k < BINS; k++) s += sq(d[k] - baseline[k]);
  noise = max(1, sqrt(s / (calibFrames.size() * BINS)));
  history.clear();
  toast("baseline set · noise ±" + nf(noise, 0, 1));
}

void recognise(Frame fr) {
  if (templates.isEmpty()) {
    sim = new float[SLOTS];
    return;
  }
  int maxLen = 0;
  for (Template t : templates) maxLen = max(maxLen, t.frames.length);
  int m = min(history.size(), round(maxLen * 1.6) + 2);
  if (m < 3) return;
  float[][] stream = new float[m][];
  for (int i = 0; i < m; i++) stream[i] = history.get(history.size() - m + i).f;

  sim = similarities(templates, stream, 3 * noise, SLOTS);

  if (quiet > 0) {
    quiet--;
    peakSim = new float[SLOTS];
    return;
  }

  // Fire on the peak: wait until a slot above threshold starts to drop
  int fire = -1;
  float fireSim = 0;
  for (int c = 0; c < SLOTS; c++) {
    if (sim[c] >= threshold && sim[c] >= peakSim[c]) {
      peakSim[c] = sim[c];
    } else if (peakSim[c] > 0) {
      if (peakSim[c] > fireSim) {
        fire = c;
        fireSim = peakSim[c];
      }
      peakSim[c] = 0;
    }
  }
  if (fire >= 0) {
    detected = fire;
    detectedSim = fireSim;
    detectedAt = millis();
    fr.det = fire;
    peakSim = new float[SLOTS];
    int len = 0;
    for (Template t : templates) if (t.cls == fire) len = max(len, t.frames.length);
    quiet = max(3, round(len * 0.6));
    println("detected " + names[fire] + " " + round(fireSim * 100) + "%");
  }
}

void startRecording(int c) {
  if (baseline == null) return;
  recording = c;
  recFrames.clear();
  recStartMs = millis();
  editing = -1;
}

void finishRecording() {
  int c = recording;
  recording = -1;
  recKey = 0;
  float[][] frames = trim(recFrames, noise);
  if (frames == null) {
    toast("nothing recorded: no clear signal, try a longer or stronger gesture");
    return;
  }
  templates.add(new Template(c, frames));
  saveGestures();
  toast("added example " + countExamples(c) + " to " + names[c] + " (" + frames.length + " frames)");
}

void removeLast(int c) {
  for (int i = templates.size() - 1; i >= 0; i--) {
    if (templates.get(i).cls == c) {
      templates.remove(i);
      sim[c] = 0;
      saveGestures();
      toast("removed an example from " + names[c]);
      return;
    }
  }
}

int countExamples(int c) {
  int n = 0;
  for (Template t : templates) if (t.cls == c) n++;
  return n;
}

void toast(String s) {
  toastMsg = s;
  toastAt = millis();
}

// ---------------------------------------------------------------- drawing

void drawHeader() {
  fill(C1);
  textSize(26);
  textAlign(LEFT, BASELINE);
  text("Touché gestures", LX, 42);

  float errPct = packets == 0 ? 0 : 100.0 * errors / packets;
  fill(C3);
  textSize(12);
  textAlign(RIGHT, BASELINE);
  text(portName + "  ·  " + round(sweepRate) + " sweeps/s  ·  " + nf(errPct, 0, 1) + "% errors",
    LX + LW, 42);
}

void drawSweep() {
  float[] xs = latestX, ys = latestY;
  boolean hasData = xs != null && ys != null && xs.length == ys.length;
  if (hasData) {
    if (shown == null || shown.length != ys.length) shown = ys.clone();
    for (int i = 0; i < ys.length; i++) shown[i] = lerp(shown[i], ys[i], 0.5);
    graph.xMin = xs[0];
    graph.xMax = xs[xs.length - 1];
    graph.yMax = lerp(graph.yMax, max(max(shown) * 1.15, 50), 0.1);
  }
  graph.drawFrame();
  if (!hasData) {
    graph.drawWaiting("waiting for sweep data…");
    return;
  }
  if (baseline != null) {
    float[] b = new float[xs.length];
    for (int i = 0; i < b.length; i++) b[i] = baseline[min(BINS - 1, i * BINS / b.length)];
    graph.drawGhost(xs, b, "baseline", false);
  }
  graph.drawCurve(xs, shown);
  graph.drawCursor(xs, shown);
}

// Time view: x = time (newest right), y = frequency, colour = change from baseline
void drawHeat() {
  noStroke();
  fill(C4);
  rect(LX, HM_Y - 30, LW, HM_H + 80, 16);

  float maxAbs = 0;
  for (Frame fr : history) for (float v : fr.f) maxAbs = max(maxAbs, abs(v));
  heatScale = lerp(heatScale, max(maxAbs, 10 * noise), 0.05);

  heat.loadPixels();
  int offset = HISTORY - history.size();
  for (int x = 0; x < HISTORY; x++) {
    Frame fr = x >= offset ? history.get(x - offset) : null;
    for (int k = 0; k < BINS; k++) {
      color c = C4;
      if (fr != null) {
        float v = constrain(fr.f[k] / heatScale, -1, 1);
        c = v >= 0 ? lerpColor(C4, C1, v) : lerpColor(C4, C3, -v);
      }
      heat.pixels[(BINS - 1 - k) * HISTORY + x] = c;
    }
  }
  heat.updatePixels();
  image(heat, HM_X, HM_Y, HM_W, HM_H);

  // recording spans and detections above the strip
  float colW = HM_W / HISTORY;
  for (int i = 0; i < history.size(); i++) {
    Frame fr = history.get(i);
    float x = HM_X + (offset + i) * colW;
    if (fr.rec >= 0) {
      fill(C1);
      rect(x, HM_Y - 12, colW + 0.5, 6);
    }
    if (fr.det >= 0) {
      stroke(C1);
      strokeWeight(1.5);
      line(x, HM_Y - 4, x, HM_Y + HM_H);
      noStroke();
      fill(C1);
      textSize(11);
      textAlign(RIGHT, BOTTOM);
      text(names[fr.det], x - 3, HM_Y - 2);
    }
  }
  noStroke();

  fill(C2);
  textSize(12);
  textAlign(CENTER, TOP);
  text("time  →  (last " + HISTORY + " sweeps)", HM_X + HM_W / 2, HM_Y + HM_H + 12);
  pushMatrix();
  translate(HM_X - 42, HM_Y + HM_H / 2);
  rotate(-HALF_PI);
  textAlign(CENTER, BOTTOM);
  text("frequency", 0, 0);
  popMatrix();

  // legend
  textSize(11);
  textAlign(RIGHT, CENTER);
  float lx = HM_X + HM_W, ly = HM_Y + HM_H + 20;
  fill(C3);
  text("below baseline", lx - 120, ly);
  for (int i = 0; i < 40; i++) {
    float v = map(i, 0, 39, -1, 1);
    fill(v >= 0 ? lerpColor(C4, C1, v) : lerpColor(C4, C3, -v));
    rect(lx - 112 + i * 2, ly - 5, 2.5, 10);
  }
  fill(C3);
  textAlign(LEFT, CENTER);
  text("above", lx - 26, ly);
}

void drawBanner() {
  float by = 705, bh = 70;
  int since = millis() - detectedAt;
  boolean showDet = detected >= 0 && since < 2500;

  noStroke();
  if (recording >= 0) {
    fill(C1);
    rect(LX, by, LW, bh, 16);
    fill(C4);
    textSize(22);
    textAlign(LEFT, CENTER);
    text("●  recording " + names[recording] + "  ·  " + nf((millis() - recStartMs) / 1000.0, 0, 1) + " s  ·  "
      + recFrames.size() + " frames", LX + 24, by + bh / 2);
  } else if (calibrating > 0 || baseline == null) {
    fill(C4);
    rect(LX, by, LW, bh, 16);
    fill(C2);
    textSize(18);
    textAlign(LEFT, CENTER);
    text("calibrating baseline — hands off the sensor…", LX + 24, by + bh / 2);
  } else if (showDet) {
    float a = since < 1800 ? 255 : map(since, 1800, 2500, 255, 0);
    fill(C1, a);
    rect(LX, by, LW, bh, 16);
    fill(C4, a);
    textSize(30);
    textAlign(LEFT, CENTER);
    text(names[detected], LX + 24, by + bh / 2 - 2);
    textSize(16);
    textAlign(RIGHT, CENTER);
    text(round(detectedSim * 100) + "% match", LX + LW - 24, by + bh / 2);
  } else {
    fill(C4);
    rect(LX, by, LW, bh, 16);
    fill(C3);
    textSize(16);
    textAlign(LEFT, CENTER);
    text(templates.isEmpty() ? "hold REC on a card while doing a gesture to record an example"
      : "perform a gesture…", LX + 24, by + bh / 2);
  }
}

float cardY(int i) {
  return CARD_Y + i * (CARD_H + CARD_GAP);
}

boolean overRec(int i, float mx, float my) {
  float cy = cardY(i);
  return mx > RX + RW - 84 && mx < RX + RW - 14 && my > cy + 12 && my < cy + 42;
}

boolean overName(int i, float mx, float my) {
  float cy = cardY(i);
  return mx > RX + 52 && mx < RX + RW - 140 && my > cy + 8 && my < cy + 44;
}

void drawCards() {
  boolean recent = millis() - detectedAt < 2500;
  for (int i = 0; i < SLOTS; i++) {
    float cy = cardY(i);
    boolean rec = recording == i;
    boolean det = recent && detected == i;
    boolean dark = rec || det;
    color fg = dark ? C4 : C1;
    color dim = dark ? C4 : C3;

    noStroke();
    fill(rec ? C1 : det ? C2 : C4);
    rect(RX, cy, RW, CARD_H, 16);

    fill(dark ? C3 : C5);
    ellipse(RX + 28, cy + 27, 28, 28);
    fill(fg);
    textSize(13);
    textAlign(CENTER, CENTER);
    text(i + 1, RX + 28, cy + 26);

    textSize(18);
    textAlign(LEFT, CENTER);
    String nm = names[i] + (editing == i && (frameCount / 30) % 2 == 0 ? "|" : "");
    text(nm, RX + 52, cy + 25);
    if (editing == i) {
      stroke(fg);
      strokeWeight(1);
      line(RX + 52, cy + 38, RX + RW - 140, cy + 38);
      noStroke();
    }

    int n = countExamples(i);
    fill(dim);
    textSize(11);
    textAlign(RIGHT, CENTER);
    text(n == 0 ? "no examples" : n + (n == 1 ? " example" : " examples"), RX + RW - 94, cy + 27);

    // REC pill
    boolean hot = rec || overRec(i, mouseX, mouseY);
    fill(rec ? C4 : hot ? C2 : C1);
    rect(RX + RW - 84, cy + 12, 70, 30, 15);
    fill(rec ? C1 : C4);
    textSize(12);
    textAlign(CENTER, CENTER);
    text(rec ? "● " + nf((millis() - recStartMs) / 1000.0, 0, 1) + "s" : "REC", RX + RW - 49, cy + 26);

    // similarity bar + threshold tick
    float bx = RX + 18, by = cy + 58, bw = RW - 80;
    fill(dark ? C3 : C5);
    rect(bx, by, bw, 10, 5);
    if (n > 0) {
      fill(dark ? C4 : (shownSim[i] >= threshold ? C1 : C2));
      rect(bx, by, max(10, bw * shownSim[i]), 10, 5);
    }
    stroke(dark ? C4 : C1);
    strokeWeight(2);
    float tx = bx + bw * threshold;
    line(tx, by - 4, tx, by + 14);
    noStroke();
    fill(dim);
    textSize(11);
    textAlign(RIGHT, CENTER);
    text(n > 0 ? round(shownSim[i] * 100) + "%" : "–", RX + RW - 18, by + 5);
  }
}

float sliderX0() {
  return RX + 20;
}

float sliderX1() {
  return RX + RW - 20;
}

float sliderY() {
  return SL_Y + 74;
}

boolean overBaselineButton(float mx, float my) {
  return mx > RX + 20 && mx < RX + RW - 20 && my > SL_Y + 100 && my < SL_Y + 130;
}

void drawSlider() {
  noStroke();
  fill(C4);
  rect(RX, SL_Y, RW, SL_H, 16);

  fill(C2);
  textSize(12);
  textAlign(LEFT, BASELINE);
  text("match threshold", RX + 20, SL_Y + 30);
  fill(C3);
  textSize(11);
  text("lower = more detections, higher = stricter", RX + 20, SL_Y + 46);
  fill(C1);
  textSize(24);
  textAlign(RIGHT, BASELINE);
  text(round(threshold * 100) + "%", RX + RW - 20, SL_Y + 38);

  float x0 = sliderX0(), x1 = sliderX1(), y = sliderY();
  float kx = lerp(x0, x1, threshold);
  strokeCap(ROUND);
  strokeWeight(8);
  stroke(C5);
  line(x0, y, x1, y);
  stroke(C2);
  line(x0, y, kx, y);
  noStroke();
  fill(C1);
  ellipse(kx, y, dragging ? 24 : 20, dragging ? 24 : 20);

  boolean hot = overBaselineButton(mouseX, mouseY);
  fill(hot ? C2 : C5);
  rect(RX + 20, SL_Y + 100, RW - 40, 30, 15);
  fill(hot ? C4 : C1);
  textSize(12);
  textAlign(CENTER, CENTER);
  text("recalibrate baseline (b)  ·  noise ±" + nf(noise, 0, 1), RX + RW / 2, SL_Y + 114);
}

void drawHint() {
  textSize(11);
  textAlign(LEFT, BASELINE);
  if (millis() - toastAt < 3500) {
    fill(C1);
    text(toastMsg, LX, height - 18);
  } else {
    fill(C3);
    text("hold REC / keys 1–6 to record  ·  right-click card removes last example  ·  click name to rename  ·  b baseline  ·  r reconnect",
      LX, height - 18);
  }
}

// ---------------------------------------------------------------- input

void setThreshold(float mx) {
  threshold = constrain(map(mx, sliderX0(), sliderX1(), 0, 1), 0.05, 0.99);
}

void mousePressed() {
  if (abs(mouseY - sliderY()) < 16 && mouseX > sliderX0() - 12 && mouseX < sliderX1() + 12) {
    dragging = true;
    setThreshold(mouseX);
    return;
  }
  if (overBaselineButton(mouseX, mouseY)) {
    startCalibration();
    return;
  }
  for (int i = 0; i < SLOTS; i++) {
    float cy = cardY(i);
    if (mouseX < RX || mouseX > RX + RW || mouseY < cy || mouseY > cy + CARD_H) continue;
    if (mouseButton == RIGHT) removeLast(i);
    else if (overRec(i, mouseX, mouseY)) startRecording(i);
    else if (overName(i, mouseX, mouseY)) editing = i;
    return;
  }
  if (editing >= 0) {
    editing = -1;
    saveGestures();
  }
}

void mouseDragged() {
  if (dragging) setThreshold(mouseX);
}

void mouseReleased() {
  if (dragging) {
    dragging = false;
    saveGestures();
  }
  if (recording >= 0 && recKey == 0) finishRecording();
}

void keyPressed() {
  if (editing >= 0) {
    if (key == ENTER || key == RETURN || key == ESC) {
      if (key == ESC) key = 0; // don't quit the sketch
      editing = -1;
      saveGestures();
    } else if (key == BACKSPACE) {
      if (names[editing].length() > 0) names[editing] = names[editing].substring(0, names[editing].length() - 1);
    } else if (key != CODED && key >= ' ' && names[editing].length() < 18) {
      names[editing] += key;
    }
    return;
  }
  if (key >= '1' && key <= '0' + SLOTS && recording < 0) {
    startRecording(key - '1');
    if (recording >= 0) recKey = key;
  }
  if (key == 'b') startCalibration();
  if (key == 'r') connect();
}

void keyReleased() {
  if (recording >= 0 && key == recKey) finishRecording();
}
