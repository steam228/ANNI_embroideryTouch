// Touché-style swept-frequency touch sensor (Illutron / Dzl 2012)
// Processing 4 port of the original Processing_graph, restyled.
// Arduino side: src/main.cpp + src/SendData.cpp
//
// Click a gesture card (or press 1-4) while doing that gesture to record it.
// Right-click a card to clear it. c = clear all, a = autoscale, r = reconnect.

// Palette, highlight -> bg
final color C1 = #32213A;
final color C2 = #383B53;
final color C3 = #66717E;
final color C4 = #D4D6B9;
final color C5 = #D1CAA1;

Graph graph;
String[] names = {"Nothing", "Touch", "Grab", "In water"};
float[][] gestures = new float[4][]; // recorded sweep per gesture, null = not set
float[] confidence = new float[4];
int best = -1;

float[] shown;          // eased copy of the latest sweep, for drawing
boolean autoscale = false;

final float CARD_X = 800, CARD_Y = 60, CARD_W = 250, CARD_H = 121, CARD_GAP = 12;

void setup() {
  size(1100, 620);
  pixelDensity(displayDensity());
  smooth(8);
  textFont(createFont("SansSerif", 14));
  graph = new Graph(110, 90, 640, 440);
  connect();
}

void draw() {
  readSerial();
  background(C5);

  float[] xs = latestX, ys = latestY;
  boolean hasData = ys != null && xs != null && xs.length == ys.length;

  if (hasData) {
    if (shown == null || shown.length != ys.length) shown = ys.clone();
    for (int i = 0; i < ys.length; i++) shown[i] = lerp(shown[i], ys[i], 0.35);
    graph.xMin = xs[0];
    graph.xMax = xs[xs.length - 1];
    float target = autoscale ? max(max(shown) * 1.15, 50) : 1024;
    graph.yMax = lerp(graph.yMax, target, 0.1);
    classify(ys);
  }

  drawHeader();
  graph.drawFrame();
  if (hasData) {
    for (int i = 0; i < gestures.length; i++) {
      if (gestures[i] != null && gestures[i].length == xs.length) {
        graph.drawGhost(xs, gestures[i], names[i], i == best);
      }
    }
    graph.drawCurve(xs, shown);
    graph.drawCursor(xs, shown);
  } else {
    graph.drawWaiting("waiting for sweep data…");
  }
  drawCards();
  drawHint();
}

// Compare the whole sweep (RMS distance) against each recorded gesture
void classify(float[] s) {
  float[] d = new float[gestures.length];
  float sumInv = 0, bestD = Float.MAX_VALUE;
  best = -1;
  for (int i = 0; i < gestures.length; i++) {
    float[] g = gestures[i];
    if (g == null || g.length != s.length) {
      d[i] = -1;
      continue;
    }
    float acc = 0;
    for (int j = 0; j < s.length; j++) acc += sq(s[j] - g[j]);
    d[i] = sqrt(acc / s.length);
    sumInv += 1 / (d[i] + 1);
    if (d[i] < bestD) {
      bestD = d[i];
      best = i;
    }
  }
  for (int i = 0; i < gestures.length; i++) {
    float target = d[i] < 0 ? 0 : (1 / (d[i] + 1)) / sumInv;
    confidence[i] = lerp(confidence[i], target, 0.2);
  }
}

void drawHeader() {
  fill(C1);
  textSize(26);
  textAlign(LEFT, BASELINE);
  text("Touché sweep", 50, 42);

  float errPct = packets == 0 ? 0 : 100.0 * errors / packets;
  fill(C3);
  textSize(12);
  textAlign(RIGHT, BASELINE);
  text(portName + "  ·  " + BAUD + " baud  ·  " + sweeps + " sweeps  ·  " + nf(errPct, 0, 1) + "% errors",
    CARD_X - 20, 42);
}

void drawCards() {
  for (int i = 0; i < names.length; i++) {
    float cy = CARD_Y + i * (CARD_H + CARD_GAP);
    boolean on = i == best;
    color bg = on ? C1 : C4;
    color fg = on ? C4 : C1;
    color dim = on ? C4 : C3;

    noStroke();
    fill(bg);
    rect(CARD_X, cy, CARD_W, CARD_H, 16);

    // number badge
    fill(on ? C2 : C5);
    ellipse(CARD_X + 30, cy + 32, 30, 30);
    fill(fg);
    textSize(14);
    textAlign(CENTER, CENTER);
    text(i + 1, CARD_X + 30, cy + 31);

    textSize(20);
    textAlign(LEFT, CENTER);
    text(names[i], CARD_X + 56, cy + 31);

    float bx = CARD_X + 18, by = cy + CARD_H - 34, bw = CARD_W - 36;
    if (gestures[i] == null) {
      fill(dim);
      textSize(12);
      textAlign(LEFT, CENTER);
      text("click or press " + (i + 1) + " to record", bx, by + 6);
    } else {
      fill(on ? C2 : C5);
      rect(bx, by, bw, 12, 6);
      fill(on ? C4 : C2);
      rect(bx, by, max(12, bw * confidence[i]), 12, 6);
      fill(dim);
      textSize(11);
      textAlign(RIGHT, BOTTOM);
      text(str(round(confidence[i] * 100)) + "%", bx + bw, by - 4);
    }
  }
}

void drawHint() {
  fill(C3);
  textSize(11);
  textAlign(LEFT, BASELINE);
  text("click card / 1–4 record  ·  right-click clears  ·  c clear all  ·  a autoscale " + (autoscale ? "on" : "off") + "  ·  r reconnect",
    50, height - 18);
}

int cardAt(float mx, float my) {
  for (int i = 0; i < names.length; i++) {
    float cy = CARD_Y + i * (CARD_H + CARD_GAP);
    if (mx > CARD_X && mx < CARD_X + CARD_W && my > cy && my < cy + CARD_H) return i;
  }
  return -1;
}

void record(int i) {
  if (latestY != null) gestures[i] = latestY.clone();
}

void mousePressed() {
  int i = cardAt(mouseX, mouseY);
  if (i < 0) return;
  if (mouseButton == RIGHT) {
    gestures[i] = null;
    confidence[i] = 0;
  } else {
    record(i);
  }
}

void keyPressed() {
  if (key >= '1' && key <= '4') record(key - '1');
  if (key == 'c') {
    gestures = new float[4][];
    confidence = new float[4];
    best = -1;
  }
  if (key == 'a') autoscale = !autoscale;
  if (key == 'r') connect();
}
