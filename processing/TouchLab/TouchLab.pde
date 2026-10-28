// Touché lab: live sweep, change-from-baseline view, feature tracks,
// dynamic gesture recognition and a labelled dataset recorder.
// Needs firmware v2 (src/v2/main.cpp, 500000 baud).
//
// Set the embroidery pattern name (top), keep hands off while the baseline calibrates,
// then hold REC on a card (or keys 1-6) while performing a gesture.
// Each recording becomes a recognition example AND a CSV take in data/dataset/<pattern>/.
// Live gestures are cut into segments (rise above noise ... pause) and each segment is
// matched once: temporal pattern first, intensity only to break near-ties (see Gestures.pde).
// Right-click a card removes its last example (and its CSV). Click names to rename.
// d = absolute / change-from-baseline view, b = recalibrate baseline, r = reconnect.

// Palette, highlight -> bg
final color C1 = #32213A;
final color C2 = #383B53;
final color C3 = #66717E;
final color C4 = #D4D6B9;
final color C5 = #D1CAA1;

final int SLOTS = 6;
final int HISTORY = 300;      // frames kept for the time views (~8 s)
final int CALIB_FRAMES = 20;
final int WARMUP_FRAMES = 30; // let the sensor settle (and hands move away) before calibrating
final int MAX_SEGMENT_MS = 8000; // a segment this long is closed and its level becomes the new rest
final float DRIFT_RATE = 0.01;   // per sweep while idle: baseline follows slow drift (~3 s)

class Frame {
  float[] raw;  // full sweep
  float[] f;    // BINS, minus baseline (recognition)
  float[] feat; // features()
  int t;        // millis() on arrival
  int rec = -1; // slot being recorded on this frame
  int det = -1; // slot detected at the end of a segment on this frame
  boolean seg;  // part of a live segment
}

Graph graph;
float[] shown;
boolean deltaView = true;
float deltaRange = 20;
ArrayList<Frame> history = new ArrayList<Frame>();

float[] baseline;    // BINS
float[] baselineRaw; // full sweep
float[] baseFeat;
float noise = 2;
int calibrating = 0, warmup = 0;
ArrayList<float[]> calibFrames = new ArrayList<float[]>();

String pattern = "pattern 1";
String[] names = new String[SLOTS];
ArrayList<Template> templates = new ArrayList<Template>();
float[] sim = new float[SLOTS];   // pattern score of the last segment
float[] inten = new float[SLOTS]; // intensity score of the last segment
float[] shownSim = new float[SLOTS];
float threshold = 0.6;
int gapMs = 350;

// live segmentation
boolean inSeg = false;
ArrayList<Frame> segFrames = new ArrayList<Frame>();
int lastActiveT;

// last decision
int detected = -1, detectedAt = -100000, unknownAt = -100000;
float detectedSim;
boolean detByIntensity;
int closest = -1;
float closestSim;
float[] segEnv;
Template matchT;

int recording = -1;
char recKey = 0;
ArrayList<Frame> recFrames = new ArrayList<Frame>();
int recStartMs;

final int EDIT_PATTERN = -2;
int editing = -1; // card index, EDIT_PATTERN, or -1
int dragSlider = -1;

String toastMsg = "";
int toastAt = -100000;
int rateCount = 0, rateAt = 0;
float sweepRate = 0;
float heatScale = 20;
PImage heat;

// Layout
final float LX = 40, LW = 960;                     // left column
final float RX = 1020, RW = 380;                   // right column
final float HM_X = 100, HM_Y = 420, HM_W = 870, HM_H = 130;
final float FT_Y = 620, FT_H = 215;                // feature tracks panel
final float BAN_Y = 70, BAN_H = 80;
final float CARD_Y = 162, CARD_H = 72, CARD_GAP = 8;
final float SL_Y = 646, SL_H = 189;
final float PAT_X = 300, PAT_W = 240;              // pattern field
final float TOG_X = 580;                           // view toggle

void setup() {
  size(1440, 900);
  pixelDensity(displayDensity());
  smooth(8);
  textFont(createFont("SansSerif", 14));
  graph = new Graph(100, 100, 870, 230);
  heat = createImage(HISTORY, BINS, RGB);
  for (int i = 0; i < SLOTS; i++) names[i] = "Gesture " + char('A' + i);
  loadGestures();
  connect();
  startCalibration();
}

void draw() {
  readSerial();
  for (float[] s : newSweeps) onSweep(latestX, s);
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
  drawFeatures();
  drawBanner();
  drawCards();
  drawSliders();
  drawFooter();
}

// ---------------------------------------------------------------- data

float onLevel() {
  return max(4 * noise, 2);
}

float offLevel() {
  return max(2.5 * noise, 1.2);
}

void onSweep(float[] xs, float[] ys) {
  rateCount++;
  if (xs == null || xs.length != ys.length) return;
  if (baselineRaw != null && baselineRaw.length != ys.length) startCalibration(); // firmware changed

  if (warmup > 0) {
    warmup--;
    return;
  }
  if (calibrating > 0) {
    calibFrames.add(ys);
    if (--calibrating == 0) finishCalibration(xs);
    return;
  }
  if (baseline == null) return;

  Frame fr = new Frame();
  fr.raw = ys;
  fr.t = millis();
  fr.feat = features(xs, ys);
  float[] d = downsample(ys);
  fr.f = new float[BINS];
  for (int k = 0; k < BINS; k++) fr.f[k] = d[k] - baseline[k];
  history.add(fr);
  if (history.size() > HISTORY) history.remove(0);

  if (recording >= 0) {
    recFrames.add(fr);
    fr.rec = recording;
  } else {
    segmentStep(fr);
    if (!inSeg && rms(fr.f) < offLevel()) followDrift(xs, ys);
  }
}

// While idle, let the baseline slowly follow the signal (temperature, humidity, cable movement...)
void followDrift(float[] xs, float[] ys) {
  for (int i = 0; i < ys.length; i++) baselineRaw[i] += DRIFT_RATE * (ys[i] - baselineRaw[i]);
  baseline = downsample(baselineRaw);
  baseFeat = features(xs, baselineRaw);
}

// The signal settled at a new level: take the last second as the new baseline
void rebaseline(float[] xs) {
  int n = min(history.size(), max(5, round(sweepRate)));
  if (n == 0) return;
  float[] b = new float[baselineRaw.length];
  for (int j = history.size() - n; j < history.size(); j++) {
    float[] r = history.get(j).raw;
    for (int i = 0; i < b.length; i++) b[i] += r[i] / n;
  }
  baselineRaw = b;
  baseline = downsample(baselineRaw);
  baseFeat = features(xs, baselineRaw);
  toast("signal stayed away from rest for " + MAX_SEGMENT_MS / 1000 + " s: took it as the new baseline (press b to recalibrate properly)");
}

void startCalibration() {
  calibFrames.clear();
  warmup = WARMUP_FRAMES;
  calibrating = CALIB_FRAMES;
  inSeg = false;
  segFrames.clear();
}

void finishCalibration(float[] xs) {
  int n = calibFrames.get(0).length;
  baselineRaw = new float[n];
  for (float[] c : calibFrames) for (int i = 0; i < n; i++) baselineRaw[i] += c[i] / calibFrames.size();
  baseline = downsample(baselineRaw);
  baseFeat = features(xs, baselineRaw);
  float s = 0;
  for (float[] c : calibFrames) {
    float[] d = downsample(c);
    for (int k = 0; k < BINS; k++) s += sq(d[k] - baseline[k]);
  }
  noise = max(0.5, sqrt(s / (calibFrames.size() * BINS)));
  history.clear();
  toast("baseline set for " + pattern + " · noise ±" + nf(noise, 0, 2));
}

ArrayList<Template> activeTemplates() {
  ArrayList<Template> out = new ArrayList<Template>();
  for (Template t : templates) if (t.pattern.equals(pattern)) out.add(t);
  return out;
}

// Gesture = rise above onLevel ... until it stays below offLevel for gapMs
void segmentStep(Frame fr) {
  float e = rms(fr.f);
  if (!inSeg) {
    if (e <= onLevel()) return;
    inSeg = true;
    segFrames.clear();
    for (int i = max(0, history.size() - 1 - PRE_FRAMES); i < history.size() - 1; i++) {
      segFrames.add(history.get(i));
      history.get(i).seg = true;
    }
    lastActiveT = fr.t;
  }
  segFrames.add(fr);
  fr.seg = true;
  if (e > offLevel()) lastActiveT = fr.t;
  if (fr.t - lastActiveT > gapMs) {
    endSegment();
  } else if (fr.t - segFrames.get(0).t > MAX_SEGMENT_MS) {
    endSegment();
    rebaseline(latestX);
  }
}

void endSegment() {
  inSeg = false;
  ArrayList<float[]> fs = new ArrayList<float[]>();
  for (Frame fr : segFrames) fs.add(fr.f);
  int[] r = activeRange(fs, onLevel(), offLevel());
  Frame last = segFrames.get(segFrames.size() - 1);
  segFrames.clear();
  if (r == null) return;

  float[][] seg = new float[r[1] - r[0] + 1][];
  for (int i = r[0]; i <= r[1]; i++) seg[i - r[0]] = fs.get(i);
  segEnv = envelope(seg);

  ArrayList<Template> active = activeTemplates();
  if (active.isEmpty()) {
    matchT = null;
    toast("gesture segment of " + seg.length + " sweeps · record examples to recognise it");
    return;
  }
  Match m = classify(seg, active, SLOTS);
  sim = m.pattern;
  inten = m.intensity;
  matchT = m.winner >= 0 ? m.best[m.winner] : null;
  float top = 0;
  for (float v : m.pattern) top = max(top, v);

  if (m.winner >= 0 && top >= threshold) {
    detected = m.winner;
    detectedSim = m.pattern[m.winner];
    detByIntensity = m.byIntensity;
    detectedAt = millis();
    last.det = m.winner;
    println("detected " + names[m.winner] + "  pattern " + round(m.pattern[m.winner] * 100)
      + "%  intensity " + round(m.intensity[m.winner] * 100) + "%" + (m.byIntensity ? "  (intensity decided)" : ""));
  } else {
    closest = m.winner;
    closestSim = top;
    unknownAt = millis();
    println("no match, closest " + (closest >= 0 ? names[closest] : "-") + " " + round(top * 100) + "%");
  }
}

void startRecording(int c) {
  if (baseline == null) return;
  recording = c;
  recFrames.clear();
  recStartMs = millis();
  editing = -1;
  inSeg = false;
  segFrames.clear();
}

void finishRecording() {
  int c = recording;
  recording = -1;
  recKey = 0;
  if (recFrames.isEmpty()) return;
  ArrayList<float[]> fs = new ArrayList<float[]>();
  for (Frame fr : recFrames) fs.add(fr.f);
  int[] r = activeRange(fs, onLevel(), offLevel());
  if (r == null) {
    toast("nothing recorded: no clear signal, try a longer or stronger gesture");
    return;
  }
  float[][] frames = new float[r[1] - r[0] + 1][];
  for (int i = r[0]; i <= r[1]; i++) frames[i - r[0]] = fs.get(i);
  String file = saveTake(names[c], recFrames, r[0], r[1], latestX);
  templates.add(new Template(c, pattern, file, frames));
  saveGestures();

  int pause = longestPause(recFrames, r[0], r[1], offLevel());
  if (pause > gapMs) {
    toast("saved, but it contains a " + pause + " ms pause: raise \"gesture ends after\" above that or it will be split");
  } else {
    toast("saved " + file + "  ·  example " + countExamples(c) + " of " + names[c]);
  }
}

void removeLast(int c) {
  for (int i = templates.size() - 1; i >= 0; i--) {
    Template t = templates.get(i);
    if (t.cls == c && t.pattern.equals(pattern)) {
      templates.remove(i);
      deleteTake(t.file);
      sim[c] = 0;
      if (matchT == t) matchT = null;
      saveGestures();
      toast("removed last example of " + names[c] + (t.file.length() > 0 ? " and " + t.file : ""));
      return;
    }
  }
}

int countExamples(int c) {
  int n = 0;
  for (Template t : templates) if (t.cls == c && t.pattern.equals(pattern)) n++;
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
  text("Touché lab", LX, 46);

  // pattern field
  boolean ed = editing == EDIT_PATTERN;
  fill(C3);
  textSize(11);
  textAlign(RIGHT, CENTER);
  text("pattern", PAT_X - 10, 37);
  noStroke();
  fill(ed ? C1 : C4);
  rect(PAT_X, 22, PAT_W, 30, 15);
  fill(ed ? C4 : C1);
  textSize(14);
  textAlign(LEFT, CENTER);
  text(pattern + (ed && (frameCount / 30) % 2 == 0 ? "|" : ""), PAT_X + 14, 36);

  // view toggle
  String[] opts = {"absolute", "Δ baseline"};
  float x = TOG_X;
  for (int i = 0; i < 2; i++) {
    boolean on = deltaView == (i == 1);
    float w = textWidth(opts[i]) + 28;
    fill(on ? C1 : C4);
    rect(x, 22, w, 30, 15);
    fill(on ? C4 : C1);
    textAlign(CENTER, CENTER);
    text(opts[i], x + w / 2, 36);
    x += w + 6;
  }

  // baseline button
  boolean hot = overBaselineButton(mouseX, mouseY);
  fill(hot ? C2 : C4);
  rect(RX, 22, RW, 30, 15);
  fill(hot ? C4 : C1);
  textSize(12);
  textAlign(CENTER, CENTER);
  text("recalibrate baseline (b)  ·  noise ±" + nf(noise, 0, 2), RX + RW / 2, 36);
}

boolean overPattern(float mx, float my) {
  return mx > PAT_X && mx < PAT_X + PAT_W && my > 22 && my < 52;
}

boolean overBaselineButton(float mx, float my) {
  return mx > RX && mx < RX + RW && my > 22 && my < 52;
}

int overToggle(float mx, float my) {
  if (my < 22 || my > 52) return -1;
  textSize(14);
  float w0 = textWidth("absolute") + 28, w1 = textWidth("Δ baseline") + 28;
  if (mx > TOG_X && mx < TOG_X + w0) return 0;
  if (mx > TOG_X + w0 + 6 && mx < TOG_X + w0 + 6 + w1) return 1;
  return -1;
}

void drawSweep() {
  float[] xs = latestX, ys = latestY;
  boolean hasData = xs != null && ys != null && xs.length == ys.length;
  boolean delta = deltaView && baselineRaw != null && hasData && baselineRaw.length == ys.length;

  if (hasData) {
    if (shown == null || shown.length != ys.length) shown = ys.clone();
    for (int i = 0; i < ys.length; i++) shown[i] = lerp(shown[i], ys[i], 0.5);
    graph.xMin = xs[0];
    graph.xMax = xs[xs.length - 1];
  }
  if (delta) {
    float m = 0;
    for (int i = 0; i < shown.length; i++) m = max(m, abs(shown[i] - baselineRaw[i]));
    deltaRange = lerp(deltaRange, max(m * 1.2, 10 * noise), 0.1);
    graph.yMin = -deltaRange;
    graph.yMax = deltaRange;
    graph.yLabel = "Δ amplitude";
  } else {
    graph.yMin = 0;
    if (hasData) graph.yMax = lerp(graph.yMax, max(max(shown) * 1.15, 50), 0.1);
    graph.yLabel = "amplitude";
  }

  graph.drawFrame();
  if (!hasData) {
    graph.drawWaiting("waiting for sweep data…");
    return;
  }
  if (delta) {
    float[] d = new float[shown.length];
    for (int i = 0; i < d.length; i++) d[i] = shown[i] - baselineRaw[i];
    graph.drawCurve(xs, d);
    graph.drawCursor(xs, d);
  } else {
    if (baselineRaw != null && baselineRaw.length == xs.length) graph.drawGhost(xs, baselineRaw, "baseline", false);
    graph.drawCurve(xs, shown);
    graph.drawCursor(xs, shown);
  }
}

float colX(int i) {
  return HM_X + (HISTORY - history.size() + i) * HM_W / HISTORY;
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
      heat.pixels[k * HISTORY + x] = c; // row 0 (top) = first sweep step = highest frequency
    }
  }
  heat.updatePixels();
  image(heat, HM_X, HM_Y, HM_W, HM_H);
  drawMarks(HM_Y, HM_H);

  fill(C2);
  textSize(12);
  textAlign(LEFT, TOP);
  text("time  →", HM_X, HM_Y + HM_H + 10);
  pushMatrix();
  translate(HM_X - 42, HM_Y + HM_H / 2);
  rotate(-HALF_PI);
  textAlign(CENTER, BOTTOM);
  text("frequency", 0, 0);
  popMatrix();
  if (latestX != null) {
    fill(C3);
    textSize(11);
    textAlign(RIGHT, TOP);
    text(round(latestX[0]) + " kHz", HM_X - 6, HM_Y);
    textAlign(RIGHT, BOTTOM);
    text(round(latestX[latestX.length - 1]) + " kHz", HM_X - 6, HM_Y + HM_H);
  }

  // legend
  textSize(11);
  float lx = HM_X + HM_W, ly = HM_Y + HM_H + 18;
  fill(C3);
  textAlign(RIGHT, CENTER);
  text("segment", lx - 400, ly);
  fill(C3);
  rect(lx - 390, ly - 3, 18, 5);
  fill(C1);
  rect(lx - 360, ly - 3, 18, 5);
  fill(C3);
  textAlign(LEFT, CENTER);
  text("recording", lx - 336, ly);
  textAlign(RIGHT, CENTER);
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

// Recording / segment spans (bar above) and detections (line + label) for a time view
void drawMarks(float y, float h) {
  float colW = HM_W / HISTORY;
  for (int i = 0; i < history.size(); i++) {
    Frame fr = history.get(i);
    float x = colX(i);
    if (fr.rec >= 0 || fr.seg) {
      noStroke();
      fill(fr.rec >= 0 ? C1 : C3);
      rect(x, y - 10, colW + 0.5, 5);
    }
    if (fr.det >= 0) {
      stroke(C1);
      strokeWeight(1.5);
      line(x, y - 3, x, y + h);
      noStroke();
      fill(C1);
      textSize(11);
      textAlign(RIGHT, BOTTOM);
      text(names[fr.det], x - 3, y - 1);
    }
  }
  noStroke();
}

// Four sparklines sharing the time axis of the heat view
void drawFeatures() {
  noStroke();
  fill(C4);
  rect(LX, FT_Y, LW, FT_H, 16);

  int nFeat = FEATURE_NAMES.length;
  float gap = 20, x0 = LX + 20, w = (LW - 40 - gap * (nFeat - 1)) / nFeat;
  float sy = FT_Y + 92, sh = FT_H - 112;

  for (int k = 0; k < nFeat; k++) {
    float x = x0 + k * (w + gap);
    Frame last = history.isEmpty() ? null : history.get(history.size() - 1);

    fill(C2);
    textSize(12);
    textAlign(LEFT, BASELINE);
    text(FEATURE_NAMES[k], x, FT_Y + 28);
    fill(C1);
    textSize(24);
    String val = last == null ? "–" : nf(last.feat[k], 0, 1) + FEATURE_UNITS[k];
    text(val, x, FT_Y + 60);
    if (last != null && baseFeat != null) {
      float d = last.feat[k] - baseFeat[k];
      fill(C3);
      textSize(11);
      text((d >= 0 ? "+" : "") + nf(d, 0, 1) + " vs baseline", x, FT_Y + 78);
    }

    fill(C5);
    rect(x, sy, w, sh, 8);
    if (history.size() < 2) continue;

    float lo = Float.MAX_VALUE, hi = -Float.MAX_VALUE;
    for (Frame fr : history) {
      lo = min(lo, fr.feat[k]);
      hi = max(hi, fr.feat[k]);
    }
    if (baseFeat != null) {
      lo = min(lo, baseFeat[k]);
      hi = max(hi, baseFeat[k]);
    }
    float pad = max((hi - lo) * 0.15, 0.5);
    lo -= pad;
    hi += pad;

    // recording / segment spans
    float step = w / HISTORY;
    int offset = HISTORY - history.size();
    fill(C4);
    for (int i = 0; i < history.size(); i++) {
      Frame fr = history.get(i);
      if (fr.rec >= 0 || fr.seg) rect(x + (offset + i) * step, sy, step + 0.5, sh);
    }

    if (baseFeat != null) {
      stroke(C3);
      strokeWeight(1);
      float by = map(baseFeat[k], lo, hi, sy + sh - 4, sy + 4);
      for (float dx = 0; dx < w; dx += 8) line(x + dx, by, x + min(dx + 4, w), by);
    }

    noFill();
    stroke(C1);
    strokeWeight(1.8);
    beginShape();
    for (int i = 0; i < history.size(); i++) {
      vertex(x + (offset + i) * step, map(history.get(i).feat[k], lo, hi, sy + sh - 4, sy + 4));
    }
    endShape();

    for (int i = 0; i < history.size(); i++) {
      if (history.get(i).det < 0) continue;
      stroke(C2);
      strokeWeight(1);
      float dx = x + (offset + i) * step;
      line(dx, sy, dx, sy + sh);
    }
    noStroke();
  }
}

// Intensity over time of the last segment (solid) on top of the example it matched (dotted)
void drawMatchView(float x, float y, float w, float h, boolean dark, float alpha) {
  if (segEnv == null || segEnv.length < 2) return;
  float[] te = matchT != null ? matchT.env : null;
  float top = max(segEnv);
  if (te != null) top = max(top, max(te));
  top = max(top, 1e-3);

  if (te != null && te.length > 1) {
    stroke(dark ? C3 : C3, alpha);
    strokeWeight(1.5);
    for (int i = 0; i < te.length; i++) {
      float px = x + w * i / (te.length - 1), py = y + h - h * te[i] / top;
      point(px, py);
      if (i > 0) {
        float qx = x + w * (i - 1) / (te.length - 1), qy = y + h - h * te[i - 1] / top;
        if (i % 2 == 0) line(qx, qy, px, py);
      }
    }
  }
  noFill();
  stroke(dark ? C4 : C1, alpha);
  strokeWeight(2);
  beginShape();
  for (int i = 0; i < segEnv.length; i++) vertex(x + w * i / (segEnv.length - 1), y + h - h * segEnv[i] / top);
  endShape();
  noStroke();
}

void drawBanner() {
  int sinceDet = millis() - detectedAt, sinceUnk = millis() - unknownAt;
  float cy = BAN_Y + BAN_H / 2;
  float mvX = RX + RW - 150, mvY = BAN_Y + 16, mvW = 130, mvH = BAN_H - 32;

  noStroke();
  if (recording >= 0) {
    fill(C1);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C4);
    textSize(18);
    textAlign(LEFT, CENTER);
    text("●  recording " + names[recording], RX + 20, cy - 11);
    textSize(12);
    text(nf((millis() - recStartMs) / 1000.0, 0, 1) + " s  ·  " + recFrames.size() + " sweeps", RX + 20, cy + 15);
  } else if (calibrating > 0 || baseline == null) {
    fill(C4);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C2);
    textSize(15);
    textAlign(LEFT, CENTER);
    text("calibrating baseline…", RX + 20, cy - 10);
    fill(C3);
    textSize(12);
    text("hands off the sensor", RX + 20, cy + 13);
  } else if (inSeg) {
    fill(C2);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C4);
    textSize(16);
    textAlign(LEFT, CENTER);
    text("gesture…", RX + 20, cy - 10);
    textSize(12);
    text(nf((millis() - segFrames.get(0).t) / 1000.0, 0, 1) + " s  ·  ends after a " + gapMs + " ms pause", RX + 20, cy + 13);
  } else if (detected >= 0 && sinceDet < 3000) {
    float a = sinceDet < 2300 ? 255 : map(sinceDet, 2300, 3000, 255, 0);
    fill(C1, a);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C4, a);
    textSize(24);
    textAlign(LEFT, CENTER);
    text(names[detected], RX + 20, cy - 12);
    textSize(11);
    text("pattern " + round(detectedSim * 100) + "%  ·  intensity " + round(inten[detected] * 100) + "%"
      + (detByIntensity ? "  ·  intensity decided" : ""), RX + 20, cy + 16);
    drawMatchView(mvX, mvY, mvW, mvH, true, a);
  } else if (sinceUnk < 3000) {
    fill(C4);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C2);
    textSize(18);
    textAlign(LEFT, CENTER);
    text("no match", RX + 20, cy - 12);
    fill(C3);
    textSize(11);
    text(closest >= 0 ? "closest: " + names[closest] + " " + round(closestSim * 100) + "%" : "", RX + 20, cy + 14);
    drawMatchView(mvX, mvY, mvW, mvH, false, 255);
  } else {
    fill(C4);
    rect(RX, BAN_Y, RW, BAN_H, 16);
    fill(C3);
    textSize(14);
    textAlign(LEFT, CENTER);
    text(activeTemplates().isEmpty() ? "hold REC while doing a gesture\nto record an example" : "perform a gesture…", RX + 20, cy);
  }
}

float cardY(int i) {
  return CARD_Y + i * (CARD_H + CARD_GAP);
}

boolean overRec(int i, float mx, float my) {
  float cy = cardY(i);
  return mx > RX + RW - 84 && mx < RX + RW - 14 && my > cy + 8 && my < cy + 38;
}

boolean overName(int i, float mx, float my) {
  float cy = cardY(i);
  return mx > RX + 52 && mx < RX + RW - 180 && my > cy + 4 && my < cy + 40;
}

void drawCards() {
  boolean recent = millis() - detectedAt < 3000;
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
    ellipse(RX + 28, cy + 23, 28, 28);
    fill(fg);
    textSize(13);
    textAlign(CENTER, CENTER);
    text(i + 1, RX + 28, cy + 22);

    textSize(18);
    textAlign(LEFT, CENTER);
    text(names[i] + (editing == i && (frameCount / 30) % 2 == 0 ? "|" : ""), RX + 52, cy + 21);
    if (editing == i) {
      stroke(fg);
      strokeWeight(1);
      line(RX + 52, cy + 34, RX + RW - 180, cy + 34);
      noStroke();
    }

    int n = countExamples(i);
    fill(dim);
    textSize(11);
    textAlign(RIGHT, CENTER);
    text(n == 0 ? "no examples" : n + (n == 1 ? " example" : " examples"), RX + RW - 94, cy + 23);

    // REC pill
    boolean hot = rec || overRec(i, mouseX, mouseY);
    fill(rec ? C4 : hot ? C2 : C1);
    rect(RX + RW - 84, cy + 8, 70, 30, 15);
    fill(rec ? C1 : C4);
    textSize(12);
    textAlign(CENTER, CENTER);
    text(rec ? "● " + nf((millis() - recStartMs) / 1000.0, 0, 1) + "s" : "REC", RX + RW - 49, cy + 22);

    // pattern score bar + threshold tick, intensity as a thin line under it
    float bx = RX + 18, by = cy + CARD_H - 24, bw = RW - 80;
    fill(dark ? C3 : C5);
    rect(bx, by, bw, 9, 4.5);
    if (n > 0) {
      fill(dark ? C4 : (shownSim[i] >= threshold ? C1 : C2));
      rect(bx, by, max(9, bw * shownSim[i]), 9, 4.5);
      fill(dark ? C4 : C3, 160);
      rect(bx, by + 12, bw * inten[i], 3, 1.5);
    }
    stroke(dark ? C4 : C1);
    strokeWeight(2);
    float tx = bx + bw * threshold;
    line(tx, by - 4, tx, by + 13);
    noStroke();
    fill(dim);
    textSize(11);
    textAlign(RIGHT, CENTER);
    text(n > 0 ? round(shownSim[i] * 100) + "%" : "–", RX + RW - 18, by + 5);
  }
}

// ---------------------------------------------------------------- sliders

final String[] SL_LABEL = {"pattern match threshold", "gesture ends after a pause of", "intensity decides within"};
final String[] SL_HINT = {"lower = more detections, higher = stricter", "raise for double taps or gestures with pauses",
  "pattern scores this close → closest intensity wins"};

float slValue(int i) {
  if (i == 0) return threshold;
  if (i == 1) return map(gapMs, 100, 1500, 0, 1);
  return PATTERN_MARGIN / 0.3;
}

void slSet(int i, float v) {
  v = constrain(v, 0, 1);
  if (i == 0) threshold = constrain(v, 0.05, 0.99);
  else if (i == 1) gapMs = round(lerp(100, 1500, v) / 10) * 10;
  else PATTERN_MARGIN = lerp(0, 0.3, v);
}

String slText(int i) {
  if (i == 0) return round(threshold * 100) + "%";
  if (i == 1) return gapMs + " ms";
  return "±" + round(PATTERN_MARGIN * 100) + "%";
}

float slRowY(int i) {
  return SL_Y + 16 + i * 58;
}

float slTrackY(int i) {
  return slRowY(i) + 38;
}

void drawSliders() {
  noStroke();
  fill(C4);
  rect(RX, SL_Y, RW, SL_H, 16);

  float x0 = RX + 20, x1 = RX + RW - 20;
  for (int i = 0; i < SL_LABEL.length; i++) {
    float y = slRowY(i);
    fill(C2);
    textSize(12);
    textAlign(LEFT, TOP);
    text(SL_LABEL[i], x0, y);
    fill(C3);
    textSize(10);
    text(SL_HINT[i], x0, y + 15);
    fill(C1);
    textSize(18);
    textAlign(RIGHT, TOP);
    text(slText(i), x1, y);

    float ty = slTrackY(i), kx = lerp(x0, x1, slValue(i));
    strokeCap(ROUND);
    strokeWeight(6);
    stroke(C5);
    line(x0, ty, x1, ty);
    stroke(C2);
    line(x0, ty, kx, ty);
    noStroke();
    fill(C1);
    float r = dragSlider == i ? 20 : 16;
    ellipse(kx, ty, r, r);
  }
}

void drawFooter() {
  textSize(11);
  textAlign(LEFT, BASELINE);
  if (millis() - toastAt < 5000) {
    fill(C1);
    text(toastMsg, LX, height - 22);
  } else {
    fill(C3);
    text("hold REC / 1–6 to record  ·  right-click card removes last  ·  click names to rename  ·  d view  ·  b baseline  ·  r reconnect",
      LX, height - 22);
  }
  float errPct = packets == 0 ? 0 : 100.0 * errors / packets;
  fill(C3);
  textAlign(RIGHT, BASELINE);
  String fw = fwCount > 0 ? fwCount + " steps ×" + fwOversample + "  ·  " : "";
  text(portName + "  ·  " + fw + round(sweepRate) + " sweeps/s  ·  " + nf(errPct, 0, 1) + "% errors", RX + RW, height - 22);
}

// ---------------------------------------------------------------- input

void endEditing() {
  if (editing == EDIT_PATTERN) {
    pattern = pattern.trim().length() == 0 ? "pattern 1" : pattern.trim();
    toast("pattern: " + pattern + "  ·  changed the embroidery? press b to recalibrate the baseline");
  }
  editing = -1;
  saveGestures();
}

void mousePressed() {
  if (editing != -1) endEditing();
  for (int i = 0; i < SL_LABEL.length; i++) {
    if (abs(mouseY - slTrackY(i)) < 14 && mouseX > RX + 8 && mouseX < RX + RW - 8) {
      dragSlider = i;
      slSet(i, map(mouseX, RX + 20, RX + RW - 20, 0, 1));
      return;
    }
  }
  if (overBaselineButton(mouseX, mouseY)) {
    startCalibration();
    return;
  }
  if (overPattern(mouseX, mouseY)) {
    editing = EDIT_PATTERN;
    return;
  }
  int tog = overToggle(mouseX, mouseY);
  if (tog >= 0) {
    deltaView = tog == 1;
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
}

void mouseDragged() {
  if (dragSlider >= 0) slSet(dragSlider, map(mouseX, RX + 20, RX + RW - 20, 0, 1));
}

void mouseReleased() {
  if (dragSlider >= 0) {
    dragSlider = -1;
    saveGestures();
  }
  if (recording >= 0 && recKey == 0) finishRecording();
}

void keyPressed() {
  if (editing != -1) {
    String s = editing == EDIT_PATTERN ? pattern : names[editing];
    if (key == ENTER || key == RETURN || key == ESC) {
      if (key == ESC) key = 0; // don't quit the sketch
      endEditing();
      return;
    } else if (key == BACKSPACE) {
      if (s.length() > 0) s = s.substring(0, s.length() - 1);
    } else if (key != CODED && key >= ' ' && s.length() < 22) {
      s += key;
    }
    if (editing == EDIT_PATTERN) pattern = s;
    else names[editing] = s;
    return;
  }
  if (key >= '1' && key <= '0' + SLOTS && recording < 0) {
    startRecording(key - '1');
    if (recording >= 0) recKey = key;
  }
  if (key == 'd') deltaView = !deltaView;
  if (key == 'b') startCalibration();
  if (key == 'r') connect();
}

void keyReleased() {
  if (recording >= 0 && key == recKey) finishRecording();
}
