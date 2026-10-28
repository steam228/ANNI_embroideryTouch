// Sweep plot: rounded panel, soft grid, filled curve, peak marker, mouse readout

class Graph {
  float x, y, w, h;
  float xMin = 0, xMax = 160, yMin = 0, yMax = 1024;
  String xLabel = "frequency (kHz)";
  String yLabel = "amplitude";

  Graph(float x, float y, float w, float h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }

  float px(float v) {
    return map(v, xMin, xMax, x, x + w);
  }

  float py(float v) {
    return map(constrain(v, yMin, yMax), yMin, yMax, y + h, y);
  }

  void drawFrame() {
    noStroke();
    fill(C4);
    rect(x - 60, y - 30, w + 90, h + 80, 16);

    textSize(11);
    for (int i = 0; i <= 4; i++) {
      float v = lerp(yMin, yMax, i / 4.0);
      stroke(C5);
      strokeWeight(1);
      line(x, py(v), x + w, py(v));
      noStroke();
      fill(C3);
      textAlign(RIGHT, CENTER);
      text(str(round(v)), x - 10, py(v));
    }
    for (int i = 0; i <= 8; i++) {
      float v = lerp(xMin, xMax, i / 8.0);
      stroke(C5);
      line(px(v), y, px(v), y + h);
      noStroke();
      fill(C3);
      textAlign(CENTER, TOP);
      text(str(round(v)), px(v), y + h + 8);
    }

    if (yMin < 0 && yMax > 0) { // zero line in the change-from-baseline view
      stroke(C3);
      strokeWeight(1.5);
      line(x, py(0), x + w, py(0));
      noStroke();
    }

    fill(C2);
    textSize(12);
    textAlign(CENTER, TOP);
    text(xLabel, x + w / 2, y + h + 28);
    pushMatrix();
    translate(x - 42, y + h / 2);
    rotate(-HALF_PI);
    textAlign(CENTER, BOTTOM);
    text(yLabel, 0, 0);
    popMatrix();
  }

  void curvePath(float[] xs, float[] ys) {
    int n = ys.length;
    curveVertex(px(xs[0]), py(ys[0]));
    for (int i = 0; i < n; i++) curveVertex(px(xs[i]), py(ys[i]));
    curveVertex(px(xs[n - 1]), py(ys[n - 1]));
  }

  int peak(float[] ys) {
    int p = 0;
    for (int i = 1; i < ys.length; i++) if (ys[i] > ys[p]) p = i;
    return p;
  }

  // A recorded gesture, drawn faintly behind the live curve
  void drawGhost(float[] xs, float[] ys, String label, boolean strong) {
    noFill();
    stroke(strong ? C2 : C3, strong ? 200 : 110);
    strokeWeight(strong ? 2 : 1.5);
    beginShape();
    curvePath(xs, ys);
    endShape();

    int p = peak(ys);
    noStroke();
    fill(strong ? C2 : C3);
    textSize(11);
    textAlign(LEFT, BOTTOM);
    text(label, px(xs[p]) + 6, py(ys[p]) - 4);
  }

  void drawCurve(float[] xs, float[] ys) {
    int n = ys.length;

    noStroke();
    fill(C2, 50);
    beginShape();
    vertex(px(xs[0]), py(0));
    for (int i = 0; i < n; i++) vertex(px(xs[i]), py(ys[i]));
    vertex(px(xs[n - 1]), py(0));
    endShape(CLOSE);

    noFill();
    stroke(C1);
    strokeWeight(3);
    strokeJoin(ROUND);
    beginShape();
    curvePath(xs, ys);
    endShape();

    int p = peak(ys);
    float cx = px(xs[p]), cy = py(ys[p]);
    noFill();
    stroke(C1);
    strokeWeight(2);
    ellipse(cx, cy, 18, 18);
    noStroke();
    fill(C1);
    ellipse(cx, cy, 6, 6);
    textSize(12);
    textAlign(CENTER, BOTTOM);
    text(str(round(ys[p])) + " @ " + str(round(xs[p])), cx, cy - 14);
  }

  void drawCursor(float[] xs, float[] ys) {
    if (mouseX < x || mouseX > x + w || mouseY < y || mouseY > y + h) return;
    int i = 0;
    for (int j = 1; j < xs.length; j++) {
      if (abs(px(xs[j]) - mouseX) < abs(px(xs[i]) - mouseX)) i = j;
    }
    float cx = px(xs[i]), cy = py(ys[i]);
    stroke(C3, 120);
    strokeWeight(1);
    line(cx, y, cx, y + h);
    noStroke();
    fill(C1);
    ellipse(cx, cy, 8, 8);

    String s = str(round(xs[i])) + "  →  " + str(round(ys[i]));
    textSize(12);
    float tw = textWidth(s) + 16;
    float bx = min(cx + 10, x + w - tw);
    rect(bx, y + 8, tw, 22, 11);
    fill(C4);
    textAlign(LEFT, CENTER);
    text(s, bx + 8, y + 19);
  }

  void drawWaiting(String msg) {
    fill(C3);
    textSize(14);
    textAlign(CENTER, CENTER);
    text(msg, x + w / 2, y + h / 2);
  }
}
