/* Dynamic gesture recognition: DTW + nearest neighbour.

   Each sweep is reduced to BINS values and the resting baseline is subtracted,
   so a frame describes how the curve has moved away from rest. A gesture is
   the sequence of those frames over time.

   Recognition compares the most recent frames against every recorded example
   with subsequence Dynamic Time Warping: it finds the best alignment that ends
   now and may start anywhere in the window, so faster or slower performances
   still match. Similarity = 1 - distance / example energy:
   100% = identical, 0% = no closer than doing nothing. */

final int BINS = 32;

class Template {
  int cls;
  float[][] frames;
  float energy; // mean RMS of the frames: how far the gesture moves from rest

  Template(int cls, float[][] frames) {
    this.cls = cls;
    this.frames = frames;
    float e = 0;
    for (float[] f : frames) e += rms(f);
    energy = e / frames.length;
  }
}

float rms(float[] a) {
  float s = 0;
  for (float v : a) s += v * v;
  return sqrt(s / a.length);
}

float frameDist(float[] a, float[] b) {
  float s = 0;
  for (int k = 0; k < a.length; k++) s += sq(a[k] - b[k]);
  return sqrt(s / a.length);
}

// Average sweep values into BINS buckets
float[] downsample(float[] ys) {
  float[] out = new float[BINS];
  int n = ys.length;
  for (int k = 0; k < BINS; k++) {
    int a = k * n / BINS, b = max(a + 1, (k + 1) * n / BINS);
    float s = 0;
    for (int i = a; i < b; i++) s += ys[i];
    out[k] = s / (b - a);
  }
  return out;
}

// Open-begin DTW of template t against stream s (alignment must end at the last frame of s).
// Returns the mean frame distance along the best path.
float subDTW(float[][] t, float[][] s) {
  int n = t.length, m = s.length;
  float[] prevD = new float[m], curD = new float[m];
  int[] prevL = new int[m], curL = new int[m];

  for (int j = 0; j < m; j++) {
    prevD[j] = frameDist(t[0], s[j]); // free start anywhere in the stream
    prevL[j] = 1;
  }
  for (int i = 1; i < n; i++) {
    curD[0] = prevD[0] + frameDist(t[i], s[0]);
    curL[0] = prevL[0] + 1;
    for (int j = 1; j < m; j++) {
      float d = prevD[j - 1];
      int l = prevL[j - 1];
      if (prevD[j] < d) {
        d = prevD[j];
        l = prevL[j];
      }
      if (curD[j - 1] < d) {
        d = curD[j - 1];
        l = curL[j - 1];
      }
      curD[j] = d + frameDist(t[i], s[j]);
      curL[j] = l + 1;
    }
    float[] tD = prevD;
    prevD = curD;
    curD = tD;
    int[] tL = prevL;
    prevL = curL;
    curL = tL;
  }
  return prevD[m - 1] / prevL[m - 1];
}

// Best similarity per gesture slot for the current stream window
float[] similarities(ArrayList<Template> templates, float[][] stream, float noiseFloor, int slots) {
  float[] best = new float[slots];
  for (Template t : templates) {
    float d = subDTW(t.frames, stream);
    float v = constrain(1 - d / max(t.energy, noiseFloor), 0, 1);
    best[t.cls] = max(best[t.cls], v);
  }
  return best;
}

// Drop the quiet frames at the start/end of a recording (button held before/after the gesture)
float[][] trim(ArrayList<float[]> frames, float noise) {
  float peak = 0;
  for (float[] f : frames) peak = max(peak, rms(f));
  float gate = max(3 * noise, 0.15 * peak);
  int a = 0, b = frames.size() - 1;
  while (a <= b && rms(frames.get(a)) < gate) a++;
  while (b >= a && rms(frames.get(b)) < gate) b--;
  if (b - a + 1 < 3) return null;
  a = max(0, a - 1);
  b = min(frames.size() - 1, b + 1);
  float[][] out = new float[b - a + 1][];
  for (int i = a; i <= b; i++) out[i - a] = frames.get(i);
  return out;
}

void saveGestures() {
  JSONObject j = new JSONObject();
  j.setInt("bins", BINS);
  j.setFloat("threshold", threshold);
  JSONArray n = new JSONArray();
  for (String s : names) n.append(s);
  j.setJSONArray("names", n);
  JSONArray ts = new JSONArray();
  for (Template t : templates) {
    JSONObject o = new JSONObject();
    o.setInt("cls", t.cls);
    JSONArray fr = new JSONArray();
    for (float[] f : t.frames) {
      JSONArray row = new JSONArray();
      for (float v : f) row.append(round(v * 10) / 10.0);
      fr.append(row);
    }
    o.setJSONArray("frames", fr);
    ts.append(o);
  }
  j.setJSONArray("templates", ts);
  saveJSONObject(j, dataPath("gestures.json"));
}

void loadGestures() {
  File file = new File(dataPath("gestures.json"));
  if (!file.exists()) return;
  JSONObject j = loadJSONObject(file);
  if (j.getInt("bins", -1) != BINS) {
    toast("gestures.json uses a different BINS, ignored");
    return;
  }
  threshold = j.getFloat("threshold", threshold);
  JSONArray n = j.getJSONArray("names");
  for (int i = 0; i < min(n.size(), names.length); i++) names[i] = n.getString(i);
  JSONArray ts = j.getJSONArray("templates");
  for (int i = 0; i < ts.size(); i++) {
    JSONObject o = ts.getJSONObject(i);
    JSONArray fr = o.getJSONArray("frames");
    float[][] frames = new float[fr.size()][];
    for (int f = 0; f < fr.size(); f++) frames[f] = fr.getJSONArray(f).toFloatArray();
    int cls = o.getInt("cls");
    if (cls < names.length) templates.add(new Template(cls, frames));
  }
  toast("loaded " + templates.size() + " examples");
}
