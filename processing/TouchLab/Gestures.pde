/* Dynamic gesture recognition: segment + banded DTW + nearest neighbour.

   Frames: each sweep is reduced to BINS values and the resting baseline is subtracted,
   so a frame says how the curve has moved away from rest (shape AND intensity).

   Segments: a gesture starts when the frame energy rises above ON and ends after
   the signal has stayed below OFF for gapMs. The whole segment is classified once,
   so holding a touch is one gesture, not a stream of detections.

   Comparison (segment vs each recorded example of the current pattern):
   - every frame also carries its velocity (change since the previous frame),
     so rise/fall speed and rhythm count, not only how strong the signal is
   - DTW aligns the two sequences but may only warp within BAND around the
     tempo-scaled diagonal: a tap cannot be stretched into a long hold
   Decision, pattern first:
   1. pattern score: both sequences are scaled to the same energy before DTW, so only
      the shape over time counts (how hard you press does not), then scaled down by
      the duration mismatch (DUR_WEIGHT). Threshold applies to this score.
   2. intensity score: ratio of the two energies (1 = same strength).
      Only used when gestures' pattern scores are within PATTERN_MARGIN of the best:
      among those, the closest intensity wins.
   Score per gesture = its best example (1-nearest-neighbour). */

final int BINS = 32;

final float VEL_W = 1.5;       // weight of the velocity part of each frame
final float BAND = 0.2;        // allowed warping, as a fraction of the sequence length
final float DUR_WEIGHT = 0.5;  // 0 = duration ignored, 1 = 2x longer halves the score
final int PRE_FRAMES = 2;      // frames kept before the onset
float PATTERN_MARGIN = 0.12;  // pattern scores closer than this -> intensity decides

class Template {
  int cls;
  String pattern;
  String file; // dataset take this example came from (relative to data/), "" if none
  float[][] frames; // BINS, minus baseline
  float[][] seq;    // frames + velocity, used for matching
  float[] env;      // energy per frame, for the match view
  float energy;

  Template(int cls, String pattern, String file, float[][] frames) {
    this.cls = cls;
    this.pattern = pattern;
    this.file = file;
    this.frames = frames;
    seq = seqFeatures(frames);
    env = envelope(frames);
    energy = meanEnergy(seq);
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

// Each frame followed by its velocity (change since the previous frame)
float[][] seqFeatures(float[][] frames) {
  float[][] out = new float[frames.length][2 * BINS];
  for (int t = 0; t < frames.length; t++) {
    for (int k = 0; k < BINS; k++) {
      float prev = t > 0 ? frames[t - 1][k] : 0;
      out[t][k] = frames[t][k];
      out[t][BINS + k] = VEL_W * (frames[t][k] - prev);
    }
  }
  return out;
}

float[] envelope(float[][] frames) {
  float[] e = new float[frames.length];
  for (int t = 0; t < frames.length; t++) e[t] = rms(frames[t]);
  return e;
}

float meanEnergy(float[][] seq) {
  float e = 0;
  for (float[] f : seq) e += rms(f);
  return e / seq.length;
}

// DTW restricted to a band around the tempo-scaled diagonal.
// Diagonal steps count double (symmetric step pattern), so dividing by n + m
// gives a fair mean frame distance whatever the path.
float bandedDTW(float[][] a, float[][] b) {
  int n = a.length, m = b.length;
  int r = max(2, ceil(BAND * max(n, m)));
  float INF = Float.MAX_VALUE / 4;
  float[][] D = new float[n + 1][m + 1];
  for (float[] row : D) java.util.Arrays.fill(row, INF);
  D[0][0] = 0;
  for (int i = 1; i <= n; i++) {
    int c = round((float) i * m / n);
    for (int j = max(1, c - r); j <= min(m, c + r); j++) {
      float d = frameDist(a[i - 1], b[j - 1]);
      D[i][j] = min(D[i - 1][j - 1] + 2 * d, min(D[i - 1][j] + d, D[i][j - 1] + d));
    }
  }
  return D[n][m] / (n + m);
}

float[][] scaled(float[][] seq, float by) {
  float[][] out = new float[seq.length][];
  for (int t = 0; t < seq.length; t++) {
    out[t] = new float[seq[t].length];
    for (int k = 0; k < seq[t].length; k++) out[t][k] = seq[t][k] / by;
  }
  return out;
}

class Match {
  float[] pattern;   // per slot, 0..1 (thresholded)
  float[] intensity; // per slot, 0..1
  Template[] best;   // per slot, the example that gave the pattern score
  int winner = -1;   // slot chosen (before thresholding)
  boolean byIntensity = false; // true if intensity broke a near-tie
}

// Score a whole segment against every example of the current pattern
Match classify(float[][] segment, ArrayList<Template> active, int slots) {
  Match m = new Match();
  m.pattern = new float[slots];
  m.intensity = new float[slots];
  m.best = new Template[slots];

  float[][] s = seqFeatures(segment);
  float eS = max(meanEnergy(s), 1e-6);
  float[][] sN = scaled(s, eS);
  for (Template t : active) {
    float d = bandedDTW(scaled(t.seq, max(t.energy, 1e-6)), sN);
    float ratio = (float) min(t.seq.length, s.length) / max(t.seq.length, s.length);
    float p = constrain(1 - d, 0, 1) * lerp(1, ratio, DUR_WEIGHT);
    float in = min(t.energy, eS) / max(t.energy, eS);
    int c = t.cls;
    // best example per gesture: by pattern, near-ties by intensity
    boolean better = m.best[c] == null || p > m.pattern[c] + PATTERN_MARGIN
      || (p > m.pattern[c] - PATTERN_MARGIN && in > m.intensity[c]);
    if (better) {
      m.pattern[c] = p;
      m.intensity[c] = in;
      m.best[c] = t;
    }
  }

  // winner: best pattern; gestures within the margin of it are decided by intensity
  float top = 0;
  for (int c = 0; c < slots; c++) top = max(top, m.pattern[c]);
  int byPattern = -1;
  for (int c = 0; c < slots; c++) {
    if (m.best[c] == null || m.pattern[c] < top - PATTERN_MARGIN) continue;
    if (byPattern < 0 || m.pattern[c] > m.pattern[byPattern]) byPattern = c;
    if (m.winner < 0 || m.intensity[c] > m.intensity[m.winner]) m.winner = c;
  }
  m.byIntensity = m.winner != byPattern;
  return m;
}

// Same start/end rule for recordings and live segments: from PRE_FRAMES before the first
// frame above on, to one frame after the last frame above off. null if nothing crosses on.
int[] activeRange(ArrayList<float[]> frames, float on, float off) {
  int a = -1, b = -1;
  for (int i = 0; i < frames.size(); i++) {
    float e = rms(frames.get(i));
    if (a < 0 && e > on) a = i;
    if (a >= 0 && e > off) b = i;
  }
  if (a < 0 || b - a + 1 < 2) return null;
  return new int[] {max(0, a - PRE_FRAMES), min(frames.size() - 1, b + 1)};
}

// Longest quiet stretch inside a recording, in ms (a pause longer than gapMs would split it live)
int longestPause(ArrayList<Frame> frames, int a, int b, float off) {
  int longest = 0, quietFrom = -1;
  for (int i = a; i <= b; i++) {
    boolean quietNow = rms(frames.get(i).f) <= off;
    if (quietNow && quietFrom < 0) quietFrom = frames.get(i).t;
    if (!quietNow && quietFrom >= 0) {
      longest = max(longest, frames.get(i).t - quietFrom);
      quietFrom = -1;
    }
  }
  return longest;
}

void saveGestures() {
  JSONObject j = new JSONObject();
  j.setInt("bins", BINS);
  j.setFloat("threshold", threshold);
  j.setInt("gapMs", gapMs);
  j.setFloat("margin", PATTERN_MARGIN);
  j.setString("pattern", pattern);
  JSONArray n = new JSONArray();
  for (String s : names) n.append(s);
  j.setJSONArray("names", n);
  JSONArray ts = new JSONArray();
  for (Template t : templates) {
    JSONObject o = new JSONObject();
    o.setInt("cls", t.cls);
    o.setString("pattern", t.pattern);
    o.setString("file", t.file);
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
  gapMs = j.getInt("gapMs", gapMs);
  PATTERN_MARGIN = j.getFloat("margin", PATTERN_MARGIN);
  pattern = j.getString("pattern", pattern);
  JSONArray n = j.getJSONArray("names");
  for (int i = 0; i < min(n.size(), names.length); i++) names[i] = n.getString(i);
  JSONArray ts = j.getJSONArray("templates");
  for (int i = 0; i < ts.size(); i++) {
    JSONObject o = ts.getJSONObject(i);
    JSONArray fr = o.getJSONArray("frames");
    float[][] frames = new float[fr.size()][];
    for (int f = 0; f < fr.size(); f++) frames[f] = fr.getJSONArray(f).toFloatArray();
    int cls = o.getInt("cls");
    if (cls < names.length && frames.length > 0) {
      templates.add(new Template(cls, o.getString("pattern", ""), o.getString("file", ""), frames));
    }
  }
  toast("loaded " + templates.size() + " examples");
}
