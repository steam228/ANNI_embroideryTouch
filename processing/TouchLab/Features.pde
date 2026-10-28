/* Interpretable features of one sweep, tracked over time:
   peak frequency - where the resonance is (kHz). Touch/capacitance tends to move it.
   peak height    - amplitude at the resonance.
   peak width     - frequency span above half height (between the curve's floor and its peak).
   mean level     - average amplitude over the whole sweep (area under the curve). */

final String[] FEATURE_NAMES = {"peak frequency", "peak height", "peak width", "mean level"};
final String[] FEATURE_UNITS = {" kHz", "", " kHz", ""};

float[] features(float[] xs, float[] ys) {
  int n = ys.length;
  int p = 0;
  float lo = ys[0], sum = 0;
  for (int i = 0; i < n; i++) {
    if (ys[i] > ys[p]) p = i;
    lo = min(lo, ys[i]);
    sum += ys[i];
  }

  // sub-step peak position from a parabola through the 3 points around the max
  float pos = p;
  if (p > 0 && p < n - 1) {
    float den = ys[p - 1] - 2 * ys[p] + ys[p + 1];
    if (den != 0) pos = p + constrain(0.5 * (ys[p - 1] - ys[p + 1]) / den, -0.5, 0.5);
  }

  float half = lo + (ys[p] - lo) / 2;
  int a = p, b = p;
  while (a > 0 && ys[a - 1] >= half) a--;
  while (b < n - 1 && ys[b + 1] >= half) b++;

  return new float[] {
    sampleAt(xs, pos),
    ys[p],
    abs(xs[a] - xs[b]),
    sum / n
  };
}

// Linear interpolation of xs at a fractional index
float sampleAt(float[] xs, float idx) {
  int i = constrain(floor(idx), 0, xs.length - 1);
  int j = min(i + 1, xs.length - 1);
  return lerp(xs[i], xs[j], idx - i);
}
