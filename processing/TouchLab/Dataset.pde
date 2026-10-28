/* Labelled dataset: every recorded example is also saved as one CSV file
   data/dataset/<pattern>/<gesture>_<take>_<timestamp>.csv

   Columns: pattern, gesture, take, frame, t_ms, active, f<kHz>...
   - one row per sweep, full resolution (one column per frequency)
   - frame -1 is the baseline (resting curve) in use when the take was recorded
   - active = 1 for the frames kept as the gesture after trimming the quiet ends
   Every file has the same layout, so in Python:
     pd.concat(pd.read_csv(f) for f in Path("data/dataset").rglob("*.csv")) */

String safeName(String s) {
  String r = s.trim().replaceAll("[^A-Za-z0-9_-]+", "_");
  return r.length() == 0 ? "unnamed" : r;
}

String csvText(String s) {
  return s.replace(",", " ").replace("\"", "'");
}

// Next free take number for this gesture in this pattern's folder
int nextTake(File dir, String gestureFile) {
  int take = 0;
  String[] files = dir.list();
  if (files == null) return 1;
  for (String f : files) {
    if (!f.startsWith(gestureFile + "_")) continue;
    String rest = f.substring(gestureFile.length() + 1);
    if (rest.matches("\\d{3}_.*\\.csv")) take = max(take, int(rest.substring(0, 3)));
  }
  return take + 1;
}

// Saves a take, returns its path relative to the data folder (or "" on failure)
String saveTake(String gesture, ArrayList<Frame> frames, int a, int b, float[] xs) {
  String dirRel = "dataset/" + safeName(pattern);
  File dir = new File(dataPath(dirRel));
  dir.mkdirs();
  String g = safeName(gesture);
  int take = nextTake(dir, g);
  String rel = dirRel + "/" + g + "_" + nf(take, 3) + "_" + year() + nf(month(), 2) + nf(day(), 2)
    + "-" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".csv";

  PrintWriter out = createWriter(dataPath(rel));
  StringBuilder head = new StringBuilder("pattern,gesture,take,frame,t_ms,active");
  for (float x : xs) head.append(",f").append(nf(x, 0, 1));
  out.println(head);

  String prefix = csvText(pattern) + "," + csvText(gesture) + "," + take + ",";
  if (baselineRaw != null) out.println(prefix + "-1,0,0," + joinValues(baselineRaw));
  int t0 = frames.get(0).t;
  for (int i = 0; i < frames.size(); i++) {
    Frame fr = frames.get(i);
    out.println(prefix + i + "," + (fr.t - t0) + "," + (i >= a && i <= b ? 1 : 0) + "," + joinValues(fr.raw));
  }
  out.flush();
  out.close();
  return rel;
}

String joinValues(float[] v) {
  StringBuilder s = new StringBuilder();
  for (int i = 0; i < v.length; i++) {
    if (i > 0) s.append(',');
    s.append(round(v[i] * 100) / 100.0);
  }
  return s.toString();
}

void deleteTake(String rel) {
  if (rel == null || rel.length() == 0) return;
  File f = new File(dataPath(rel));
  if (f.exists()) f.delete();
}
