// Host test of the REAL src/sin_cos_angle_transform.h against SensESP stubs.
#include <cstdio>
#include <cmath>
#include <vector>
#include "sin_cos_angle_transform.h"

using sensesp::SinCosAngle;
static int failures = 0;
static void check(bool ok, const char* what, double got = 0, double want = 0) {
  if (ok) { printf("  PASS  %s\n", what); }
  else { printf("  FAIL  %s   (got %.6f, want %.6f)\n", what, got, want); failures++; }
}
static double deg(double r) { return r * 180.0 / M_PI; }
static double rad(double d) { return d * M_PI / 180.0; }

struct Rig {
  SinCosAngle* a;
  std::vector<float> out;
  Rig(int samples, float sign = 1.0f, float offset = 0.0f) {
    a = new SinCosAngle(1.0f, offset, 0.05f, sign, samples);
    a->on_emit = [this](float v) { out.push_back(v); };
  }
  void feed(double angle_deg) {   // one full sin/cos pair, as the ADS delivers it
    a->sin_input().set((float)sin(rad(angle_deg)));
    a->cos_input().set((float)cos(rad(angle_deg)));
  }
  void feed_raw(float s, float c) {
    a->sin_input().set(s);
    a->cos_input().set(c);
  }
  float last() const { return out.empty() ? NAN : out.back(); }
};

int main() {
  // ---- A. full-circle accuracy, no smoothing --------------------------------
  {
    Rig r(1);
    double worst = 0; int worst_at = 0;
    for (int d = 0; d < 360; d++) {
      r.out.clear();
      r.feed(d);
      double want = d > 180 ? d - 360 : d;          // (-180, 180]
      double err = fabs(deg(r.last()) - want);
      if (err > 180) err = fabs(err - 360);
      if (err > worst) { worst = err; worst_at = d; }
    }
    printf("A. full circle, samples=1\n");
    check(worst < 0.01, "every degree within 0.01 deg", worst, 0);
    if (worst >= 0.01) printf("     worst at %d deg\n", worst_at);
  }

  // ---- B. the wrap bug: smoothing through dead astern -----------------------
  {
    printf("B. smoothing across the +/-pi wrap (was: read dead ahead)\n");
    const double jitter[] = {178, -179, 179, -178, 180};
    Rig r(5);
    for (double d : jitter) r.feed(d);
    double got = fabs(deg(r.last()));

    // The same samples through the old angle-domain average, for contrast.
    double sum = 0;
    for (double d : jitter) sum += (d > 180 ? d - 360 : d);
    double old_mean = sum / 5.0;
    double old_err = fabs(fabs(old_mean) - 180.0);
    double new_err = fabs(got - 180.0);
    printf("     angle-domain mean = %.1f deg (%.0f deg off); vector mean = %.1f deg (%.1f deg off)\n",
           old_mean, old_err, got, new_err);
    check(new_err < 5.0, "vector mean is within 5 deg of the true direction", new_err, 0);
    check(old_err > 90.0, "angle-domain mean is more than 90 deg wrong", old_err, 90);
  }

  // ---- C. one emit per sin/cos pair ----------------------------------------
  {
    printf("C. emit count\n");
    Rig r(1);
    for (int i = 0; i < 20; i++) r.feed(45);
    check(r.out.size() == 20, "20 pairs in, 20 angles out", (double)r.out.size(), 20);
  }

  // ---- D. NaN does not poison the average permanently ----------------------
  {
    printf("D. NaN recovery\n");
    Rig r(5);
    for (int i = 0; i < 5; i++) r.feed(90);
    bool good_before = isfinite(r.last());
    r.feed_raw(NAN, 0.0f);
    bool nan_emitted = !isfinite(r.last());
    // The upstream raw-volt gate in main.cpp relies on this: NaN must clear the
    // window, so the very first sample after recovery is already correct and
    // no pre-fault sample blends into it.
    r.feed(0);
    bool first_clean = isfinite(r.last()) && fabs(deg(r.last())) < 0.01;
    for (int i = 0; i < 5; i++) r.feed(90);
    bool recovered = isfinite(r.last()) && fabs(deg(r.last()) - 90) < 0.01;
    check(good_before, "finite before the NaN");
    check(nan_emitted, "NaN in -> NaN out");
    check(first_clean, "window cleared: first sample after NaN is exact", deg(r.last()), 0);
    check(recovered, "recovers to 90 deg afterwards", deg(r.last()), 90);
  }

  // ---- E. dead sensor: both channels at 0 V, centred ------------------------
  {
    printf("E. dead transducer seen by the magnitude guard alone\n");
    Rig r(1);
    // 0 V on both channels, through the default centering (V-4.0)/1.5
    float centred = (0.0f - 4.0f) / 1.5f;
    r.feed_raw(centred, centred);
    check(isfinite(r.last()),
          "magnitude guard does NOT catch it (why the raw-volt gate exists)",
          deg(r.last()), 0);
    printf("     reports a steady %.1f deg — main.cpp turns such volts into NaN before they get here\n",
           deg(r.last()));
  }

  // ---- F. direction sign and offset ----------------------------------------
  {
    printf("F. sign and offset\n");
    Rig flip(1, -1.0f);
    flip.feed(45);
    check(fabs(deg(flip.last()) + 45) < 0.01, "sin_sign=-1 mirrors 45 -> -45", deg(flip.last()), -45);

    Rig off(1, 1.0f, (float)rad(90));
    off.feed(45);
    check(fabs(deg(off.last()) - 135) < 0.01, "offset +90 shifts 45 -> 135", deg(off.last()), 135);

    Rig wrap(1, 1.0f, (float)rad(90));
    wrap.feed(170);   // 170 + 90 = 260 -> must wrap to -100
    check(fabs(deg(wrap.last()) + 100) < 0.01, "offset wraps past pi correctly", deg(wrap.last()), -100);

    Rig neg(1, 1.0f, (float)rad(-90));
    neg.feed(-170);   // -170 - 90 = -260 -> must wrap to +100
    check(fabs(deg(neg.last()) - 100) < 0.01, "negative offset wraps past -pi correctly", deg(neg.last()), 100);

    // A fat-fingered offset must not hang the board (the old while-loop did).
    Rig huge(1, 1.0f, 1e30f);
    huge.feed(45);
    check(isfinite(huge.last()) && fabs(huge.last()) <= M_PI + 1e-6,
          "offset 1e30 still yields a finite angle in [-pi, pi]", huge.last(), 0);
  }

  // ---- G. samples config round-trip ----------------------------------------
  {
    printf("G. samples config\n");
    SinCosAngle a(1.0f, 0.0f, 0.05f, 1.0f, 5);
    JsonObject j;
    a.to_json(j);
    check(j["samples"].is<int>(), "samples serialised as an integer");
    JsonObject in;
    in["samples"] = 999;              // out of range
    a.from_json(in);
    JsonObject back;
    a.to_json(back);
    check((int)back["samples"] == SinCosAngle::kMaxSamples, "999 clamped to 32",
          (double)(int)back["samples"], SinCosAngle::kMaxSamples);
    JsonObject zero;
    zero["samples"] = 0;
    a.from_json(zero);
    a.to_json(back);
    check((int)back["samples"] == 1, "0 clamped to 1 (no modulo by zero)",
          (double)(int)back["samples"], 1);
    // legacy config without "samples" must not disturb the current value
    JsonObject legacy;
    legacy["gain"] = 1.0f;
    a.from_json(legacy);
    a.to_json(back);
    check((int)back["samples"] == 1, "missing samples key leaves it alone",
          (double)(int)back["samples"], 1);
  }

  // ---- H. sign and gain are sanitised --------------------------------------
  {
    printf("H. sign/gain sanitising\n");
    SinCosAngle zero_sign(1.0f, 0.0f, 0.05f, 0.0f, 1);
    float got = NAN;
    zero_sign.on_emit = [&](float v) { got = v; };
    zero_sign.sin_input().set((float)sin(rad(45)));
    zero_sign.cos_input().set((float)cos(rad(45)));
    check(fabs(deg(got) - 45) < 0.01, "sin_sign 0 is treated as +1", deg(got), 45);

    JsonObject j;
    j["sin_sign"] = -0.3f;   // any negative value means "flip"
    zero_sign.from_json(j);
    zero_sign.sin_input().set((float)sin(rad(45)));
    zero_sign.cos_input().set((float)cos(rad(45)));
    check(fabs(deg(got) + 45) < 0.01, "sin_sign -0.3 is treated as -1", deg(got), -45);

    SinCosAngle zero_gain(0.0f, 0.0f, 0.05f, 1.0f, 1);
    float g = NAN;
    zero_gain.on_emit = [&](float v) { g = v; };
    zero_gain.sin_input().set((float)sin(rad(30)));
    zero_gain.cos_input().set((float)cos(rad(30)));
    check(isfinite(g) && fabs(deg(g) - 30) < 0.01, "gain 0 falls back to 1 (no permanent NaN)", deg(g), 30);
  }

  printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
