#include "unity.h"
#include "control.h"
#include "board.h"
#include <math.h>

static Controller C;
static const float Q_ID[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static const float ZERO3[3] = { 0.0f, 0.0f, 0.0f };

void setUp(void) { Ctrl_Init(&C); }
void tearDown(void) {}

static void test_starts_off_with_centred_fins(void)
{
    float fin[4] = { 9.0f, 9.0f, 9.0f, 9.0f };
    const float w[3] = { 1.0f, 2.0f, 3.0f };
    TEST_ASSERT_EQUAL(CTRL_MODE_OFF, C.mode);
    Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, fin[i]); }
}

static void test_rate_damper_opposes_roll_rate(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };                    /* +1 rad/s roll */
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, ZERO3, w, FRAME_DT_S, fin);
#if CTRL_ROLL_LAW >= 1
    /* scheduled law at the fallback q (lowest gain), no trim */
    const float want = -(CTRL_ROLL_C / CTRL_ROLL_Q_FALLBACK) / (FIN_DEFLECT_DEG * 0.0174532925f);
#else
    const float want = -CTRL_RD_GAIN_ROLL;
#endif
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, want, fin[i]); }
}

static void test_rate_damper_uses_gyro_not_estimator(void)
{
    float fin[4];
    const float w_est[3] = { 0.0f, 0.0f, 50.0f };               /* garbage estimate */
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, w_est, ZERO3, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, fin[i]); }
}

static void test_rate_damper_saturates(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, -1000.0f };
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, ZERO3, w, FRAME_DT_S, fin);
#if CTRL_ROLL_LAW >= 1
    const float lim = CTRL_ROLL_TRAVEL_RAD / (FIN_DEFLECT_DEG * 0.0174532925f);   /* canard travel */
#else
    const float lim = CTRL_FIN_LIMIT;
#endif
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, lim, fin[i]); }
}

static void test_pitch_yaw_disabled(void)
{
    float fin[4];
    const float w[3] = { 3.0f, -3.0f, 0.0f };
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 50; k++) { Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fin[i]); }
}

static void test_full_mode_damps_roll_rate(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 20; k++) { Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(fin[i] < 0.0f);                         /* opposes +roll */
        TEST_ASSERT_TRUE(fin[i] >= -CTRL_FIN_LIMIT);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, fin[0], fin[i]);         /* pure roll: common mode */
    }
}

static void test_full_mode_quiescent_is_zero(void)
{
    float fin[4];
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 100; k++) { Ctrl_Update(&C, Q_ID, ZERO3, ZERO3, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fin[i]); }
}

static void test_bumpless_entry_preloads_actuator_filter(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin);
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, fin[i], C.delta_f[i]); }
    /* Steady-state preload: a constant input reproduces itself. */
    Biquad s = C.bq_delta[0];
    const float u = fin[0];
    float y = 0.0f;
    for (int k = 0; k < 5; k++) {
        y = C.b0 * u + s.z1;
        s.z1 = C.b1 * u - C.a1 * y + s.z2;
        s.z2 = C.b2 * u - C.a2 * y;
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, u, y);
    }
}

static void test_mode_change_resets_integral(void)
{
    C.e_I[2] = 0.2f;
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.e_I[2]);
}

#if CTRL_ROLL_LAW >= 1
/* Kp = c/q: gain halves when q doubles; floor at Q_MIN; fallback when invalid. */
static void test_roll_schedule_scales_with_q(void)
{
    Ctrl_SetQbar(&C, 10000.0f, true);
    const float d1 = Ctrl_RollLaw(&C, 1.0f, FRAME_DT_S, false);
    Ctrl_SetQbar(&C, 20000.0f, true);
    const float d2 = Ctrl_RollLaw(&C, 1.0f, FRAME_DT_S, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -CTRL_ROLL_C / 10000.0f, d1);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.5f * d1, d2);
    Ctrl_SetQbar(&C, 10.0f, true);                       /* below floor */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -CTRL_ROLL_C / CTRL_ROLL_Q_MIN, Ctrl_RollLaw(&C, 1.0f, FRAME_DT_S, false));
    Ctrl_SetQbar(&C, 1000.0f, false);                    /* invalid -> fallback */
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, -CTRL_ROLL_C / CTRL_ROLL_Q_FALLBACK, Ctrl_RollLaw(&C, 1.0f, FRAME_DT_S, false));
    Ctrl_SetQbar(&C, NAN, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, -CTRL_ROLL_C / CTRL_ROLL_Q_FALLBACK, Ctrl_RollLaw(&C, 1.0f, FRAME_DT_S, false));
}

/* Trim: dead band, rate limit, clamp; frozen while the output saturates. */
static void test_roll_trim_bounded(void)
{
    Ctrl_SetQbar(&C, 6000.0f, true);
    const float Kp = CTRL_ROLL_C / 6000.0f;
    const float band = CTRL_TRIM_BAND_K * CTRL_TRIM_DEADBAND_RAD / Kp;
    (void)Ctrl_RollLaw(&C, 0.9f * band, FRAME_DT_S, true);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.trim);                /* inside the band */
    for (int k = 0; k < 100; k++) { (void)Ctrl_RollLaw(&C, 2.0f, FRAME_DT_S, true); }   /* 1 s */
    TEST_ASSERT_TRUE(C.trim < 0.0f);
    TEST_ASSERT_TRUE(C.trim >= -(CTRL_TRIM_RATE * 1.0f) - 1e-6f);   /* rate limit */
    for (int k = 0; k < 2000; k++) { (void)Ctrl_RollLaw(&C, 2.0f, FRAME_DT_S, true); }
    TEST_ASSERT_TRUE(C.trim >= -CTRL_TRIM_CLAMP - 1e-6f);
    const float t0 = C.trim;
    (void)Ctrl_RollLaw(&C, 1000.0f, FRAME_DT_S, true);   /* saturated: frozen */
    TEST_ASSERT_EQUAL_FLOAT(t0, C.trim);
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    Ctrl_SetMode(&C, CTRL_MODE_OFF);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.trim);                /* reset on OFF */
}
#endif

#if CTRL_ROLL_LAW == 2
/* A roll plant for the observer tests: pdot = beta q delta(t - 2 frames) + f. */
typedef struct { float p, d[3]; } Plant;
static float plant_step(Plant *pl, float cmd, float q, float f)
{
    pl->p += FRAME_DT_S * ((0.559f * q * pl->d[0]) + f);
    pl->d[0] = pl->d[1]; pl->d[1] = pl->d[2]; pl->d[2] = cmd;
    return pl->p;
}
static float run(Plant *pl, int n, float q, float f, bool eso)
{
    float out = 0.0f;
    C.eso_enable = eso;
    for (int k = 0; k < n; k++) { out = Ctrl_RollLaw(&C, pl->p, FRAME_DT_S, true); (void)plant_step(pl, out, q, f); }
    return out;
}

/* SYSID-01: the observer cancels a constant disturbance torque within a second; the
 * trim integrator alone (observer off) is still far off */
static void test_eso_rejects_constant_disturbance(void)
{
    Ctrl_SetQbar(&C, 4000.0f, true);
    Plant a = { 0 }, b = { 0 };
    (void)run(&a, 200, 4000.0f, 40.0f, true);
    const float p_eso = a.p;
    Ctrl_Init(&C); Ctrl_SetQbar(&C, 4000.0f, true);
    (void)run(&b, 200, 4000.0f, 40.0f, false);
    TEST_ASSERT_TRUE(fabsf(p_eso) < 0.02f);              /* < 1.2 dps after 1 s */
    TEST_ASSERT_TRUE(fabsf(b.p) > (10.0f * fabsf(p_eso)));
}

/* entering the observer from the trim, and leaving it, keep the command continuous */
static void test_eso_bumpless_entry_and_exit(void)
{
    Ctrl_SetQbar(&C, 4000.0f, true);
    Plant a = { 0 };
    float out = run(&a, 400, 4000.0f, 20.0f, false);    /* trim builds up */
    TEST_ASSERT_TRUE(fabsf(C.trim) > 0.001f);
    C.eso_enable = true;
    float o2 = Ctrl_RollLaw(&C, a.p, FRAME_DT_S, true);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, out, o2);           /* < 0.12 deg step at entry */
    out = run(&a, 300, 4000.0f, 20.0f, true);
    C.eso_enable = false;
    o2 = Ctrl_RollLaw(&C, a.p, FRAME_DT_S, true);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, out, o2);           /* and at exit */
    TEST_ASSERT_FALSE(C.eso_init);
    TEST_ASSERT_TRUE(fabsf(C.trim) <= CTRL_TRIM_CLAMP + 1e-6f);
}

/* the excitation is a known input to the observer: it reaches the canard and is not cancelled */
static void test_eso_passes_excitation(void)
{
    Ctrl_SetQbar(&C, 4000.0f, true);
    Plant a = { 0 };
    (void)run(&a, 300, 4000.0f, 10.0f, true);
    const float t0 = C.trim;
    C.u_exc = 0.005f;
    const float out = Ctrl_RollLaw(&C, a.p, FRAME_DT_S, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.005f, out - (C.trim + (CTRL_ROLL_C / 4000.0f) * (0.0f - a.p)));
    (void)plant_step(&a, out, 4000.0f, 10.0f);
    for (int k = 0; k < 40; k++) { (void)plant_step(&a, Ctrl_RollLaw(&C, a.p, FRAME_DT_S, true), 4000.0f, 10.0f); }
    TEST_ASSERT_TRUE(fabsf(C.trim - t0) < 0.5f * 0.005f);   /* the cancellation term barely moves */
    C.u_exc = 0.0f;
}

/* outside BOOST/COAST the flight code leaves the observer off: no response from the
 * canards (a bench) then gives no drift */
static void test_eso_off_no_drift_without_torque(void)
{
    Ctrl_SetQbar(&C, 1000.0f, true);
    C.eso_enable = false;
    float out = 0.0f;
    for (int k = 0; k < 60000; k++) { out = Ctrl_RollLaw(&C, 0.0005f, FRAME_DT_S, true); }   /* 5 min, 0.03 dps residual bias */
    TEST_ASSERT_TRUE(fabsf(out) < 0.01f);
}
#endif

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_off_with_centred_fins);
    RUN_TEST(test_rate_damper_opposes_roll_rate);
    RUN_TEST(test_rate_damper_uses_gyro_not_estimator);
    RUN_TEST(test_rate_damper_saturates);
    RUN_TEST(test_pitch_yaw_disabled);
    RUN_TEST(test_full_mode_damps_roll_rate);
    RUN_TEST(test_full_mode_quiescent_is_zero);
    RUN_TEST(test_bumpless_entry_preloads_actuator_filter);
    RUN_TEST(test_mode_change_resets_integral);
#if CTRL_ROLL_LAW >= 1
    RUN_TEST(test_roll_schedule_scales_with_q);
    RUN_TEST(test_roll_trim_bounded);
#endif
#if CTRL_ROLL_LAW == 2
    RUN_TEST(test_eso_rejects_constant_disturbance);
    RUN_TEST(test_eso_bumpless_entry_and_exit);
    RUN_TEST(test_eso_passes_excitation);
    RUN_TEST(test_eso_off_no_drift_without_torque);
#endif
    return UNITY_END();
}
