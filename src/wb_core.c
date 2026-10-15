/*
 * wb_core.c — noyau de pesée embarquée.
 *
 * À chaque cycle (50 Hz conseillé) :
 *   1. conversion des voies capteurs en kg, vote entre les voies A et B ;
 *   2. contrôle des conditions de mesure (au sol, freiné, immobile, à plat) ;
 *   3. au parking : moyenne glissante sur une fenêtre, rejetée si l'avion bouge ;
 *      chaque fenêtre stable remplace la mesure précédente (l'embarquement
 *      peut continuer après la mise sous tension) ;
 *   4. dès que l'avion quitte le parking, la dernière mesure est figée ; la
 *      masse sans carburant reste constante et le carburant vient du jaugeage.
 */
#include "wb.h"

#include <math.h>
#include <string.h>

#define DEG2RAD (3.14159265358979323846 / 180.0)
#define ALL_CAL_TEMP_REF 20.0

double wb_pct_mac(const wb_aircraft_t *ac, double x_m)
{
    return (x_m - ac->lemac_m) / ac->mac_m * 100.0;
}

double wb_x_of_mac(const wb_aircraft_t *ac, double pct)
{
    return ac->lemac_m + pct / 100.0 * ac->mac_m;
}

/* Point dans le polygone (masse, centrage) — lancer de rayon. */
int wb_in_envelope(const wb_aircraft_t *ac, double mass_kg, double cg_mac)
{
    int i, j, in = 0;
    for (i = 0, j = ac->n_env - 1; i < ac->n_env; j = i++) {
        double yi = ac->env_mass_kg[i], yj = ac->env_mass_kg[j];
        double xi = ac->env_cg_mac[i],  xj = ac->env_cg_mac[j];
        if ((yi > mass_kg) != (yj > mass_kg)) {
            double x = xi + (mass_kg - yi) * (xj - xi) / (yj - yi);
            if (cg_mac < x) in = !in;
        }
    }
    return in;
}

int wb_channel_kg(const wb_cal_t *c, int32_t raw, double temp_c, double *kg)
{
    double dt, counts;
    if (raw < 0 || raw > WB_ADC_MAX) return 0;
    dt = temp_c - ALL_CAL_TEMP_REF;
    counts = (double)raw - c->offset_counts - c->tc_zero_counts * dt;
    *kg = counts * c->gain_kg_per_count * (1.0 + c->tc_gain_ppm * 1e-6 * dt);
    return 1;
}

const char *wb_state_name(wb_state_t s)
{
    switch (s) {
    case WB_ST_SELFTEST:  return "AUTOTEST";
    case WB_ST_WAIT:      return "ATTENTE";
    case WB_ST_MEASURING: return "MESURE";
    case WB_ST_FROZEN:    return "FIGÉE";
    case WB_ST_FLIGHT:    return "VOL";
    case WB_ST_FAIL:      return "PANNE";
    }
    return "?";
}

static void reset_window(wb_ctx_t *c)
{
    int i;
    c->n = 0;
    c->s_mx = c->s_my = c->s_total = 0.0;
    c->w_pitch = c->w_roll = 0.0;
    c->win_alerts = 0;
    for (i = 0; i < WB_MAX_LEGS; i++) c->s_leg[i] = 0.0;
}

void wb_init(wb_ctx_t *ctx, const wb_aircraft_t *ac)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->ac = ac;
    ctx->state = WB_ST_SELFTEST;
    if (!ac || !ac->cal_loaded || wb_validate(ac, NULL, 0) != 0)
        ctx->state = WB_ST_FAIL;
    reset_window(ctx);
}

/* Charge de chaque jambe pour ce cycle. Renvoie les alertes capteurs. */
static uint32_t read_legs(const wb_aircraft_t *ac, const wb_inputs_t *in, double *leg_kg)
{
    uint32_t al = 0;
    int l, k;
    for (l = 0; l < ac->n_legs; l++) {
        const wb_leg_t *L = &ac->legs[l];
        double t = in->leg_temp_c[l];
        leg_kg[l] = 0.0;
        for (k = 0; k < L->n_points; k++) {
            int p = L->first_point + k;
            double a = 0.0, b = 0.0;
            int oka = wb_channel_kg(&ac->cal[2 * p],     in->raw[2 * p],     t, &a);
            int okb = wb_channel_kg(&ac->cal[2 * p + 1], in->raw[2 * p + 1], t, &b);
            if (oka && okb) {
                double big = fabs(a) > fabs(b) ? fabs(a) : fabs(b);
                double tol = ac->p.chan_tol_pct / 100.0 * big;
                if (tol < ac->p.chan_tol_kg) tol = ac->p.chan_tol_kg;
                if (fabs(a - b) > tol) {
                    /* Deux voies valides mais en désaccord : impossible de
                     * savoir laquelle croire, le point est perdu. */
                    al |= WB_AL_SENSOR_FAIL;
                } else {
                    leg_kg[l] += 0.5 * (a + b);
                }
            } else if (oka || okb) {
                leg_kg[l] += oka ? a : b;
                al |= WB_AL_SENSOR_DEGRADED;
            } else {
                al |= WB_AL_SENSOR_FAIL;
            }
        }
    }
    return al;
}

static void fuel(const wb_aircraft_t *ac, const wb_inputs_t *in, double *kg, double *mx)
{
    int i;
    *kg = 0.0;
    *mx = 0.0;
    for (i = 0; i < ac->n_tanks; i++) {
        double q = in->tank_kg[i] > 0.0 ? in->tank_kg[i] : 0.0;
        *kg += q;
        *mx += q * ac->tanks[i].x_m;
    }
}

/* Fenêtre complète et stable : calcul de la masse et du centrage. */
static void finalize(wb_ctx_t *c, const wb_inputs_t *in)
{
    const wb_aircraft_t *ac = c->ac;
    double n = (double)c->n;
    double N = c->s_total / n;
    double p = c->w_pitch / n * DEG2RAD, r = c->w_roll / n * DEG2RAD;
    double W, xc, yc, fob, fmx;
    int l;

    /* Sur un parking en pente, les trains ne portent que W·cos(pente) ;
     * et la gravité, qui n'est plus perpendiculaire au plancher, reporte
     * de la charge vers le bas de la pente (CG à hauteur h). */
    W  = N / (cos(p) * cos(r));
    xc = (c->s_mx / n) / N - ac->cg_height_m * tan(p);
    yc = (c->s_my / n) / N - ac->cg_height_m * tan(r);

    fuel(ac, in, &fob, &fmx);
    c->zfw_kg  = W - fob;
    c->zfw_mx  = W * xc - fmx;
    c->zfw_my  = W * yc;           /* carburant supposé symétrique */
    c->meas_fob_kg = fob;
    for (l = 0; l < ac->n_legs; l++) c->meas_leg[l] = c->s_leg[l] / n;
    c->meas_alerts  = c->win_alerts;
    c->meas_time_ms = in->time_ms;
    c->has_meas = 1;
}

void wb_step(wb_ctx_t *c, const wb_inputs_t *in, wb_outputs_t *out)
{
    const wb_aircraft_t *ac = c->ac;
    double leg[WB_MAX_LEGS];
    uint32_t sens_al, reasons = 0, alerts = 0;
    double pitch, roll;
    int l;

    memset(out, 0, sizeof *out);

    if (c->state == WB_ST_FAIL) {
        out->state = WB_ST_FAIL;
        return;
    }
    if (c->state == WB_ST_SELFTEST) {
        if (!c->started) { c->started = 1; c->t_start_ms = in->time_ms; }
        if (in->time_ms - c->t_start_ms < WB_SELFTEST_MS) {
            out->state = WB_ST_SELFTEST;
            return;
        }
        c->state = WB_ST_WAIT;
    }

    /* 1. Capteurs */
    sens_al = read_legs(ac, in, leg);
    {
        /* Plausibilité : un avion vide pèse bien plus du quart de sa MZFW */
        double tot = 0.0;
        for (l = 0; l < ac->n_legs; l++) tot += leg[l];
        if (in->on_ground && (tot < 0.25 * ac->mzfw_kg || tot > 1.25 * ac->mrw_kg)) sens_al |= WB_AL_SENSOR_FAIL;
    }
    alerts |= sens_al;
    if (sens_al & WB_AL_SENSOR_FAIL) reasons |= WB_WAIT_SENSOR;

    /* 2. Conditions */
    if (fabs(in->pitch_deg[0] - in->pitch_deg[1]) > ac->p.incl_tol_deg
        || fabs(in->roll_deg[0] - in->roll_deg[1]) > ac->p.incl_tol_deg) {
        reasons |= WB_WAIT_INCLINOMETER;
        alerts  |= WB_AL_INCL_DISAGREE;
    }
    pitch = 0.5 * (in->pitch_deg[0] + in->pitch_deg[1]);
    roll  = 0.5 * (in->roll_deg[0] + in->roll_deg[1]);
    if (sqrt(pitch * pitch + roll * roll) > ac->p.max_slope_deg) reasons |= WB_WAIT_SLOPE;
    if (!in->on_ground)     reasons |= WB_WAIT_AIRBORNE;
    if (!in->parking_brake) reasons |= WB_WAIT_BRAKE;
    if (in->groundspeed_kt > ac->p.max_gs_kt) reasons |= WB_WAIT_MOVING;
    if (in->ext_contact)    reasons |= WB_WAIT_EXT_CONTACT;

    /* 3. Mesure ou maintien */
    if (reasons == 0) {
        int i;
        double tot = 0.0;
        c->state = WB_ST_MEASURING;
        for (l = 0; l < ac->n_legs; l++) {
            tot += leg[l];
            c->s_mx += leg[l] * ac->legs[l].x_m;
            c->s_my += leg[l] * ac->legs[l].y_m;
            c->s_leg[l] += leg[l];
        }
        c->w_total[c->n] = tot;
        c->s_total += tot;
        c->w_pitch += pitch;
        c->w_roll  += roll;
        c->win_alerts |= sens_al;
        c->n++;
        if (c->n >= ac->p.window_samples) {
            /* Stabilité : la moyenne de la 2e moitié de la fenêtre ne doit
             * pas s'écarter de celle de la 1re. Insensible au bruit des
             * capteurs, sensible à un chargement en cours. */
            int h = c->n / 2;
            double m1 = 0.0, m2 = 0.0;
            for (i = 0; i < h; i++)    m1 += c->w_total[i];
            for (i = h; i < c->n; i++) m2 += c->w_total[i];
            m1 /= h;
            m2 /= (c->n - h);
            if (fabs(m2 - m1) > ac->p.stab_pct / 100.0 * ac->mtow_kg) {
                c->unstable = 1;   /* quelqu'un charge ou l'avion bouge */
            } else {
                c->unstable = 0;
                finalize(c, in);
            }
            reset_window(c);
        }
        if (c->unstable) reasons |= WB_WAIT_UNSTABLE;
    } else {
        reset_window(c);
        c->unstable = 0;
        if (c->has_meas) c->state = in->on_ground ? WB_ST_FROZEN : WB_ST_FLIGHT;
        else             c->state = WB_ST_WAIT;
    }

    out->state = c->state;
    out->wait_reasons = reasons;
    out->progress_pct = c->has_meas ? 100 : (c->n * 100) / ac->p.window_samples;

    if (c->has_meas) {
        double fob, fmx, gw, mx;
        fuel(ac, in, &fob, &fmx);
        gw = c->zfw_kg + fob;
        mx = c->zfw_mx + fmx;
        out->mass_valid = 1;
        out->meas_age_ms = in->time_ms - c->meas_time_ms;
        out->fob_kg     = fob;
        out->gw_kg      = gw;
        out->cg_mac     = wb_pct_mac(ac, mx / gw);
        out->cg_y_m     = c->zfw_my / gw;
        out->zfw_kg     = c->zfw_kg;
        out->zfw_cg_mac = wb_pct_mac(ac, c->zfw_mx / c->zfw_kg);
        for (l = 0; l < ac->n_legs; l++) out->leg_kg[l] = c->meas_leg[l];

        alerts |= c->meas_alerts & WB_AL_SENSOR_DEGRADED;
        if (in->on_ground && gw > ac->mrw_kg) alerts |= WB_AL_OVER_MRW;
        if (c->state != WB_ST_MEASURING && gw > ac->mtow_kg) alerts |= WB_AL_OVER_MTOW;
        if (c->zfw_kg > ac->mzfw_kg) alerts |= WB_AL_OVER_MZFW;
        if (!wb_in_envelope(ac, gw, out->cg_mac)) alerts |= WB_AL_CG_OUT;
        if (fabs(out->cg_y_m) > ac->p.lat_cg_max_m) alerts |= WB_AL_LATERAL;

        if (in->loadsheet_zfw_kg > 0.0) {
            out->xcheck_dmass_pct = (c->zfw_kg - in->loadsheet_zfw_kg) / in->loadsheet_zfw_kg * 100.0;
            out->xcheck_dcg_mac   = out->zfw_cg_mac - in->loadsheet_zfw_cg_mac;
            if (fabs(out->xcheck_dmass_pct) > ac->p.xcheck_mass_pct) alerts |= WB_AL_XCHECK_MASS;
            if (fabs(out->xcheck_dcg_mac) > ac->p.xcheck_cg_mac)     alerts |= WB_AL_XCHECK_CG;
        }
    } else {
        for (l = 0; l < ac->n_legs; l++) out->leg_kg[l] = leg[l];
    }
    out->alerts = alerts;
}
