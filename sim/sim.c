#include "sim.h"

#include <math.h>
#include <string.h>

#define DEG2RAD (3.14159265358979323846 / 180.0)

void sim_default(sim_truth_t *t)
{
    memset(t, 0, sizeof *t);
    t->temp_c = 20.0;
    t->noise_kg = 5.0;
    t->fault_ch = -1;
    t->seed = 12345u;
}

static double uniform(uint32_t *s)
{
    /* xorshift32 : déterministe, sans dépendance */
    uint32_t x = *s ? *s : 1u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return ((double)x + 1.0) / 4294967297.0;
}

static double gauss(uint32_t *s)
{
    double u1 = uniform(s), u2 = uniform(s);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);
}

static int is_wing(const char *name)
{
    return strncmp(name, "WING", 4) == 0;
}

void sim_split_fuel(const wb_aircraft_t *ac, double fuel_kg, double *tank_kg)
{
    int i, n_wing = 0;
    double wing_cap = 0.0, rest;
    for (i = 0; i < ac->n_tanks; i++) {
        tank_kg[i] = 0.0;
        if (is_wing(ac->tanks[i].name)) { n_wing++; wing_cap += ac->tanks[i].capacity_kg; }
    }
    rest = fuel_kg;
    if (n_wing) {
        double w = rest < wing_cap ? rest : wing_cap;
        for (i = 0; i < ac->n_tanks; i++)
            if (is_wing(ac->tanks[i].name)) tank_kg[i] = w * ac->tanks[i].capacity_kg / wing_cap;
        rest -= w;
    }
    for (i = 0; i < ac->n_tanks && rest > 0.0; i++) {
        if (is_wing(ac->tanks[i].name)) continue;
        tank_kg[i] = rest < ac->tanks[i].capacity_kg ? rest : ac->tanks[i].capacity_kg;
        rest -= tank_kg[i];
    }
}

void sim_truth_gw(const wb_aircraft_t *ac, const sim_truth_t *t, double *gw, double *cg_mac)
{
    double tank[WB_MAX_TANKS], fob = 0.0, fmx = 0.0;
    int i;
    sim_split_fuel(ac, t->fuel_kg, tank);
    for (i = 0; i < ac->n_tanks; i++) { fob += tank[i]; fmx += tank[i] * ac->tanks[i].x_m; }
    *gw = t->zfw_kg + fob;
    *cg_mac = wb_pct_mac(ac, (t->zfw_kg * wb_x_of_mac(ac, t->zfw_cg_mac) + fmx) / *gw);
}

/* Résout 3x3 par Cramer. */
static int solve3(double m[3][3], const double b[3], double x[3])
{
    double d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    int c, r;
    if (fabs(d) < 1e-12) return 0;
    for (c = 0; c < 3; c++) {
        double mc[3][3];
        memcpy(mc, m, sizeof mc);
        for (r = 0; r < 3; r++) mc[r][c] = b[r];
        x[c] = (mc[0][0] * (mc[1][1] * mc[2][2] - mc[1][2] * mc[2][1])
              - mc[0][1] * (mc[1][0] * mc[2][2] - mc[1][2] * mc[2][0])
              + mc[0][2] * (mc[1][0] * mc[2][1] - mc[1][1] * mc[2][0])) / d;
    }
    return 1;
}

void sim_fill(const wb_aircraft_t *ac, sim_truth_t *t, wb_inputs_t *in)
{
    double gw, cg, x, y, N, p, r, b[3], lam[3], m[3][3] = {{0}}, leg[WB_MAX_LEGS];
    int l, k;

    sim_truth_gw(ac, t, &gw, &cg);
    sim_split_fuel(ac, t->fuel_kg, in->tank_kg);
    x = wb_x_of_mac(ac, cg);
    y = t->cg_y_m * t->zfw_kg / gw;
    p = t->pitch_deg * DEG2RAD;
    r = t->roll_deg * DEG2RAD;

    /* Efforts sur les trains : équilibre des forces et des deux moments.
     * Avec plus de 3 jambes le problème est hyperstatique : on prend la
     * solution de norme minimale, suffisante pour le banc logiciel. */
    N = in->on_ground ? gw * cos(p) * cos(r) : 0.0;
    b[0] = N;
    b[1] = N * (x + ac->cg_height_m * tan(p));
    b[2] = N * (y + ac->cg_height_m * tan(r));
    for (l = 0; l < ac->n_legs; l++) {
        double v[3] = {1.0, ac->legs[l].x_m, ac->legs[l].y_m};
        int i, j;
        for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) m[i][j] += v[i] * v[j];
    }
    if (!solve3(m, b, lam)) lam[0] = lam[1] = lam[2] = 0.0;
    for (l = 0; l < ac->n_legs; l++)
        leg[l] = lam[0] + lam[1] * ac->legs[l].x_m + lam[2] * ac->legs[l].y_m;

    for (l = 0; l < ac->n_legs; l++) {
        const wb_leg_t *L = &ac->legs[l];
        double dt = t->temp_c - 20.0;
        in->leg_temp_c[l] = t->temp_c;
        for (k = 0; k < L->n_points; k++) {
            int pt = L->first_point + k, ab;
            for (ab = 0; ab < 2; ab++) {
                int ch = 2 * pt + ab;
                const wb_cal_t *c = &ac->cal[ch];
                double kg = leg[l] / L->n_points + t->noise_kg * gauss(&t->seed);
                double raw;
                if (ch == t->fault_ch) kg += t->fault_kg;
                raw = c->offset_counts + c->tc_zero_counts * dt
                    + kg / (c->gain_kg_per_count * (1.0 + c->tc_gain_ppm * 1e-6 * dt));
                in->raw[ch] = (int32_t)floor(raw + 0.5);
                if (ch == t->fault_ch && t->fault_dead) in->raw[ch] = -1;
            }
        }
    }
    in->pitch_deg[0] = in->pitch_deg[1] = t->pitch_deg;
    in->roll_deg[0]  = in->roll_deg[1]  = t->roll_deg;
}
