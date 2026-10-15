/*
 * test_wb.c — tests du noyau sur les 4 types d'avion fournis.
 * Chaque test simule les capteurs (sim.c) pour une masse connue et vérifie
 * ce que le noyau en déduit.
 */
#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0, g_count = 0;

#define CHECK(cond, ...) do { g_count++; if (!(cond)) { g_fail++; \
    printf("  ÉCHEC %s:%d : ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const char *TYPES[] = {"NB180", "NB189", "TP70", "WB4"};
#define N_TYPES 4

static void load(wb_aircraft_t *ac, const char *type)
{
    char path[128], err[256];
    snprintf(path, sizeof path, "configs/types/%s.acf", type);
    if (wb_load_type_file(path, ac, err, sizeof err) != 0) { printf("  %s : %s\n", path, err); g_fail++; return; }
    snprintf(path, sizeof path, "configs/tails/DEMO-%s.cal", type);
    if (wb_load_cal_file(path, ac, err, sizeof err) != 0) { printf("  %s : %s\n", path, err); g_fail++; }
}

/* Banc : un contexte, une vérité, des entrées. */
typedef struct {
    wb_aircraft_t ac;
    wb_ctx_t      ctx;
    sim_truth_t   tr;
    wb_inputs_t   in;
    wb_outputs_t  out;
} bench_t;

static bench_t B;

static void bench_start(const char *type)
{
    double lo = 1e9, hi = -1e9;
    int i;
    memset(&B, 0, sizeof B);
    load(&B.ac, type);
    sim_default(&B.tr);
    B.tr.zfw_kg = 0.85 * B.ac.mzfw_kg;
    for (i = 0; i < B.ac.n_env; i++) {
        if (B.ac.env_cg_mac[i] < lo) lo = B.ac.env_cg_mac[i];
        if (B.ac.env_cg_mac[i] > hi) hi = B.ac.env_cg_mac[i];
    }
    B.tr.zfw_cg_mac = 0.5 * (lo + hi);
    B.tr.fuel_kg = 0.12 * B.ac.mtow_kg;
    B.in.on_ground = 1;
    B.in.parking_brake = 1;
    wb_init(&B.ctx, &B.ac);
}

/* Avance de `seconds` à 50 Hz. */
static void run(double seconds)
{
    int i, n = (int)(seconds * 50.0 + 0.5);
    for (i = 0; i < n; i++) {
        B.in.time_ms += 20;
        sim_fill(&B.ac, &B.tr, &B.in);
        wb_step(&B.ctx, &B.in, &B.out);
    }
}

/* Temps pour l'autotest et deux fenêtres complètes */
static double settle(void) { return WB_SELFTEST_MS / 1000.0 + 2.0 * B.ac.p.window_samples / 50.0 + 0.1; }

/* ------------------------------------------------------------------------ */

static void test_configs_load(void)
{
    int t;
    printf("chargement des 4 types et de leurs étalonnages\n");
    for (t = 0; t < N_TYPES; t++) {
        bench_start(TYPES[t]);
        CHECK(B.ac.cal_loaded, "%s : étalonnage non chargé", TYPES[t]);
        CHECK(B.ctx.state == WB_ST_SELFTEST, "%s : le noyau refuse la configuration", TYPES[t]);
    }
}

static void test_accuracy_all_types(void)
{
    /* Cas variés : masse, centrage, carburant, pente, dévers, température */
    static const double cases[][6] = {
        /* zfw %MZFW, ΔCG %CMA, carburant %MTOW, tangage°, roulis°, °C */
        {0.70, -4.0, 0.05,  0.0,  0.0, 20.0},
        {0.85,  0.0, 0.12,  1.5, -0.8, -25.0},
        {0.99,  3.0, 0.20, -2.0,  1.0, 45.0},
        {0.80,  5.0, 0.15,  0.5,  2.0, 5.0},
    };
    int t, k;
    printf("précision : 4 types x 4 cas (pente, température, bruit capteurs)\n");
    for (t = 0; t < N_TYPES; t++) {
        for (k = 0; k < 4; k++) {
            double gw, cg, mid;
            bench_start(TYPES[t]);
            mid = B.tr.zfw_cg_mac;
            B.tr.zfw_kg     = cases[k][0] * B.ac.mzfw_kg;
            B.tr.zfw_cg_mac = mid + cases[k][1];
            B.tr.fuel_kg    = cases[k][2] * B.ac.mtow_kg;
            if (B.tr.zfw_kg + B.tr.fuel_kg > B.ac.mtow_kg - 300)   /* rester dans les limites */
                B.tr.fuel_kg = B.ac.mtow_kg - 300 - B.tr.zfw_kg;
            B.tr.pitch_deg  = cases[k][3];
            B.tr.roll_deg   = cases[k][4];
            B.tr.temp_c     = cases[k][5];
            B.tr.cg_y_m     = 0.05;
            run(settle());
            sim_truth_gw(&B.ac, &B.tr, &gw, &cg);
            CHECK(B.out.state == WB_ST_MEASURING && B.out.mass_valid,
                  "%s cas %d : pas de mesure (état %s, raisons %#x)", TYPES[t], k, wb_state_name(B.out.state), B.out.wait_reasons);
            CHECK(fabs(B.out.gw_kg - gw) / gw < 0.0005,
                  "%s cas %d : masse %.0f au lieu de %.0f", TYPES[t], k, B.out.gw_kg, gw);
            CHECK(fabs(B.out.cg_mac - cg) < 0.1,
                  "%s cas %d : centrage %.2f au lieu de %.2f", TYPES[t], k, B.out.cg_mac, cg);
            CHECK(fabs(B.out.zfw_kg - B.tr.zfw_kg) / B.tr.zfw_kg < 0.0005,
                  "%s cas %d : ZFW %.0f au lieu de %.0f", TYPES[t], k, B.out.zfw_kg, B.tr.zfw_kg);
            CHECK(fabs(B.out.cg_y_m - B.tr.cg_y_m * B.tr.zfw_kg / gw) < 0.02,
                  "%s cas %d : déport %.3f", TYPES[t], k, B.out.cg_y_m);
            CHECK(B.out.alerts == 0, "%s cas %d : alertes inattendues %#x", TYPES[t], k, B.out.alerts);
        }
    }
}

static void test_selftest_and_progress(void)
{
    printf("autotest au démarrage puis mesure\n");
    bench_start("NB180");
    run(1.0);
    CHECK(B.out.state == WB_ST_SELFTEST && !B.out.mass_valid, "aucune valeur pendant l'autotest");
    run(1.5);
    CHECK(B.out.state == WB_ST_MEASURING && !B.out.mass_valid && B.out.progress_pct > 0,
          "mesure en cours attendue (état %s, %d %%)", wb_state_name(B.out.state), B.out.progress_pct);
    run(2.0);
    CHECK(B.out.mass_valid, "masse disponible après une fenêtre");
}

static void test_missing_calibration_fails(void)
{
    static wb_aircraft_t ac;
    static wb_ctx_t ctx;
    wb_inputs_t in;
    wb_outputs_t out;
    char err[256];
    printf("sans étalonnage, le système se déclare en panne\n");
    memset(&in, 0, sizeof in);
    CHECK(wb_load_type_file("configs/types/NB180.acf", &ac, err, sizeof err) == 0, "%s", err);
    wb_init(&ctx, &ac);
    wb_step(&ctx, &in, &out);
    CHECK(out.state == WB_ST_FAIL && !out.mass_valid, "état PANNE attendu");
}

static void test_conditions_block_measure(void)
{
    printf("conditions : pente, frein, mouvement, engin connecté, inclinomètres\n");

    bench_start("NB180");
    B.tr.pitch_deg = 3.5;
    run(settle());
    CHECK(B.out.state == WB_ST_WAIT && (B.out.wait_reasons & WB_WAIT_SLOPE) && !B.out.mass_valid, "pente trop forte");

    bench_start("NB180");
    B.in.parking_brake = 0;
    run(settle());
    CHECK(B.out.state == WB_ST_WAIT && (B.out.wait_reasons & WB_WAIT_BRAKE), "frein desserré");

    bench_start("NB180");
    B.in.groundspeed_kt = 3.0;
    run(settle());
    CHECK(B.out.wait_reasons & WB_WAIT_MOVING, "avion en mouvement");

    bench_start("NB180");
    B.in.ext_contact = 1;
    run(settle());
    CHECK(B.out.wait_reasons & WB_WAIT_EXT_CONTACT, "barre de tractage connectée");

    /* Un inclinomètre dérive : sim_fill écrit les deux, on corrompt B ensuite */
    bench_start("NB180");
    {
        int i;
        for (i = 0; i < (int)(settle() * 50); i++) {
            B.in.time_ms += 20;
            sim_fill(&B.ac, &B.tr, &B.in);
            B.in.pitch_deg[1] += 0.6;
            wb_step(&B.ctx, &B.in, &B.out);
        }
    }
    CHECK((B.out.wait_reasons & WB_WAIT_INCLINOMETER) && (B.out.alerts & WB_AL_INCL_DISAGREE),
          "inclinomètres en désaccord");
}

static void test_loading_in_progress(void)
{
    int i;
    printf("chargement en cours : la mesure attend que la masse se stabilise\n");
    bench_start("NB180");
    run(WB_SELFTEST_MS / 1000.0);
    for (i = 0; i < 6 * 50; i++) {      /* 150 kg/s pendant 6 s */
        B.tr.zfw_kg += 3.0;
        B.in.time_ms += 20;
        sim_fill(&B.ac, &B.tr, &B.in);
        wb_step(&B.ctx, &B.in, &B.out);
    }
    CHECK(!B.out.mass_valid && (B.out.wait_reasons & WB_WAIT_UNSTABLE), "pas de mesure pendant le chargement");
    run(4.2);
    CHECK(B.out.mass_valid && fabs(B.out.zfw_kg - B.tr.zfw_kg) < 30, "mesure après stabilisation (%.0f / %.0f)",
          B.out.zfw_kg, B.tr.zfw_kg);
}

static void test_boarding_after_power_up(void)
{
    double first;
    printf("embarquement après la mise sous tension : la mesure suit\n");
    bench_start("NB180");
    run(settle());
    first = B.out.zfw_kg;
    B.tr.zfw_kg += 4200;     /* 50 passagers montent */
    run(settle());
    CHECK(fabs(B.out.zfw_kg - first - 4200) < 30, "ZFW passe de %.0f à %.0f", first, B.out.zfw_kg);
}

static void test_freeze_taxi_and_flight(void)
{
    double zfw, gw, cg;
    int i;
    printf("roulage et vol : ZFW figée, carburant suivi\n");
    bench_start("NB189");
    run(settle());
    zfw = B.out.zfw_kg;

    B.in.parking_brake = 0;
    B.in.groundspeed_kt = 15.0;
    B.tr.zfw_kg += 2000;              /* une perturbation au roulage ne doit rien changer */
    run(1.0);
    CHECK(B.out.state == WB_ST_FROZEN && fabs(B.out.zfw_kg - zfw) < 0.01, "valeur figée au roulage");
    B.tr.zfw_kg -= 2000;

    B.in.on_ground = 0;
    for (i = 0; i < 100; i++) {
        B.tr.fuel_kg -= 30.0;
        B.in.time_ms += 1000;
        sim_fill(&B.ac, &B.tr, &B.in);
        wb_step(&B.ctx, &B.in, &B.out);
    }
    sim_truth_gw(&B.ac, &B.tr, &gw, &cg);
    CHECK(B.out.state == WB_ST_FLIGHT, "état VOL");
    CHECK(fabs(B.out.gw_kg - gw) < 40 && fabs(B.out.cg_mac - cg) < 0.1,
          "masse en vol %.0f / %.0f, centrage %.2f / %.2f", B.out.gw_kg, gw, B.out.cg_mac, cg);
    CHECK(!(B.out.alerts & WB_AL_SENSOR_FAIL), "trains déchargés en vol : pas une panne capteur");

    /* Atterrissage, parking : nouvelle mesure */
    B.in.on_ground = 1;
    B.in.groundspeed_kt = 0.0;
    B.in.parking_brake = 1;
    B.tr.zfw_kg -= 8000;              /* débarquement */
    run(2.0 * B.ac.p.window_samples / 50.0 + 0.1);
    CHECK(B.out.state == WB_ST_MEASURING && fabs(B.out.zfw_kg - B.tr.zfw_kg) < 30,
          "nouvelle mesure au parking (%.0f / %.0f)", B.out.zfw_kg, B.tr.zfw_kg);
}

static void test_sensor_faults(void)
{
    double gw, cg;
    printf("pannes capteurs : voie muette, voies en désaccord\n");

    bench_start("NB180");
    B.tr.fault_ch = 3;
    B.tr.fault_dead = 1;
    run(settle());
    sim_truth_gw(&B.ac, &B.tr, &gw, &cg);
    CHECK(B.out.mass_valid && (B.out.alerts & WB_AL_SENSOR_DEGRADED) && !(B.out.alerts & WB_AL_SENSOR_FAIL),
          "voie muette : mesure sur l'autre voie (alertes %#x)", B.out.alerts);
    CHECK(fabs(B.out.gw_kg - gw) / gw < 0.0005, "voie muette : masse %.0f / %.0f", B.out.gw_kg, gw);

    bench_start("NB180");
    B.tr.fault_ch = 4;
    B.tr.fault_kg = 1500.0;
    run(settle());
    CHECK(!B.out.mass_valid && (B.out.alerts & WB_AL_SENSOR_FAIL) && (B.out.wait_reasons & WB_WAIT_SENSOR),
          "voies A/B en désaccord : aucune masse fausse affichée");

    /* Panne après une bonne mesure : on garde la bonne mesure */
    bench_start("WB4");
    run(settle());
    B.tr.fault_ch = 12;
    B.tr.fault_kg = 5000.0;
    run(1.0);
    CHECK(B.out.mass_valid && B.out.state == WB_ST_FROZEN && (B.out.alerts & WB_AL_SENSOR_FAIL),
          "panne après mesure : dernière mesure conservée, alerte affichée");
}

static void test_limit_alerts(void)
{
    printf("alertes : centrage, MZFW, déséquilibre, vérification croisée\n");

    bench_start("NB180");
    B.tr.zfw_cg_mac = 44.0;
    run(settle());
    CHECK(B.out.alerts & WB_AL_CG_OUT, "centrage arrière hors limites");

    bench_start("TP70");
    B.tr.zfw_kg = B.ac.mzfw_kg + 400;
    B.tr.fuel_kg = 1500;
    run(settle());
    CHECK(B.out.alerts & WB_AL_OVER_MZFW, "MZFW dépassée");

    bench_start("NB180");
    B.tr.cg_y_m = 0.9;
    run(settle());
    CHECK(B.out.alerts & WB_AL_LATERAL, "déséquilibre latéral");

    bench_start("NB180");
    B.in.loadsheet_zfw_kg = B.tr.zfw_kg * 0.97;     /* devis faux de 3 % */
    B.in.loadsheet_zfw_cg_mac = B.tr.zfw_cg_mac;
    run(settle());
    CHECK((B.out.alerts & WB_AL_XCHECK_MASS) && !(B.out.alerts & WB_AL_XCHECK_CG),
          "écart de 3 %% avec le devis détecté (%.2f %%)", B.out.xcheck_dmass_pct);

    bench_start("NB180");
    B.in.loadsheet_zfw_kg = B.tr.zfw_kg * 1.004;
    B.in.loadsheet_zfw_cg_mac = B.tr.zfw_cg_mac;
    run(settle());
    CHECK(!(B.out.alerts & (WB_AL_XCHECK_MASS | WB_AL_XCHECK_CG)), "écart de 0,4 %% toléré");
}

static void test_envelope(void)
{
    printf("enveloppe de centrage\n");
    bench_start("NB180");
    CHECK(wb_in_envelope(&B.ac, 60000, 25), "60 t à 25 %% : dans l'enveloppe");
    CHECK(!wb_in_envelope(&B.ac, 60000, 15), "60 t à 15 %% : trop avant");
    CHECK(!wb_in_envelope(&B.ac, 76000, 38), "76 t à 38 %% : trop arrière à forte masse");
    CHECK(!wb_in_envelope(&B.ac, 80000, 30), "80 t : au-dessus de l'enveloppe");
}

static void test_parser_errors(void)
{
    static wb_aircraft_t ac;
    char err[256];
    const char *base =
        "name = T\nmac_m = 4\nlemac_m = 18\nmrw_kg = 80000\nmtow_kg = 79000\nmlw_kg = 66000\nmzfw_kg = 62000\n"
        "envelope = 40000:15 79000:20 79000:35 40000:35\n"
        "leg = N 8 0 1\nleg = L 21 -3 2\nleg = R 21 3 2\ntank = WING 19 10000\n";
    char buf[1024];

    printf("contrôle des fichiers de configuration\n");
    CHECK(wb_parse_type(base, &ac, err, sizeof err) == 0, "configuration minimale refusée : %s", err);

    snprintf(buf, sizeof buf, "%sfoo = 1\n", base);
    CHECK(wb_parse_type(buf, &ac, err, sizeof err) != 0 && strstr(err, "foo"), "clé inconnue détectée");

    snprintf(buf, sizeof buf, "%smzfw_kg = 70000\n", base);
    CHECK(wb_parse_type(buf, &ac, err, sizeof err) != 0, "MZFW > MLW refusée");

    snprintf(buf, sizeof buf, "%senvelope = 40000:15 79000:80 40000:35\n", base);
    CHECK(wb_parse_type(buf, &ac, err, sizeof err) != 0 && strstr(err, "empattement"),
          "enveloppe hors des trains refusée (%s)", err);

    CHECK(wb_parse_type(base, &ac, err, sizeof err) == 0, "rechargement");
    CHECK(wb_parse_cal("tail = X\ntype = AUTRE\ndefault = 0.004 1 0 0\n", &ac, err, sizeof err) != 0,
          "étalonnage d'un autre type refusé");
    CHECK(wb_parse_cal("tail = X\ntype = T\ndefault = 0.004 1 0 0\nch10 = 0.004 1 0 0\n", &ac, err, sizeof err) != 0,
          "voie inexistante refusée");
    CHECK(wb_parse_cal("tail = X\ntype = T\ndefault = 0.004 1 0 0\nch9 = 0.004 1 0 0\n", &ac, err, sizeof err) == 0,
          "étalonnage valide accepté : %s", err);
}

int main(void)
{
    test_configs_load();
    test_accuracy_all_types();
    test_selftest_and_progress();
    test_missing_calibration_fails();
    test_conditions_block_measure();
    test_loading_in_progress();
    test_boarding_after_power_up();
    test_freeze_taxi_and_flight();
    test_sensor_faults();
    test_limit_alerts();
    test_envelope();
    test_parser_errors();
    printf("\n%d vérifications, %d échec(s)\n", g_count, g_fail);
    return g_fail ? 1 : 0;
}
