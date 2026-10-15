/*
 * wbsim — banc d'essai : rejoue un départ complet (mise sous tension,
 * embarquement, repoussage, roulage, vol) et affiche ce que verrait le pilote.
 *
 *   wbsim TYPE.acf AVION.cal [options]
 *     --zfw KG          masse sans carburant réelle      (défaut : 85 % MZFW)
 *     --zfw-cg PCT      centrage sans carburant réel     (défaut : milieu d'enveloppe)
 *     --fuel KG         carburant au bloc                (défaut : 15 % MTOW)
 *     --trip KG         carburant consommé en vol        (défaut : 60 % du carburant)
 *     --pitch DEG --roll DEG   pente du parking
 *     --temp C          température des trains
 *     --loadsheet KG    masse sans carburant du devis, pour la vérification croisée
 *     --loadsheet-cg PCT
 *     --fault VOIE:KG   biais sur une voie ; --dead VOIE : voie muette
 */
#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HZ 50

static const char *ALERT_TXT[] = {
    "CAPTEUR PERDU", "CAPTEUR DÉGRADÉ (une voie)", "INCLINOMÈTRES EN DÉSACCORD",
    "MASSE > MAX ROULAGE", "MASSE SANS CARBURANT > MAX", "MASSE > MAX DÉCOLLAGE",
    "CENTRAGE HORS LIMITES", "DÉSÉQUILIBRE LATÉRAL",
    "ÉCART MASSE AVEC LE DEVIS", "ÉCART CENTRAGE AVEC LE DEVIS",
};
static const char *WAIT_TXT[] = {
    "en vol", "frein de parking desserré", "avion en mouvement", "engin connecté",
    "pente trop forte", "inclinomètres", "capteur", "chargement en cours",
};

static void print_bits(uint32_t v, const char **txt, int n, const char *sep)
{
    int i, first = 1;
    for (i = 0; i < n; i++) if (v & (1u << i)) { printf("%s%s", first ? "" : sep, txt[i]); first = 0; }
}

static void show(const wb_aircraft_t *ac, const wb_outputs_t *o, uint32_t t_ms, const char *event)
{
    printf("\n[%6.1f s] %s — état %s\n", t_ms / 1000.0, event, wb_state_name(o->state));
    if (o->state == WB_ST_FROZEN || o->state == WB_ST_FLIGHT) {
        printf("           dernière mesure au parking conservée, carburant mis à jour par le jaugeage\n");
    } else if (o->wait_reasons) {
        printf("           mesure suspendue : ");
        print_bits(o->wait_reasons, WAIT_TXT, 8, ", ");
        printf("\n");
    }
    if (o->mass_valid) {
        int l;
        printf("           MASSE      %9.0f kg   centrage %5.2f %% CMA   déport %+.2f m\n",
               o->gw_kg, o->cg_mac, o->cg_y_m);
        printf("           SANS CARB. %9.0f kg   centrage %5.2f %% CMA   carburant %.0f kg\n",
               o->zfw_kg, o->zfw_cg_mac, o->fob_kg);
        printf("           trains    ");
        for (l = 0; l < ac->n_legs; l++) printf(" %s %.0f", ac->legs[l].name, o->leg_kg[l]);
        printf("  (mesure il y a %.0f s)\n", o->meas_age_ms / 1000.0);
        if (o->xcheck_dmass_pct != 0.0 || o->xcheck_dcg_mac != 0.0)
            printf("           devis      écart masse %+.2f %%   écart centrage %+.2f\n",
                   o->xcheck_dmass_pct, o->xcheck_dcg_mac);
    } else if (o->state == WB_ST_MEASURING) {
        printf("           mesure en cours : %d %%\n", o->progress_pct);
    }
    if (o->alerts) {
        printf("           ALERTES : ");
        print_bits(o->alerts, ALERT_TXT, 10, " | ");
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    static wb_aircraft_t ac;
    static wb_ctx_t ctx;
    wb_inputs_t in;
    wb_outputs_t out;
    sim_truth_t tr;
    char err[256];
    double trip = -1, final_zfw, block, gw_true, cg_true;
    wb_state_t last = WB_ST_FAIL;
    uint32_t last_alerts = 0, last_wait = 0, t = 0;
    int i, s;

    if (argc < 3) {
        fprintf(stderr, "usage : wbsim TYPE.acf AVION.cal [options]\n");
        return 2;
    }
    if (wb_load_type_file(argv[1], &ac, err, sizeof err) != 0) { fprintf(stderr, "%s : %s\n", argv[1], err); return 1; }
    if (wb_load_cal_file(argv[2], &ac, err, sizeof err) != 0)  { fprintf(stderr, "%s : %s\n", argv[2], err); return 1; }

    memset(&in, 0, sizeof in);
    sim_default(&tr);
    tr.zfw_kg = 0.85 * ac.mzfw_kg;
    {
        double lo = 1e9, hi = -1e9;
        for (i = 0; i < ac.n_env; i++) { if (ac.env_cg_mac[i] < lo) lo = ac.env_cg_mac[i]; if (ac.env_cg_mac[i] > hi) hi = ac.env_cg_mac[i]; }
        tr.zfw_cg_mac = 0.5 * (lo + hi);
    }
    tr.fuel_kg = 0.15 * ac.mtow_kg;

    for (i = 3; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!v) { fprintf(stderr, "%s : valeur manquante\n", a); return 2; }
        if      (!strcmp(a, "--zfw"))          tr.zfw_kg = atof(v);
        else if (!strcmp(a, "--zfw-cg"))       tr.zfw_cg_mac = atof(v);
        else if (!strcmp(a, "--fuel"))         tr.fuel_kg = atof(v);
        else if (!strcmp(a, "--trip"))         trip = atof(v);
        else if (!strcmp(a, "--pitch"))        tr.pitch_deg = atof(v);
        else if (!strcmp(a, "--roll"))         tr.roll_deg = atof(v);
        else if (!strcmp(a, "--temp"))         tr.temp_c = atof(v);
        else if (!strcmp(a, "--loadsheet"))    in.loadsheet_zfw_kg = atof(v);
        else if (!strcmp(a, "--loadsheet-cg")) in.loadsheet_zfw_cg_mac = atof(v);
        else if (!strcmp(a, "--fault"))        { tr.fault_ch = atoi(v); tr.fault_kg = strchr(v, ':') ? atof(strchr(v, ':') + 1) : 1000.0; }
        else if (!strcmp(a, "--dead"))         { tr.fault_ch = atoi(v); tr.fault_dead = 1; }
        else { fprintf(stderr, "option inconnue %s\n", a); return 2; }
        i++;
    }
    if (trip < 0) trip = 0.6 * tr.fuel_kg;
    final_zfw = tr.zfw_kg;
    block = tr.fuel_kg;

    printf("Avion %s (type %s) — %d jambes, %d points de mesure, %d voies\n",
           ac.tail, ac.type_name, ac.n_legs, ac.n_points, 2 * ac.n_points);
    sim_truth_gw(&ac, &tr, &gw_true, &cg_true);
    printf("Valeurs réelles simulées : masse %.0f kg, centrage %.2f %% CMA, sans carburant %.0f kg à %.2f %% CMA\n",
           gw_true, cg_true, tr.zfw_kg, tr.zfw_cg_mac);

    wb_init(&ctx, &ac);
    in.on_ground = 1;
    in.parking_brake = 1;

    /* Phase 1 : parking, fin d'embarquement (3 t chargées en 10 s), 40 s au total */
    for (s = 0; s < 40 * HZ; s++, t += 1000 / HZ) {
        double ts = t / 1000.0;
        tr.zfw_kg = ts < 5 ? final_zfw - 3000 : ts < 15 ? final_zfw - 3000 * (15 - ts) / 10 : final_zfw;
        in.time_ms = t;
        sim_fill(&ac, &tr, &in);
        wb_step(&ctx, &in, &out);
        if (out.state != last || out.alerts != last_alerts || out.wait_reasons != last_wait) {
            show(&ac, &out, t, out.state == WB_ST_SELFTEST ? "mise sous tension" : "parking");
            last = out.state; last_alerts = out.alerts; last_wait = out.wait_reasons;
        }
    }
    show(&ac, &out, t, "fin d'embarquement, avant repoussage");

    /* Phase 2 : repoussage (barre de tractage) puis roulage 5 min, 1 % du carburant */
    in.parking_brake = 0; in.ext_contact = 1;
    for (s = 0; s < 5 * 60 * HZ; s++, t += 1000 / HZ) {
        if (s == 60 * HZ) in.ext_contact = 0;
        in.groundspeed_kt = s < 60 * HZ ? 2.0 : 12.0;
        tr.fuel_kg = block - 0.01 * block * s / (5.0 * 60 * HZ);
        in.time_ms = t;
        sim_fill(&ac, &tr, &in);
        wb_step(&ctx, &in, &out);
    }
    show(&ac, &out, t, "alignement piste");
    sim_truth_gw(&ac, &tr, &gw_true, &cg_true);
    printf("           (réel : %.0f kg à %.2f %% CMA)\n", gw_true, cg_true);

    /* Phase 3 : vol d'une heure à 1 Hz */
    in.on_ground = 0; in.groundspeed_kt = 250;
    {
        double f0 = tr.fuel_kg;
        for (s = 0; s <= 3600; s++, t += 1000) {
            tr.fuel_kg = f0 - trip * s / 3600.0;
            in.time_ms = t;
            sim_fill(&ac, &tr, &in);
            wb_step(&ctx, &in, &out);
            if (s == 1) show(&ac, &out, t, "décollage");
        }
    }
    show(&ac, &out, t, "avant atterrissage");
    sim_truth_gw(&ac, &tr, &gw_true, &cg_true);
    printf("           (réel : %.0f kg à %.2f %% CMA)\n", gw_true, cg_true);
    return 0;
}
