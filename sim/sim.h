/*
 * sim.h — simulateur d'avion : produit les mesures brutes que les capteurs
 * fourniraient pour une masse et un centrage connus. Sert au banc d'essai
 * logiciel et aux tests ; il n'est pas embarqué.
 */
#ifndef SIM_H
#define SIM_H

#include "wb.h"

typedef struct {
    double   zfw_kg;          /* vérité : masse sans carburant      */
    double   zfw_cg_mac;      /* vérité : centrage sans carburant   */
    double   cg_y_m;          /* vérité : déport latéral            */
    double   fuel_kg;         /* carburant à bord                   */
    double   pitch_deg, roll_deg;
    double   temp_c;
    double   noise_kg;        /* bruit de mesure par voie (écart-type) */
    int      fault_ch;        /* voie en défaut (-1 : aucune)       */
    double   fault_kg;        /* biais ajouté à cette voie          */
    int      fault_dead;      /* 1 : la voie ne répond plus         */
    uint32_t seed;
} sim_truth_t;

void   sim_default(sim_truth_t *t);
/* Répartit le carburant : ailes d'abord, puis les autres réservoirs. */
void   sim_split_fuel(const wb_aircraft_t *ac, double fuel_kg, double *tank_kg);
/* Masse et centrage réels (avec carburant) pour comparer aux sorties. */
void   sim_truth_gw(const wb_aircraft_t *ac, const sim_truth_t *t, double *gw, double *cg_mac);
/* Remplit les mesures capteurs, l'assiette et le jaugeage de *in.
 * in->on_ground doit être réglé avant l'appel (en vol, les trains ne portent rien).
 * Les champs time_ms, on_ground, parking_brake, etc. restent à l'appelant. */
void   sim_fill(const wb_aircraft_t *ac, sim_truth_t *t, wb_inputs_t *in);

#endif
