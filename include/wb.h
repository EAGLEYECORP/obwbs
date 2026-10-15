/*
 * wb.h — Pesée embarquée (OBWBS) : API publique du noyau.
 *
 * Le noyau ne connaît aucun avion en particulier. Tout ce qui dépend du type
 * (géométrie des trains, limites, enveloppe, réservoirs) vient d'un fichier
 * de type (.acf) ; tout ce qui dépend de l'avion lui-même (étalonnage de
 * chaque capteur) vient d'un fichier d'immatriculation (.cal).
 *
 * Règles de conception (inspirées des pratiques DO-178C) :
 *   - aucune allocation dynamique, aucune récursion, tailles bornées ;
 *   - aucune entrée/sortie dans le noyau : l'hôte remplit wb_inputs_t,
 *     appelle wb_step() à cadence fixe et lit wb_outputs_t ;
 *   - fonctions déterministes : mêmes entrées => mêmes sorties.
 */
#ifndef WB_H
#define WB_H

#include <stddef.h>
#include <stdint.h>

#define WB_MAX_LEGS      10   /* jambes de train (avant + principaux + ventraux) */
#define WB_MAX_POINTS    32   /* points de mesure (un par essieu en général)     */
#define WB_MAX_CHANNELS  (2 * WB_MAX_POINTS)  /* chaque point a 2 voies A/B      */
#define WB_MAX_ENV       24   /* sommets de l'enveloppe de centrage              */
#define WB_MAX_TANKS     10   /* réservoirs carburant                            */
#define WB_MAX_WINDOW    512  /* échantillons par fenêtre de mesure              */
#define WB_NAME_LEN      24

#define WB_ADC_MAX       16777215L  /* convertisseur 24 bits */

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */

typedef struct {
    char   name[WB_NAME_LEN];
    double x_m;          /* bras de levier longitudinal (m depuis la référence) */
    double y_m;          /* position latérale (m, droite positive)              */
    int    n_points;     /* nombre de points de mesure sur cette jambe          */
    int    first_point;  /* index du premier point (rempli au chargement)       */
} wb_leg_t;

typedef struct {
    char   name[WB_NAME_LEN];
    double x_m;          /* bras de levier du carburant de ce réservoir */
    double capacity_kg;
} wb_tank_t;

typedef struct {
    double gain_kg_per_count;   /* sensibilité                               */
    double offset_counts;       /* zéro à 20 °C                              */
    double tc_gain_ppm;         /* dérive de sensibilité (ppm/°C)            */
    double tc_zero_counts;      /* dérive du zéro (points/°C)                */
} wb_cal_t;

typedef struct {
    double max_slope_deg;     /* pente maxi du parking pour mesurer          */
    double max_gs_kt;         /* vitesse sol maxi (avion immobile)           */
    int    window_samples;    /* échantillons moyennés par mesure            */
    double stab_pct;          /* dérive maxi entre les 2 moitiés de fenêtre (% MTOW) */
    double chan_tol_pct;      /* écart maxi entre voies A et B (% de la voie)*/
    double chan_tol_kg;       /* plancher absolu de cet écart                */
    double incl_tol_deg;      /* écart maxi entre les deux inclinomètres     */
    double xcheck_mass_pct;   /* écart maxi mesure / devis (masse)           */
    double xcheck_cg_mac;     /* écart maxi mesure / devis (% CMA)           */
    double lat_cg_max_m;      /* déport latéral maxi du CG                   */
} wb_params_t;

typedef struct {
    /* type */
    char        type_name[WB_NAME_LEN];
    double      mac_m, lemac_m;   /* corde moyenne et position de son bord d'attaque */
    double      cg_height_m;      /* hauteur du CG au-dessus du plan des roues       */
    double      mrw_kg, mtow_kg, mlw_kg, mzfw_kg;
    int         n_env;
    double      env_mass_kg[WB_MAX_ENV];
    double      env_cg_mac[WB_MAX_ENV];
    int         n_legs;
    wb_leg_t    legs[WB_MAX_LEGS];
    int         n_points;         /* total, = somme des n_points des jambes  */
    int         n_tanks;
    wb_tank_t   tanks[WB_MAX_TANKS];
    wb_params_t p;
    /* immatriculation */
    char        tail[WB_NAME_LEN];
    int         cal_loaded;
    wb_cal_t    cal[WB_MAX_CHANNELS];
} wb_aircraft_t;

/* Analyse d'un fichier de type (.acf) déjà lu en mémoire. 0 si OK. */
int wb_parse_type(const char *text, wb_aircraft_t *ac, char *err, size_t errlen);
/* Analyse d'un fichier d'étalonnage (.cal) ; le type doit être chargé. */
int wb_parse_cal(const char *text, wb_aircraft_t *ac, char *err, size_t errlen);
/* Contrôles de cohérence après chargement (enveloppe, limites, géométrie). */
int wb_validate(const wb_aircraft_t *ac, char *err, size_t errlen);

/* Utilitaires hôte (wb_file.c) : lecture d'un fichier puis analyse. */
int wb_load_type_file(const char *path, wb_aircraft_t *ac, char *err, size_t errlen);
int wb_load_cal_file(const char *path, wb_aircraft_t *ac, char *err, size_t errlen);

/* ------------------------------------------------------------------------ */
/* Entrées / sorties cycliques                                               */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t time_ms;
    int32_t  raw[WB_MAX_CHANNELS];     /* mesures brutes des capteurs (points ADC) */
    double   leg_temp_c[WB_MAX_LEGS];  /* température de chaque jambe             */
    double   pitch_deg[2];             /* inclinomètres A et B, cabré positif      */
    double   roll_deg[2];              /* inclinomètres A et B, aile droite basse + */
    int      on_ground;                /* contact sol (WOW)                        */
    int      parking_brake;            /* frein de parking serré                   */
    double   groundspeed_kt;
    int      ext_contact;              /* passerelle / barre de tractage connectée  */
    double   tank_kg[WB_MAX_TANKS];    /* jaugeage carburant par réservoir          */
    double   loadsheet_zfw_kg;         /* devis : masse sans carburant (0 = absent) */
    double   loadsheet_zfw_cg_mac;     /* devis : centrage sans carburant           */
} wb_inputs_t;

typedef enum {
    WB_ST_SELFTEST = 0,  /* autotest en cours                                  */
    WB_ST_WAIT,          /* pas encore de mesure, conditions non réunies       */
    WB_ST_MEASURING,     /* avion au parking : mesure en continu               */
    WB_ST_FROZEN,        /* au sol en mouvement : dernière mesure + carburant  */
    WB_ST_FLIGHT,        /* en vol : dernière mesure + carburant               */
    WB_ST_FAIL           /* autotest échoué : aucune valeur fournie            */
} wb_state_t;

/* Raisons pour lesquelles la mesure ne peut pas se faire maintenant */
enum {
    WB_WAIT_AIRBORNE      = 1u << 0,
    WB_WAIT_BRAKE         = 1u << 1,
    WB_WAIT_MOVING        = 1u << 2,
    WB_WAIT_EXT_CONTACT   = 1u << 3,
    WB_WAIT_SLOPE         = 1u << 4,
    WB_WAIT_INCLINOMETER  = 1u << 5,
    WB_WAIT_SENSOR        = 1u << 6,
    WB_WAIT_UNSTABLE      = 1u << 7
};

/* Alertes à présenter au pilote */
enum {
    WB_AL_SENSOR_FAIL     = 1u << 0,  /* un point de mesure perdu            */
    WB_AL_SENSOR_DEGRADED = 1u << 1,  /* un point sur une seule voie         */
    WB_AL_INCL_DISAGREE   = 1u << 2,
    WB_AL_OVER_MRW        = 1u << 3,
    WB_AL_OVER_MZFW       = 1u << 4,
    WB_AL_OVER_MTOW       = 1u << 5,  /* en vol ou au roulage                 */
    WB_AL_CG_OUT          = 1u << 6,
    WB_AL_LATERAL         = 1u << 7,
    WB_AL_XCHECK_MASS     = 1u << 8,
    WB_AL_XCHECK_CG       = 1u << 9
};

typedef struct {
    wb_state_t state;
    uint32_t   wait_reasons;
    uint32_t   alerts;
    int        mass_valid;        /* 1 si les valeurs ci-dessous sont utilisables */
    int        progress_pct;      /* avancement de la fenêtre de mesure           */
    uint32_t   meas_age_ms;       /* âge de la dernière mesure                    */
    double     gw_kg;             /* masse actuelle                               */
    double     cg_mac;            /* centrage actuel (% CMA)                      */
    double     cg_y_m;            /* déport latéral du CG                         */
    double     zfw_kg;            /* masse sans carburant                         */
    double     zfw_cg_mac;
    double     fob_kg;            /* carburant à bord                             */
    double     leg_kg[WB_MAX_LEGS];
    double     xcheck_dmass_pct;  /* (mesure - devis) / devis                     */
    double     xcheck_dcg_mac;
} wb_outputs_t;

typedef struct {
    const wb_aircraft_t *ac;
    wb_state_t state;
    int        started;
    uint32_t   t_start_ms;
    /* fenêtre glissante */
    int        n;
    double     w_total[WB_MAX_WINDOW];
    double     s_mx, s_my, s_total;
    double     s_leg[WB_MAX_LEGS];
    double     w_pitch, w_roll;
    int        unstable;               /* dernière fenêtre rejetée (avion en mouvement) */
    uint32_t   win_alerts;             /* alertes capteurs vues pendant la fenêtre      */
    /* dernière mesure retenue (au moment de la mesure) */
    int        has_meas;
    uint32_t   meas_time_ms;
    double     zfw_kg, zfw_mx, zfw_my; /* masse et moments sans carburant */
    double     meas_fob_kg;
    double     meas_leg[WB_MAX_LEGS];
    uint32_t   meas_alerts;            /* alertes capteurs pendant la mesure */
} wb_ctx_t;

/* Durée de l'autotest au démarrage */
#define WB_SELFTEST_MS 2000u

void wb_init(wb_ctx_t *ctx, const wb_aircraft_t *ac);
void wb_step(wb_ctx_t *ctx, const wb_inputs_t *in, wb_outputs_t *out);

/* Fonctions de calcul exposées pour les tests */
double wb_pct_mac(const wb_aircraft_t *ac, double x_m);
double wb_x_of_mac(const wb_aircraft_t *ac, double pct);
int    wb_in_envelope(const wb_aircraft_t *ac, double mass_kg, double cg_mac);
/* Conversion brute -> kg pour une voie ; renvoie 0 si hors plage */
int    wb_channel_kg(const wb_cal_t *c, int32_t raw, double temp_c, double *kg);

const char *wb_state_name(wb_state_t s);

#endif
