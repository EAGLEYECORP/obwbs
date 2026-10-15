/*
 * wb_config.c — lecture des fichiers de type (.acf) et d'étalonnage (.cal).
 *
 * Format commun : une donnée par ligne, « clé = valeur », « # » commente
 * le reste de la ligne. Aucune allocation : tout tient dans wb_aircraft_t.
 */
#include "wb.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX_LEN 512
#define MAX_TOKENS   (WB_MAX_ENV + 4)

static int fail(char *err, size_t errlen, int line, const char *fmt, ...)
{
    if (err && errlen) {
        va_list ap;
        int n = 0;
        if (line > 0) n = snprintf(err, errlen, "ligne %d : ", line);
        if (n < 0) n = 0;
        if ((size_t)n < errlen) {
            va_start(ap, fmt);
            vsnprintf(err + n, errlen - (size_t)n, fmt, ap);
            va_end(ap);
        }
    }
    return -1;
}

static char *trim(char *s)
{
    char *e;
    while (*s && isspace((unsigned char)*s)) s++;
    e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}

/* Découpe s sur les blancs ; modifie s. Renvoie le nombre de jetons. */
static int split(char *s, char **tok, int max)
{
    int n = 0;
    while (*s) {
        while (*s && isspace((unsigned char)*s)) s++;
        if (!*s) break;
        if (n == max) return -1;
        tok[n++] = s;
        while (*s && !isspace((unsigned char)*s)) s++;
        if (*s) *s++ = '\0';
    }
    return n;
}

static int to_num(const char *s, double *out)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) return 0;
    *out = v;
    return 1;
}

static void copy_name(char *dst, const char *src)
{
    strncpy(dst, src, WB_NAME_LEN - 1);
    dst[WB_NAME_LEN - 1] = '\0';
}

/* Itère sur les lignes ; cb reçoit clé, valeur, numéro de ligne. */
typedef int (*line_cb)(wb_aircraft_t *, char *, char *, int, char *, size_t);

static int for_each_line(const char *text, wb_aircraft_t *ac, line_cb cb,
                         char *err, size_t errlen)
{
    char buf[LINE_MAX_LEN];
    int  line = 0;
    const char *p = text;

    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        char *s, *eq, *hash;
        line++;
        if (len >= sizeof buf) return fail(err, errlen, line, "ligne trop longue");
        memcpy(buf, p, len);
        buf[len] = '\0';
        p += len + (nl ? 1 : 0);

        hash = strchr(buf, '#');
        if (hash) *hash = '\0';
        s = trim(buf);
        if (!*s) continue;
        eq = strchr(s, '=');
        if (!eq) return fail(err, errlen, line, "« = » attendu");
        *eq = '\0';
        if (cb(ac, trim(s), trim(eq + 1), line, err, errlen) != 0) return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Fichier de type                                                           */
/* ------------------------------------------------------------------------ */

static int type_line(wb_aircraft_t *ac, char *key, char *val, int line,
                     char *err, size_t errlen)
{
    char *tok[MAX_TOKENS];
    int   n;
    double v;

    struct { const char *k; double *dst; } scalars[] = {
        {"mac_m", &ac->mac_m},           {"lemac_m", &ac->lemac_m},
        {"cg_height_m", &ac->cg_height_m},
        {"mrw_kg", &ac->mrw_kg},         {"mtow_kg", &ac->mtow_kg},
        {"mlw_kg", &ac->mlw_kg},         {"mzfw_kg", &ac->mzfw_kg},
        {"max_slope_deg", &ac->p.max_slope_deg},
        {"max_gs_kt", &ac->p.max_gs_kt},
        {"stab_pct", &ac->p.stab_pct},
        {"chan_tol_pct", &ac->p.chan_tol_pct},
        {"chan_tol_kg", &ac->p.chan_tol_kg},
        {"incl_tol_deg", &ac->p.incl_tol_deg},
        {"xcheck_mass_pct", &ac->p.xcheck_mass_pct},
        {"xcheck_cg_mac", &ac->p.xcheck_cg_mac},
        {"lat_cg_max_m", &ac->p.lat_cg_max_m},
    };
    size_t i;

    if (strcmp(key, "name") == 0) { copy_name(ac->type_name, val); return 0; }
    if (strcmp(key, "description") == 0) return 0;

    for (i = 0; i < sizeof scalars / sizeof scalars[0]; i++) {
        if (strcmp(key, scalars[i].k) == 0) {
            if (!to_num(val, scalars[i].dst))
                return fail(err, errlen, line, "%s : nombre attendu", key);
            return 0;
        }
    }

    if (strcmp(key, "window_samples") == 0) {
        if (!to_num(val, &v) || v < 10 || v > WB_MAX_WINDOW || v != floor(v))
            return fail(err, errlen, line, "window_samples : entier entre 10 et %d", WB_MAX_WINDOW);
        ac->p.window_samples = (int)v;
        return 0;
    }

    n = split(val, tok, MAX_TOKENS);

    if (strcmp(key, "envelope") == 0) {
        int k;
        if (n < 3 || n > WB_MAX_ENV)
            return fail(err, errlen, line, "envelope : entre 3 et %d sommets masse:centrage", WB_MAX_ENV);
        for (k = 0; k < n; k++) {
            char *c = strchr(tok[k], ':');
            if (!c) return fail(err, errlen, line, "envelope : « %s » doit être masse:centrage", tok[k]);
            *c = '\0';
            if (!to_num(tok[k], &ac->env_mass_kg[k]) || !to_num(c + 1, &ac->env_cg_mac[k]))
                return fail(err, errlen, line, "envelope : sommet %d illisible", k + 1);
        }
        ac->n_env = n;
        return 0;
    }

    if (strcmp(key, "leg") == 0) {
        wb_leg_t *L;
        double np;
        if (n != 4) return fail(err, errlen, line, "leg = NOM x_m y_m nb_points");
        if (ac->n_legs == WB_MAX_LEGS) return fail(err, errlen, line, "plus de %d jambes", WB_MAX_LEGS);
        L = &ac->legs[ac->n_legs];
        copy_name(L->name, tok[0]);
        if (!to_num(tok[1], &L->x_m) || !to_num(tok[2], &L->y_m) || !to_num(tok[3], &np)
            || np < 1 || np != floor(np))
            return fail(err, errlen, line, "leg : valeurs illisibles");
        if (ac->n_points + (int)np > WB_MAX_POINTS)
            return fail(err, errlen, line, "plus de %d points de mesure", WB_MAX_POINTS);
        L->n_points = (int)np;
        L->first_point = ac->n_points;
        ac->n_points += L->n_points;
        ac->n_legs++;
        return 0;
    }

    if (strcmp(key, "tank") == 0) {
        wb_tank_t *T;
        if (n != 3) return fail(err, errlen, line, "tank = NOM x_m capacité_kg");
        if (ac->n_tanks == WB_MAX_TANKS) return fail(err, errlen, line, "plus de %d réservoirs", WB_MAX_TANKS);
        T = &ac->tanks[ac->n_tanks];
        copy_name(T->name, tok[0]);
        if (!to_num(tok[1], &T->x_m) || !to_num(tok[2], &T->capacity_kg) || T->capacity_kg <= 0)
            return fail(err, errlen, line, "tank : valeurs illisibles");
        ac->n_tanks++;
        return 0;
    }

    return fail(err, errlen, line, "clé inconnue « %s »", key);
}

static void set_defaults(wb_aircraft_t *ac)
{
    memset(ac, 0, sizeof *ac);
    ac->p.max_slope_deg   = 3.0;
    ac->p.max_gs_kt       = 0.5;
    ac->p.window_samples  = 100;   /* 2 s à 50 Hz */
    ac->p.stab_pct        = 0.10;
    ac->p.chan_tol_pct    = 1.0;
    ac->p.chan_tol_kg     = 50.0;
    ac->p.incl_tol_deg    = 0.3;
    ac->p.xcheck_mass_pct = 1.0;
    ac->p.xcheck_cg_mac   = 1.0;
    ac->p.lat_cg_max_m    = 0.5;
}

int wb_parse_type(const char *text, wb_aircraft_t *ac, char *err, size_t errlen)
{
    set_defaults(ac);
    if (for_each_line(text, ac, type_line, err, errlen) != 0) return -1;
    return wb_validate(ac, err, errlen);
}

/* ------------------------------------------------------------------------ */
/* Fichier d'étalonnage                                                      */
/* ------------------------------------------------------------------------ */

static int read_cal(char *val, wb_cal_t *c, int line, char *err, size_t errlen)
{
    char *tok[MAX_TOKENS];
    int n = split(val, tok, MAX_TOKENS);
    if (n != 4 || !to_num(tok[0], &c->gain_kg_per_count) || !to_num(tok[1], &c->offset_counts)
        || !to_num(tok[2], &c->tc_gain_ppm) || !to_num(tok[3], &c->tc_zero_counts))
        return fail(err, errlen, line, "étalonnage = gain offset tc_gain_ppm tc_zero");
    if (c->gain_kg_per_count <= 0)
        return fail(err, errlen, line, "gain nul ou négatif");
    return 0;
}

static int cal_line(wb_aircraft_t *ac, char *key, char *val, int line,
                    char *err, size_t errlen)
{
    int n_ch = 2 * ac->n_points;

    if (strcmp(key, "tail") == 0) { copy_name(ac->tail, val); return 0; }
    if (strcmp(key, "type") == 0) {
        if (strcmp(val, ac->type_name) != 0)
            return fail(err, errlen, line, "étalonnage prévu pour le type %s, type chargé %s", val, ac->type_name);
        return 0;
    }
    if (strcmp(key, "default") == 0) {
        wb_cal_t c;
        int i;
        if (read_cal(val, &c, line, err, errlen) != 0) return -1;
        for (i = 0; i < n_ch; i++) ac->cal[i] = c;
        ac->cal_loaded |= 1;
        return 0;
    }
    if (key[0] == 'c' && key[1] == 'h') {
        double idx;
        if (!to_num(key + 2, &idx) || idx < 0 || idx >= n_ch || idx != floor(idx))
            return fail(err, errlen, line, "voie « %s » inexistante (0 à %d)", key, n_ch - 1);
        if (!(ac->cal_loaded & 1))
            return fail(err, errlen, line, "« default » doit précéder les voies");
        return read_cal(val, &ac->cal[(int)idx], line, err, errlen);
    }
    return fail(err, errlen, line, "clé inconnue « %s »", key);
}

int wb_parse_cal(const char *text, wb_aircraft_t *ac, char *err, size_t errlen)
{
    if (ac->n_points == 0) return fail(err, errlen, 0, "charger le type avant l'étalonnage");
    ac->cal_loaded = 0;
    ac->tail[0] = '\0';
    if (for_each_line(text, ac, cal_line, err, errlen) != 0) return -1;
    if (!ac->cal_loaded) return fail(err, errlen, 0, "étalonnage : ligne « default » absente");
    if (!ac->tail[0]) return fail(err, errlen, 0, "étalonnage : immatriculation (tail) absente");
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Cohérence                                                                 */
/* ------------------------------------------------------------------------ */

int wb_validate(const wb_aircraft_t *ac, char *err, size_t errlen)
{
    int i, nose = 0, left = 0, right = 0;
    double xmin = 1e9, xmax = -1e9;

    if (!ac->type_name[0]) return fail(err, errlen, 0, "name absent");
    if (ac->mac_m <= 0) return fail(err, errlen, 0, "mac_m absent ou nul");
    if (ac->mtow_kg <= 0 || ac->mlw_kg <= 0 || ac->mzfw_kg <= 0 || ac->mrw_kg <= 0)
        return fail(err, errlen, 0, "limites de masse incomplètes (mrw, mtow, mlw, mzfw)");
    if (!(ac->mzfw_kg <= ac->mlw_kg && ac->mlw_kg <= ac->mtow_kg && ac->mtow_kg <= ac->mrw_kg))
        return fail(err, errlen, 0, "limites incohérentes : il faut mzfw ≤ mlw ≤ mtow ≤ mrw");
    if (ac->n_env < 3) return fail(err, errlen, 0, "enveloppe de centrage absente");
    if (ac->n_legs < 3) return fail(err, errlen, 0, "au moins 3 jambes de train sont nécessaires");
    if (ac->n_tanks < 1) return fail(err, errlen, 0, "au moins un réservoir est nécessaire");

    for (i = 0; i < ac->n_legs; i++) {
        const wb_leg_t *L = &ac->legs[i];
        if (L->x_m < xmin) xmin = L->x_m;
        if (L->x_m > xmax) xmax = L->x_m;
        if (L->y_m < -0.01) left = 1;
        else if (L->y_m > 0.01) right = 1;
        else nose = 1;
    }
    (void)nose;
    if (!left || !right) return fail(err, errlen, 0, "il faut des jambes à gauche (y<0) et à droite (y>0)");
    if (xmax - xmin < 1.0) return fail(err, errlen, 0, "empattement trop court : les jambes doivent être séparées en x");

    /* L'enveloppe doit rester entre les trains, sinon l'avion basculerait */
    for (i = 0; i < ac->n_env; i++) {
        double x = ac->lemac_m + ac->env_cg_mac[i] / 100.0 * ac->mac_m;
        if (x <= xmin || x >= xmax)
            return fail(err, errlen, 0, "sommet %d de l'enveloppe hors de l'empattement", i + 1);
    }
    if (ac->p.window_samples < 10) return fail(err, errlen, 0, "window_samples trop petit");
    return 0;
}
