/*
 * wb_file.c — chargement depuis un fichier (cibles avec système de fichiers).
 * Sur une cible sans fichiers, l'hôte passe directement le texte (base de
 * données chargeable) à wb_parse_type() / wb_parse_cal().
 */
#include "wb.h"

#include <stdio.h>

#define WB_FILE_MAX 32768

static char g_buf[WB_FILE_MAX];

static int read_all(const char *path, char *err, size_t errlen)
{
    FILE *f = fopen(path, "rb");
    size_t n;
    if (!f) {
        snprintf(err, errlen, "impossible d'ouvrir %s", path);
        return -1;
    }
    n = fread(g_buf, 1, sizeof g_buf - 1, f);
    if (n == sizeof g_buf - 1 && fgetc(f) != EOF) {
        fclose(f);
        snprintf(err, errlen, "%s dépasse %d octets", path, WB_FILE_MAX - 1);
        return -1;
    }
    fclose(f);
    g_buf[n] = '\0';
    return 0;
}

int wb_load_type_file(const char *path, wb_aircraft_t *ac, char *err, size_t errlen)
{
    if (read_all(path, err, errlen) != 0) return -1;
    return wb_parse_type(g_buf, ac, err, errlen);
}

int wb_load_cal_file(const char *path, wb_aircraft_t *ac, char *err, size_t errlen)
{
    if (read_all(path, err, errlen) != 0) return -1;
    return wb_parse_cal(g_buf, ac, err, errlen);
}
