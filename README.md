# OBWBS : logiciel de pesée embarquée

Ce logiciel tourne dans le calculateur de l'avion. Il lit les capteurs de charge des trains
d'atterrissage et donne au pilote **la masse réelle et le centrage de l'avion**. La mesure démarre
dès la mise sous tension. Le logiciel continue ensuite de suivre la masse pendant le roulage et le
vol.

Le même code sert pour **tous les types d'avion**. Ce qui change d'un avion à l'autre se décrit dans
deux fichiers texte :

| Fichier | Contenu | Qui le fournit |
|---|---|---|
| `configs/types/<TYPE>.acf` | Position des trains, nombre de capteurs, limites de masse, enveloppe de centrage, réservoirs | Données du constructeur (manuel de masse et centrage) |
| `configs/tails/<IMMAT>.cal` | Étalonnage de chaque capteur de cet avion précis | Pesée sur balances à l'installation, puis à chaque grande visite |

Quatre types sont fournis comme exemples. Leurs **valeurs sont indicatives** : il faut les remplacer
par les données officielles du constructeur.

| Type | Proche de | Trains | Points de mesure |
|---|---|---|---|
| `NB180` | A320 | avant + 2 principaux | 5 |
| `NB189` | 737-800 | avant + 2 principaux | 5 |
| `TP70` | ATR 72 | avant + 2 principaux | 5 |
| `WB4` | A380 | avant + 2 trains d'aile + 2 trains de fuselage | 11 |

## Essayer

```bash
make test   # 143 vérifications sur les 4 types
make demo   # rejoue un départ complet d'un monocouloir
./build/wbsim configs/types/WB4.acf configs/tails/DEMO-WB4.cal --zfw 330000 --fuel 160000 --dead 7
```

Options de `wbsim` :
- `--zfw` : masse sans carburant réelle ;
- `--zfw-cg` : centrage sans carburant ;
- `--fuel` : carburant au bloc ;
- `--trip` : carburant consommé en vol ;
- `--pitch` et `--roll` : pente du parking ;
- `--temp` : température des trains ;
- `--loadsheet` et `--loadsheet-cg` : valeurs du devis de masse et centrage ;
- `--fault VOIE:KG` : ajoute une erreur sur une voie ;
- `--dead VOIE` : simule une voie qui ne répond plus.

Extrait de `make demo` (avion garé sur une pente de 0,8°) :

```
[   6.0 s] parking — état MESURE
           mesure suspendue : chargement en cours
[  18.0 s] parking — état MESURE
           MASSE          69300 kg   centrage 24.48 % CMA
           SANS CARB.     59500 kg   centrage 24.00 % CMA   carburant 9800 kg
           devis      écart masse +0.51 %   écart centrage +0.20
[3941.0 s] avant atterrissage — état VOL
           MASSE          62801 kg   centrage 24.18 % CMA
           (réel : 62802 kg à 24.18 % CMA)
```

## Fonctionnement

```
Capteurs (2 voies A/B par roue) ─► conversion en kg + vote A/B ─► charge par jambe
Inclinomètres A/B, frein, contact sol, vitesse ─► conditions de mesure
Jaugeage carburant par réservoir ─────────────► masse et centrage en vol
                                                       │
                                     wb_step() à 50 Hz ─┴─► masse, centrage, alertes
```

**États :**

| État | Situation | Ce que voit le pilote |
|---|---|---|
| `AUTOTEST` | 2 s après la mise sous tension | rien |
| `ATTENTE` | conditions non réunies, pas encore de mesure | la raison (frein desserré, pente, etc.) |
| `MESURE` | au parking, frein serré, immobile | masse mise à jour toutes les 2 s, l'embarquement peut continuer |
| `FIGÉE` | au sol, en mouvement | dernière mesure du parking ; le carburant vient du jaugeage |
| `VOL` | en vol | masse sans carburant mesurée + carburant restant, centrage avec la position de chaque réservoir |
| `PANNE` | configuration absente ou incohérente | aucune valeur, pour ne jamais afficher une masse fausse |

**Calculs :**
- **Masse :** somme des charges des trains, corrigée de la pente du parking : W = ΣN / (cos θ · cos φ).
- **Centrage :** moyenne des positions des trains pondérée par leurs charges. On retranche le report
  de charge dû à la pente, avec la hauteur du CG : x = Σ(N·x)/ΣN − h·tan θ.
- **Masse sans carburant :** masse mesurée moins le carburant jaugé. Elle reste fixe ensuite.
  La masse en vol vaut masse sans carburant + carburant restant.
- **Stabilité :** une mesure n'est retenue que si la moyenne de la seconde moitié de la fenêtre
  (2 s) est égale à celle de la première, à 0,1 % de la MTOW près. Une personne qui charge ou un
  camion qui pousse suspend la mesure. Le bruit des capteurs, lui, ne la suspend pas.

**Sécurité :**
- Chaque roue a **deux voies indépendantes**. Si une voie est muette, le calcul se fait sur
  l'autre et l'alerte « capteur dégradé » s'affiche. Si les deux voies répondent mais ne sont pas
  d'accord, le point est déclaré perdu et la mesure est suspendue : le logiciel ne choisit jamais au
  hasard.
- Les deux inclinomètres doivent concorder. Au-delà de 3° de pente, la mesure est refusée.
- Une charge totale invraisemblable (moins du quart de la MZFW au sol) est traitée comme une panne
  de capteur.
- Si une panne survient après une bonne mesure, cette mesure est conservée et l'alerte s'affiche.

**Alertes :** capteur perdu ou dégradé, inclinomètres en désaccord, masse maximale de roulage,
masse sans carburant maximale, masse maximale au décollage (au roulage et en vol), centrage hors
enveloppe, déséquilibre latéral. Deux alertes concernent l'**écart avec le devis de masse et
centrage** (plus de 1 % en masse ou plus de 1 % de CMA en centrage). Ce sont elles qui détectent
une erreur de chargement ou de saisie.

## Ajouter un type d'avion

1. Copiez `configs/types/NB180.acf`. Renseignez les données du manuel de masse et centrage : CMA,
   limites, enveloppe, position de chaque jambe de train, réservoirs.
   - Ligne `leg` : nom, position x (m), position y (m, droite positive), nombre de points de
     mesure (en général un par roue ou par essieu).
   - Le logiciel refuse un fichier incohérent : limites dans le désordre, enveloppe hors des
     trains, moins de 3 jambes, pas de jambe à gauche ou à droite.
2. Pour chaque avion, créez son fichier `.cal` après la pesée sur balances :
   - la ligne `default` donne les valeurs communes à toutes les voies ;
   - une ligne `chN` corrige la voie N si besoin.
   - Les voies sont numérotées dans l'ordre des jambes, puis des points : voie 2p pour la voie A du
     point p, voie 2p+1 pour sa voie B.
3. Ajoutez le type à `TYPES[]` dans `tests/test_wb.c` et lancez `make test`.

Aucune ligne de code n'est à modifier. Le nombre de jambes, de points et de réservoirs est libre
dans les limites de `wb.h` : 10 jambes, 32 points, 10 réservoirs.

## Intégration dans le calculateur

```c
static wb_aircraft_t ac;   static wb_ctx_t ctx;
wb_parse_type(texte_acf, &ac, err, sizeof err);   // base de données chargeable
wb_parse_cal(texte_cal, &ac, err, sizeof err);
wb_init(&ctx, &ac);
for (;;) {                       // tâche cyclique à 50 Hz
    lire_capteurs(&in);          // à écrire pour la carte cible : CAN 24 bits, ARINC 429, discrets
    wb_step(&ctx, &in, &out);
    publier(&out);               // vers l'écran cockpit, le FMS, la tablette pilote
}
```

Règles respectées par le noyau (`src/`) :
- aucune allocation dynamique et aucune récursion ;
- tableaux de taille fixe ;
- aucune entrée/sortie : seul `wb_file.c` lit des fichiers, et il ne sert que sur les cibles qui en
  ont ;
- compilation en C99 strict, avec tous les avertissements traités comme des erreurs ;
- testé sous AddressSanitizer et UndefinedBehaviorSanitizer.

## Ce qui reste à faire avant de voler

Ce logiciel est un **prototype de développement**, pas un logiciel certifié.

1. **Données réelles** : géométrie et limites de chaque type fournies par le constructeur, et
   étalonnage issu d'une vraie pesée.
2. **Essais avec de vrais capteurs** : le simulateur ne reproduit ni la souplesse du train, ni le
   vent, ni le frottement des pneus. Il faudra fixer les tolérances sur avion réel.
3. **Interface avion** : codage ARINC 429 ou AFDX des sorties selon les spécifications du
   constructeur, et pilotes des capteurs pour la carte cible.
4. **Certification logicielle DO-178C** :
   - exigences tracées jusqu'au code et aux tests ;
   - couverture structurelle (MC/DC si le niveau DAL B est retenu) ;
   - revue indépendante ;
   - outils qualifiés.
   - Le calcul est aujourd'hui en virgule flottante double précision. Il faudra justifier ce choix
     sur le processeur cible, ou passer en virgule fixe.
5. **Base de données chargeable** : les fichiers `.acf` et `.cal` devront suivre un processus
   DO-200B ou équivalent (intégrité, contrôle de version, signature).
6. **Approbation** : modification de type (STC) auprès de l'EASA ou de la FAA. La circulaire FAA
   AC 20-161 décrit les exigences propres aux systèmes de pesée embarquée.
