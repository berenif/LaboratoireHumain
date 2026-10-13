# LaboratoireHumain — Parité native et prochain protocole causal

Rapport consolidé du 13 octobre 2026. Code et sources publiés sur `investigate/physical-support`, commit vérifié `83b8447285333ec2feb765781c3d84aac3d2e4dd`. Aucun changement de `main`.


---

# Parité native démontrée sur le premier tick contraint

**44 sous-pas natifs exactement exécutés en 1,477787 seconde murale.** Le lot
correctif est `closed-success` : intégrité, préfixe archivé, commandes installées,
parité copie/exécution et présence des diagnostics sont tous vérifiés.
**Aucun relevage, transfert local ni stabilité prolongée n’est qualifié.**

Le gel a été publié et son SHA distant vérifié avant exécution :
`b37eb9a566e1c819a7077c067ded5696f425ca8b` sur `investigate/physical-support`.
`main` reste à `c597959f46f37715035349c8cd0a699b6822111a`.
Le lot a démarré le 13 octobre à **10:22:22,086 UTC** ; plafond 44 pas et 120 s,
un processus, un thread. Aucun worker natif ne reste actif après sa fermeture.

## Ce que les pas prouvent

Le préfixe réel de **12 pas à 240 Hz** correspond aux archives, y compris
2 341 motifs binaires f32 de corps et moteurs par état. Depuis ce checkpoint,
chaque facteur possède une copie prédite de quatre pas et une exécution de
quatre pas sur une instance distincte. Tous les pas sont comptés :
`12 + 4 × (4 + 4) = 44`.

| Facteur | Itérations du solveur resserré | Commandes exactes par chemin | Parité native |
| ---: | ---: | ---: | --- |
| 0 | 8 | 4 | 4 états et fin de tick égaux |
| 0,5 | 8 | 4 | 4 états et fin de tick égaux |
| 1 | 12 | 4 | 4 états et fin de tick égaux |
| 1,5 | 12 | 4 | 4 états et fin de tick égaux |

L’audit indépendant retrouve **32 installations de commandes**, soit
**1 472 vérifications d’axes**, les 16 paires d’états après pas, les 16 paires
d’installations et les quatre états après tick. Positions, vitesses, mémoire,
contacts, impulsions et état sérialisé concordent exactement. Les gains
canoniques, l’actionnement `ForceBased` et les caps sont conservés ; la somme
des caps relue reste inférieure ou égale au budget de 760 Nm. Cela ne transforme
pas un cap moteur en couple effectivement livré.

Chaque comparaison couvre **16,67 ms**, de t=0,0500 à t=0,0667 s. Le test utilise
la fixture enregistrée `landed-crouch-left`, sans nouvel impact. Les diagnostics
locaux restent faux et les talons sont à 0 N sur ces derniers échantillons :
le test de transfert de quatre secondes reste à faire. Une prévision à deux
secondes, le relevage ≤25 s et la seconde stable ne sont pas validés par ce lot.
La récupération reste désactivée par défaut ; aucune qualification des 24 poses,
cycles ou impacts n’est effectuée.

Le journal CCD du chemin réel est archivé séparément. La comparaison des états
exposés ne certifie pas des caches internes que la sérialisation Rapier n’expose
pas. La preuve porte sur les quatre pas effectivement comparés, pas sur une
équivalence universelle de toute prévision future.

## Refus conservés et correction

La tentative du matin avait été arrêtée avant tout worker par l’interface
`/proc/.../children` absente. Le lanceur corrigé a ensuite démarré le worker,
mais le [lot renouvelé](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/coordination-native-parity-renewal-2026-10-13.md) s’est
interrompu sur une erreur de sérialisation JSON. Son compteur officiel reste
inconnu, avec une borne de code de **0–1 appel Rapier**, jamais remplacé par zéro.
Ses 44 pas réservés ne sont pas remboursés.

Le défaut était reproductible sans physique : `serde_json::Value` ne conserve
pas les entiers u128 dépassant u64. Le masque réel de contact
`170141183460469231731687303715884105728` (2^127) apparaît dès le premier pas.
Le monde est désormais archivé comme texte JSON exact dans une enveloppe
`{encoding, text}`, sans conversion en flottant ni changement global de serde.
Un compteur minimal est écrit durablement avant l’état détaillé ; une erreur
interceptée produit un reçu, ou des bornes explicites si le compte est incertain.

**8 tests purs**, la compilation de production et la compilation release dédiée
ont passé avant lancement. Aucun monde ni intégration n’est créé par ces tests.
Le nouveau binaire est distinct ; les anciens binaires et registres sont conservés.
Les 326 fichiers gelés et sept pièces protégées ont été vérifiés après fermeture.

## Preuves et capture

- [Registre clos et comptage](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/native-run/physical-ledger.json).
- [Résultat natif complet compressé](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/native-run/native-result.json.gz).
- [Trace de tous les états compressée](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/native-run/native-trace.jsonl.gz).
- [Empreintes et validation sans perte](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/archive-index.json).
- [Audit indépendant de budget](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/post-run-budget-audit.md).
- [Audit indépendant des commandes et états physiques](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/post-run-physical-parity-audit.md).
- [Capture des poses enregistrées](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/recorded-poses.png).

La capture transforme seulement les maillages selon les 25 poses archivées :
aucune intégration, interpolation, FK ou requête de collision nouvelle. Le
compteur final distingue 44 admissions et 44 pas terminés, zéro réservation
locale pendante. La trace contient aussi 11 répétitions de comptage en fin de
tick : ses 55 lignes de compteurs ne sont pas 55 intégrations.

## Prochain jalon causal, protocole implémenté

Préparer **une paire A/B de quatre secondes chacune**, mêmes référence, fixture,
profil, gains, caps et critères. A désactive la coordination ; B utilise les
quatre facteurs déjà figés avec sélection prédictive à 10 Hz maximum. Aucun
réglage après lecture du premier résultat. Le coût plafonné inclut :

- vérification native dans le même processus : **16 pas** ;
- deux histoires réelles : **1 920 pas** ;
- au plus 40 appels, quatre prévisions de 480 pas : **76 800 pas** ;
- total **78 736 pas**, zéro sonde supplémentaire, plafond mural **2 h**.

Les 16 pas sont payés explicitement : le reçu du lot court n’est pas transformé
en admission fictive pour un autre processus. Le nouveau mode `local-pair` du registre et du lanceur ferme après B,
que son critère local réussisse ou échoue, sans lancer B-repeat ou B60.
**128 tests purs passent** (87 du registre, 29 du lanceur, 12 du builder),
sans worker natif ni mutation des anciens registres. Le mode historique reste disponible.
Cette préparation entre dans l’enveloppe de préparation existante ;
aucun nouveau lot A/B n’a été ouvert ou lancé.

Avant exécution, il reste à reconstruire le worker de transition dans une
entrée dédiée, exécuter ses contrôles purs actuels, établir les preuves de
préparation puis geler les empreintes dans un nouveau manifeste. La parité
courte ne remplace pas cette préparation liée au binaire qui exécutera A/B.
Le lanceur impose ensuite la vérification native de 16 pas dans ce même
processus et ce même registre ; un échec ferme le lot avant les histoires.

La continuation vers debout reste une décision distincte, avec les commandes,
appuis courants, passage de propriétaire, terminal original et stabilité à
vérifier avant une histoire complète. Le succès local ne peut pas devenir un
succès de relevage. La [proposition chiffrée de continuation](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-parity-renewal-2026-10-13/next-causal-milestone.md)
expose les coûts ; une histoire de 26 s dépasse le périmètre des histoires
locales de quatre secondes et n’est pas exécutée ici.

La [préparation détaillée et ses prérequis restants](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/next-local-pair-preparation.md)
décrit le builder, les critères et la décision de continuation. Elle est
implémentée au niveau du protocole ; son manifeste exécutable reste à geler.

## Vérification et état de livraison

La vérification Rust de ce lot est ciblée : huit tests purs, compilation
de production et compilation release sans exécution, puis les 44 pas natifs
autorisés. La suite globale Rust/TypeScript n’a pas été relancée ; ses échecs
historiques ne sont ni attribués à cette correction ni présentés comme résolus.
Les preuves du succès sont publiées au commit
`18005749aa2dc45fb38e6973846a844187d49660`, SHA distant vérifié.
Les ajouts de protocole A/B font l’objet d’un jalon distinct sur la même
branche ; aucun merge ni changement de `main`.


---

# Audit du lot natif de sérialisation — 13 octobre 2026

Audit indépendant réalisé uniquement par lecture du code et des preuves, calcul d’empreintes et comparaison des données archivées. Aucune intégration Rapier, reconstruction géométrique, compilation ou exécution de worker pendant cet audit. Ce document est ajouté après exécution, hors du manifeste figé ; aucun registre ni fichier figé n’a été modifié.

## Verdict et périmètre

Le lot distinct `coordination-numeric-parity-serialization-2026-10-13T10:06Z` est correctement clos en `closed-success` : **44 sous-pas réservés avant démarrage, 44 admis, 44 terminés, aucun pas en attente et aucun remboursement**. Les preuves qualifient la parité numérique du préfixe archivé et de quatre comparaisons courtes copie/réel. Elles ne qualifient ni une continuation complète, ni un relevage, ni la stabilité finale.

| Partie | Pas originaux | Pas de copie | Total |
| --- | ---: | ---: | ---: |
| Préfixe commun archivé | 12 | 0 | 12 |
| Échelle 0 | 4 | 4 | 8 |
| Échelle 0,5 | 4 | 4 | 8 |
| Échelle 1 | 4 | 4 | 8 |
| Échelle 1,5 | 4 | 4 | 8 |
| **Total** | **28** | **16** | **44** |

Le reçu natif fournit `nativeAccounting.exact=true`, `minimumSteps=maximumSteps=completedClockSteps=admittedSteps=44`. Son budget local donne `locallyReservedSteps=reservedSteps=admittedIntegrations=44` et `pendingReservedSteps=0`. Ces valeurs concordent avec le reçu NDJSON et `physical-ledger.json` ; `run-result.json` reproduit exactement le contenu du registre.

## Compteurs, ordre et délai

- Début du plafond mural : **10:22:22.085649 UTC** ; réservation durable : **10:22:22.086454 UTC** ; clôture : **10:22:23.563447 UTC**.
- Durée monotone enregistrée : **1,4777865719988768 s**, inférieure aux **120 s** autorisées, démarrage et échanges compris. L’écart UTC est cohérent à la précision des dates sérialisées. Le délai monotone figé dépasse exactement le début de 120 s.
- Un seul message `run`, précédé du `ready` du PID **6288**, puis un seul `result`. Identité enregistrée : binaire attendu, **1 thread**, aucun enfant. Le PID a disparu ; la lecture de `/proc/*/exe` après clôture ne trouve aucun worker de coordination ou de parité actif. Cette constatation complète les contrôles du superviseur sans prétendre constituer un enregistrement continu des processus.
- Le reçu contient **44 installations avant intégration** : horloges 0–11 pour le préfixe, puis 12–15 pour chacune des huit branches copie/réel.
- La trace comporte **112 lignes** : 1 état initial, 12 états de préfixe, 16 prédictions, 16 états réels, 1 certification du préfixe, 11 états de fin de tick et **55 checkpoints de comptage**. Ces derniers couvrent les valeurs distinctes **1 à 44**, dans l’ordre, avec **11 répétitions de fin de tick**. Les répétitions ne sont pas des intégrations supplémentaires.

Les compteurs ne reposent donc pas uniquement sur un booléen de succès : les horloges, les installations et les états archivés fournissent des décomptes concordants.

## Comparaisons indépendantes des archives

La lecture hors simulation confirme :

- **12/12** tableaux `bodyAndMotorBits` du préfixe égaux à ceux de `expected.json` ; horloges terminées 1–12.
- **16/16** états sérialisés copie/réel égaux par paires, y compris les tableaux de bits et la représentation du monde ; horloges 13–16 pour chaque échelle.
- **16/16** installations copie/réel égales par paires dans le reçu, en conservant les valeurs JSON et les bits archivés.
- Pour les quatre échelles, reçu de parité `initial_substep=12`, `verified_substeps=4`, commandes figées vérifiées sur chacun des deux chemins, vérification des moteurs et de l’état après tick positive.

Les quatre critères du reçu sont vrais : `integrity_ok`, `prefix_matches_archive`, `parity_verified`, `diagnostic_present`. Les limites déclarées restent applicables : espaces de travail CCD omis par la sérialisation non certifiés par cette comparaison ; aucune qualification de récupération (`recoveryQualified=false`, `fullRecoveryQualified=false`) et aucune histoire complète autorisée par ce reçu (`fullHistoryAuthorized=false`).

## Intégrité du gel et conservation des lots clos

**326/326 fichiers** correspondent à leurs empreintes dans le manifeste. Les **6 anciens registres et l’ancien binaire du prototype**, soit 7 pièces protégées par `preparation.json`, correspondent aussi à leurs empreintes. L’ancien binaire de parité `9e125785…2760cc` est conservé ; le nouveau binaire est distinct.

Les claims, manifestes, registres physiques et résultats des **deux tentatives closes précédentes** ont été vérifiés contre les empreintes du nouveau gel, soit 8 pièces concordantes. Leurs réservations de 44 pas chacune restent consommées et leurs compteurs officiels inconnus restent `null` : le succès présent ne rembourse ni ne réécrit ces lots. L’enveloppe SHA-256 du registre présent est valide ; les empreintes du reçu et de la trace inscrites dans ce registre correspondent aux fichiers.

La préparation figée conserve **23 989 s réservées sur 144 000 s** et **9 173 inspections sur 20 000** ; ce lot ne crée aucune extension de préparation (`additionalPreparationReservedSeconds=0`). Ces nombres sont des charges conservatrices réservées, non des mesures de calcul effectif. Le champ figé `nativeExecutionStarted=false` décrit l’état avant lancement ; le claim et le registre distincts portent l’exécution effective.

| Pièce | SHA-256 |
| --- | --- |
| `manifest.json` | `179bfe3cb58819cd2ba1b73642969c0eb2a343a052dee83ae89e2850e0830356` |
| Binaire dédié | `83bddf95897acc6015de7d2947b04ee2f5aaf2eb49b4dcb85ba0a67764f95e1b` |
| `authorization-claim.json` | `64477a600eaf5b334bac4031d2b5b3e2f1eae52f29858d751a4569e5f63611fe` |
| `native-run/physical-ledger.json` | `191d3216953bfb4aca5b7abcea277454d7d97ad5b4ca72e28ffbc260399af250` |
| `native-run/run-result.json` | `87c88f9d00e80d4d68c8218e3451c7fed243e674ffaf2baa1bec8ed78d8b6fed` |
| `native-run/native-result.json` | `91af43ee6a6ee5420b381ddefb5d722876304b25f403154d6e326fe4dbe0924c` |
| `native-run/native-trace.jsonl` | `88802a9348cc7e8ff074b3396d6261e71e43026018168f972f748b3b5f891766` |
| `native-run/protocol.ndjson` | `f98d8e60a9ee535974f89c0be23321354912ba4d87818fbd82482d47ae7e230d` |

Le lot est épuisé et clos. Aucun crédit de calcul résiduel ou droit de rejouer ne découle de sa réussite.


---

# Audit indépendant des preuves de parité native

Audit du 13 octobre 2026 après clôture du lot
`coordination-numeric-parity-serialization-2026-10-13T10:06Z`.
Les opérations de cet audit sont des lectures de fichiers, décompressions,
comparaisons de valeurs/bits, vérifications de sommes et d'empreintes. Elles
n'exécutent **aucun worker natif, aucune construction de modèle ou de monde,
aucune FK, aucun pas Rapier**. La seule écriture est cette note, hors manifeste.

**Verdict : les preuves établissent la parité annoncée sur les quatre pas
comparés de chacun des quatre cas.** Les commandes numériques archivées sont
bien celles installées dans les moteurs, et les états copie/réel comparés sont
identiques. Ce verdict ne constitue ni une acquisition des talons ni un relevage.

## Comptage et intégrité des preuves

Le registre porte `closed-success`, sans motif de refus. Il réserve 44 pas,
en rapporte exactement 44, ne rembourse aucun pas, et mesure
`1.4777865719988768` seconde, sous le plafond de 120 secondes. Son intervalle
commence à `2026-10-13T10:22:22.085649+00:00` et sa clôture est journalisée à
`10:22:23.563447+00:00`. Le processus lié est le PID 6288, un thread,
sans enfant déclaré par le superviseur.

Le rapport natif confirme simultanément :

- 44 admissions, 44 pas achevés, borne minimale = borne maximale = 44 ;
- 44 réservations locales et zéro pas réservé restant à intégrer ;
- préfixe commun de 12 pas, puis quatre cas de 4 pas copiés + 4 pas réels ;
- les quatre portes `integrity_ok`, `prefix_matches_archive`,
  `parity_verified`, `diagnostic_present` vraies.

Les 55 lignes de comptage durable comprennent les 44 nombres de pas de 1 à 44
dans l'ordre, avec 11 répétitions de fin de tick. Elles ne représentent donc
pas 55 intégrations. Les traces contiennent 12 observations du préfixe,
16 observations prédites, 16 réelles et 11 états après finalisation de tick.
Le journal stderr du worker est vide.

L'enveloppe SHA-256 du registre est valide. Les **326 fichiers gelés** du
manifeste et les **7 fichiers protégés** de la préparation ont été relus :
leurs empreintes sont inchangées. Les contenus bruts du rapport et de la
trace, directement lus ou décompressés, correspondent aux empreintes du
registre. Aucun ancien registre n'a été modifié par l'audit.

| Pièce | SHA-256 vérifié |
| --- | --- |
| Manifeste | `179bfe3cb58819cd2ba1b73642969c0eb2a343a052dee83ae89e2850e0830356` |
| Registre, fichier complet | `191d3216953bfb4aca5b7abcea277454d7d97ad5b4ca72e28ffbc260399af250` |
| Rapport natif brut | `91af43ee6a6ee5420b381ddefb5d722876304b25f403154d6e326fe4dbe0924c` |
| Trace native brute | `88802a9348cc7e8ff074b3396d6261e71e43026018168f972f748b3b5f891766` |
| Binaire lié par le superviseur | `83bddf95897acc6015de7d2947b04ee2f5aaf2eb49b4dcb85ba0a67764f95e1b` |
| Source `numeric_parity_worker.rs` | `ac89b99890d1842205bb1071967697a221c7fa2b929fd253d54e08035ccd04e0` |

## Comparaisons refaites indépendamment sur les archives

Les comparaisons n'ont pas seulement relu les booléens du rapport. Elles ont
parcouru les observations et installations enregistrées :

1. **12 états du préfixe** correspondent aux anciennes archives : tableaux
   u32 de corps/moteurs égaux, puis comparaison récursive des champs après la
   projection explicitement gelée. Cette projection exclut les métadonnées
   de trace et journaux CCD distincts ; le nouveau `numericAttempt` est exigé
   nul avant retrait. Les flottants sont comparés par leurs bits, zéros signés
   compris.
2. **32 commandes installées** — quatre commandes × quatre échelles × deux
   chemins — correspondent bit à bit aux commandes certifiées archivées aux
   indices 12–15. Les 75 positions/vitesses stockées, les cibles du simulateur
   et la mémoire de Recovery concordent avec cet oracle indépendant.
3. **1 472 installations d'axes moteurs** ont été vérifiées : 32 × 46 axes.
   Les `target_pos` et `target_vel` effectivement relus dans les joints sont
   ceux de l'oracle. `dq` est exactement la différence finie f32 de q sur dt.
   Les moteurs sont `ForceBased`, leurs axes canoniques sont activés, le
   plafond vectoriel est actif et les gains correspondent aux formules
   canoniques avec l'ordre d'arrondi f32 du profil gelé.
4. Les plafonds finaux relus respectent chaque plafond anatomique. La somme
   f64 des 46 `max_force` varie de **401,6021146774292 à
   759,999991953373 Nm**, donc reste ≤ 760 Nm. La vérification utilise les
   moteurs après éventuelle réparation ULP, pas seulement les caps journalisés
   avant cette réparation.
5. **16 paires d'installations copie/réel**, **16 paires d'états après pas** et
   **4 paires d'états après finalisation de tick** sont égales. La comparaison
   inclut le JSON exact du monde, corps, moteurs, contacts/charges/persistance,
   horloges, propriétaire, `modeSignature` et la mémoire Recovery complète.
   Chaque état final porte `nativeSubstep=16` et `tick=4`.

Les certificats du premier tick contraint sont effectivement présents, avec
quatre commandes certifiées par cas et la tentative finale
`certified guarded exact f32 commands` :

| Facteur | Itérations | Erreur dure déclarée | Résidu du gradient projeté |
| --- | ---: | ---: | ---: |
| 0 | 8 | 1,7347 × 10⁻¹⁸ | 6,4889 × 10⁻⁹ |
| 0,5 | 8 | 5,5511 × 10⁻¹⁷ | 9,3625 × 10⁻⁹ |
| 1 | 12 | 5,5511 × 10⁻¹⁷ | 1,7328 × 10⁻⁷ |
| 1,5 | 12 | 1,1102 × 10⁻¹⁶ | 1,7785 × 10⁻⁷ |

Ces valeurs respectent les seuils gelés de `1e-8` et `1e-6`. Les tentatives
directes refusées sont conservées comme diagnostics ; elles ne sont pas
utilisées comme oracle des commandes installées.

## Conservation effective du bit 127

Les mondes sont archivés avec l'encodage explicite
`serde-json-text-preserving-u128-v1`. Leur champ `text` a été comparé exactement
entre copie et réel, puis lu avec des entiers Python arbitrairement grands pour
contrôler les valeurs, sans conversion en flottant.

Le premier état achevé contient notamment :

```text
step1/narrow_phase/body_solver_color_masks/19
= 170141183460469231731687303715884105728
= 2^127
```

L'audit retrouve cette valeur exacte 56 fois dans les 29 textes de mondes
distincts conservés. Le défaut précédent de passage forcé dans un `Value`
borné à u64 est donc effectivement dépassé sur un état natif contenant le
bit 127, au-delà du seul test synthétique. Aucune comparaison ne tronque cette
valeur à u64 ni ne l'arrondit en f64.

## Portée et limites physiques

La preuve est limitée à la fixture native `landed-crouch-left`, orientation
0 rad, telle qu'archivée. Ce lot ne simule aucun nouvel impact.

Chaque cas compare **4 / 240 s = 16,67 ms** depuis le même checkpoint au pas 12.
Le préfixe commun représente 50 ms ; chaque branche s'arrête à 66,67 ms depuis
la fixture initiale. Les 44 pas sont un coût total de calcul sur ces branches,
pas une histoire continue de 44 pas.

Les quatre diagnostics locaux finaux portent `passed=false`. Les deux talons
ont une charge courante nulle (encodée `-0.0` dans les traces) et aucune
persistance qualifiée. Les maxima de vitesse finale vont de 0,436 à 0,454 m/s
et de 3,43 à 3,97 rad/s selon le cas : cette courte vérification n'établit pas
la stabilité locale. Elle ne démontre ni l'impossibilité de l'objectif causal,
ni une préférence physique entre les quatre facteurs.

Les journaux CCD du chemin réel sont conservés séparément ; le chemin copié
n'en collecte pas. L'égalité prouvée porte sur les états persistants natifs et
leurs contacts/impulsions comparés, pas sur une égalité de journaux CCD absents
ni sur tous les espaces de travail omis par la sérialisation. Aucun bilan
angulaire global supplémentaire n'est qualifié.

**Le résultat permet de poursuivre la préparation d'un prochain jalon causal ;
il ne vaut pas succès local, continuation longue, relevage après impact ou
qualification des 24 poses/cycles/impacts.** Aucun nouveau calcul physique ni
crédit de budget n'est autorisé par cet audit.


---

# Prochain jalon : paire locale A/B préparée, aucune exécution

**Préparer une seule paire A/B de quatre secondes, plafonnée à 78 736 pas natifs et deux heures de physique.** La parité courte a réellement passé : [reçu compressé sans perte](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/native-run/native-result.json.gz), [audit indépendant](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/post-run-budget-audit.md), 44 pas exacts en 1,477787 s. Elle couvre les douze états du préfixe et quatre commandes copie/réel pour chacune des quatre échelles. Elle ne démontre pas encore une prévision de 480 pas, un talon porteur, la stabilité locale ou la continuation debout.

Cette livraison ajoute uniquement des scripts de préparation, des critères et des tests factices. Aucun worker n'a été lancé, aucun monde construit, aucune FK ou inspection géométrique nouvelle, aucun pas physique. Les anciens manifestes, traces et registres restent intacts. La préparation demeure dans l'enveloppe initiale de 40 h ; elle n'ouvre ni ne renouvelle une allocation physique.

| Poste du nouveau lot | Borne |
| --- | ---: |
| Vérification dans le même processus : A copie4/réel4 puis B copie4/réel4 | 16 |
| A : référence commune, coordination désactivée, heading 0° | 960 |
| B : même référence, coordination activée, heading 0° | 960 |
| B : au plus 40 appels × 4 prévisions × 480 pas | 76 800 |
| **Total réservé maximal** | **78 736** |

Les 16 pas sont payés dans le nouveau lot et produisent les deux `NativeKernelAdmission` conservés par `transition_worker`. Aucun reçu externe n'est importé comme permission, aucun crédit des 44 pas antérieurs n'est réutilisé. Les réservations restent consommées en cas de refus ou d'arrêt anticipé. A et B sont deux instances natives indépendantes issues de la même fixture ; chaque histoire conserve son instance et son horloge jusqu'à sa fin, sans reset intermédiaire.

Le [mode `local-pair`](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/scripts/coordination-prototype-budget.py) et le [runner](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/scripts/run-coordination-prototype.py) sont maintenant préparés : `protocolMode: "local-pair"`, prérequis `local_pair_frozen`, ordre limité à A puis B, vérification exactement 16, zéro probe court et paquets de prévision exactement `[480,480,480,480]`. Les plafonds figés sont `histories=2`, `totalNativeSteps=78736`, `predictionAndVerificationSteps=76816`, `verificationSteps=16`. B termine le registre en succès local comme en échec local ; aucun B-repeat, B60 ou prolongement n'est admis. L'absence de ce mode conserve l'ancien protocole et ses anciens plafonds.

Le worker Rust conserve ses limites générales de quatre histoires et 300 000 pas ; le registre local plus strict autorise seulement les permis ci-dessus et le runner ferme son unique worker après B. Changer le plafond général du worker n'est pas nécessaire pour cette paire. Le délai natif commun de 7 200 s commence à la réservation de vérification, après `ready` ; le démarrage est borné séparément à 30 s. Il ne faut donc pas présenter ce runner comme un superviseur de deux heures incluant lancement et toute la préparation. Le contrôle `/proc` du processus/thread et les échéances avant chaque batch demeurent en place ; la supervision récente des descendants du runner de parité n'a pas été transplantée ici.

**Programme et continuation sont figés avant admission.** Le programme commun reste [la référence à six clés](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json), SHA `f14de3871ef9aba18083b36d10eeb56c8587c66a73b1ab2ec54e01830e06e59d`, avec [les tâches à 60 Hz](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json), SHA `a96d03d736ca185a0457ae29c33846b69af28ecd5eb81609f60d281463756dd6`. Les clés sont à 0, 0,70, 1,25, 1,80, 7 et 8 s ; le dernier intervalle conserve la cible debout pendant une seconde. Les deux archives natives couvrent chacune les 1 921 états à 240 Hz, sans refus à 0° et 60°. Les sources `trajectory_program.rs`, `trajectory_geometry.rs` et `planner/continuation.rs` correspondent encore aux hashes du gel original. Le builder vérifie ces identités, la liaison programme/tâches et les hashes des traces compressées, sans recalcul géométrique.

Ces archives prouvent une route géométrique complète, pas l'équilibre chargé du transfert : les talons à environ 1 mm du sol et les points de pivot conservés restent des cibles. Le CoM initial de la fixture peu fléchie est à environ 1,0103 m ; aucun settle physique caché ne transforme l'entrée en accroupi profond. La racine inspectée n'est jamais une commande corporelle. La géométrie ne prouve ni les charges, ni les couples réellement livrés, ni la conservation de l'appui pendant l'extension.

Le [driver actuel](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/rust/crates/sim/src/recovery/planner/transition_worker.rs) réévalue B aux pas 0,24,…,936, soit au plus 10 Hz. Chaque appel copie l'état natif complet et le diagnostic, puis exécute les quatre échelles **0 ; 0,5 ; 1 ; 1,5**, sans recherche récursive. Le contrôleur commun travaille à 60 Hz et l'installateur natif à 240 Hz ; modèle, profil v24, anatomie, gains, frottement et caps restent inchangés. Une copie n'est utilisable que si elle termine ses 480 pas sans refus. Le score lexicographique existant privilégie diagnostic acquis, CoM/capture dans le support, charge minimale des talons, fraction de support, puis faible vitesse du CoM ; une égalité conserve la première échelle. Aucun réglage de score ou de famille n'est proposé.

La première tranche réelle de quatre pas après sélection est comparée à sa copie avant de poursuivre. La preuve courte actuelle ne garantit pas l'admissibilité de tous les ticks suivants : un nouveau refus numérique ou une divergence ferme le lot. La dernière prévision commence à 3,9 s et peut aller à 5,9 s de temps simulé ; ses 480 pas sont déjà comptés. Cette copie au-delà de quatre secondes n'autorise aucune continuation réelle. La référence de huit secondes fournit les cibles nécessaires.

Les [critères locaux](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/local-pair-criteria.json) restent ceux du prototype : talons natifs du hindfoot avec `local_z < −0,02 m`, chacun ≥3 N pendant ses propres douze pas ; support admis ≥80 % du poids ; aucun appui non-pied persistant ≥3 N ; CoM et capture dans le support observé avec marge de 3 mm ; les 25 corps à ≤0,22 m/s et ≤0,65 rad/s. Exiger **132 échantillons consécutifs**, dont le début est **strictement avant 2 s**, et l'histoire réelle complète de 960 pas sans refus. La variable `passed` mémorise un intervalle réussi ; elle ne certifie pas la posture terminale à 4 s. Les faibles charges non-pieds doivent aussi être rapportées.

Un diagnostic A négatif permet B si l'intégrité technique de A est valide. Une violation d'intégrité, un refus de support/prévision ou l'absence de diagnostic ferme immédiatement l'admission. B négatif ferme le lot sans substitution de commande. Si A et B réussissent, un avantage propre de la coordination n'est pas établi ; si B seul réussit, l'effet concerne l'ensemble coordination + choix prédictif dans cette paire. Une répétabilité ou une récupération complète ne découle d'aucun de ces cas.

Pour rendre le prochain gel concret, il reste à :

1. **Reconstruire séparément le `coordination-worker` corrigé**, sans démarrage et sans compilation simultanée au futur lot. Le binaire de parité `numeric_parity_worker_serialization` est une autre entrée ; son succès ne certifie pas directement le nouveau binaire. Conserver la commande de build, la toolchain, le log et le hash du binaire. Aucun binaire n'a été reconstruit pour cette note.
2. Conserver les contrôles purs courants du solveur, de la certification f32, des tâches observées, de la continuation, du diagnostic et de la comptabilité. Employer des filtres explicitement sans intégration ; une suite Rust générale contiendrait des tests physiques non autorisés. Conserver la revue du propriétaire unique, des copies natives, des moteurs et du handoff original.
3. Utiliser le [nouveau builder](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/prepare_local_pair_manifest.py) avec `--binary CHEMIN --template RECU_PENDING.json`. Il produit des prérequis **pending**, jamais vrais par défaut. Pour chaque prérequis, fournir ensuite `status: "passed"` et une liste `evidence` de `{path,sha256}` pointant sur une preuve réellement obtenue. `checkedFiles` lie la revue aux versions exactes des sources, dépendances vendues, tests, binaire, critères et entrées. Le builder contrôle la complétude et les empreintes ; une attestation demeure une responsabilité de revue, pas une preuve scientifique automatiquement déduite de son booléen.
4. Exécuter le builder avec `--binary CHEMIN --readiness RECU.json --output NOUVEAU_MANIFESTE.json`. Les sorties sont créées exclusivement ; preuve manquante, pending, périmée ou non liée au code courant = refus. Le builder ne crée aucun registre, n'importe aucune admission et n'exécute rien. Après revue finale et annonce préalable au parent de la justification et du plafond distinct, un **nouveau** répertoire de registre/trace pourra être créé avec l'origine et les consommations de préparation conservées. La consigne du propriétaire permet de poursuivre sans nouvelle question d'autorisation de routine ; les prérequis techniques restent nécessaires. Ni les anciens registres clos ni leur enveloppe ne sont réouverts.

Les tests factices du registre/runner passent : [**87**](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/local-pair-budget-tests.log) + [**29**](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/local-pair-runner-tests.log), dont 13 nouveaux cas couvrant clôture positive/négative de B, plafond exact, absence de réexécution et invariance du préflight. Le builder dispose de [**12 tests purs**](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-serialization-2026-10-13/local-pair-manifest-tests.log) : reçus pending/incomplets/périmés refusés, ancien mode refusé, preuves modifiées refusées, sortie existante préservée, inclusion des données/configurations de compilation vendues et archives de continuation cohérentes. Ces tests ne satisfont pas à eux seuls les autres prérequis Rust ni la reconstruction ci-dessus. Aucun manifeste prêt à exécuter n'est produit par cette livraison.

**Décision de continuation après la paire :** si B est refusé ou ne satisfait pas le diagnostic, archiver le premier refus et sa provenance ; ne pas prolonger, remplacer la référence ou régler une variante dans ce lot. Si B réussit, examiner les charges, la dérive des points matériels, les efforts et le raccord à 4 s ; demander au parent une décision distincte pour l'histoire complète. Le driver actuel finit à 960 pas et n'est pas un driver de continuation.

Le [plan antérieur de 26 s](https://github.com/berenif/LaboratoireHumain/blob/83b8447285333ec2feb765781c3d84aac3d2e4dd/docs/evidence/coordination-native-parity-renewal-2026-10-13/next-causal-milestone.md) demeure une proposition d'extension, chiffrée à 204 480 pas hors admission : deux histoires de 6 240 pas et au plus 100 appels de quatre copies de 480 pas. Une nouvelle exécution exigeant verify16 coûterait au plus **204 496**, et non un crédit récupéré sur la paire. Il faudrait garder les mêmes instances pendant toute chaque histoire, préserver le settle/handoff et exiger le terminal original `recoveries == 1` avec `Motion::Upright` au plus tard à 25 s, puis 240 pas réels de stabilité. Le maintien de cible 7–8 s et le latch local ne remplacent pas ce terminal. Cette extension n'est ni implémentée ni admise par le mode local-pair ; `fullRecoveryQualified` et `fullHistoryAuthorized` restent faux.
