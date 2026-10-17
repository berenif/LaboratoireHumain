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
