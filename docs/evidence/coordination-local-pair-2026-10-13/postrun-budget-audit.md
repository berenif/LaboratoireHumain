# Audit budgétaire après clôture de la paire A/B — 13 octobre 2026

Audit indépendant réalisé par lecture des registres, reçus, journaux durables avant/après intégration, traces et code gelé. Aucun calcul physique, FK, renderer, compression ni archivage exécuté pour cet audit. Seul ce nouveau document est écrit ; aucun registre n’est modifié.

**Le lot est clos en échec après exactement 1 040 intégrations documentées : vérification 16 + A 960 + prévisions B 64 + B réel 0. Les 3 856 pas réservés restent entièrement débités.** Le registre parent conserve intentionnellement une borne moins précise, **1 040–2 000**, car il n’a pas admis le reçu final de B après l’échec de planification. Cet audit rapproche les preuves externes sans modifier cette borne officielle.

## Réconciliation de chaque opération

| Opération | Réservés au parent | Réalisés inscrits au parent | Preuve native indépendante | État au parent |
| --- | ---: | ---: | ---: | --- |
| Vérification A/B : copies et originaux | 16 | 16 | 16 | `completed` |
| Histoire A | 960 | 960 | 960 | `completed` |
| Histoire B réelle | 960 | Non renseigné | 0 | `reserved` dans un lot clos |
| Premier appel de planification B | 1 920 | 64 | 4 × 16 = 64 | `failed-reported` |
| **Total** | **3 856** | **1 040 connus** | **1 040 exacts documentés** | **`closed-native-failure`** |

Le registre `33c9dd5f-73f2-472a-a2eb-b866ae7c9f2b` annonce `actualNativeStepsExact=false`, `unreportedReservedNativeSteps=960`, `actualNativeStepsLowerBound=1040`, `actualNativeStepsUpperBound=2000` et `authorizesIntegration=false`. Ces champs sont cohérents avec ses quatre réservations ; le zéro de B ne lui a pas été ajouté rétroactivement.

La précision supplémentaire est établie par plusieurs preuves concordantes :

- Les journaux `*.json.accounting.jsonl` contiennent respectivement **16, 960, 64 et 0** couples admission/fin pour vérification, A, planification et B. Chaque admission est suivie de sa fin, sans compteur manquant ; les compteurs de fin sont continus depuis 1 jusqu’au total de leur permis.
- Le code gelé place `before_integration` immédiatement avant `world.step_with_events` et `after_integration` immédiatement après. Chaque checkpoint est écrit, vidé puis synchronisé par `sync_data`. Les quatre journaux se terminent avec admission = complétion, `actualNativeStepsExact=true` et aucun emplacement natif en attente.
- Le journal réel B n’a qu’une ligne `permit-activated`, avec 0 admis et 0 terminé. Son [rapport final](native-run/002-B.json) confirme `actualNativeSteps=0`, `localReservedRealSteps=0`, `workerAccountingExact=true`, `workerActualAllOperations=1040` et `workerReservedAllOperations=3856`. Ses poses initiale/finale sont identiques, avec horloge 0.
- Les traces contiennent **16** états de vérification (quatre groupes de quatre), **960** lignes `actual-history` de A aux pas 1–960, et **64** lignes B exclusivement prédictives (quatre groupes de pas 1–16). Aucune ligne réelle B n’apparaît.
- Le protocole reçoit la planification en échec, puis ferme l’admission. Il ne contient aucun `continue-history`, second appel de planification, nouvelle histoire ou nouveau permis.

Les **80** réservations locales du rapport prédictif sont des emplacements de ticks : 64 intégrés et **16 abandonnés avant intégration**, quatre par branche. Elles sont incluses dans les 1 920 pas du permis parent ; elles ne sont ni 80 intégrations ni un débit supplémentaire. Au total, les **2 816 pas réservés mais non intégrés documentés** — 960 pour B réel et 1 856 pour ses copies — restent consommés.

## Arrêt et portée du résultat

A termine 4 s sans refus d’intégrité, mais ne franchit pas le diagnostic local. Les quatre prévisions B sont refusées au tick 4, avant le pas 17, par la projection des contraintes du solveur dans son budget fixe. Chacune a donc exécuté **16/480 pas** ; aucune n’est utilisable, aucune échelle n’est sélectionnée et aucun pas réel de B ne suit.

Le champ parent `integrity_ok=false` est un motif de fermeture du protocole ; il ne prouve pas un dépassement physique de couple ou une impossibilité anatomique. L’échec observé est l’absence de prévision admissible dans cette famille figée. Le succès de parité courte antérieur ne qualifiait pas les 480 pas demandés. Aucune comparaison causale A/B réelle complète, acquisition des talons ou récupération complète n’est établie.

## Gel, preuves et processus

**347/347 entrées gelées** concordent avec leurs empreintes, ainsi que les **7 anciennes pièces protégées** — quatre registres clos et trois binaires historiques. Les trois reçus admis par le parent (vérification, A, planification) correspondent à leurs empreintes enregistrées. L’enveloppe SHA-256 du registre est valide ; le résultat du runner reproduit ses compteurs et son statut clos.

Le manifeste sur disque et sa version normalisée dans le registre ont des empreintes différentes, conformément au code : la normalisation conserve fichiers/prérequis/mode mais retire `workerInputs`. Le contenu normalisé recalcule exactement l’empreinte stockée ; les 347 entrées et prérequis sont identiques dans les deux représentations. Le manifeste sur disque correspond aussi au hash du préflight. Aucune anomalie de gel n’en découle.

Un seul PID **12156**, un thread, binaire `c1c8a12825428758643a7f6e2fafb0a2710f7dca787c1daee07c27ec59849e9c`. Tous les permis portent ce même PID et la même échéance native. Le PID a disparu ; le recensement en lecture de `/proc/*/exe` après clôture ne trouve aucun worker natif de coordination ou de parité actif. Les six essais de supervision avec programme C inerte restent des tests sans physique et ne sont pas ajoutés au compteur natif.

## Délais : mesures et bornes distinctes

- Première réservation native : **11:10:50.135208 UTC** ; clôture durable : **11:11:02.456529 UTC**. Fenêtre mesurée par ces dates : **12,321321 s**.
- Le début monotone de supervision précède cette réservation de **0,079525382 s**. Du début de supervision à la clôture du registre : environ **12,400846382 s**, en combinant ce décalage monotone avec les dates UTC.
- Le reçu de supervision ne donne pas l’instant monotone final de récupération du processus : ces nombres ne sont donc pas présentés comme une durée exacte de tout le lancement/arrêt. L’échéance externe est bornée à **7 137 s** (30 + 7 100 + 7), et l’échéance native commune à **13:09:10.135208 UTC**. La clôture intervient bien avant ces plafonds ; aucun délai n’a été renouvelé.
- La somme des fenêtres documentées de la famille est désormais **21,951356994 s** : 9,630035994 s antérieures + 12,321321 s présentes. Les démarrages/arrêts historiques incomplets empêchent d’en faire une durée totale exacte. La borne conservatrice avant exécution de **7 183,630036 s**, démarrages/arrêts inclus selon les phases bornées, demeure inférieure aux 7 200 s familiales ; ce résultat effectif ne requiert aucune extension.

## Cumul familial et préparation

| Poste | Débit après clôture |
| --- | ---: |
| Ancien prototype et trois lots techniques | 3 988 |
| Nouvelle paire : permis effectivement réservés | 3 856 |
| **Famille** | **7 844 / 300 000** |
| Histoires réelles réservées, quatre emplacements | 3 840 |
| Prévisions longues réservées, deux appels B | 3 840 |
| Vérifications techniques, tous lots compris | 164 / 21 120 |

Les **82 724** pas de la préparation étaient la projection si les 40 appels B du nouveau lot étaient tous réservés. Le compteur effectif est **7 844** : les appels jamais admis ne constituent pas un remboursement. Les **quatre emplacements d’histoire sont utilisés**, en comptant les deux B réservés même lorsqu’ils n’intègrent aucun pas réel. Ni les sous-pas inutilisés ni l’arrêt précoce n’ouvrent une cinquième histoire, un B60 ou un renouvellement automatique.

La somme des quantités connues dans les registres familiaux devient **2 108** ; le total familial exact reste non certifié à cause des anciens compteurs inconnus, notamment le renouvellement technique clos après erreur de sérialisation. La certitude propre au présent lot n’efface pas cette incertitude historique.

Préparation : **28 489 s réservées sur 144 000 s**, **9 173 inspections sur 20 000**, échéance générale inchangée au **17 octobre à 20:14 UTC**. Le plafond de préparation supplémentaire de 3 600 s n’est pas renouvelé et aucun débit n’est remboursé. Les deux campagnes historiques hors famille et les tests factices restent exclus pour les raisons détaillées dans [l’audit cumulatif avant lancement](cumulative-budget-audit.md).

Les reçus de post-traitement produits séparément par le parent ont été lus : `archive-index.json` déclare zéro intégration ; `rendered/render-receipt.json` déclare zéro intégration, FK et requête collision. Hachage, compression, transformation visuelle des poses stockées et arithmétique sur matrices QP archivées consomment la préparation déjà réservée ; ils ne constituent pas de nouvelles inspections géométriques. Le compteur reste donc 9 173 et la charge conservatrice 28 489 s. Cette distinction suppose de rester sur les données archivées, sans reconstruire un monde ni recalculer la cinématique ; elle ne certifie pas le résultat du contrôle rationnel encore en cours au moment de cette note.

## Empreintes principales

| Pièce | SHA-256 |
| --- | --- |
| Registre clos `budget/ledger.json` | `e7c6831159ac1ae913a05f2af545a0521181d7142dbabd967908dfe7958fba97` |
| Manifeste sur disque | `2fbcb3db0cd1bccf14082e772b1efba63e567fe7fce35ed8cccbbbf2ac2cdde9` |
| Manifeste normalisé du registre | `2e6a9fe308763d940b0076f8b172bc88557bfaa8d06f30e9c7090043b6470604` |
| Rapport final B, non admis par le parent | `72ec39bf64e1f637cc1ed1099eff42cf8836ade2c21bb99dc12c843f502aa723` |
| Journal durable de vérification | `fcde8d9825d21d5892fb7a31adf382adb40225f0d0fd7d0e01bff5a2477ee8fa` |
| Journal durable A | `a66c893eb4d89d7a127ad04b999e4deb0fa9239e74485d0a42b385517156d406` |
| Journal durable des prévisions B | `8f277ded9d8a2c0226698e30561d78420d4438f9b59c21da5e4f6e82f97536c9` |
| Journal durable réel B | `d0449e433e8a0ac8eb05fff998e45c93fc2f95ddfa4f95839a58a4cd425a8965` |
| `native-run/run-result.json` | `ade23ba9efb1462f96780b86ecd66d910385a341952ab78535b5d8d00033b846` |
| `native-run/protocol.ndjson` | `f6af31c19c1064b7bea4f574797859eaefe6ec44e80cde5c51a00c1254b924aa` |

**Conclusion : clôture cohérente, comptabilité conservatrice intacte, aucun dépassement identifié. Aucun nouveau calcul physique ou relancement n’est autorisé par cette note.**
