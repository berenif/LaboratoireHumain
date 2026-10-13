# Audit cumulatif avant la dernière paire locale — 13 octobre 2026

Lecture indépendante des registres, reçus, protocoles et réservations de préparation. Aucun worker, test natif, calcul géométrique ou intégration physique exécuté pour cet audit ; aucun ancien registre modifié.

**La paire prévue tient dans les plafonds cumulatifs initiaux : 82 724 sous-pas débités au maximum sur 300 000, quatre histoires sur quatre et 28 489 secondes de préparation réservées sur 144 000.** Elle consomme les deux derniers emplacements d’histoire. La nouvelle instruction explicite A/B remplace, pour ces emplacements, les B répété/B60 du plan initial ; elle ne crée pas une cinquième histoire. Le délai natif de la paire est réduit à **7 100 s**, distinct du plafond général historique de 7 200 s.

Il s’agit d’une admissibilité comptable avant gel/exécution, pas d’un succès physique ni d’une admission automatique des prérequis techniques. Le nouveau registre doit rester distinct ; les quatre registres physiques antérieurs demeurent clos.

## Périmètre de la famille

Origine : [préparation du nouveau prototype](../coordination-prototype-2026-10-12/preparation-start.json), **12 octobre à 20:14 UTC**, sous l’autorisation de revoir et implémenter la coordination. Plafonds : quatre histoires de 4 s/960 pas chacune, 300 000 sous-pas comprenant toutes copies et vérifications, 2 h de calcul physique, 40 h de préparation active, 20 000 inspections statiques et échéance calendaire **17 octobre à 20:14 UTC**.

Les autorisations techniques distinctes de parité sont conservées séparément et également débitées dans cette synthèse familiale. Aucun échec, arrêt précoce, zéro reconstruit ou absence de reçu ne rembourse sa réservation.

## Tous les débits physiques antérieurs

| Lot / opération | Débit réservé | Réalisé inscrit au registre | Observation complémentaire |
| --- | ---: | ---: | --- |
| Prototype : vérification A/B, originaux et copies | 16 | 16 | Admission technique terminée. |
| Prototype : histoire A | 960 | 960 | 4 s terminées, diagnostic local négatif. |
| Prototype : histoire B | 960 | Non renseigné | Histoire réservée et lancée ; reçu final B indique 0 pas réels. Son premier appel prédictif échoue. |
| Prototype : premier appel B, quatre prévisions | 1 920 | 48 | Quatre branches de 12 pas ; aucun second appel ni sonde courte. |
| Parité du 13 octobre, autorisation 06:44 | 44 | `null` | Arrêt du superviseur avant création du worker ; zéro intégration établi par le chemin d’échec, sans réécriture du registre. |
| Renouvellement de parité, autorisation 10:06 | 44 | `null` | Worker lancé ; erreur de sérialisation après l’état initial. Borne documentaire prudente 0–1 appel Rapier ; exactement 1 non mesuré. |
| Correctif de sérialisation, lot technique distinct | 44 | 44 | Préfixe 12 + quatre paires copie4/réel4 ; parité courte passée. |
| **Total antérieur définitivement débité** | **3 988** | **1 068 connus** | **La somme connue n’est pas un total physique exact certifié : les anciens champs inconnus restent inconnus.** |

Le premier registre charge 3 856 pas, les trois lots techniques 132 de plus. Les 64 réservations internes par ticks mentionnées dans le premier rapport prédictif ne sont pas un autre débit : elles sont couvertes par les 1 920 pas de son permis parent. Les préfixes des lots de parité sont compris dans leurs 44 pas ; les lignes de comptage répétées après tick ne sont pas des intégrations supplémentaires.

Sources : [audit du prototype](../coordination-prototype-2026-10-12/budget-final-review.md), [premier lot de parité](../coordination-native-parity-2026-10-13/post-run-budget-audit.md), [audit du renouvellement](../coordination-native-parity-renewal-2026-10-13/independent-budget-audit.json), [audit du lot de sérialisation](../coordination-native-serialization-2026-10-13/post-run-budget-audit.md), ainsi que les quatre registres physiques listés plus bas.

## Histoires et nouvelle paire

| Emplacement familial | Histoire | Charge maximale | État à cet audit |
| --- | --- | ---: | --- |
| 1 | A initiale, 0° | 960 | Terminée. |
| 2 | B initiale, 0° | 960 | Réservée et engagée jusqu’aux prévisions ; conserve son emplacement malgré 0 pas réels. |
| 3 | Nouvelle A, 0° | 960 | Prévue par la nouvelle instruction A/B. |
| 4 | Nouvelle B, 0° | 960 | Prévue par cette même instruction. |

Les essais techniques de 4 pas et leur préfixe commun de 12 pas sont des vérifications déjà explicitement budgétées, pas des histoires supplémentaires de récupération. Aucun reset au milieu d’une histoire ni changement de fixture/référence entre la nouvelle A et B n’est admis. Les deux anciens emplacements ne sont pas remis à disposition.

La nouvelle instruction déléguée demande explicitement une unique paire A/B. Elle modifie donc la destination des emplacements 3–4, initialement prévus pour B répété et B60 uniquement après succès de B. Ce remplacement est déclaré avant lancement ; aucune répétition ni essai à 60° supplémentaire ne reste dans le plafond initial de quatre histoires.

| Nouveau poste | Maximum |
| --- | ---: |
| Vérification dans le même processus | 16 |
| Deux histoires réelles de 960 pas | 1 920 |
| B : 40 appels × 4 copies × 480 pas | 76 800 |
| **Nouveau lot** | **78 736** |
| **Famille, réservations antérieures comprises** | **82 724 / 300 000** |

Contrôles croisés : histoires réelles réservées **3 840 / 3 840** ; prévisions et vérifications **78 884 / 296 160** ; seules vérifications techniques **164 / 21 120**. Zéro sonde courte prévue. La marge arithmétique de 217 276 pas ne crée aucun droit à une autre histoire ou à un renouvellement ; le lot doit se fermer après B ou au premier refus applicable.

## Temps mural cumulé

Durées déjà inscrites ou déduites des dates de clôture :

| Lot | Secondes |
| --- | ---: |
| Prototype, première réservation → clôture | 7,930429 |
| Premier lot technique | 0,028446437 |
| Renouvellement technique | 0,193373985 |
| Correctif de sérialisation | 1,477786572 |
| **Total documenté** | **9,630035994** |

Un nouveau délai natif plein de 7 200 s ne respecterait pas un plafond familial cumulé de 2 h. Ce point a été signalé au parent, qui a réduit **uniquement `LOCAL_PAIR_LIMITS.physicsWallSeconds` à 7 100 s**, en conservant le mode historique inchangé. [La réservation nouvelle](preparation-reservation.json) prévoit 30 s de démarrage et 7 s d’arrêt, soit **7 146,630036 s** cumulées pour ces phases et les durées documentées.

Le temps historique de 7,930429 s commence à la réservation de vérification : il n’inclut pas le démarrage et l’arrêt de ce premier worker. En ajoutant conservativement **30 + 7 s** pour ces deux phases anciennes non mesurées, le total projeté des phases bornées est **7 183,630036 s**, laissant **16,369964 s** sous 7 200 s. Cette somme est une borne de phases, pas une mesure passée ni une preuve de supervision à elle seule : les délais du lanceur et l’arrêt du processus doivent effectivement être respectés. Le démarrage, les I/O et la clôture ne doivent pas être présentés comme inclus dans le seul délai natif de 7 100 s. Aucun dépassement ou renouvellement après délai ne peut être justifié par la marge en sous-pas.

## Préparation, sans remboursement ni double comptage

| Jalon | Charge supplémentaire (s) | Cumul (s) | Inspections réservées cumulées |
| --- | ---: | ---: | ---: |
| Préparation initiale : 589 antérieures + 7 200 réservées | 7 789 | 7 789 | 6 923 |
| Correction numérique pure | 7 200 | 14 989 | 7 923 |
| Préparation de la parité du 13 octobre | 7 200 | 22 189 | 8 923 |
| Renouvellement et correction de sérialisation | 1 800 | 23 989 | 9 173 |
| Réserve finale de préparation/livraison | 900 | 24 889 | 9 173 |
| Nouvelle préparation de la paire locale | 3 600 | **28 489** | **9 173** |

Plafond conservé : **144 000 s** ; marge arithmétique **115 511 s**. Inspections : **9 173 / 20 000**, sans nouvelle inspection réservée pour cette phase ; marge arithmétique **10 827**. Ces charges ne prétendent pas mesurer le temps actif exact ni le nombre effectif cumulé d’inspections. Les réservations chevauchantes de 900 s et 3 600 s restent toutes deux entièrement chargées.

La nouvelle réserve commence le **13 octobre à 10:44:39.937862 UTC**, finit au plus tard à **11:44:39.937862 UTC** et conserve l’échéance générale du **17 octobre à 20:14 UTC**. Les deux inspections initiales et les inspections FK/géométrie des sous-registres de continuation sont déjà incluses dans la charge initiale ; les recompter ici gonflerait artificiellement la dépense. Ces inspections ont déclaré zéro intégration physique. La correction de portabilité et le correctif de sérialisation réutilisent les réserves de préparation annoncées ; ils ne créent pas une seconde charge égale à leur cumul recopié.

## Exclusions explicites

- `coordination-launcher-portability-2026-10-13/e2e` : **16 essais de processus C factices**, 13 claims de l’autorisation `SYNTHETIC-launcher-portability-physical-integrations-zero`. Les nombres 12/44 de leurs reçus sont fabriqués. Sources et audit déclarent zéro worker Rust/Rapier et zéro intégration physique ; ils sont exclus du total natif et du nombre d’histoires. Leur développement relève néanmoins de la préparation, déjà réservée.
- `foot-coordination-2026-10-12` : campagne distincte autorisée à 11:39 UTC, avant l’origine de cette famille. Ses **2 histoires, 12 480 pas réservés et 6 235 connus** restent dans son propre registre clos, non ajoutés à l’enveloppe accordée à 20:14 UTC et non effacés.
- `intermediate-transfer-prep-2026-10-12` : campagne distincte autorisée à 15:01 UTC, close sans ouverture de budget physique, **0 nouvelle histoire et 0 pas natif**. Ses anciens registres référencés (`getup-ab`, `native-hybrid`, `recovery-coordinated-retarget`) sont historiques, antérieurs et hors de cette famille ; aucune réexécution n’est impliquée par leur consultation.
- Tests Python synthétiques, tests Rust purs explicitement filtrés, lectures de traces, hachage, builds sans lancement et comparaisons archivées ne sont pas des intégrations. Le journal initial `native_plan_preview-tests.log` distingue notamment son noyau factice et la copie statique sans intégration. Aucune nouvelle physique ne résulte du présent audit.

## Empreintes et état des prérequis

Les quatre registres physiques et les trois binaires historiques protégés par la nouvelle réservation ont été relus et leurs **7 empreintes concordent**. Le hash du prédécesseur de préparation correspond également à son fichier.

| Registre ou réservation | SHA-256 |
| --- | --- |
| Prototype : `budget/ledger.json` | `2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07` |
| Parité initiale : `physical-ledger.json` | `73b2e3bff52aadf0414be2f7a4a96fa4cd5da7b02de0945434e4f136bbf476a6` |
| Renouvellement : `physical-ledger.json` | `1411bfcc1a17caa681359cfb39d7c0429f2188375925c22ba4b037df96d44bc2` |
| Correctif de sérialisation : `physical-ledger.json` | `191d3216953bfb4aca5b7abcea277454d7d97ad5b4ca72e28ffbc260399af250` |
| Réserve finale de préparation de 900 s | `23c8b93f7a5eb7c20af2c518ea119a36b143c22897ff245e1191ec6e58dff27b` |
| Nouvelle réserve de préparation de 3 600 s | `971e85ade9ba0eb322ad0151866eee6d43110f4b8152a946be65e477ddf7c0c8` |

Au moment de cet audit, la nouvelle réservation indique `nativeExecutionStarted=false`. Le budget est compatible avec le périmètre corrigé ci-dessus. Restent à produire et geler les preuves techniques de la nouvelle paire ; le succès des 44 pas précédents ne garantit ni une prévision de 480 pas ni la faisabilité locale. Aucun relevage complet ou qualification de 24 cas n’est autorisé ou certifié par cet audit.
