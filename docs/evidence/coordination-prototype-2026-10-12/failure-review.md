# Revue du refus natif — 12 octobre 2026

Le prototype est **arrêté sur un refus numérique de production de commande**, avant toute application du contrôle coordonné dans l'histoire réelle B. Ce résultat ne démontre ni une impossibilité anatomique, ni une impossibilité physique du mouvement. La continuation physique complète de 8 s reste refusée ; le protocole est clos, sans retouche ni relance.

Cette revue lit les preuves déjà produites et le code gelé au commit `b6e711d3dc58909979295df81437e6a5579c41db`. Les sommes SHA-256 des fichiers de solveur, compilateur, propriétaire de commande, tâches géométriques, worker et preview lus concordent avec le [manifeste gelé](frozen-manifest.json). Travail effectué : lecture du code, extraction et comparaison de JSON existants. Aucun solveur, test, FK, inspection géométrique ou sous-pas physique supplémentaire n'a été exécuté. Seule cette note est ajoutée.

## Ce que le lot a effectivement exécuté

| Étape | Intégrations natives | Résultat établi |
|---|---:|---|
| Vérification A/B, copie et réel | 16 | Parité exacte des quatre premiers pas de chaque mode |
| A, référence commune sans correction coordonnée | 960 | 4 s complètes, aucun refus d'intégrité original, diagnostic local jamais satisfait |
| Première planification B, quatre copies | 48 = 4 × 12 | Chaque copie s'arrête à 50 ms, avant la treizième intégration |
| Histoire réelle B | 0 | Aucun candidat sélectionné ; aucune commande B appliquée |
| B répété, B à 60°, continuation de 8 s | 0 | Non exécutés |

Total connu : **1 024 intégrations**, pour **3 856 réservées**, conformément au [résultat du lot](native-run/run-result.json). Les réservations restent consommées. Les 64 réservations locales du premier planning sont cohérentes avec quatre copies ayant chacune réservé quatre ticks : trois ticks intégrés, puis le tick refusé avant son premier sous-pas. Les 1 920 pas du permis de planning et les 960 pas du permis B ne sont pas des pas réellement exécutés.

Le [rapport de vérification](native-run/000-native-verification.json.gz) certifie uniquement les quatre premiers pas. Ceux-ci appartiennent au préfixe commun sans appui matériel acquis par le propriétaire coordonné ; le premier solveur avec contraintes d'appui intervient au pas 12. Cette vérification ne constitue donc pas une observation de parité dans le régime coordonné actif, et ne certifie pas un horizon de 2 s.

## Localisation exacte du refus B

Le [premier planning](native-run/002-B-planning-000.json.gz) possède quatre cas inutilisables, sans échelle sélectionnée. Le [journal B](native-run/002-B.states.jsonl.gz) contient exactement quatre groupes de douze états, tous nommés `preview-0-*`, et aucune ligne d'histoire réelle. Les snapshots physiques et moteurs natifs des quatre groupes sont strictement identiques à chacun de leurs douze pas.

Au dernier pas intégré, les avant-pieds gauche et droit ont chacun douze pas de charge persistante. Au début de la commande suivante, le propriétaire acquiert deux patches `Hold` sur les segments 19 et 24, puis tente sa première résolution contrainte. Toutes les erreurs portent `check: ownership`, `tick: 3`, `substep: 0`. Elles surviennent avant l'appel d'intégration du treizième pas, et avant l'installation de la commande candidate.

| Échelle | Refus archivé | Conclusion prouvée |
|---:|---|---|
| 0 | `Emitted rates violate a hard task or total command bound` | La solution continue f64 est acceptée, puis au moins une des quatre commandes f32 d'essai est refusée par la vérification des contraintes |
| 0,5 | `Common task solve did not converge within fixed iteration budget` | La boucle extérieure de résolution atteint sa limite de 512 itérations sans satisfaire son critère d'arrêt |
| 1 | Même refus | Même portée |
| 1,5 | Même refus | Même portée |

L'échelle 0 **n'est pas le témoin A** : `enabled` reste vrai, les contraintes dures, le solveur et l'objectif de posture restent actifs. Cette échelle annule les poids de correction CoM/orientation. La famille de quatre candidats ne permet pas de substituer silencieusement A à un planning refusé.

## Cause établie et limites de l'explication numérique

Pour l'échelle 0, [`whole_body_controller.rs`](../../../rust/crates/sim/src/recovery/planner/whole_body_controller.rs) appelle `problem.solve()`, stocke les taux f64, puis compile et vérifie successivement les quatre commandes futures. Le message observé ne peut provenir que de `problem.verify_rates()`. La résolution f64 avait donc accepté ses contraintes et sa convergence ; `runtime::command()` avait réussi pour la commande examinée, y compris ses contrôles de finitude, d'anatomie et de vitesse totale. Le refus établit une **discordance entre les taux f64 acceptés et les taux effectivement représentés par la commande f32**, sous le même problème de contraintes. Il ne constitue pas un refus anatomique du compilateur.

[`coordinated_runtime.rs`](../../../rust/crates/sim/src/recovery/planner/coordinated_runtime.rs) arrondit la position à f32, puis calcule la vitesse depuis la différence entre les positions f32 divisée par le pas natif. Les taux réellement émis peuvent donc différer des taux optimisés. [`whole_body_tasks.rs`](../../../rust/crates/sim/src/recovery/planner/whole_body_tasks.rs) réapplique une tolérance numérique de `1e-8` aux contraintes et bornes. Le mécanisme de désaccord de représentation est établi par ce chemin de code ; **la ligne fautive, le dépassement exact et le rang de la commande parmi les quatre ne sont pas archivés**. Il serait injustifié d'affirmer une erreur mesurée de telle amplitude, ou d'attribuer le refus à un talon, une articulation ou une borne précise.

Pour les trois autres échelles, le message terminal provient exclusivement de la sortie de la boucle extérieure après 512 itérations. Les projections internes sont revenues `Ok` ; une erreur de projection dure aurait produit un autre message. Ces cas ne prouvent donc pas l'infaisabilité des contraintes. De plus, les quatre copies ont le même état physique, les mêmes patches et les mêmes bornes dures ; l'échelle modifie leurs objectifs souples. L'acceptation du vecteur f64 de l'échelle 0 constitue ainsi une preuve de faisabilité numérique du système dur commun à la tolérance du solveur. Elle ne prouve pas l'existence d'une commande f32 acceptable sur les quatre pas, ni la faisabilité dynamique de ce mouvement.

Le solveur emploie un gradient projeté, une borne globale de courbure par somme des normes des lignes pondérées, un démarrage à la projection de zéro, et un seuil de résidu `1e-6`. Une progression trop lente liée à cette borne, au conditionnement ou aux contraintes actives est une explication plausible de l'épuisement des itérations. **Aucun résidu final, historique de convergence, nombre de cycles de projection ni valeur de courbure n'est archivé pour ces échecs** : leur rôle exact demeure une hypothèse. La revue ne résout pas de nouveau le problème et ne propose aucun réglage à appliquer au lot clos.

## Attention à la mémoire archivée et aux indicateurs agrégés

Le propriétaire n'est pas perdu lors de l'erreur : `Simulation::configure_motors()` restaure `coordination` après `owner.configure()`, même lorsque celui-ci retourne une erreur. Le rapport conserve donc une mémoire **partiellement mise à jour**, pas seulement l'état de la dernière commande installée.

À l'échelle 0, `terminalOwner.rates` contient le candidat f64 obtenu avant le refus ; son maximum absolu vaut environ `0,970556 rad/s`. En revanche, `last.commandSubstep` vaut toujours 11, `last.solver` est nul et `nextSubstep` reste 12. Aux trois autres échelles, la résolution ayant échoué avant l'affectation, les taux mémoire sont ceux du préfixe précédent. Aucun de ces champs ne permet de présenter le candidat refusé comme une commande appliquée. Le compte rendu de résolution est écrit seulement après la validation des quatre commandes, ce qui explique la perte des détails d'échec.

Les quatre booléens de l'`outcome` du planning sont affectés ensemble depuis `best.is_some()`. Leur valeur fausse signifie ici « aucune prédiction admissible », **pas quatre diagnostics indépendants d'intégrité physique, d'infaisabilité d'appui, de prédiction et d'absence de mesure**. Des diagnostics locaux existent bien pour les 48 pas des copies. Le `integrity_ok:false` final agrège un refus logiciel d'ownership ; il ne doit pas être décrit comme une violation physique mesurée. La même prudence s'applique à `support_feasible:true` du témoin A : ce champ ne certifie pas le transfert recherché.

Les copies déclarent `ccdJournalCollected:false`. Elles archivent contacts courants, charges persistantes et états/moteurs, mais aucune impulsion CCD journalière ; aucune certification du bilan angulaire n'est revendiquée. Leurs charges d'appui ne doivent pas être confondues avec des intégrales d'impulsion CCD.

## Portée du diagnostic local et décision de continuation

Les douze pas B sont beaucoup trop courts pour qualifier un intervalle de 132 pas. À 50 ms, la fraction de poids soutenue par les pieds est environ 0,6554, les charges de talon sont nulles, le CoM et le point de capture sont hors du support mesuré. Ce sont des observations du préfixe ; le délai d'acquisition de 2 s n'est pas arrivé à échéance. **Le refus B n'est donc pas un échec démontré du diagnostic de transfert à son échéance.**

Le [témoin A](native-run/001-A.json), lui, a effectivement parcouru 960 pas : les deux charges de talon sélectionnées par le diagnostic restent nulles sur toutes les lignes ; aucune ligne ne satisfait simultanément les conditions locales et aucun intervalle stable ne commence. Le compteur original de récupération reste nul. Ce constat est valable pour cette référence commune exécutée sans correction ; il ne démontre pas qu'un contrôleur coordonné fonctionnel aurait échoué. L'amplitude de correction de B n'a jamais été comparée physiquement à A après acquisition des appuis.

Le résultat n'autorise **ni la répétition B, ni B à 60°, ni l'histoire physique complète de 8 s**, et encore moins une récupération debout qualifiée en moins de 25 s avec sa stabilité supplémentaire. La route géométrique préparée conserve sa portée cinématique ; elle ne remplace pas une exécution physique réussie. Le verrouillage du lot est approprié malgré le budget natif restant.

Un préalable à un éventuel travail ultérieur distinct serait une instrumentation purement numérique : conserver le problème complet, le vecteur f64 et ses certificats, chaque q/dq f32 essayé, la première ligne ou borne refusée et son résidu ; conserver aussi les derniers résidus/itérés et statistiques de projection en cas de non-convergence. Cela permettrait d'étudier le compilateur et le solveur sans modifier les forces, les limites anatomiques ou les critères de réussite. **Cette note n'effectue ni cette instrumentation, ni un nouveau calcul, ni une réouverture du protocole.**

Les liens des grandes preuves désignent leurs archives gzip sans perte ; les
SHA des originaux et des archives sont dans `native-run/archive-index.json`.
Décompresser rétablit exactement les fichiers cités par les reçus natifs.
