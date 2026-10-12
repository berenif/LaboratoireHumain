# Revue indépendante du prototype de coordination — 12 octobre 2026

**Verdict sur le point de départ : préparation possible, lancement physique bloqué tant que les prérequis ci-dessous ne sont pas livrés.** Le budget annoncé est arithmétiquement cohérent. Il ne démontre ni l'existence d'une référence admissible, ni la fidélité d'une prévision longue, ni un coût d'exécution compatible avec deux heures. Un arrêt avant physique est un résultat correct de cette étape.

Lecture du HEAD `e3b4ed3b2ddaac5450c2846ec12c736a1287082e`, branche `investigate/physical-support`, de la [synthèse](../../coordination-architecture-review-2026-10-12.md), de la [contre-revue précédente](../coordination-architecture-2026-10-12/counterreview.md) et de l'[audit SLOW](../coordination-architecture-2026-10-12/getup-audit.md). Les constats de code ci-dessous décrivent ce HEAD, avant les changements parallèles du prototype. Aucun test, intégration physique, politique tierce, entraînement ou programme scientifique tiers exécuté pour cette revue. Seule cette note est écrite par son auteur. Une seconde lecture indépendante a vérifié l'arithmétique et les anciens lanceurs, également sans exécution.

## 1. Objections qui bloquent une expérience interprétable

| Point | Défaut concret au HEAD inspecté | Condition minimale de levée |
|---|---|---|
| Référence | Le corridor accroupi → talons → recentrage est une intention ; la synthèse demande encore les clés numériques, pivots, tâches et préscreen complet. | Une référence numérique et ses empreintes, inspectée sur toute l'interpolation, avec entrée native et continuation préparée. Un JSON qui déclare seulement la route prête ne constitue pas cette preuve. |
| Prévision | `PreparedNativeStep` copie `PhysicsWorld`, emprunte le filtre, et garde les commandes installées ou réémet une même liste. Il ne copie pas le contrôleur, ses compteurs, les ancres ou les décisions futures. | Même transition de décision/installation/observation et mêmes horloges dans le monde réel et les copies ; plan candidat fixé, sans appel récursif au planificateur. |
| Identification | `MotorTargetDelta` modifie uniquement `target_vel`, en conservant position, gains et cap. La commande proposée modifie vitesse **et position intégrée**, puis le cap dépend de la demande. | Les sondes de quatre sous-pas passent par le même compilateur de commandes et le même installateur/allocation que le candidat. |
| Observation | Le journal CCD et l'installateur de trajectoire ont chacun un `thread_local ACTIVE`. Un processus mono-thread ne sépare pas leurs états entre monde réel et copies. | Contexte explicite ou garde correctement empilée/restaurée ; les copies ne peuvent ni incrémenter la persistance réelle, ni remplir son journal, ni déclencher son handoff. |
| Terminal | L'observateur historique attend un `Controller` en Stand, ses ancres, le handoff puis Ordinary. `full_trajectory` cesse d'installer au handoff. | Adaptateur terminal documenté, propriétaire unique des moteurs, arbitre original conservé. Aucun compteur expérimental de récupération. |
| Registre | Le prédicteur existant possède seulement un compteur local recréable. Les anciens lanceurs n'ont pas les nouveaux plafonds. | Registre global persistant, réservation avant toute intégration, plafond mural unique ; mêmes limites pour réel, sondes, copies, vérifications et préfixes. |

Sources de ces constats : [`native_predictor.rs`](../../../rust/crates/sim/src/recovery/native_predictor.rs), [`full_trajectory.rs`](../../../rust/crates/sim/src/recovery/support_probe/full_trajectory.rs), [`contact_observation.rs`](../../../rust/crates/sim/src/recovery/planner/contact_observation.rs), [`observe_recovery`](../../../rust/crates/sim/src/recovery.rs#L1366), [`install_targets`](../../../rust/crates/sim/src/recovery/support_probe/offline_trajectory.rs#L329).

Une autre incompatibilité demande un choix explicite : [`Program::compile`](../../../rust/crates/sim/src/recovery/trajectory_program.rs#L7) impose une durée de 8 à 25 s et des vitesses nulles aux deux extrémités. Un plan local de 0,5–2 s, ou une référence totale de 4 s, ne peut lui être passé tel quel. Deux solutions conservatrices : extraire un horizon d'une référence complète admissible de durée ≥8 s, ou écrire un type court distinct réutilisant les validations de commandes, sans diminuer le contrat du programme complet. Exécuter seulement quatre secondes d'un programme plus long est possible ; confondre durée du programme et durée d'histoire ne l'est pas.

## 2. Plus petit périmètre correctement implémentable

Une route fixe depuis `landed-crouch-left`, un plan candidat à quatre–six clés communes, un problème de vitesse des **46 coordonnées**, un unique chemin de compilation et d'installation. Les bras et le tronc doivent rester disponibles dans la résolution ; le bassin libre est un état/objectif, jamais une commande. Ni modèle inverse complet, ni apprentissage, ni recherche de topologies ne sont nécessaires pour cette première question locale.

Le contrôleur commun décide tous les quatre sous-pas natifs : 60 Hz. Entre deux décisions, la cible totale suit la vitesse admise à 240 Hz et l'installateur recalcule les commandes/caps selon la même règle dans A, B et les copies. L'identification/planification ne se produit qu'aux indices autorisés, au plus tous les 24 sous-pas : 10 Hz. La mémoire contient l'origine entière de ces horloges, la référence, le résidu, les patches, les persistances, la carte native et son état de validité. Une phase en attente ne remet pas à zéro le temps physique de l'histoire.

**La limite de 1 rad/s concerne la commande totale, pas seulement le résidu.** Si la référence atteint déjà cette limite, une correction de même signe n'a plus de marge. Bornes anatomiques, vitesse totale, déplacement admissible de cible et continuité de `q/dq` doivent être imposés ensemble. `validate_command` valide un échantillon ; la validation complète doit aussi vérifier le slew entre échantillons, l'interpolation réelle en f32, les changements de plan et les raccords de continuation. Des clés valides ne suffisent pas à rendre une interpolation valide.

Un problème de moindres carrés commun ne donne pas, à lui seul, des priorités strictes. Les appuis retenus, les limites et les refus de collision doivent être des contraintes contrôlées, avec résidus explicitement bornés. Les tâches incompatibles produisent un refus. L'arrêt sur nombre maximal d'itérations ne signifie pas convergence ; les résidus primal/optimalité pertinents et le statut doivent être publiés. Un solveur saturé ou de rang insuffisant ne doit pas continuer en supprimant silencieusement une tâche d'appui. Les unités des sorties de carte — m/s, rad/s, N — exigent une normalisation documentée pour interpréter le compromis entre objectifs souples.

La carte locale estimée en quatre sous-pas est une approximation conditionnelle au mode courant. Elle peut être singulière, non lisse ou changer au contact. Une carte expirée doit être invalidée dès qu'un patch/son état de charge change ou que le résidu prédit dépasse la borne annoncée. La cadence maximale de 10 Hz n'autorise pas une identification immédiate supplémentaire : attendre avec une commande déjà admissible, ou refuser. Le repli doit être défini avant expérience ; « conserver la dernière commande » ne prouve pas automatiquement qu'elle reste sûre sous un nouveau mode.

L'horizon de 0,5 s n'est **pas** suffisant pour anticiper une nouvelle acquisition suivie des 0,55 s de stabilité demandées : 120 sous-pas sont inférieurs aux 132 sous-pas de maintien seuls. La durée doit couvrir le déplacement admissible, l'acquisition et la persistance restante. Si cela exige plus de 480 sous-pas, ce candidat est refusé dans cette famille ; le maximum de deux secondes ne devient pas implicitement extensible. Un horizon court peut servir à une continuation déjà stabilisée, mais ne peut être décrit comme preuve complète d'un nouveau transfert.

## 3. Contacts persistants et impulsions CCD : deux observations nécessaires

[`measurement::solver_normal_impulse`](../../../rust/crates/sim/src/measurement.rs#L13) lit les contacts sélectionnés du **dernier intervalle CCD**, en excluant les points de cache non sélectionnés. Le journal expérimental accumule, lui, les intervalles du sous-pas. Ces deux valeurs répondent à des questions différentes. Somme des impulsions du sous-pas divisée par sa durée : effort moyen pour le bilan. Impulsion du dernier intervalle avec sa durée et son état géométrique : information locale actuelle. Une impulsion d'impact passée ne prouve pas qu'une main porte encore.

Le chemin original divise actuellement la lecture de contact par `profile.dt_s` puis met à jour `integrated_support_substeps` par **segment** ([boucle native](../../../rust/crates/sim/src/lib.rs#L1450)). Ces mesures doivent conserver leur sens et leurs valeurs pour l'arbitre historique. Les nouvelles mesures doivent publier explicitement intervalle, durée, point, repère et méthode de conversion ; ne pas renommer une grandeur pour la faire passer pour l'autre.

La persistance d'un segment Foot ne constitue pas celle d'un talon. [`planted_contacts`](../../../rust/crates/sim/src/recovery.rs#L169) applique un filtre de persistance segment puis choisit les points actuellement sélectionnés. Un nouveau point arrière peut donc apparaître après une charge persistante ailleurs sur le même segment. Le diagnostic talon exige son propre compteur à partir des points arrière réellement chargés (`local_z < -0,02 m`), sur chaque pied, avec seuil et durée originaux. Le compteur doit retomber quand cette condition disparaît ; il ne s'hérite pas de l'avant-pied.

Une ancre conservée identifie segment + point local et mémorise sa position mondiale d'acquisition. Mesurer sa dérive depuis cette position ; remplacer à chaque pas l'ancre par le point actuel effacerait exactement le glissement à détecter. Un pivot est une transition explicite entre anciens et nouveaux patches observés, avec ancien point archivé. Les contacts de cache sans sélection actuelle, les charges hors fenêtre et les points géométriquement disparus ne peuvent servir au retrait d'une main. Toutes les impulsions hors pieds restent rapportées, même sous le seuil de dépendance persistante.

Le bilan angulaire demeure un diagnostic : `angular_balance_verified` vaut false dans le journal. Cela ne bloque pas nécessairement une expérience de commande bornée, mais interdit de la présenter comme une certification exhaustive de conservation ou d'effort centroidal.

## 4. Budget et définition de l'expérience

Le calcul maximal de la synthèse est exact :

| Poste | Calcul | Sous-pas natifs |
|---|---|---:|
| Histoires réelles A, B, B répété, B à 60° | `4 × 4 × 240` | 3 840 |
| Prévisions longues des trois B | `3 × 40 × 4 × 480` | 230 400 |
| Identification des trois B | `3 × 40 × (1 + 2 × 46) × 4` | 44 640 |
| Vérifications techniques réservées | borne restante | 21 120 |
| Total | tous postes | **300 000** |

Pour une histoire B de quatre secondes : appels aux indices `0,24,…,936`, soit 40 appels. Ajouter un appel à l'état final 960 en ferait 41. Une prévision témoin longue, une nouvelle vérification du candidat retenu ou une reprise après changement de mode compte parmi les quatre prévisions de l'appel, sauf réservation technique explicitement prévue. Les 93 sondes comprennent déjà le témoin court ; elles ne comprennent pas un second témoin implicite. Les copies autorisées consomment des sous-pas, même si leur horizon dépasse la fin de l'histoire réelle.

La borne de 300 000 porte sur les intégrations natives de 1/240 s ; les intervalles CCD internes sont journalisés à part. Le registre doit identifier cette unité pour éviter de confondre les deux. Une réservation consignée avant exécution reste consommée après crash/timeout, même si l'on ne connaît plus le nombre exact de pas terminés. Un refus purement statique n'intègre rien ; une vérification de neutralité à monde copié, elle, consomme son allocation. Aucun nouveau registre après un arrêt partiel.

Le coût maximal moyen compatible avec deux heures est de 24 ms par sous-pas si tous les 300 000 pas sont consommés ; copies, décisions, écritures et inspections réduisent encore cette marge. Ce calcul n'est pas une mesure de débit, encore moins une garantie de contrôle en temps réel. La deadline globale doit être consultée avant chaque intégration, y compris pendant l'identification et les prévisions. Pas de benchmark physique préalable hors registre.

Les lanceurs existants ne peuvent être supposés conformes : [`whole_history.rs`](../../../rust/crates/sim/src/recovery/support_probe/whole_history.rs#L110) conserve une boucle 25+1 s ; [`run-getup-ab.py`](../../../scripts/run-getup-ab.py) garde l'ancien budget trois histoires de 6 240 pas et 600 s. Les réutiliser exige une nouvelle enveloppe explicite.

A et B doivent partager référence, état initial, profil, installateur, règle de caps, critères et frontière des observateurs. Les valeurs numériques des caps peuvent différer puisque les commandes diffèrent ; c'est la **règle** d'allocation qui doit rester identique. Une simple répétition de B vérifie la reproductibilité de cette histoire, pas une robustesse statistique. B à 60° teste aussi la cohérence des repères ; les objectifs matériels doivent tourner avec le corps et le sol, pas conserver des directions mondiales accidentelles.

## 5. Continuation debout et comparaison SLOW

Le diagnostic de transfert ne vaut pas relevage. Il faut préparer la continuation sans greffer un état prédit dans le monde réel : raccord des `q/dq`, route des appuis, même temps écoulé, remontée, settle, handoff, Ordinary puis observation stable originale. L'entrée debout avant 25 s et la seconde supplémentaire doivent avoir des indices distincts, tout en conservant exactement la sémantique de l'arbitre existant. Le contrat contient aussi 0,30 s de settle et 0,55 s de persistance locale : aucune de ces durées ne remplace la seconde d'observation finale.

Un fichier de continuation géométrique peut être préparé maintenant. Il ne prouve pas que l'état réel au transfert rejoindra ce fichier ; il faut documenter le raccord admissible et refuser s'il manque. Aucune continuation physique au-delà des quatre secondes n'entre implicitement dans les quatre histoires autorisées, ni dans la réserve technique. Le succès court permet seulement la conclusion locale prévue.

La comparaison utile avec SLOW reste celle de l'[audit épinglé](../coordination-architecture-2026-10-12/getup-audit.md) : acteur commun 118 entrées/21 sorties à 40 Hz, PD à 800 Hz, réseau corrigeant une référence selon l'état. Le prototype proposé partage l'idée d'un retour commun vers les commandes ; il ajoute une représentation explicite des patches et des prévisions natives. Il ne reproduit ni l'apprentissage, ni le modèle, ni les garanties de SLOW. Les masses, corps, pieds articulés, enveloppes moteurs et critères diffèrent. Les 21 sorties conjointes source ne démontrent pas que 46 sorties jointes locales suffisent ; la preuve manquante reste l'exécution native du transfert puis du relevage.

## 6. Propriétés indispensables avant ouverture du chemin physique

1. **Commande pure et unique :** mêmes observation/plan/mémoire donnent exactement même `q/dq` et mémoire ; rejet d'une racine commandée, d'un axe absent, d'une borne/slew violés ou d'un résidu de contrainte dure excessif. Aucune mutation des corps lors de décision/installation.
2. **Isolation :** copies complètes et indépendantes, notamment mémoire, persistances, horloges, allocation, caches natifs et journaux ; le compteur budgétaire commun reste partagé volontairement. Nettoyage garanti sur toute erreur.
3. **Cadences et budget :** tests purs de tous les indices autorisés, limite initiale comprise, overflow, épuisement, réservation refusée et reprise après interruption. Aucun test qui intègre un monde n'est classé « purement technique » pour échapper au compteur.
4. **Supports :** séquences synthétiques différenciant impact CCD terminé, contact actuel, cache périmé, nouveau talon, glissement d'une ancre et pivot déclaré ; l'exclusion des membres libérés intervient avant la preuve du support restant.
5. **Prévision physique ultérieure, comptée :** égalité du premier sous-pas sous même commande sur les 25 corps et livraison moteur, puis fidélité conditionnelle des décisions/horloges pendant l'horizon. La propriété au premier pas est nécessaire, pas suffisante pour les 480 suivants.
6. **Interprétation honnête :** refus de préscreen = route non préparée ; refus numérique = solveur/carte insuffisant ; échec physique = famille testée rejetée. Aucun de ces résultats ne démontre une impossibilité anatomique générale ou ne permet d'assouplir les critères.

**Conclusion de revue :** le minimum correct comprend la préparation numérique, le contrat d'observation, le solveur borné commun, la compilation cohérente des commandes et le registre. Une façade qui appelle le prédicteur actuel avec un horizon augmenté manquerait l'essentiel. Si la référence complète, la séparation des états ou le terminal restent absents, livrer ces briques avec un gate fermé et zéro physique est préférable à une expérience dont le résultat ne serait pas attribuable.
