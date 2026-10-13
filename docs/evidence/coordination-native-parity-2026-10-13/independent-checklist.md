# Revue indépendante préalable de la parité native

Lecture du 13 octobre 2026 à partir de `8c050515d7fd9d8cb28f92742057e03cdc2dd9f5`.
Aucun test, construction de modèle/monde, FK ou sous-pas physique exécuté par
cette revue. Aucun registre antérieur modifié. Cette note prépare la revue du
nouveau worker ; elle ne certifie pas son implémentation future ni son résultat.

Le périmètre transmis par le parent est un contrôle distinct de **44 sous-pas
maximum / 120 secondes**, comprenant le préfixe natif commun de 12 pas puis
quatre paires copie/réel de 4 + 4 pas, pour les échelles 0, 0,5, 1 et 1,5.
Une divergence du préfixe doit arrêter avant tout pas de comparaison. Aucun
nouvel essai, prévision longue, stabilisation ou qualification n'est inclus.

## Critères à vérifier dans le nouveau worker

| Point | Preuve concrète attendue | Faux positif à exclure |
| --- | --- | --- |
| Identité de l'expérience | SHA de base, diff préparatoire, binaire et entrées gelés ; budget distinct écrit avant exécution ; cas et horloge explicitement nommés | Réutiliser l'ancien reçu 16 pas ou rouvrir l'ancien registre |
| Préfixe commun | 12 pas réellement intégrés, comparaison aux archives avant le pas 13, copie de l'état natif complet en mémoire | Recréer au pas 12 un monde depuis les seules poses puis le présenter comme le checkpoint original |
| Décision effectivement exercée | Patches retenus non vides, `numericAttempt` au pas 12, statut certifié, quatre commandes planifiées ; échelle exacte du cas | `enabled=true` mais exécution du seul préfixe sans solveur |
| Oracle numérique distinct | Comparaison bit à bit à `validation/certified-commands.json` gelé, pour les commandes 12, 13, 14 et 15 ; quatre commandes exactement | Comparer seulement deux invocations du même compilateur, susceptibles de partager la même erreur |
| Choix de l'oracle | Vérifier `audit.stage`, choisir la tentative certifiée ; les quatre archives actuelles réussissent dans `guardedAttempt.attemptedCommands` | Utiliser `directAttempt`, qui contient les commandes refusées avant réparation |
| Application de q/dq | Relire les 46 `target_pos`/`target_vel` effectivement installés ; comparer leurs bits à l'oracle et contrôler `dq=(q_n-q_prev)/dt` en f32 | Lire seulement `owner.last`, `sim.targets` ou un rapport du solveur |
| Anatomie et actionnement | 25 corps, 46 axes autorisés, racine sans commande, axes absents sans action ; modèle `ForceBased`, gains canoniques, limites, `enforce_motor_vector_limit`, masses/frictions/collisions inchangés | Parité parfaite de deux mondes également altérés ou moteurs devenus `AccelerationBased` |
| Allocation effective | Somme f64 des `max_force` finaux ≤ plafond du profil ; chaque cap fini, positif ou nul, ≤ cap anatomique ; règle `OwnedRequests` commune | Assimiler les couples demandés aux couples délivrés ; comparer des caps journalisés avant la réparation d'arrondi |
| Instances et copies | Deux `Simulation` distinctes pour chaque paire, état initial identique, aucune mémoire mutable partagée, aucun retour d'état copié vers le réel | Faire avancer deux fois le même objet ou comparer deux vues du même objet |
| États par sous-pas | Bits des poses/vitesses des 25 corps, q/dq et moteurs, état du propriétaire, signatures et horloges ; 4 observations consécutives | Omettre un pas ou confondre commande au pas 12 et état achevé au pas 13 |
| État de fin de tick | Comparaison supplémentaire après retour du noyau : tick 4, sous-pas 16, contacts et persistance finalisés, compteurs et mémoire | Utiliser seulement le callback du quatrième sous-pas, qui précède la finalisation du tick |
| Contacts et impulsions | Identifiants, points locaux/monde, normales, sélection solveur, impulsions normales/tangentielles, charges et persistance comparés ; impulsions moteur signées, absolues et angulaires distinctes | Déduire toutes les impulsions CCD d'un seul manifold final ou de la seule charge du segment |
| Observation CCD | Présence/absence de journal déclarée ; journal désactivé entre branches ; aucune contamination du contexte thread-local | Présenter `ccd=[]` côté copie comme un journal CCD égal au réel |
| Refus | Premier écart identifié avec cas, sous-pas, champ et deux valeurs/bits ; conservation des dernières preuves ; aucun pas suivant | Réessayer un cas ou rendre le budget réservé après un échec |
| Dernière admission avant Rapier | Après `configure_motors` et toute préparation, contrôle de l'échéance murale/monotone et du compteur global immédiatement avant `world.step_with_events` ; pas refusé si le permis manque | Contrôle au début du tick puis solveur/écriture longue permettant une intégration après expiration |
| Résultat | Parité bornée aux quatre pas observés ; succès local, relevage et qualification restent faux/non établis | Transformer une égalité de noyaux en preuve de stabilité ou de support faisable |

La même fonction d'intégration doit être utilisée dans les deux chemins : c'est
une condition souhaitable de cohérence. L'oracle archivé et la relecture des
moteurs apportent le contrôle indépendant qui manque à une simple égalité des
deux chemins. Une commande certifiée à 60 Hz est ensuite installée quatre fois
sur le noyau à 240 Hz ; chaque installation doit être observée, pas seulement
le solveur initial.

La comparaison de mémoire doit inclure explicitement
`owner.memory.mode_signature` et tous les champs de `recovery::Controller` :
`status`, `targets`, `hands`, `lead`, `stand_feet`, `stand_root`, `stand_goal`,
`best_progress`, `velocities`, `foot_anchors`, `hand_anchors`, `knee_anchors`,
`handoff_from`, `stable_substeps`, `handoff_substeps`, `settle_substeps`.
Contrôler leur présence et leur égalité au checkpoint initial, après chaque
sous-pas comparé et après la finalisation du tick. Les seules sorties publiques
`recovery_status` ne couvrent pas cette mémoire qui influence la continuation.

## Constats sur le code de base

- `native_plan_preview.rs:44–107` compare le monde sérialisé, les corps et
  certains champs moteurs bit à bit. Le tableau de bits omet
  `delivered_impulse` signé ; il inclut `impulse` et `delivered_abs_impulse`,
  qui ont des sens différents. Le monde sérialisé apporte d'autres champs,
  mais ne remplace pas un contrôle explicite des nombres finis et des bits
  sur les grandeurs revendiquées.
- `native_plan_preview.rs:120–181` impose des instances distinctes et quatre
  sous-pas consécutifs. `transition_worker.rs:386–423` enregistre les traces
  dans les callbacks du noyau. Or `lib.rs:1516–1533` finalise les contacts,
  la persistance et `tick` après le quatrième callback. Une comparaison
  après le retour du tick est donc nécessaire.
- `Controller::evidence`, `whole_body_controller.rs:181`, omet
  `memory.mode_signature`. L'ancienne parité passe `&()` comme mémoire
  externe. `recovery_status()` ne contient pas non plus toute la mémoire
  du contrôleur original : cibles/vitesses, ancres, compteurs de stabilité
  et transfert sont des champs séparés dans `recovery.rs:726–744`.
- `copy_static_snapshot` copie explicitement les champs de `Simulation` et
  clone les états Rapier. `PhysicsWorld` ne sérialise ni `physics_pipeline`
  ni `ccd_solver` (`physics_world.rs:67,86`) ; le second est explicitement
  documenté comme espace de travail, et il est cloné par l'adaptateur.
  Cette omission n'est pas à elle seule un blocage, mais il faut limiter
  la portée aux états persistants effectivement comparés.
- `install_targets`, `offline_trajectory.rs:329–511`, installe bien des
  moteurs `ForceBased` à gains canoniques avec allocation `OwnedRequests`.
  Les entrées `requests.installedCapNm` sont produites ligne 483 avant la
  réparation ULP éventuelle lignes 489–505 : la preuve du cap final doit
  lire `joint.data.motors[..].max_force`, pas ce seul journal.
- `configure_motors`, `lib.rs:536`, donne la main au propriétaire explicite.
  `offline_trajectory_before_step`, `recovery.rs:19–31`, retourne sans autre
  installation si `coordination.is_some()`. Les flags de compilation et la
  présence de ce propriétaire doivent être gelés et constatés pour que
  cette isolation soit applicable ; la récupération publique reste désactivée.
- Le journal CCD est actif dans `actual_tick` et inactif dans `verify_tick`.
  La copie observe donc les contacts natifs mais archive `ccd=[]`. Comparer
  ces contacts ne prouve pas l'égalité des historiques CCD complets ; aucun
  bilan angulaire global n'est certifié par cette vérification.

Ces remarques ont été transmises directement à l'agent chargé du worker et au
parent avant l'exécution envisagée. La revue du worker écrit reste à faire.

## Première lecture du nouveau worker, avant admission

Le raccordement expérimental dans `lib.rs` est placé après configuration des
moteurs et préparation du noyau, immédiatement avant `world.step_with_events`.
Un refus marque `first_failure` et `Halted`, puis retourne sans intégration.
Le mélange des deux features de workers est refusé à la compilation. Le
superviseur Python réserve durablement les 44 pas avant le lancement, lie le
processus/binaire, conserve l'échéance originale et contrôle les fichiers gelés
et protégés après arrêt. Cette revue en lecture n'a pas relevé de blocage
supplémentaire sur ce superviseur ; les résultats des tests Python sont fournis
par le parent, sans réexécution ici.

Le module `numeric_parity_worker.rs` nouvellement écrit inclut les 17 champs
de `Recovery`, déstructurés exhaustivement, ainsi que `modeSignature`, une
comparaison après finalisation du tick, l'impulsion moteur signée et une
dernière vérification d'échéance après les contrôles préparatoires.

Deux corrections ont été demandées avant feu vert :

1. Comparer chaque état réel à l'état prédit du même sous-pas dans son callback,
   avant tout pas suivant, et comparer chaque installation à l'installation
   prédite correspondante avant Rapier. Dans la première version l'égalité
   copie/réel était décidée après les quatre pas réels : un écart au pas 13
   pouvait donc être suivi des pas 14–16.
2. Vérifier explicitement les gains canoniques en f32, les plafonds anatomiques
   et la somme agrégée des plafonds réellement installés. La première version
   de `verify_command` vérifiait seulement leur finitude et non-négativité,
   insuffisantes pour les critères gelés.

Verdict de cette première lecture : **admission suspendue jusqu'à correction
et relecture de ces deux points**. Aucun calcul physique effectué par la revue.

## Relecture corrective — avis favorable borné

Les deux blocages ci-dessus sont levés dans la version relue :

- `before_integration` (`numeric_parity_worker.rs:123`) compare maintenant
  l'installation réelle à celle prédite du même sous-pas **avant** Rapier,
  puis contrôle à nouveau l'échéance juste avant le débit et le retour au
  noyau. Le compteur, la réservation de quatre pas et l'horloge attendue sont
  contrôlés ; aucun permis absent ne permet d'intégrer.
- `real_tick` (`:337–362`) compare `common_state` dans **chaque callback réel**
  à l'état copié du même sous-pas. Un écart retourne un échec au noyau avant
  l'intégration suivante. Les contrôles supplémentaires après finalisation
  du tick et la parité des traces restent présents.
- `verify_command` (`:161–214`) contrôle les bits de q/dq sur les 46 axes
  canoniques, leur activation, `ForceBased`, le plafond vectoriel, les gains
  canoniques avec le même ordre f32 et les caps anatomiques ; la somme f64
  des `max_force` finaux doit respecter le plafond agrégé original.
- `memory` (`:245`) couvre par déstructuration exhaustive tout `Recovery`
  et ajoute explicitement `modeSignature`. Ces données participent aux
  comparaisons avant installation, par sous-pas et après le tick.

La projection du préfixe est cohérente avec l'extraction : le nouveau
`numericAttempt` doit être nul avant retrait ; les champs CCD exclus sont
explicitement distingués ; les corps et moteurs sont aussi protégés par les
2 341 valeurs u32 encodant leurs bits archivés. Les commandes de l'oracle proviennent uniquement de
la tentative resserrée certifiée, jamais de la tentative directe refusée.

Empreintes relues avant éventuel reformatage final :

| Source | SHA-256 |
| --- | --- |
| `numeric_parity_worker.rs` | `fd1f7c95d2fc635b0e14e4119b0a4a0ab56017b3d6c97a1e4c52a0b8a7a3be71` |
| `lib.rs` | `bbab790f5cff501bb1ab88c22d162d44f55b7d7a305c706c56c8af3dec58b08b` |
| `run-coordination-numeric-parity.py` | `9d57bcefac46a1f769378e237edbcdf63a01a10204ba9e7cf6bfdf3daa513db5` |

**Avis favorable de revue pour le seul lot 44 pas / 120 secondes**, sous le
gel de ces comportements, des entrées et du binaire, avec les vérifications
préparatoires requises par le parent. L'auteur du worker rapporte quatre
tests purs réussis et une compilation sans exécution réussie ; leur lecture
confirme qu'ils ne construisent aucun `Simulation`. La présente revue n'a
relancé aucun test et a exécuté **0 construction de modèle ou monde, 0 FK,
0 sous-pas physique**. Les anciens registres sont intacts. Ce feu vert de revue
ne préjuge pas du résultat physique : les journaux CCD copiés, le bilan
angulaire, la stabilité, le relevage et sa qualification ne sont pas établis.
