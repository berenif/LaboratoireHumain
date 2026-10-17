# Revue de préparation de la télémétrie — 15 octobre 2026

**Les champs de télémétrie suffisent à classer la cause observable à une frontière native. Le blocage de compatibilité du préfixe identifié ci-dessous est corrigé dans les sources relues ; les tests centralisés et le gel restent requis.** Revue en lecture seule du commit `a5c8ff36af4dfbe784277aebc91021040b947a52`. Zéro sous-pas, monde, FK, requête de collision ou test exécuté. Les empreintes exactes des sources examinées figurent dans `readiness-review.json`.

## Blocage avant lancement : CD-READINESS-01

Dans la version initialement examinée, `transfer_worker::archived_projection` retire `kind` et `owner.transfer` uniquement. `native_frame` sérialise intégralement `support`, qui comporte désormais `native_contact_boundary`. Ce champ est absent des **36** frames historiques vérifiées. `exact_json` exige le même nombre de clés : sans adaptation explicite du schéma, le préfixe refusera dès son premier pas, avant de pouvoir diagnostiquer la transition visée.

L'adaptation doit valider et conserver la nouvelle télémétrie, puis exclure **uniquement ce champ ajouté** de la comparaison avec le préfixe ancien. Tous les anciens champs et la comparaison numérique exacte doivent rester vérifiés. Les trackers matériels du propriétaire sont vides dans les 36 frames historiques : aucun retrait des nouveaux champs matériels n'est nécessaire pour ce préfixe. Des tests purs doivent accepter cette seule addition et refuser une modification des anciennes données ou une horloge incohérente. Aucune modification Rust n'est effectuée par cette revue.

## Ce que le diagnostic peut distinguer

| Observation | Preuve disponible | Limite d'interprétation |
|---|---|---|
| Point toujours sélectionné, impulsion nulle | Index sélectionné vers le point brut, `normal_impulse_ns = 0`, exclusion `zero_impulse`, géométrie courante | Déchargement à la frontière du solveur ; ne représente pas à lui seul une force moyenne sur tous les intervalles CCD. |
| Point brut encore présent, non sélectionné | Positions locales et monde, features, liste de sélection vide et motif `unselected_manifold_point` | Une impulsion positive mise en cache sur ce point n'est pas une charge actuelle. |
| Point sélectionné mais écarté par l'observateur | Normale complète, séparation par ancres courantes, motifs normal/distance explicites | Les seuils restent ceux du contrat existant. |
| Point brut remplacé ou supprimé | Paire et générations, subshapes, points géométriques et features avant/après, collections brute/cluster distinctes | Les indices et features ne constituent pas une identité matérielle durable. Une correspondance géométrique reste nécessaire. |
| Paire ou manifold disparu | Absence explicite de paire ou de manifold courant, charges actuelles et persistantes séparées | Cela ne démontre pas automatiquement la perte de tout l'appui du pied. |

Les coordonnées des points bruts sont résolues avec les poses actuelles des colliders et sous-formes ; elles restent distinctes des ancres du solveur. La sélection des contacts et le diagnostic partagent la même fonction de classification : aucune divergence de seuil, de conversion impulsion/charge ou de pas natif n'est introduite.

## Conservation nécessaire

Chaque frontière prédite doit conserver `support.native_contact_boundary`, son horloge et les 25 entrées de segment. L'endpoint doit être observé à nouveau et conservé lors du refus : un rejet atomique peut laisser `owner.lastObserved` à 77 alors que le monde et sa télémétrie sont à 78. Les observations précédentes et l'endpoint, ainsi que les motifs d'exclusion, sont nécessaires pour conclure ; un résumé du propriétaire seul ne suffit pas.

Les points admis et leurs charges doivent être réconciliés avec `support.segments[*].contacts`. Une télémétrie ou un endpoint absent constitue un diagnostic manquant et impose la clôture, sans remplacement ni relance. Le mode borné demandé doit s'arrêter après le préfixe et l'unique prévision, sans installer de transfert réel.

## Incertitudes résiduelles explicites

L'état privé de recyclage Rapier et le drapeau de mise à jour complète de la narrow phase ne sont pas exposés. Le bit « nouveau contact » ne prouve pas à lui seul qu'une mise à jour complète vient d'avoir lieu. La télémétrie permet donc d'établir la transformation observable des contacts ; elle ne garantit pas d'identifier chaque décision intermédiaire interne du moteur.

Une disparition du manifold ou une redistribution des points ne prouve pas une continuité mécaniquement équivalente de la surface chargée. Les obligations de support, de dérive, de continuation vers debout et de stabilité finale restent séparées. Aucun résultat futur à 78, transfert ou relevage n'est présumé par cette revue.

## Résolution de CD-READINESS-01 vérifiée dans les sources

L'agent du superviseur a ajouté la projection du seul chemin `support.native_contact_boundary`. Le nouveau `compare_live_prefix` exige la présence d'une télémétrie désérialisable complète de schéma 1, la concordance des horloges de frame/support/frontière, et 25 segments dans l'ordre canonique. Tous les champs historiques restent soumis à `exact_json`. Les tests purs préparés couvrent l'absence de télémétrie, une horloge différente, la variation du signe de zéro d'une charge historique, une commande modifiée, un champ homonyme au mauvais emplacement et l'apparition d'un patch matériel. Aucun de ces tests n'est exécuté par cette revue. L'empreinte de la version corrigée relue est inscrite dans le JSON. Aucun blocage supplémentaire de télémétrie n'est identifié avant les vérifications centralisées et le gel.

## Fraîcheur des poses et limite des points bruts

Dans le Rapier fourni, `physics_pipeline/substep.rs` appelle `advance_to_final_positions` après le solveur et le CCD. Cette fonction affecte la pose finale du corps puis celle de chacun de ses colliders attachés. `Simulation::advance_tick_observed` appelle le callback après le retour de `world.step_with_events`. À une frontière normale, les poses corps/collider sont donc propagées au même état final. Une vérification sur les valeurs archivées reste nécessaire après exécution.

En revanche, après le **dernier** intervalle CCD, Rapier rafraîchit uniquement la broad phase et ne recalcule pas la narrow phase. Les points locaux, features et distances stockés du manifold peuvent provenir de la narrow phase précédente ou d'un recyclage. Les champs `current_world_p1_m`, `current_world_p2_m` et `live_geometric_separation_m` ne font que transformer ces points stockés avec les poses courantes, selon la normale stockée du manifold : ils ne constituent pas une nouvelle requête de contact ou de point le plus proche.

La conclusion causale doit donc se fonder d'abord sur la collection active, la sélection, les impulsions, les exclusions explicites et les ancres du solveur résolues par les corps courants. Les points bruts servent de corroboration. Une modification du manifold brut seule ne démontre ni une perte réelle d'appui ni un renouvellement mécaniquement équivalent. Cette revue n'introduit aucune correction spéculative des poses ou de la télémétrie.

## Garde final de l'endpoint relu

La version définitive de `transfer_worker.rs` examinée a pour SHA256 `811df09bae544476e03026a9d832f7ea306ff774fe85e42929f974efd2c3f89c`. Le garde `require_diagnostic_endpoint` exige une observation valide, une frontière de schéma 1 présente, 25 segments canoniques, et l'égalité des horloges du monde, de l'observation, de la frontière et de l'échantillon `localDiagnostic`. Une preuve manquante ou un échantillon ancien ne peut donc pas devenir un diagnostic déclaré complet.

L'écriture de l'endpoint précède ce refus de complétude. Si le monde et le support sont à 78 tandis que l'échantillon mémorisé reste à 77, la frame `preview-transfer-end` conserve le support et la télémétrie **à 78**, avec l'échantillon explicitement horodaté 77. En l'absence d'échantillon, `diagnostic-endpoint-incomplete` conserve également le support courant. Le refus secondaire `diagnostic-endpoint-refused` enregistre séparément `endpointRefusal` et `originalForecastOutcome` ; le message d'erreur final conserve les deux. Le motif causal initial de la prévision ne disparaît donc pas derrière l'échec secondaire de complétude.

Les autres empreintes déjà examinées restent inchangées. Les cas purs préparés couvrent l'absence de frontière, l'absence ou l'ancienneté de l'échantillon, une horloge d'endpoint incohérente, une horloge de frontière incohérente et un nombre incomplet de segments. Leur exécution et la compilation finale restent centralisées. Cette mise à jour de revue n'a modifié aucun Rust ni exécuté de calcul physique, FK, worker ou test.
