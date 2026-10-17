# Préfixe archivé pour le replay numérique du pas 12

L'[extrait autonome](archived-step12.json) contient 224 368 octets ; SHA-256 `79c0feb2d4bf3ea4b336b8d23782bf897cecadda826d081a83d2d880209d873d`. Le [script](extract_archived_step12.py) recopie les valeurs des archives ; le [reçu](archived-step12-extraction-receipt.json) conserve les empreintes des six sources et les vérifications. Cette extraction n'exécute aucun pas physique, FK, assemblage de tâches ni calcul géométrique.

## Données et frontière temporelle

| Champ | Contenu et usage |
|---|---|
| `physical.snapshot.segments` | Les 25 `lh_model::Pose` originales, positions, quaternions xyzw, vitesses linéaires et angulaires. |
| `physical.support` | L'observation complète du pas 12, dont les 25 poses/vitesses, contacts courants, charges persistantes, CoM global et poids déjà mesurés. |
| `physical.motors` | Les 24 enregistrements de joints, avec les six états moteur par joint et les impulsions déjà mesurées. |
| `physical.targets`, `restTargets`, `persistence`, `integratedSupport*` | Préfixe de la simulation copié sans modification. |
| `ownerBeforeConfigure` | Contrôleur accepté après l'intégration du pas 12 : `previousStep=11`, `nextSubstep=12`, aucun patch acquis. Pour chaque cas, ne remplacer que `feedbackScale`. |
| `cases[i].terminalOwner` | Contrôleur après la tentative refusée de `configure(12)` : deux patches déjà acquis, compteurs modifiés ; il ne faut pas le traiter comme un préfixe intact. |
| `cases[i].rejectedF64Rates` | Les 46 taux f64 écrits avant refus pour le cas 0 uniquement. `null` pour les autres : leur solveur a refusé avant de remplacer les taux du préfixe. |
| `archivedLocalMassProperties` | Propriétés inertielles locales natives, notamment `inv_mass` et `local_com`, extraites de mondes déjà sérialisés. |
| `commandProgram`, `referenceRow`, `materialTargets` | Programme complet figé, référence corporelle au pas 12 et points matériels prévus. Aucune nouvelle interpolation dans cette extraction. |
| `modelSegmentAttributes` | Attributs canoniques des 25 segments, sans maillage ni pose initiale, copiés pour l'inspection. Le modèle complet reste identifié par empreinte. |

Les quatre lignes 12, 24, 36 et 48 de `002-B.states.jsonl` sont **identiques pour l'ensemble des champs physiques**, moteurs compris. Leurs owners ne diffèrent que par `feedbackScale`. Les poses terminales du planning sont identiques à leurs poses au pas 12. Le stamp archivé a `tick=2` ; la tentative suivante refuse à `tick=3, substep=0`. B n'a exécuté aucun pas dans l'histoire réelle.

Le cas 0 (`feedbackScale=0`) refuse « Emitted rates violate a hard task or total command bound ». Les cas 0,5, 1 et 1,5 refusent « Common task solve did not converge within fixed iteration budget ». Le champ `last.solver` est nul et reste celui de la dernière commande acceptée : il n'est pas un rapport du solveur ayant refusé.

## Interface recommandée sans monde physique

Extraire un noyau commun `assemble_observed(model, observed, retained, goals)` alimenté par 25 poses/vitesses f32, 25 masses f32 et 25 CoM mondiaux f32 ; conserver `assemble(&Simulation, ...)` comme simple adaptateur. Les corps de ce noyau n'ont besoin ni de handles, ni de colliders, ni de joints Rapier. L'ascendance et les frames articulaires viennent du modèle ; `measurement::coordinates` utilise les rotations observées, sans FK de chaîne.

Pour le replay, cloner `ownerBeforeConfigure`, appliquer le scale du cas, puis effectuer une seule mise à jour des patches depuis `physical.support`. L'assemblage peut aussi recevoir les patches déjà acquis dans `cases[i].terminalOwner.patches`, à condition d'identifier explicitement cette frontière après observation. Garder la mémoire q/rates du préfixe accepté. Reprendre exactement les calculs des objectifs et bornes du contrôleur ; la solution f64 ne remplace pas la vérification des quatre commandes f32 émises par `runtime::command`.

Précautions de parité : conserver xyzw et les signes des quaternions, aucune renormalisation ; chart couplé natif en f32 ; transformation des points locaux f64→f32 puis pose f32 puis élargissement f64 ; horloge `f64::from(SUBSTEP_S)*4`, somme des masses/CoM dans l'ordre natif en f32. Le poids archivé divisé par 9,81 ne garantit pas la masse f32 originale. Parser directement `Model` via serde évite `Model::canonical()/validate()`, qui réintègre les maillages.

## Propriétés inertielles et cache CoM

Les 16 mondes archivés des quatre premiers pas de chacun des quatre cas possèdent les mêmes propriétés inertielles locales. L'extracteur associe chacun des 25 corps à son segment par égalité des bits f32 de la pose et des deux vitesses déjà enregistrées, puis copie `mprops.local_mprops`. Il ne reconstruit aucune masse ni aucun CoM mondial au pas 12. Les empreintes des 16 mondes source sont conservées dans `archivedLocalMassPropertiesSources`.

Le CoM mondial individuel au pas 12 n'est pas directement archivé. Le code vendored précise toutefois sa frontière : `physics_pipeline/substep.rs::advance_to_final_positions` affecte la pose finale puis appelle `update_world_mass_properties` (lignes 148 et 206 dans cette version) ; `rigid_body_components.rs:528` calcule `world_com = local_mprops.world_com(position)`. Pour les corps actifs sans quarantaine, utiliser la même méthode native f32 sur la pose finale et les propriétés locales archivées suit donc cette frontière. La masse doit venir de la même méthode `MassProperties::mass()` que `RigidBody::mass()`, et non de `model.massKg` supposé identique. La comparaison bit à bit du CoM global obtenu au CoM déjà archivé est un contrôle de reconstruction à réaliser par le replay. Les positions et quaternions doivent rester ceux de l'archive, pas ceux de la mémoire de commande.
