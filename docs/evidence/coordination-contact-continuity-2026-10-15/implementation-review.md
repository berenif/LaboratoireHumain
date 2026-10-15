# Contre-revue indépendante de la correction de continuité — 15 octobre 2026

**Conclusion : aucune anomalie bloquante détectée dans la correction statique examinée après les deux durcissements ci-dessous.** Cette conclusion ne qualifie aucune exécution native, acquisition des talons ou continuation vers debout.

La revue a lu le code, les tests préparés et les archives existantes. Elle a utilisé **zéro sous-pas natif, zéro monde, zéro FK et zéro exécution de test**. Les résultats des tests exécutés par le responsable sont consignés séparément. Les sources exactes examinées et leurs SHA256 figurent dans `implementation-review.json`.

## Constats corrigés pendant la revue

| Constat | Risque identifié | Correction vérifiée à la lecture |
|---|---|---|
| CR-01 — certificat mis en cache | Un ancien `EquivalentLoadedSurface` pouvait être considéré courant sans vérifier son horloge, son identifiant de patch ou son segment. | `has_current_support` exige désormais le mode `Hold` et l’égalité des trois identifiants. Le cache est exclu de la désérialisation. |
| CR-02 — seuil direct après un cache surfacique | Le contrôle du calendrier choisissait sa branche à partir de `current_contact_present`, après un recalcul booléen. Un cache surfacique pouvait donc éviter le seuil historique de 3 N d’un nouveau point direct dans une observation différente de même horloge. | `has_matching_material_support` branche désormais sur le **type de preuve recalculé**. Le chemin direct exige séparément `has_matching_load`; le chemin surfacique conserve trois témoins de 3 N; le refus reste fermé. Le test de régression correspondant a été relu. |

## Garanties examinées

L’observation ne déplace ni le point local d’origine, ni son ancre monde, ni son horloge d’acquisition. Le plan local est établi à l’acquisition et n’est pas reconstruit lorsque le point disparaît. La politique de transfert compare ce plan et les références matérielles à ses obligations initiales.

Le certificat supplémentaire est limité aux segments de pied tenus en `Hold`. Il est indisponible pour `Pivot`, `Unload`, `Free`, mains et tibias. Il exige trois témoins actuellement chargés à au moins 3 N chacun, une surface non colinéaire, la cohérence avec le plan initial et la pose courante, puis la couverture du point original par un triangle chargé. Sa tolérance numérique de 1 µm ne dilate pas cette enveloppe de 12 mm.

La correspondance directe historique reste inchangée. La continuité surfacique ne prouve pas l’absence de glissement : la dérive reste mesurée depuis l’ancre initiale et l’adaptateur applique séparément son plafond existant. Ses validations travaillent sur une copie et n’engagent l’observation qu’après succès.

Les consommateurs disposant de l’observation recalculent la preuve. Les consommateurs internes sans cette observation emploient le cache identifié après validation. Un JSON ne restaure pas `continuity_evidence` et ne peut pas créer un reçu natif. `Adapter::observe` précède toujours la permission native de commande, et les horloges de continuation et de stabilité finale restent distinctes.

## Le refus archivé reste un refus

À 78, deux contacts seulement atteignent 3 N; le troisième porte 0,989888184 N. Le certificat surfacique est donc refusé. Le test synthétique qui augmente les trois charges à 5 N isole ensuite la condition géométrique : le point original reste **69,992784709 mm hors du triangle** et demeure refusé. Cette modification de fixture n’est pas une nouvelle observation physique.

La charge du segment de 85,55 N et la faible dérive ne suffisent pas à démontrer une redistribution équivalente du support chargé. Les archives filtrées ne permettent toujours pas de décider si le point a été déchargé, non sélectionné, renouvelé ou exclu pour une autre cause. Elles ne prouvent ni perte totale du pied ni impossibilité anatomique.

## Télémétrie et identités

Le nouvel observateur conserve la voie des contacts sélectionnés, avec la même classification de normale, impulsion, séparation et pas natif. Les données de manifolds supplémentaires sont uniquement lues. Les collections actives et inactives, indices, identités de features, normales complètes, distances, impulsions, charges et raisons d’exclusion sont distingués. Les impulsions positives sur des points non sélectionnés restent explicitement susceptibles d’être anciennes et n’ajoutent aucun appui.

Le choix de collection est conforme au code Rapier fourni dans le dépôt : `solver_manifolds` emploie `solver_clusters` si cette collection est non vide. La résolution des points monde lit les poses existantes; elle n’intègre pas la dynamique.

Les champs de surface, de certificat et de frontière de contact modifient les observations sérialisées du propriétaire et donc ses identités. **La parité de l’ancien binaire ne qualifie pas le nouveau.** La télémétrie augmente aussi le volume d’archives et le travail d’observation; aucun coût natif supplémentaire n’a été mesuré ici. Une éventuelle exécution future doit employer un gel distinct et conserver ses plafonds de temps, de calcul et d’arrêt.

## Portée

Cette correction établit une condition géométrique conservatrice de continuité autour d’un point immuable. Elle ne certifie pas une force appliquée à ce point, une capacité de moment, le maintien de toute la semelle, une trajectoire dynamique ou un relevage. Le protocole physique clos reste clos. Aucun succès local ni seconde stable n’est revendiqué, et aucun résultat global Rust/TypeScript n’est déduit de cette revue.

## Complément : observation complète à chaque pas prédit

La revue complémentaire porte sur l’unique champ ajouté au journal `transfer-preview-step` dans `transfer_worker.rs:472` : `"support": observed`. Cette variable est l’observation déjà calculée au début du callback; le changement n’ajoute aucun appel d’observation, aucune intégration, collision ou FK.

Chaque ligne de prévision conservera ainsi son observation de support courante et `native_contact_boundary`, au lieu de conserver seulement des résumés et une proposition provenant de l’observation précédente. L’enregistrement complet de l’endpoint lors d’un refus reste inchangé. Cette correction de conservation des preuves est nécessaire pour pouvoir expliquer une future transition de contacts à chaque sous-pas.

Les 17 empreintes précédentes sont inchangées et celle de `transfer_worker.rs` devient la **18e source** du JSON de revue. Le coût de sérialisation et la taille des traces augmentent; aucun coût natif ni parité du nouveau binaire ne sont mesurés par cette vérification statique.

## Complément : provenance du diagnostic archivé

La correction de `forensics.py/json/md` a été vérifiée : les sept sources du dépôt sont épinglées aux blobs du commit exécuté `da4575170c6ae9b0858c0851a6f1ed15fe1ce858`, la référence Parry externe garde son empreinte imposée, et les quatre entrées du protocole clos sont inchangées. Le hash du script correspond également au JSON. Aucun de ces fichiers n’est substitué silencieusement par une version du worktree.

L’empreinte canonique des résultats, après exclusion des quatre champs de provenance explicités dans le JSON de revue, est bien `a6e14bb58ebd690805af903552dbe485d502ddbfa787b231ddf839751b3a0772`. L’entrée `forensics.json` de cette revue est actualisée; les 17 autres empreintes restent identiques. Les empreintes Rust de la présente revue concernent la **nouvelle correction statique**; celles du diagnostic concernent **le code de l’essai clos**. Aucun nouveau calcul physique, test, FK ou réexécution du script de diagnostic n’a été nécessaire. Aucune incohérence détectée.
