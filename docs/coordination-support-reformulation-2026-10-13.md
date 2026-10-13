# Reformulation statique : commandes, intentions et appuis observés

La paire physique précédente reste close. Cette préparation corrige la
confusion entre vitesse de consigne et vitesse physique dans les tâches
d'appui, sans changer le corps, les moteurs ou les critères de réussite.
**Aucun nouveau sous-pas natif, monde, FK ou inspection d'une nouvelle
trajectoire géométrique.** Les tests manipulent des nombres archivés ou
synthétiques, notamment les transformations de poses fournies et les
polygones de support de l'observateur existant. Ils ne reconstruisent aucune
anatomie ni trajectoire.

La [réservation distincte de préparation](evidence/coordination-support-reformulation-2026-10-13/preparation-reservation.json)
débite 7 200 s supplémentaires, soit 35 689/144 000 s. Son échéance est
13:32:52 UTC le 13 octobre. Les cinq registres physiques restent clos ;
quatre histoires familiales sont déjà consommées. Aucun budget physique
supplémentaire n'est ouvert.

## Diagnostic et décisions

Les inconnues du problème refusé sont les 46 vitesses des **références**
articulaires. La relation `J × vitesse_référence + twist_racine_observé`
n'est pas une prédiction validée de la vitesse matérielle sous `ForceBased`.
La contraindre comme telle, en exigeant de rejoindre l'ancre d'origine en
un tick, ajoute une hypothèse de commande qui n'est pas une loi du contact.

Huit sous-systèmes contradictoires ont été isolés : un par pied et par facteur.
Chacun comprend trois faces de bandes matérielles et cinq bornes de commande.
Leur minimalité **par inclusion**, et non leur cardinalité minimale globale,
est prouvée par huit certificats rationnels et 64 témoins après retrait d'une
inégalité. Ces témoins ne sont pas des mouvements admissibles du corps entier.
Les quatre bornes à ±1 rad/s limitent le slew des consignes ; la cinquième,
sur la cheville, vient de la marge restante sous la limite anatomique du
prochain tick. Les trois faces matérielles ajoutent l'hypothèse cinématique
contestée. Ni la borne de cheville seule, ni une impossibilité anatomique
globale ne découlent de ces conflits. Les [certificats et leur portée exacte](evidence/coordination-support-reformulation-2026-10-13/minimal-conflicts-review.md)
incluent la tolérance d'origine de chaque inégalité, sans l'augmenter.

Le correctif sépare les domaines :

| Domaine | Statut conservé |
| --- | --- |
| Anatomie, amplitude et vitesse des consignes, représentation f32 | Contraintes dures de commande |
| Masses, friction, collisions, caps, actionnement, intégrité native | Contrat physique inchangé |
| Charges, persistance, dérive et perte d'appui, support restant | Obligations sur les observations et prévisions natives |
| CoM désiré, orientation, posture, retour vers une ancre | Guides de commande, jamais certificats de soutien |
| Retrait et repositionnement d'un membre | Intention explicite, progression seulement avec preuves de transfert |

`command_proposal` conserve les bornes d'origine et toutes les contraintes
dures qui ne sont pas les lignes matérielles explicitement identifiées. Il
conserve aussi les ancres et les bandes matérielles dans ses preuves, avec
leurs résidus pour chacune des quatre commandes f32. Ces lignes deviennent
des guides heuristiques à poids unitaire fixe ; aucune recherche de poids ou
de gains n'est effectuée. Le résultat porte le statut `NativePreviewRequired`
et ne peut pas autoriser une exécution réelle.

`support_schedule` ajoute les phases `Retain → Unload → Released →
Reposition → Acquire`. La demande de retrait exclut le **pied entier**
(arrière-pied et avant-pied) du soutien restant. Elle garde les obligations
tant que la garde d'origine refuse. Le repositionnement nomme point local,
but mondial et échéance ; ce but n'est jamais converti en contact. Une
réacquisition exige une disparition de soutien observée après le retrait,
puis douze observations propres du nouveau point chargé. Les anciens
identifiants et ancres restent archivés. Le pivot garde son ancre ; le
roulement avec remplacement continu d'un point n'est pas implémenté.

Cette séparation ne certifie pas que les contacts supporteront la commande.
Elle retire une fausse équivalence du domaine de faisabilité des consignes.
Une nouvelle carte commande→état devrait avoir une provenance native et un
domaine de validité explicites ; les quatre facteurs archivés ne permettent
pas d'identifier une telle carte à 46 dimensions.

## Résultats vérifiés

| Vérification ciblée | Résultat |
| --- | --- |
| Certificats et témoins rationnels | 7 tests réussis ; 8 conflits minimaux par inclusion, 64 témoins |
| Séparation et compilateur q/dq | 5 tests réussis ; 4 fixtures × 4 commandes × 46 axes |
| Machine de transfert | 9 tests réussis ; 8 refus archivés et contre-exemples synthétiques |
| Compilation de production `lh-sim --lib --locked` | Réussie ; le module expérimental demeure exclu de production |
| Nouvelle intégration physique | 0 ; famille close, quatre emplacements consommés |

La compilation des tests avec `experimental-solver-step-observation` vérifie
le nouveau Rust. Les deux filtres n'exécutent que les tests purs nommés ; les
tests construisant un monde sont exclus. La suite globale Rust/TypeScript
n'a pas été relancée : ses échecs historiques ne sont pas déclarés corrigés.

Les [commandes et résidus exportés](evidence/coordination-support-reformulation-2026-10-13/static-command-results.json)
conservent les coefficients et bandes d'origine. Chaque problème corrigé
est résolu en deux itérations, avec erreur de contrainte du QP continu
nulle. Les quatre commandes f32 sont ensuite vérifiées séparément :
amplitudes anatomiques, bornes d'origine et égalité bit à bit entre dq et
différence des positions installables. Aucune commande n'a été installée.

| Facteur archivé | Excès maximal hors ancienne bande matérielle, heuristique (m/s) | Admission physique |
| ---: | ---: | --- |
| 0 | 0,325154207 | Refusée / prévision requise |
| 0,5 | 0,392244093 | Refusée / prévision requise |
| 1 | 0,403367732 | Refusée / prévision requise |
| 1,5 | 0,410739638 | Refusée / prévision requise |

Ces excès importants restent visibles. Ce sont des résidus algébriques
sous une réponse moteur unitaire non validée, **pas des vitesses physiques
mesurées**. La faisabilité des consignes ne démontre donc pas la faisabilité
du maintien. Même une fixture donnant un résidu nul ne reçoit aucune
admission réelle.

Les [huit décisions détaillées](evidence/coordination-support-reformulation-2026-10-13/static-support-release-results.json)
refusent le retrait de chacun des deux pieds dans les quatre observations
archivées : le support restant ne représente que 42,534–43,157 % du poids,
contre 80 % requis, et le membre à retirer dépasse toujours la limite de
25 %. CoM et capture sont hors du polygone restant dans les huit cas.
Ces refus n'établissent pas l'impossibilité d'une préparation antérieure.
Les tests positifs de transfert utilisent des observations
synthétiques explicitement nommées. Les contre-exemples vérifient aussi
l'absence d'héritage de persistance, la conservation d'une dérive de 48 cm,
le refus d'utiliser le talon du pied retiré comme support restant, et
l'impossibilité de réancrer par une simple boucle de phases.

Les [journaux de commande](evidence/coordination-support-reformulation-2026-10-13/command-proposal-tests.log),
[de transfert](evidence/coordination-support-reformulation-2026-10-13/support-schedule-tests.log)
et [de compilation](evidence/coordination-support-reformulation-2026-10-13/production-check.log)
conservent les résultats et avertissements. Les [revues contractuelle](evidence/coordination-support-reformulation-2026-10-13/physical-contract-review.md)
et [contradictoire](evidence/coordination-support-reformulation-2026-10-13/static-architecture-review.md)
acceptent ce jalon statique et explicitent ses limites.

Reproduction depuis la racine du dépôt, avec la toolchain Rust 1.89.0
installée (les exports optionnels exigent un chemin inexistant) :

```sh
cargo +1.89.0 test --manifest-path rust/Cargo.toml -p lh-sim --lib --features experimental-solver-step-observation recovery::planner::command_proposal::tests --locked -- --test-threads=1
cargo +1.89.0 test --manifest-path rust/Cargo.toml -p lh-sim --lib --features experimental-solver-step-observation recovery::planner::support_schedule::tests --locked -- --test-threads=1
cargo +1.89.0 check --manifest-path rust/Cargo.toml -p lh-sim --lib --locked
```

Les variables `LH_COMMAND_PROPOSAL_STATIC_EXPORT` et
`LH_SUPPORT_SCHEDULE_STATIC_EXPORT` ajoutent les reçus JSON. La seconde
a motivé une unique répétition des neuf tests après instrumentation,
consignée dans un [journal distinct](evidence/coordination-support-reformulation-2026-10-13/support-schedule-tests-evidence.log).
Ne pas remplacer ces filtres par la suite expérimentale entière : celle-ci
contient des tests physiques hors du budget courant.

## Limites et prochaine décision

Le correctif est isolé : `configure` et le worker physique historique ne
sont pas modifiés. Le propriétaire possède un point de préparation sans
installation ni avancement de son horloge ; ce point, qui assemblerait les
tâches depuis les poses observées, a été compilé mais pas exécuté ici.
La machine de transfert n'est pas encore raccordée à un adaptateur natif.
Les propositions sont inspectables : une tranche Rust de commandes n'est
pas un verrou universel empêchant un futur appelant de les copier. Le
refus actuel repose sur l'absence d'adaptateur et sur un statut ne possédant
aucune variante d'admission. Un futur adaptateur devra vérifier chaque
obligation à chaque pas, inclure toute la machine dans les copies et exiger
une preuve native portant sur les commandes réellement installées.

L'entrée réellement accroupie, son raccord q/dq, les branches complètes de
repositionnement et leur continuation jusqu'au terminal debout restent à
établir. Le seed historique est analytique et peu fléchi. La référence de
huit secondes ne certifie pas ces nouvelles branches. Aucun relevage,
acquisition de talon ou impossibilité anatomique n'est revendiqué.

La séparation nouvelle ne reproduit pas SLOW : le transfert historique a
importé une trajectoire, pas son acteur de retour d'état à 21 axes. Ici les
46 consignes restent soumises à l'actionnement natif. Aucun réseau tiers,
entraînement ou service externe n'a été exécuté.

Les lots causaux proposés sont distincts : local `78 736 + 4P` sous-pas,
puis continuation `204 496 + 4P`, où P est un préfixe natif encore non
identifié. Ces nombres ne sont ni une réservation ni une autorisation.
Chaque lot exigerait une entrée, une continuation, un gel et une admission
propres ; l'ancien plafond ne peut pas être rouvert. La qualification de
24 cas reste séparée.

L'accès Library reste bloqué au niveau de l'outil officiel, avant la
découverte des outils d'envoi. Aucun nouvel essai ni contournement n'est
effectué dans ce jalon, aucun nouvel ID/version n'est annoncé. Les preuves
sont livrées par les fichiers Git de la branche de progrès.

## TODO

- [x] Préserver les cinq registres, quatre binaires et cinq manifestes historiques.
- [x] Extraire les problèmes et observations archivés sans reconstruction du monde.
- [x] Établir des ensembles incompatibles minimaux par inclusion et leurs témoins.
- [x] Implémenter la séparation commande/appui et son raccord statique au propriétaire.
- [x] Terminer la machine de transfert, avec exclusions du membre entier et nouvelle acquisition mesurée.
- [x] Tester les quatre fixtures, la compilation f32 et les contre-exemples sans physique.
- [x] Contre-revoir le correctif et préciser les limites d'intégration native.
- [ ] Publier le jalon et sa vérification distante.

La [continuation complète et les protocoles proposés](evidence/coordination-support-reformulation-2026-10-13/continuation-and-protocol.md)
distinguent la référence géométrique de huit secondes, l'entrée réellement
accroupie restant à établir et un futur relevage après impact. Aucun succès
local ou algébrique n'est converti en relevage. La récupération reste inactive
par défaut ; aucune qualification des 24 cas n'est réalisée.
