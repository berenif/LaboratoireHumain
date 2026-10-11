# C : arrêt expérimental et repos physique au sol

L'arrêt sur cuisse à 1,554 s invalide le corridor tibias/mains, mais ne constitue
pas un échec terminal du relevage complet. Une continuation diagnostique de C,
avec exactement les mêmes commandes et limites, atteint un repos ventral à
**2,016667 s**, sans garde physique violée. La tête porte encore 109,88 N : cet
état peut servir de départ à une récupération, mais n'est pas une base de
progression qualifiée. Aucune commande de sortie n'a été exécutée.

Travail local sur `investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou publication ;
récupération toujours désactivée par défaut et non qualifiée.

## Exigences et critères ajoutés au corridor

Le [contrat de physique](physics-acceptance.md#contact-and-recovery-evidence)
et les [règles de récupération](dynamic-recovery.md#contact-evidence) autorisent
les surfaces du tronc et des membres pendant le roulage. Les phases suivantes
doivent acquérir leurs vrais appuis ; la tête et la nuque ne peuvent autoriser
la progression. La station debout exige les deux pieds chargés, la posture,
les limites de mouvement et leur persistance. Le contrat ne demande pas que
toute la descente reste exclusivement sur mains, tibias et pieds.

| Critère | Portée conservée |
| --- | --- |
| Anatomie, corps dynamiques inchangés, aucun reset ou setter de pose/vitesse | Toute l'intégration |
| Caps anatomiques, somme 760 Nm, aucune force/couple au bassin | Toute l'intégration |
| Ancrages 80 mm, sol 80 mm, auto-pénétration 5 mm, erreur articulaire 0,06 rad, états finis | Gardes numériques strictes, inchangées |
| Refus `hand-unplanned-support` sur une autre surface porteuse 3 N persistants | Restriction du corridor expérimental, pas critère terminal général |
| Mains/tibias bilatéraux,80% du poids, COM/capture à 3 mm, tous les corps calmes 0,55 s | Cible intermédiaire propre à ce corridor |
| Station debout soutenue 0,55 s, puis observation ordinaire 1 s et cinq cycles sans Reset | Qualification finale toujours ouverte |

Le mode explicite `LandingOnly`, dans le banc sous `cfg(test)`, enregistre
les deux restrictions du corridor sans arrêter le monde : surface non déclarée
et perte des seuls supports déclarés. Toutes les gardes physiques restent
strictes. Le résultat original reste faux et son premier refus est conservé.
Observer un contact tête/torse ne l'autorise pas à faire progresser une phase.

## Continuation et état effectivement obtenu

Le [plan et ses octets](../evidence/c-framing-continuation-2026-10-11/unchanged-c-plan.json)
sont inchangés. Un monde neuf est construit uniquement depuis le départ
original pour rejouer le préfixe ; **les 373 pas jusqu'au refus correspondent
exactement** aux états, commandes, contacts, impulsions, compteurs et mesures
conservés. Ce même monde continue ensuite 111 pas, sans clone vivant, reset,
transplantation ou nouvelle pose imposée. Les consignes gèlent à 2 s comme dans C.

Le repos diagnostique demande une charge persistante hors tête/nuque et tous
les 25 corps sous les seuils natifs de settling 0,65 m/s et 1,8 rad/s pendant 72 pas,
soit 0,30 s. Il est observé de 1,720833 s à l'arrêt à 2,016667 s. Il reste distinct
du maintien tibial 0,55 s et de la station debout.

| Mesure à l'arrêt | Résultat natif |
| --- | ---: |
| Énergie cinétique des 25 corps | 0,001660 J |
| COM hauteur / vitesse | 0,147154 m / 0,00313 m/s |
| Vitesse RMS linéaire / angulaire | 0,00644 m/s / 0,05755 rad/s |
| Maximum parmi les 25 corps, linéaire / angulaire | 0,02182 m/s / 0,16488 rad/s |
| Rotation sagittale du bassin | 1,98191 rad |
| Pelvis hauteur / UpY torse | 0,21471 m / −0,15483 |
| Torse / cuisses gauche et droite | 254,45 N /157,36 N /163,05 N |
| Mains gauche et droite / tête | 9,49 N /10,11 N /109,88 N |
| Pieds ou tibias porteurs | Aucun |
| Marge COM dans le polygone réel hors tête/nuque | 133,22 mm |

Les surfaces hors tête/nuque portent 594,47 N. Retirer géométriquement les mains
en plus laisse torse/cuisses avec une marge COM de 37,20 mm et 574,87 N actuellement
résolus. Cette projection ne certifie ni la redistribution des forces ni un
transfert réellement soutenu.

L'énergie est encore 321,46 J au refus original, puis chute par les impacts et
la dissipation. Ce résultat démontre un repos après chute, pas une descente
contrôlée. Les reconstructions du COM/moment restent cohérentes avec Rapier
(erreurs maximales 3,45 × 10⁻⁷m et 4,13 × 10⁻⁵kg·m²/s). Le travail moteur reconstruit
reste une estimation ; aucun travail de contact exact n'est revendiqué.

Maximums physiques : ancrages 4,03 µm, pénétration sol 16,37 mm, erreur
articulaire 0,00802 rad, auto-pénétration 0. Aucun dépassement de cap délivré ;
somme installée≤ 760 Nm, ratio anatomique≤ 1, assistance bassin exactement 0.
Le diagnostic est terminé sans refus numérique.

![États réellement intégrés et contacts](../evidence/c-framing-continuation-2026-10-11/c-landing-actual.png)

## Décision suivante, sans nouveau mouvement

C conserve les bras presque tendus : flexion des coudes≈ 0,194 rad et paumes
environ 0,717 m derrière leurs épaules. Ses références gelées à 2 s attendent une
base tibiale qui n'est pas acquise dans ce repos ventral. Continuer ce programme
ne constitue donc pas une commande de récupération depuis le sol.

La prochaine étape à examiner est une préparation de brace à partir des
coordonnées et contacts mesurés : fléchir/repositionner les bras pour établir
les paumes près des épaules, obtenir leur charge réelle, décharger la tête,
puis relever le torse tout en conservant les autres appuis. Portée, orientation
des paumes, auto-collisions et redistribution dynamique restent à vérifier.
L'analyse est [conservée séparément](../evidence/c-framing-continuation-2026-10-11/next-command-analysis.json) ;
aucune IK, pose candidate ou nouvelle commande n'est testée ce tour.

Le sélecteur Rust existant choisirait `Brace` pour cette orientation ; il n'a
pas été activé. Son helper général `has_persistent_support` compte actuellement
toute surface, y compris la tête : la sélection de route ne suffit donc pas à
certifier l'éligibilité des appuis au regard du contrat.

Pour le relevage complet, un contact transitoire devient une observation de
mode, tandis que seules les preuves intégrées autorisent la progression.
Perte d'appui ou stagnation imposent une reprise/attente de phase sur les mêmes
corps ; elles ne fabriquent ni succès ni état final. Les pieds bilatéraux≥ 3 N
persistants 0,05 s, pelvis/torse UpY≥ 0,97, pelvis> 0,93 m, bornes natives de
mouvement et maintien 0,55 s restent nécessaires avant la qualification debout,
puis observation ordinaire 1 s, horizon 25 s et cinq cycles sans Reset. Aucun de
ces critères de livraison n'a changé.

## Coût, vérifications et preuves

Une première invocation invalide a arrêté le replay à 0,416667 s : le banc
comptait le calme initial avant le refus original comme repos au sol. Elle
n'observe aucun état après 1,554 s et reste entièrement conservée dans
`rejected-prelanding-stop/`. La correction concerne uniquement le début du
comptage, désormais après le premier refus original. Deux invocations du même
programme sont donc rapportées, avec une seule continuation au-delà du refus :
100+484 pas,2,433333 s physiques et 1,025582+5,128876=6,154458 s de calcul natif.
Aucun nouvel essai de gain, trajectoire, répétition ou cap. Les compilations
release sont consignées séparément du budget natif.

Les deux tests du banc, Clippy strict et formatage passent. Le préfixe exact
constitue la vérification causale des mêmes commandes/physique. Les 31 tests
récupération et 4 mesures du tour précédent ne sont pas revendiqués comme
relancés ; la suite complète demeure non déclarée verte.

[Analyse native](../evidence/c-framing-continuation-2026-10-11/analysis.json),
[trace complète](../evidence/c-framing-continuation-2026-10-11/native/c-landing-observation.json),
[vidéo annotée](../evidence/c-framing-continuation-2026-10-11/c-landing-actual.mp4),
[plan et budget](../evidence/c-framing-continuation-2026-10-11/PLAN.md).
L'original C, son rapport et l'invocation rejetée sont conservés ; le dossier
inclut les 284 sources gelées, binaires, empreintes et scripts reproductibles.
