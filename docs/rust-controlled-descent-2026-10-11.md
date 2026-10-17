# Descente coordonnée et énergie avant appui tibial

**Contrôle de cadrage ultérieur :** le refus sur cuisse ci-dessous concerne le
corridor tibias/mains. Il n'est pas un échec terminal du relevage général.
La continuation séparée, mêmes commandes/limites, atteint un repos ventral
à 2,017 s sans garde physique violée ; tête encore porteuse, aucune récupération
qualifiée. Voir [audit et continuation](rust-c-framing-continuation-2026-10-11.md).
Le verdict et les preuves de l'expérience originale restent conservés.

La séquence tibias d'abord ne stabilise pas le corps. Elle avance le premier
contact de 37,5 ms, mais ne diminue pas l'énergie à temps physique égal : à
1,35 s, C possède 220,83 J contre 217,82 J pour B. Les tibias prennent une forte
charge verticale alors que le COM et la capture sont déjà en avant de leurs
contacts. Le corps pivote puis prend appui sur la cuisse ; refus à 1,554 s,
aucun maintien continu de 0,55 s. Aucun transfert, répétition ou essai à 60°
n'est lancé après ce refus.

Cette expérience prolonge le
[diagnostic mains/tibias](rust-hand-support-experiment-2026-10-11.md), sur
`investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. Les modifications antérieures sont
préservées. Aucun push, merge ou publication ; récupération de production
toujours désactivée et non qualifiée.

## Bilan avant le premier nouvel appui

L'énergie cinétique est reconstruite à partir des vrais états natifs des 25
corps et des tenseurs de masse intégrés de la géométrie canonique :
`K = Σ(½m|v|² + ½ωᵀIworldω)` et `U = Σmg yCOM`. La masse totale est 72,2 kg.
Les reconstructions du COM et du moment cinétique correspondent aux
observations natives à environ 3,5×10⁻⁷ m et 2,8×10⁻⁵ kg·m²/s au maximum.
Le travail des moteurs est estimé par leurs impulsions angulaires appliquées
aux corps et la moyenne des vitesses avant/après le pas. Il est approximatif ;
le résidu comprend contacts, amortissement et discrétisation. Aucun travail
de contact exact ni énergie élastique n'est inventé.

| Juste avant son premier nouvel appui | A bras fixes | B mains préparées | C tibias d'abord |
| --- | ---: | ---: | ---: |
| Temps physique | 1,416667 s | 1,387500 s | 1,350000 s |
| Énergie cinétique de tous les corps | 276,05 J | 265,82 J | 220,83 J |
| Énergie cinétique verticale du COM | 167,69 J | 159,57 J | 121,56 J |
| Énergie potentielle gravitaire libérée | 428,91 J | 415,67 J | 336,79 J |
| Travail moteur estimé | −149,59 J | −145,72 J | −113,33 J |
| Vitesse verticale du COM | −2,155 m/s | −2,102 m/s | −1,835 m/s |
| Moment sagittal autour du COM | 23,99 kg·m²/s | 22,28 kg·m²/s | 23,44 kg·m²/s |

Cette table compare des événements à des dates différentes. La baisse apparente
de 16,9 % de K dans C ne prouve donc pas un freinage supplémentaire. À **1,35 s
pour tous**, A/B/C valent respectivement 190,86/217,82/220,83 J ; les moments
sagittaux valent 17,47/17,68/23,44 kg·m²/s. C est légèrement plus énergétique
que B et son moment est supérieur de 32,5 %. C libère alors 26,23 J de gravité
en moins que B, mais ses moteurs absorbent environ 28,11 J de moins : l'effet
ne réduit pas K. L'amélioration initiale de C à 0,5–0,75 s disparaît vers 1 s.

![Énergie et contacts observés à temps égal](../evidence/controlled-descent-2026-10-11/energy-contact-comparison.png)

## Hypothèse et présélection conservées

L'hypothèse fixée dans le [plan préalable](../evidence/controlled-descent-2026-10-11/PLAN.md)
est d'établir tibias/avant-pieds avec un tronc moins incliné, puis d'attendre une
vraie stabilité avant de progresser vers les mains. L'énergie, les vitesses,
le moment cinétique et les contacts à temps égal discriminent cette hypothèse.
Une simple touche ou une baisse de K ne ferme aucun critère physique.

L'unique optimisation géométrique, depuis un seul point initial, s'arrête à
159 évaluations dans une branche pieds plats : le tibia reste à 9 cm du sol.
Ce résultat numérique n'est pas une impossibilité mécanique. Une construction
algébrique unique, sans nouvelle graine ni relance d'optimisation, conserve
l'orientation tibias/chevilles/orteils du terminal déjà connu. Les genoux sont
placés à 1° sous leur plafond anatomique et les hanches sont déduites pour un
bassin virtuel à 0,5 rad ; les coordonnées lombaires/thoraciques restent neutres.
La première phase interpole les jambes vers cette base en 2 s et garde les
bras initiaux. La posture globale virtuelle ne devient jamais une pose imposée.

La base virtuelle a son COM à 0,521 m de hauteur, z=0,433 m, entre les contacts
avant-pieds et tibias, avec environ 11 mm de marge avant. Le souhait souple de
25 mm de la recherche n'est pas un seuil de qualification ; la marge native
requise reste 3 mm. Le contrôle natif des 121 positions de la branche
préliminaire et le SAT convexe exact des 121 positions finales sont conservés
séparément. Le SAT final admet limites, sol et collisions ; les points et
quaternions évalués sont archivés. Aucune intégration n'a lieu dans ces contrôles.

Le LP statique de la base finale est admissible sous les limites originales :
239,264 Nm au total, pic anatomique 45,67 %, résidu d'équilibre
1,14×10⁻¹³. Il répartit environ 690 N sur les tibias et 18 N sur les pointes.
Cette existence statique ne certifie ni l'arrivée ni le maintien natif.

![Séquence virtuelle uniquement](../evidence/controlled-descent-2026-10-11/sequence-geometric-prescreen.png)

## Comparaison native bornée

C part de la même construction réelle que A/B : les snapshots initiaux sont
exactement identiques. Aucun clone vivant, transplantation, pose/rotation/vitesse
écrite, reset ou force directe au bassin. Les articulations restent les moteurs
anatomiques d'origine, mêmes gains, solveur, contacts et allocation par demande,
somme installée strictement ≤760 Nm. Tous les corps, articulations et colliders
conservent leurs identités surveillées. Les états/compteurs ordinaires ne sont
pas réinitialisés pour maintenir une phase.

C change jambes et tronc ensemble. Ses bras restent fixes, comme A, à seulement
1,05×10⁻⁷ rad près du fait des arrondis de référence ; leurs vitesses de consigne
sont identiques. A sert donc aussi de contrôle pour éviter d'attribuer aux jambes
un effet de préparation des bras dans B. Les caps résultants répondent aux
demandes de chaque séquence ; ils ne sont pas artificiellement égaux par axe.

Le banc peut suspendre les références à la base tibiale et exige ses charges
persistantes réelles, ≥80 % du poids, COM/capture dans le polygone résolu à 3 mm,
tous les corps ≤0,22 m/s et ≤0,65 rad/s pendant 0,55 s. Il s'arrête à cette base
pour vérifier reproductibilité avant toute progression. Ce résultat est séparé
du transfert mains/tibias, qui conserve son critère bilatéral plus complet, et
de la qualification finale du relevage. Dans C, le refus arrive avant le temps
de base prévu de 2 s : l'attente n'a donc pas été exercée physiquement.

Le budget est respecté : 159/200 évaluations de résidu, 1,364/30 s pour l'unique
optimisation, zéro relance ; une seule construction algébrique, SAT en 1,069 s,
un LP statique. Les 121 inspections natives concernent la branche préliminaire ;
les 121 vérifications finales sont algébriques/SAT, explicitement distinctes.
Un seul nouvel essai natif, 373 pas soit 1,554167 s, 5,371/120 s murales ; horizon
maximal 6 s. Le témoin conservé n'est pas rejoué. Répétition et 60° restent
conditionnés au succès et sont donc sautés. La compilation release dure 72 s,
hors du budget de simulation explicité au préalable.

## Premier obstacle causal précis

Le pivot réel diverge de la trajectoire globale virtuelle avant l'impact :

- Le moteur d'avant-pied gauche sature dès 0,079 s. À 0,783 s, son angle atteint
  0,7809 rad près de sa butée 0,7854, pour une cible 0,2141. La demande vaut
  −181,87 Nm, le cap installé 30,49 Nm, le plafond anatomique reste 35 Nm.
  Le signe brut d'impulsion de contrainte n'est pas présenté comme un couple
  généralisé de freinage.
- À 1,35 s, le bassin réel est à 0,749 rad contre 0,426 dans la géométrie
  virtuelle. Le COM avance à 1,398 m/s, descend à 1,835 m/s et reste hors des
  appuis réels. Hanche, lombaires et thorax restent pourtant proches de leurs
  références relatives : une bonne FK et un bon suivi articulaire ne fixent
  pas l'orientation globale ni le moment du corps.
- Les tibias touchent à 1,354 s et deviennent bilatéralement persistants à
  1,408 s. À 1,40 s, ils portent 677 N, mais leurs points sont à z≈0,562 m
  contre COM z=0,759 m. La capture est encore plus en avant. Les contacts
  glissent peu, environ 0,0095 m/s : ce n'est pas une perte immédiate de friction.
- À ce même pas, seuls les tibias sont porteurs et chacun a un point résolu.
  La somme de leurs forces tangentielles avant mesurées est **+293,294 N**,
  cohérente avec `M aCOM,z = +293,293 N`. Leur appui accélère encore le COM
  en avant à +4,062 m/s². Aucune tangente de manifold n'est comptée deux fois
  dans cette mesure.
- La cuisse touche à 1,508 s puis devient porteuse de façon persistante à
  1,554 s : refus original à 38,98 N. Aucun intervalle stable de 0,55 s,
  aucune main porteuse, aucune progression vers les pieds.

![Positions et appuis réellement résolus](../evidence/controlled-descent-2026-10-11/native-bc-actual.png)

Le blocage mesuré est donc le contrôle du support et du mouvement global avant
l'arrivée, suivi d'un contact tibial dont la résultante n'arrête pas le pivot.
Cette séquence de références relatives échoue ; cela ne prouve pas que les
tibias ou les mains ne peuvent absorber aucune autre arrivée. Les forces de
contact observées sont cohérentes avec l'accélération native : aucun nouveau
défaut de restitution des impulsions n'est nécessaire pour expliquer ce refus.

La prochaine décision doit porter sur la viabilité dynamique du COM, du moment
et du wrench de support pendant la descente, vérifiée par prédiction native,
plutôt que choisir une autre pose terminale ou ralentir une courbe sans preuve.
Un programme natif admissible pourrait être anticipé ou asservi ; cet essai
ne démontre pas qu'un type de contrôle particulier est indispensable. Changer
la séquence d'appuis sera justifié si ces contraintes restent incompatibles
dans un mode explicitement défini. Aucune limite mécanique globale ni nécessité
universelle de mains/genoux n'est certifiée ici.

## Vérifications et livrables

Le seul fichier Rust changé depuis l'expérience précédente est le banc sous
`cfg(test)`. Les références rejetées restent dans `evidence`, aucun contrôleur
de production n'est adopté. Le test de la pause vérifie des consignes tenues
sans écriture des corps ; il ne prétend pas valider un appui physique.

Construction release hors ligne/verrouillée, deux tests ciblés du banc,
31 tests `recovery::` (6 diagnostics ignorés), quatre tests `measurement::`,
Clippy strict et formatage passent. La suite complète n'est pas revendiquée :
le test hybride d'articulation et les quatre tests TypeScript signalés sur le
témoin Linux ne sont pas déclarés réparés. Les seuils d'intégrité et les caps
n'ont pas changé.

[Analyse complète](../evidence/controlled-descent-2026-10-11/comparison-analysis.json),
[premier obstacle exact](../evidence/controlled-descent-2026-10-11/first-obstacle.json),
[bilan d'énergie](../evidence/controlled-descent-2026-10-11/energy-comparison.json),
[vidéo native B/C](../evidence/controlled-descent-2026-10-11/native-bc-actual.mp4),
[sources gelées](../evidence/controlled-descent-2026-10-11/source-frozen-manifest.json).
L'archive portable conserve sources, moteur vendé, fixtures, contrat, deux
binaires, témoins A/B, entrée exacte C, refus, scripts et contrôles d'intégrité.
Image et vidéo sont sauvegardées **séparément** dans la Library privée, avec
leurs identifiants dans `library-delivery.json`, en plus de l'archive.
