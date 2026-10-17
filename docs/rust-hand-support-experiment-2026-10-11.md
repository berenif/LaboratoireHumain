# Acquisition native des appuis mains et tibias

La préparation des bras acquiert de vrais contacts porteurs et freine la chute,
mais ne stabilise pas cette descente. Les deux variantes sont refusées sur appui
tête : A à 1,596 s, B à 1,617 s. Le sous-problème de maintien intermédiaire reste
ouvert ; aucune remontée ni qualification de récupération n'est acquise.

L'expérience exécute le premier A/B de la
[revue stratégique](rust-recovery-strategy-review-2026-10-11.md), sur la branche
locale `investigate/physical-support`, depuis
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou publication.
Les corrections locales non publiées du fil parent ne sont pas revendiquées.

## Corridor et présélection

Le départ est la fixture réelle `landed-crouch-left`, construite normalement.
Un corridor articulaire symétrique descend pendant 1,8 s puis maintient son
intention jusqu'à 3 s. Les articulations utilisent les coordonnées natives
`2 atan2(q_i,q_w)` et leurs repères anatomiques ; les positions virtuelles
emploient la géométrie canonique complète. Le bassin virtuel sert à la
présélection géométrique, jamais à imposer une pose dans le monde vivant.

La première approche place les mains trop en arrière des épaules. Ses 61
positions passent la géométrie, mais le terminal statique est infaisable sous
les plafonds originaux. Une relaxation diagnostique séparée requiert un facteur
2,411 et environ 77,17 Nm à une ceinture scapulaire plafonnée à 32 Nm. Cette
relaxation n'est ni un critère d'admission ni une commande native.

Une seule correction géométrique rapproche l'appui de l'alignement des bras et
utilise l'autre approche de poignet admise par les mêmes limites. La surface
mince locale −Z des mains est orientée vers le sol. Le modèle ne nomme pas une
matière « paume » distincte ; aucune étiquette de paume n'est inventée.
Les 61 positions de B et les 61 de A passent les contrôles natifs de limites,
sol, auto-collision et assemblage : erreur d'ancrage maximale 1,3411×10⁻⁷ m,
aucun excès exact de limite. Il s'agit de contrôles avant intégration.

Le terminal B satisfait un LP statique avec équilibre force/moment, cône de
friction intérieur, couples anatomiques et plafond total 760 Nm. Effort total
233,270 Nm, utilisation anatomique maximale 39,80 %, résidu d'équilibre
1,14×10⁻¹³ et résidu d'inégalité 1,42×10⁻¹⁴. Cette existence statique ne certifie
ni le transitoire ni la force que Rapier délivrera. La marge terminale du
poignet à sa butée est seulement 0,005374 rad ; celle du coude supérieur est
1,59944 rad.

**Écart au budget géométrique préécrit :** deux préparations bornées à 200
évaluations ont été utilisées, la seconde après le refus statique causal de la
première, soit 400 évaluations au total et environ 2,7 s. La limite initiale
de 200 évaluations cumulées n'a donc pas été respectée. Ce dépassement est
conservé explicitement ; il ne concerne ni les seuils physiques ni une recherche
de gains. Les deux variantes géométriques et leurs refus sont archivés.

![Présélection géométrique et statique seulement](../evidence/hand-support-2026-10-11/geometry-static-prescreen.png)

## Comparaison physique contrôlée

A garde les bras initiaux ; B suit les références du corridor corrigé.
Les états initiaux, toutes les consignes de jambes/tronc et leurs vitesses sont
exactement identiques sur les 383 pas communs. La politique d'allocation par
demande et les gains sont identiques ; les caps qui résultent des demandes
diffèrent lorsque les bras agissent. Le changement causal comprend cette
réponse commune de l'allocation, pas une égalité artificielle de caps par axe.

Le banc installe uniquement des moteurs articulaires plafonnés. Aucun corps
n'est écrit, cloné, remplacé, réinitialisé ou transplanté après construction.
Les identités du monde et de ses 25 corps restent surveillées. L'effort direct
au bassin mesuré vaut zéro. Le contrôleur ordinaire conserve ses états et
compteurs ; le banc ne les remet pas à zéro pour maintenir une phase.

Les surfaces déclarées pendant cette acquisition sont pieds, avant-pieds,
mains et tibias, identiques dans A/B. Les contacts de tête, torse et des autres
segments restent refusés à la charge/persistance d'origine. Points, normales,
charges résolues, points locaux des segments, glissement et impulsion des mains
sont enregistrés à 240 Hz. La persistance et la perte de tous les appuis utilisent
les charges persistantes réelles, distinctes des charges instantanées. Le
maintien exige mains et tibias bilatéraux, ≥80 % du poids porté, COM/capture dans
le polygone réel avec marge 3 mm, tous les corps ≤0,22 m/s et ≤0,65 rad/s pendant
0,55 s. Les gardes de finitude et d'intégrité originales restent actives.

| Mesure native | A bras de référence | B mains préparées |
| --- | ---: | ---: |
| Durée intégrée avant refus | 1,595833 s | 1,616667 s |
| Premières mains bilatérales persistantes | absentes | 1,441667 s |
| Premiers tibias bilatéraux persistants | 1,475000 s | 1,466667 s |
| Première touche tête ≥3 N | 1,525000 s | 1,525000 s |
| Refus tête persistante | 354,69 N | 28,14 N |
| Impulsion horizontale totale des mains, Z | 0 N·s | −98,049 N·s |
| Maintien intermédiaire continu | 0 s | 0 s |
| Somme maximale des caps installés | 760 Nm | 760 Nm |
| Somme maximale des efforts moteurs délivrés absolus | 544,73 Nm | 640,06 Nm |

Deux essais seulement, 3,2125 s physiques et 8,281 s murales. Le refus interrompt
chaque essai avant son horizon de 3 s. B échouant, aucune répétition fraîche,
rotation à 60° ou poursuite du transfert n'est lancée. Les entrées, le binaire et
les traces sont figés et hachés ; aucun replay exact de cette nouvelle paire
n'est revendiqué. Les replays déterministes du témoin antérieur restent dans
l'archive précédente.

![États effectivement intégrés dans A et B](../evidence/hand-support-2026-10-11/native-ab-actual.png)

## Premier mécanisme causal

Avant impact, les bras suivent leur référence : à 1,35 s, le coude gauche
mesure 0,814 rad pour une consigne 0,815. Les mains touchent vers
1,392–1,396 s. À 1,40 s, le COM descend encore à −2,013 m/s et avance à
+1,255 m/s ; le moment cinétique sagittal autour du COM atteint 22,07 kg·m²/s,
puis 35,73 à l'acquisition persistante des mains. Le premier contact de main
se produit au bord distal (`y` local ≈−0,0811 m), pas sur une large paume plate
déjà stable. Les tibias prennent leur contact dans la région proximale
(`y` local ≈+0,1414 m), sans corps de genou séparé ajouté au modèle.

Après impact, le coude gauche atteint 1,518 rad pour une consigne 0,854 à
1,442 s, puis 2,064 pour 0,876 à 1,50 s. Il livre son cap, environ
37,89 puis 43,69 Nm, tandis que la demande PD dépasse 900 Nm en amplitude.
Son plafond anatomique reste 55 Nm et le budget installé total reste 760 Nm.
La ceinture scapulaire et le poignet rejoignent également leurs butées.
La petite marge statique du poignet ne laisse presque aucune réserve à ce choc.
Les mains freinent réellement : à 1,50 s, l'accélération avant du COM vaut
−4,065 m/s² dans B contre +1,802 dans A. Le freinage ne stabilise ni la rotation
ni les articulations porteuses. Le COM et la capture peuvent entrer dans le
polygone réel tout en conservant un mouvement trop rapide et un appui tête.

L'observation principale est donc une arrivée dynamique trop énergétique,
suivie du repliement des bras saturés. La portée instantanée devient disponible
et le suivi articulaire fonctionne avant contact. Une simple anticipation du
terminal est contredite par la mesure géométrique sur le tronc réel : à 1,25 s,
elle placerait le bas des mains à 0,448 m, contre 0,296 m réellement mesuré.
Ce calcul est une inspection virtuelle de portée, jamais une transplantation
dans la simulation. Une augmentation arbitraire de caps ou un nouveau réglage
de durée ne constitue donc pas une correction étayée.

![Freinage réel et maintien non acquis](../evidence/hand-support-2026-10-11/native-ab-causal.png)

La prochaine hypothèse utile est une descente coordonnée qui réduit vitesse
verticale et moment cinétique **avant** le premier appui, avec contacts et couples
dynamiquement admissibles. Elle doit être certifiée prospectivement dans le
monde natif ; la seule statique terminale ne suffit pas. Ni l'impossibilité des
pieds seuls ni la nécessité universelle des mains/genoux ne sont démontrées.
Aucune correction physique unique directement justifiée n'est retenue ici.

## Code et vérifications

Le nouveau banc est exclusivement sous `cfg(test)`, opt-in et ignoré dans le
parcours ordinaire des tests. Les corridors acceptés géométriquement ou refusés
restent dans `evidence`, hors du contrôleur de production. Il ajoute un test
d'intégrité qui vérifie qu'installer les consignes A/B ne change aucune position,
rotation, vitesse, force ou identité de corps et respecte chaque plafond.
La récupération de production reste désactivée par défaut et non qualifiée.
Le code préexistant est conservé, y compris le moteur et les diagnostics :
seuls les hooks du nouveau banc et la visibilité de ses trois helpers changent
par rapport au snapshot retenu C. Tous les fichiers moteur vendus sont identiques.

- Rust 1.89.0, construction release hors ligne et verrouillée : passe.
- Test d'intégrité des commandes dans le binaire release : 1 passe.
- Tests `recovery::` : 30 passent, 5 diagnostics ignorés, aucun échec.
- Tests `measurement::` : 4 passent.
- Clippy `lh-sim --all-targets -- -D warnings` et formatage : passent.
- Intégrité des traces : effort externe nul, somme caps ≤760, aucun cap
  anatomique dépassé et aucune livraison par axe supérieure au cap installé.

Ce sont des vérifications ciblées. La suite complète n'est pas déclarée verte :
un test hybride d'articulation et quatre tests TypeScript échouaient déjà sur
le témoin Linux signalé par le parent. Aucun seuil n'a été modifié pour les faire
passer.

## Livrables et reproduction

[Dossier de preuves](../evidence/hand-support-2026-10-11/),
[analyse native](../evidence/hand-support-2026-10-11/native-ab/analysis.json),
[vidéo A/B issue des vrais états](../evidence/hand-support-2026-10-11/native-ab-actual.mp4),
[manifest du binaire](../evidence/hand-support-2026-10-11/release-manifest.json) et
[sources complètes](../evidence/hand-support-2026-10-11/source-complete-manifest.json).
Les sources gelées comprennent le moteur vendé, les fixtures, le contrat et
les coordonnées exactes évaluées. Les scripts de préparation, LP, inspection,
analyse et rendu, ainsi que leurs journaux, sont conservés. Les anciens essais
géométriques portent le préfixe `rejected-rear-hand`.

```sh
LH_HAND_PLAN="$PWD/evidence/hand-support-2026-10-11/corridor-evaluated.json" \
LH_HAND_OUT="/tmp/lh-hand-ab-fresh" \
evidence/hand-support-2026-10-11/hand-support-release-binary \
recovery::support_probe::hand_support_experiment::hand_support_ab \
--exact --ignored --nocapture
```

Le chemin de sortie doit être neuf ; le banc refuse tout écrasement. L'archive
portable et les visuels sont livrés dans la Library privée, avec les identifiants
et sommes de contrôle consignés dans `library-delivery.json`. Cela ne publie
ni le dépôt ni une nouvelle version de la simulation.
