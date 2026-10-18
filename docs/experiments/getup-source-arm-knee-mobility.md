# Bras le long du corps et flexion légère des genoux

Essai borné du 18 octobre 2026. Le contrôleur par défaut reste inchangé.

## Protocole

Deux variantes isolées face au témoin original qui a réussi quatre pas de
30/60/90/45 mm, durée des trajectoires multipliée par deux. Même moteur natif
MuJoCo 2.1.3, modèle XML original, masses, frottement effectif 1, moteurs,
barrières d'évitement et seuils d'admission. Maximum moteur 95 %. Même état
initial q/qvel, contacts et solveur initialisés à froid. Aucun déplacement de
pose physique entre les pas, force/torque appliquée à la racine ou apprentissage.

Le témoin historique `multistep10` a été reproduit à l'identique par l'expérience
hanche seule. Les audits ici lisent ses mêmes données et les deux nouveaux essais.

- Bras : référence géométrique rapprochant les mains du corps, avec distance
  extérieure aux cuisses. Transition lisse de quatre secondes, préférences de
  posture dans le solveur existant et réaction bornée aux mesures COM, vitesse
  COM et vitesse angulaire du torse. Couplage de la matrice de masse pour
  demander une réaction de moment ; décalage de cible limité à 0,035 rad,
  variation limitée à 0,08 rad/s. Les contraintes moteur/contact restent
  prioritaires. La causalité des commandes ne prouve pas un bénéfice dynamique.
- Genoux : préférence neutre à 27,5° de flexion, sans changement de force.
  L'entrée était déjà fléchie de 20,14°/25,45°. Référence de support recalculée
  avec pieds/COM/marges de moteurs, puis réalisée par les seuls moteurs.
  C'est une préférence souple, pas un angle imposé à l'état physique.

Les deux références passent le contrôle statique sans intégration : besoins
moteur 90,159 % /90,086 %, séparations minimales 63,2/73,4 mm. Les tests vérifient
l'absence de mutation de l'état natif complet, les contre-factuels COM/vitesse/
rotation, le changement des vraies commandes QP et la limite de variation.

## Résultats physiques

Témoin : quatre pas réussis, 31,906/62,058/91,844/46,803 mm, puis cinq secondes
calmes. 49 857 intégrations, 62,32125 s simulées. Glissement chargé 8,611/5,965 mm.

Bras seuls : deux pas terminés, 31,901/61,969 mm. Arrêt sur la réception du
troisième, à 37,7825 s, au seuil inchangé de glissement chargé 10 mm :
4,654/10,005 mm. Marge de capture finale −21,60 mm. Les mains sont effectivement
plus proches du corps (distance horizontale main/épaule 11,25/10,67 cm contre
32,70/27,97 cm au témoin final), sans battement : vitesse articulaire maximale
des bras 0,185 rad/s contre 0,158 au témoin. Aucun contact pénétrant ; séparation
mesurée minimale 26,41 mm. L'essai ne démontre aucun gain d'équilibre et n'est
pas retenu comme candidat par défaut.

Genoux seuls : quatre pas réussis, 31,922/61,916/91,904/46,860 mm, puis cinq
secondes calmes. Même 49 857 intégrations et 62,32125 s. Glissement chargé
6,532/5,550 mm. Flexion finale réelle 23,879°/23,221°, contre 4,032°/23,889° au
témoin. Sur tout l'essai, flexion 14,62–44,00° et 15,44–44,03°, sans retour à
l'extension observée sur le témoin. Vitesses maximales des genoux 0,458/0,524
rad/s contre 0,461/0,813 ; couples maximaux 27,62/24,54 Nm contre 31,95/27,21.
Marge de capture finale 104,74 mm contre 104,80 mm. Séparation mesurée minimale
71,53 mm, aucune pénétration ni appui non pédestre. Moteurs toujours ≤95 %.
Le travail mécanique absolu total augmente de 235,38 à 242,90 J (+3,2 %) : aucun
gain énergétique n'est revendiqué.

Le dépassement initial hérité de limite souple de cheville reste présent sur
les 63,75 premières millisecondes, comme au témoin. L'audit de marche et de
stabilité passe pour les genoux, mais le critère strict « tous les échantillons
dans les limites » reste faux. Ni ce défaut ni les seuils n'ont été masqués.

Un seul essai par variante. La baisse de glissement n'établit pas une robustesse
générale. Aucun changement par défaut, aucune combinaison incluant les bras.
Les ROM, efforts et résultats de chaque phase figurent dans les JSON d'audit.

## Reproduction

Depuis la racine, avec les dépendances natives locales déjà présentes :

```sh
BASE=$PWD/outputs/getup-source-engine/session-20261017T233404Z
HELPER=$PWD/outputs/getup-source-step/first-liftoff/source_dynamics-v9.so
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 .venv/mvp/bin/python \
  scripts/getup-step/source_arm_knee_mobility.py --base "$BASE" --helper "$HELPER" \
  --output NEW_EMPTY_DIRECTORY --variant knees
# Remplacer knees par arms pour l'essai bras seul.
```

`source_arm_knee_mobility.py` utilise la machine d'états inchangée de
`source_multistep_probe.py`. `plan_source_arm_knee.py` reprend le planificateur
statique et ajoute seulement des préférences de posture et une pénalité de
proximité. `audit_source_arm_knee.py` relit les mesures sans intégration. Les
assets, XML, caches natifs, poids et journaux bruts restent hors du commit.

## Combinaison justifiée : hanches élargies + genoux, bras inchangés

Après les succès isolés, un seul essai additionnel combine la copie du modèle
avec six limites de hanche élargies de 10° par borne et la préférence de flexion
des genoux. Aucun changement du programme moteur, du frottement, du rythme ou
des bras. La copie XML, ses limites et son cache propre sont ceux de l'expérience
hanche ; l'état q/qvel initial est vérifié identique. Le contrôle statique sans
intégration préserve l'état natif octet par octet : besoin moteur 90,127 %,
séparation 75,37 mm. Le code exécuté est exactement celui du commit isolé.

Le combiné réussit les quatre pas, 31,936/61,961/91,934/46,882 mm, puis cinq
secondes calmes, en 49 861 intégrations /62,32625 s. Flexion finale réelle
24,146°/22,494°. Les genoux restent fléchis de 15,32–43,65° et 19,69–44,21° sur
l'ensemble de l'essai. Vitesse des bras maximale 0,158 rad/s ; aucune pénétration,
appui non pédestre, perte totale d'appui ou alerte moteur. Séparation mesurée
minimale 72,73 mm. Les 22 rayons d'articulation et huit rayons de pied restent
valides à chaque intégration. Le contrôle de marche et de stabilité passe ; le
défaut initial de limite souple demeure jusqu'à 63,75 ms, donc le contrôle strict
de tous les échantillons reste faux.

Le combiné ne domine pas les variantes isolées :

- Glissement gauche/droite : 6,774/6,253 mm, contre 7,511/5,748 pour les hanches
  seules et 6,532/5,550 pour les genoux seuls.
- Marge de capture minimale : 7,844 mm, entre les hanches seules (9,720) et les
  genoux seuls (6,920). Témoin original : 2,542 mm.
- Travail mécanique absolu : 237,815 J, contre 205,724 pour les hanches seules,
  242,901 pour les genoux seuls et 235,384 au témoin.
- Effort moteur maximal : 95,0 %, contre 93,615 % pour les hanches seules.
- La mobilité supplémentaire est réellement utilisée : 29 135 échantillons
  dépassent au moins une ancienne limite de hanche ; réserve minimale aux
  nouvelles limites 0,324°. Aucun dépassement des nouvelles limites de hanche.

C'est donc une combinaison physiquement valide avec flexion visible, qui reste
optionnelle. Aucun nouveau réglage ni essai supplémentaire n'a été lancé. Les
bras rapprochés restent exclus après leur échec isolé.

Les séparations géométriques sont mesurées aux résolutions du contrôleur à
200 Hz ; pénétration, angles/vitesses, efforts et rayons sont audités sur tous
les pas natifs à 800 Hz. Le travail mécanique absolu n'est pas une mesure de
consommation électrique ni métabolique.

Pour reproduire le combiné, reprendre le même appel `--variant knees` avec
`BASE=outputs/getup-source-step/hip-mobility-pair01/wider-session`, préparé par
l'expérience hanche. L'empreinte XML expérimentale attendue est
`216d5c0159abf07139f5556c59e244ae77cb7557ba5256e301ebc5e1c8bbc399`.
Les résultats complets et la comparaison à quatre conditions sont conservés
dans les fichiers compacts `hip-knees*.json` et `combined-comparison.json`.

## Choix retenu par l'utilisateur, 18 octobre 2026

La base source sélectionnée pour la suite cumule désormais hanches +10° et
flexion des genoux, avec les bras d'origine. Le preset est
`scripts/getup-step/source-step-selected-v1.json`. Lancer simplement
`.venv/mvp/bin/python scripts/getup-step/source_selected_step.py --output NEW` :
la copie du modèle et son cache froid correctement lié sont préparés sans
assembler les chemins d'expériences. `--prepare-only` vérifie sans intégration ;
`--variant baseline` conserve un témoin explicite. Ce choix ne modifie ni le
modèle original ni les entrées historiques ni l'application/WASM principale.
