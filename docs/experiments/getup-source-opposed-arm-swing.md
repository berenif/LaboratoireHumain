# Balancement sagittal des bras opposé aux jambes

Essai demandé le 18 octobre 2026, sur la base sélectionnée hanches élargies de
10° et genoux légèrement fléchis. Le preset sélectionné et les valeurs par
défaut de l'application restent inchangés. C'est un essai optionnel.

## Commande et définition de l'amplitude

Les bras sont d'abord placés physiquement un peu à côté du corps, avec une
transition quintique de quatre secondes. Ensuite, le changement d'angle sagittal
épaule-main est l'opposé exact du changement d'angle sagittal cuisse-genou du
même côté. Le gain angulaire demandé est −1 : jambe droite en avant, bras droit
en arrière, et réciproquement. L'excursion est mesurée crête à crête, dans le
repère du torse, vers l'avant positif. Ce ne sont ni des déplacements linéaires
égaux ni des angles individuels de charnière égaux.

Le pilote de phase est la position réellement mesurée des jambes, pas une
sinusoïde indépendante. Les axes d'épaule sont obliques et en miroir : une
inversion par la cinématique exacte transforme l'angle sagittal en deux angles
d'épaule, en conservant la composante latérale épaule-main. Le coude garde sa
préférence neutre. La table vérifie ±0,45 rad ; une demande extérieure est
refusée, jamais écrêtée. L'essai initial de table ±0,60 rad avait été refusé sans
aucune intégration, l'extrémité arrière gauche atteignant une limite d'épaule.

Le suivi des bras utilise des gains position/vitesse 100/20 et un poids QP 2,
avec dérivées de référence causales. Cela ne change pas la force des moteurs :
les limites d'effort, le modèle de contact, le frottement, les barrières et tous
les seuils restent les mêmes. Aucune réaction supplémentaire au COM n'est
ajoutée aux bras. Les tâches COM/torse existantes restent actives.

## Protocole physique

Même modèle sélectionné, cache propre lié au modèle, q/qvel initiaux et moteur
MuJoCo 2.1.3. Intégration à 800 Hz, QP de couple actualisé toutes les 5 ms. Aucun
reset entre les pas, aucune injection de pose, assistance de racine,
augmentation de puissance ou relaxation de critère.

1. Un pas de 30 mm, suivi de cinq secondes calmes : succès en 19,14 s /
   15 312 intégrations. Avance 32,018 mm ; glissement chargé 2,229/2,142 mm.
2. La séquence identique de quatre pas alternés 30/60/90/45 mm et cinq secondes
   calmes, dans un seul monde continu : succès.
3. Une répétition déterministe avec les empreintes du code figées au démarrage
   vérifie le code final : journal, commandes et cache final sont identiques
   octet par octet. Le reçu confirme que le code est resté inchangé pendant
   cette répétition ; ses empreintes sont jointes aux preuves.

## Résultats de la séquence

- Avances réelles : 31,989 /61,894 /91,933 /46,836 mm.
- 49 861 intégrations, soit 62,32625 s ; 4 000 échantillons calmes consécutifs.
- Glissement chargé gauche/droite : 6,601/5,980 mm, sous la borne globale de
  10 mm, contre 6,774/6,253 mm pour la base sélectionnée.
- Marge COM/capture minimale : 9,189 mm, contre 7,844 mm. Ce point de capture
  instantané utilise le proxy LIP existant ; ce n'est pas un certificat de
  viabilité dynamique.
- Effort moteur maximal : 94,945 %, sous le plafond inchangé de 95 %.
- Séparation géométrique minimale mesurée : 45,518 mm, contre 72,73 mm pour la
  base ; aucun contact pénétrant, appui non pédestre, perte totale d'appui ou
  avertissement moteur. La séparation est échantillonnée aux commandes à
  200 Hz ; la pénétration, les limites et les efforts le sont à 800 Hz.
- Les 22 rayons d'articulation et huit rayons de pied restent valides.
- Une reconstruction FK indépendante de 2 494 poses à 40 Hz retrouve les
  angles de bras et jambes avec erreur maximale 8,33 × 10⁻¹⁶ rad, sans
  intégration supplémentaire.

### Même amplitude, mesure réelle

Sur les phases lift/translate/land des quatre pas, côté droit/gauche :

- Jambes : 17,783° /18,700° d'excursion sagittale.
- Cibles des bras : exactement 17,783° /18,700°, sans écrêtage.
- Bras réels : 17,775° /18,688°, écart d'excursion −0,047 % /−0,066 %.
- Corrélations bras/jambe : −0,999995 /−0,999990.
- Erreur de suivi maximale : 0,0505° /0,0641° ; RMS 0,0210° /0,0236°.

Chaque pas séparé respecte aussi la tolérance physique déclarée de ±5 % sur
l'excursion ; le plus grand écart est +0,84 %. La cible reste exactement 1:1.
Cette tolérance n'est pas un changement des critères d'appui ou de sécurité.
Le placement initial des bras est exclu de ces mesures de balancement. Après ce
placement, les mains restent latéralement à 12,18–12,28 cm /9,22–9,31 cm des
épaules, dans le repère du torse. Les genoux terminent réellement fléchis à
21,88° /21,55° ; 27,5° reste une préférence, pas une pose imposée.

### Coûts et limites

Le travail mécanique absolu total est 287,127 J, contre 237,815 J pour la base
(+20,7 %). Ce total inclut le placement initial des bras et la modification des
mouvements du corps entier ; il ne mesure pas le coût du seul balancement.
La répartition descriptive figure dans work-by-phase.json : 11,433 J pendant
les quatre premières secondes contre 10,196 J, et 206,289 J pendant les phases
lift/translate/land contre 147,195 J. Ces phases comprennent toujours tout le
corps ; ce n'est pas une attribution causale aux seuls bras. Aucun gain
énergétique, électrique ou métabolique n'est revendiqué.

Le dépassement initial de limite souple hérité de l'entrée reste présent jusqu'à
63,75 ms, comme au témoin ; le maximum initial concerne la cheville. La marche
et les cinq secondes calmes passent, avec réserve articulaire minimale pendant
la marche de 0,318°. Le critère strict « tous les échantillons dans les limites »
reste faux. Il n'est ni masqué ni transformé en succès global sans réserve.

Une séquence et sa répétition déterministe ne prouvent pas une robustesse aux
perturbations. La baisse de glissement et la hausse de marge restent des
observations de ce scénario. La base sélectionnée n'est pas promue ou remplacée.

## Code et vérification

Entrée optionnelle : scripts/getup-step/source_opposed_arm_swing.py.
Audit hors intégration : scripts/getup-step/audit_source_opposed_arm_swing.py.
Le chemin historique de suivi des bras conserve littéralement son expression ;
18 comparaisons contre le contrôleur figé 749ee2d7 donnent des commandes et des
informations strictement identiques, sans intégration.

40 tests ciblés passent, dont dix nouveaux tests sans intégration : axes en
miroir, gain géométrique 1:1, dérivées de table, refus hors plage, transition
initiale, causalité des jambes sur les vraies commandes moteur, invariance du
monde natif et du modèle sélectionné. Une tentative antérieure de découverte large test_source_*.py
a exécuté 75 tests avec dix erreurs préexistantes d'environnement : quatre imports
requièrent Torch absent de .venv/mvp, et six tests de preview rencontrent leurs
dossiers de sortie déjà présents. Ce contrôle large n'est pas déclaré réussi.

Reproduction, depuis la racine et avec les dépendances natives locales :

```sh
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 .venv/mvp/bin/python \
  scripts/getup-step/source_opposed_arm_swing.py --output NEW_EMPTY_DIRECTORY
# Ajouter --one-step pour le pilote de 30 mm et cinq secondes calmes.
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 .venv/mvp/bin/python \
  scripts/getup-step/audit_source_opposed_arm_swing.py \
  NEW_EMPTY_DIRECTORY/run/generation-001
```

Les preuves compactes sont dans docs/evidence/getup-source-opposed-arms-2026-10-18.
Les assets originaux, XML, caches natifs et journaux bruts restent locaux, hors
commit. Aucun changement sur main, aucun déploiement.
