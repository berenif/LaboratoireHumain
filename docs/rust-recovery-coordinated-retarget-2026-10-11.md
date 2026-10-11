# Référence géométrique admissible ; relevage physique refusé

**Verdict du 11 octobre 2026 : 96 histoires natives, aucun relevage ni retour debout
après la descente. La campagne est close à son seuil d’abandon.** La récupération
reste désactivée par défaut et non qualifiée. Aucun seuil, couple anatomique,
contact, masse ou paramètre physique n’a été relâché.

Base vérifiée : `main` à `c597959f46f37715035349c8cd0a699b6822111a` ; travail sur
`investigate/physical-support`. L’autorisation finale couvre un checkpoint et un
push sur cette branche uniquement, sans merge ni force push. Les descriptions
antérieures sans publication décrivent leurs jalons respectifs. Les corrections
non livrées annoncées dans l’autre environnement ne sont pas revendiquées ici.

## Intervention et distinction des preuves

Le correctif de support utilise les points persistants réellement chargés du
solveur, au lieu d’élargir l’appui avec des sommets de semelle proches du sol.
Un contact ponctuel ou presque linéaire peut définir une demande de freinage
bornée ; il ne certifie pas un polygone d’appui stable. La régression vérifie ce
cas depuis les fixtures accroupies, sans écrire les poses des corps. **Ce
correctif répare une interprétation du support ; il ne réussit pas le relevage.**

L’interpréteur expérimental commun, le prédicteur natif et les outils d’essai
restent sous `cfg(test)` ; les histoires physiques sont des tests explicitement
ignorés. Aucun programme rejeté n’entre dans le contrôleur de production. Le
contrôle géométrique peut placer la racine d’une référence pour inspecter ses
maillages ; l’intégration physique commande uniquement les moteurs articulaires,
sans cible au bassin, transplantation de poses, remise à zéro ou rebasing.

## Référence complète et géométrie

L’export SLOW et sa provenance sont décrits dans
[la préparation de référence](rust-recovery-reference-preparation-2026-10-11.md).
La source compte 16 corps mobiles, 40,844 kg et 1,55 m ; le modèle natif 25 corps,
72,2 kg et 1,84 m. Les genoux restent limités à 140° et les chevilles à 20° de
dorsiflexion. Les 46 coordonnées natives participent à l’adaptation.

Le programme absolu final comporte 75 clés et dure 20,883333 s. Les polynômes
Hermite respectent les bornes articulaires et la vitesse originale de 1 rad/s :
maximum analytique **0,960097722 rad/s**, continuité C1, vitesses terminales nulles.
Son SHA-256 est `fef70abf3d016cecf6658b0bc5e7a6361d989eb265840acdc1a4fbc497a70c7d`.
Le compilateur natif vérifie aussi la grille `f32` à 240 Hz et ses annotations.

Une translation verticale géométrique couplée aux 1 526 sommets corrige les
pénétrations entre poses. Deux corrections compactes C1 corrigent les croisements
pied/avant-pied. La seconde nécessitait une normalisation oubliée : facteur
1,739855449, amplitude finale 0,018546263 rad ; aucune nouvelle optimisation de
pose dans le budget supplémentaire. La racine initiale est conservée et cette
translation n’est jamais appliquée comme une commande physique.

Le certificat final passe : 864 intervalles avec borne continue d’auto-collision,
73 intervalles d’un sous-pas vérifiés à leurs deux extrémités sur la grille 240 Hz.
**Ces 73 intervalles ne constituent pas une preuve continue entre sous-pas.**
Le contrôle supplémentaire accepte **402/402 poses natives à 0° et 60°**. Le budget
final utilise 10 533 inspections sur les 11 000 autorisées. Aucun de ces contrôles
n’intègre la physique ni ne démontre une charge portante.

Le plan figé comporte 63 fenêtres d’intention sur des triangles et points
barycentriques de surfaces existantes : pieds, avant-pieds, mains et avant-bras.
Il ne crée ni contact imposé, ni collider, ni contrainte. La référence conserve
la descente, le retournement, les appuis intermédiaires, la montée et le maintien.
Une intention de contact reste distincte d’un contact réellement résolu.

## Histoires physiques et cause observée

Le premier rejeu part une seule fois de la fixture exacte avec le profil épinglé.
Il échoue à **4,883333 s** : auto-collision `leftFoot/rightFoot` de **5,094402 mm**
pour le seuil original de **5 mm**. Bassin minimal 0,121891 m ; aucun relevage.
Les corps, articulations, colliders et la génération restent ceux du monde initial.
Le plafond total alloué est 760 Nm ; chaque plafond anatomique reste respecté
(ratio maximal 1). Le maximum de la somme des couples absolus délivrés est
680,057678 Nm ; cette somme n’est pas une mesure de travail mécanique.

Le rejeu indépendant est exactement identique sur les 1 171 sous-pas enregistrés :
valeurs numériques et empreintes des corps, moteurs et contacts. L’état final et
le refus au sous-pas 1 172 sont également identiques. L’inspection ayant interrompu
ce sous-pas avant l’observateur, son empreinte complète de contacts n’est pas
archivée. Aucun rejeu physique complet de 20,883 s n’a réussi.

Un témoin séparé emploie le contrôleur local ordinaire, avec le même profil et la
même fixture, pendant 26 s : aucune récupération, aucun refus géométrique. Il
inclut le correctif local de support décrit ci-dessus ; **ce n’est pas un témoin
intact de `main`**. Les reproductions antérieures du témoin sont conservées dans
les rapports précédents. Son champ `maximumAllocatedCapSumNm=0` signifie que le
hook expérimental est inactif, pas que les moteurs ordinaires ont un couple nul.

À 50 ms, les quatre points persistants de pied du premier rejeu ont une étendue
latérale de 270 mm et une profondeur avant/arrière de **0,08045 mm**. Le point de
capture est extérieur. À 75 ms, l’accélération avant du centre de masse est
−0,170733 m/s² pour la référence, contre +3,720962 m/s² pour le contrôleur local :
le freinage initial change, mais la stabilité durable n’est pas acquise.

Dans les captures enregistrées jusqu’à 550 ms, aucun point chargé du segment
`Foot` ne se situe sur sa région postérieure (`z` local < −20 mm, utilisé uniquement
pour ce diagnostic anatomique). Les charges `Foot` sont sur le bord avant ; elles
ne prouvent donc pas un appui de talon arrière. La capture est intérieure à 100 et
150 ms pour la référence, puis extérieure dès 200 ms. Ce recentrage est transitoire.
Les premiers appuis de main sélectionnés à au moins 3 N apparaissent à 1,291667 s
à gauche et 1,408333 s à droite ; la tête est chargée à 1,370833 s. Cette séquence
ne valide ni le pivot sur pieds seuls ni un transfert contrôlé par les mains.

Le bilan vertical des charges sélectionnées concorde bien entre 50 et 550 ms
(résidu RMS 0,133 N pour la référence, 0,073 N pour le témoin local). Il présente
cependant des écarts importants lors de certains impacts : maximum **6 460,934 N**
pour la référence, RMS 201,423 N sur l’histoire enregistrée. La cause de ce défaut
de bilan n’est pas isolée. Les charges sélectionnées ne sont pas présentées comme
un inventaire complet des impulsions aux impacts. Cette limite diagnostique
n’annule ni le refus géométrique ni l’absence de récupération.

## Campagne bornée et abandon

La famille est figée avant la première variation : 48 paramètres, soit six
durées de ralentissement monotones et six modes PCA (trois symétriques, trois
antisymétriques) sur sept clés temporelles. Les résidus s’annulent en position et
vitesse aux extrémités ; gains, allocations, critères et physique restent fixes.
Seed NumPy PCG64 : 0 ; propositions par génération : 16 ; élites : 4. Au plus trois
processus natifs, un thread chacun ; aucune compilation pendant les histoires.

Les quatre premières histoires comptent dans le même budget : référence,
contrôleur local, rejeu exact, variation. La limite générale était 384 histoires
ou quatre heures, avec **arrêt obligatoire à 96 sans retour debout après descente**.
Cet arrêt est atteint après environ 1 221 s murales : 95 programmes de référence
et un témoin local, tous intégrés. Parmi les programmes : 93 refus d’auto-contact,
deux refus de limite articulaire, zéro retour debout, zéro récupération complète.
Les durées physiques avant refus vont de 1,429167 à 4,929167 s.

Le candidat de score maximal, n° 13, échoue lui aussi à 4,920833 s par collision des
pieds de 5,048192 mm. Il n’est ni une solution ni un correctif à promouvoir.
Aucune des quatre validations finales n’est lancée, faute de succès à figer.
Le travailleur prépare ces validations, mais leur comportement après succès reste
non démontré. Une garde supplémentaire conserve hauteur, état Upright et compteur
pendant le repos ; elle est ajoutée après la campagne, sans nouvelle histoire.
Le fichier du travailleur effectivement utilisé est archivé séparément.

Les traces totalisent environ 162 MiB, sous la limite de 2 GiB. Les 96 paramètres,
rapports et journaux par sous-pas sont conservés ; les poses détaillées restent
pour les quatre premières histoires et le meilleur candidat. Aucun sous-jalon
ne renouvelle le budget. Aucun nouvel essai physique n’est autorisé par ce verdict.

## Vérifications et livrables

Les vérifications ciblées passent : **24 tests Rust** de mesures, prévision,
mapping, programme absolu, caps, projection des bras et support réel ; deux tests
Python de l’audit numérique. Les tests de rejeu commun portent sur des histoires
courtes indépendantes de la campagne, pas sur une réussite de relevage. Clippy
strict tous targets, formatage et `git diff --check` sont contrôlés pour le jalon.
La suite complète n’est pas déclarée verte : l’échec d’articulation hybride et les
quatre échecs TypeScript signalés sur le témoin ne sont pas résolus par ce travail.

[Résultats agrégés et empreintes publiables](recovery-physical-support-results-2026-10-11.json)
consigne le verdict sans inclure les programmes de mouvement. Les contrôles se
reproduisent avec `cargo test --offline --locked --release -p lh-sim --lib` et les
filtres listés dans `checkpoint-checks/focused-tests.json` ; ne pas ajouter
`--ignored` pour ces vérifications. `scripts/recovery-reference-audit-test.py`
vérifie l’audit sans charger de politique. Les scripts de préparation et de
recherche de cette campagne sont archivés localement avec leurs paramètres.
Le travailleur `run_whole_reference_native_history` exige des entrées locales
et une destination neuve ; la campagne close ne doit pas être relancée.

Les preuves locales sont dans `evidence/recovery-coordinated-retarget-2026-10-11/` :

- `reference-ready.json`, `contact-plan.json`, `continuous-command-certificate.json`,
  `geometry-certificate-final.json`, `ready-audit.json` : géométrie et commandes ;
- `native-campaign/search-spec.json`, `ledger.json`, `summary.json` : famille,
  comptage, arrêt définitif et résultats ;
- `native-campaign/repeat-equivalence.json`, `support-comparison.json`,
  `hindfoot-regions.json`, `force-balance.json` : comparaisons et limites ;
- `native-first-milestones.png` et `native-first-failed-replay.mp4` : poses physiques
  réellement enregistrées jusqu’au refus, **échec de relevage** ;
- `ready-geometry-milestones.png` : géométrie admissible, **aucune intégration** ;
- `native-support-comparison.png`, `initial-resolved-footprints.png` : charges,
  accélération, couples et points d’appui ;
- `checkpoint-checks/` : journaux des vérifications ciblées ; les essais rejetés,
  scripts et anciens certificats restent séparés dans le dossier de preuves.

Les images sont des rendus scientifiques des poses enregistrées, pas des captures
du navigateur. La vidéo ne consomme aucune nouvelle intégration. Le code tiers,
les poids, environnements et données de mouvement source/adaptées restent locaux :
aucune licence de redistribution trouvée dans le checkout officiel. Le dépôt
reçoit le code propre au projet, les fixtures natives de régression, l’outillage
et les rapports ; aucune archive volumineuse ni donnée de mouvement tierce.

Le résultat utile est la séparation entre semelle géométrique et support chargé.
Une séquence pieds seuls reste non démontrée. Un appui intermédiaire est une piste
physique, mais ces essais ne prouvent pas sa nécessité mathématique ni sa réussite.
Les blocages restants sont un pivot effectivement porteur, un recentrage durable,
un transfert de charge contrôlé et un diagnostic complet des impulsions d’impact.
