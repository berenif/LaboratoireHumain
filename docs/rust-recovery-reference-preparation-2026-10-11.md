# Référence entière extraite ; adaptation native rejetée

Travail local sur `investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`, conforme à la
[décision d’architecture](rust-recovery-architecture-decision-2026-10-11.md).
L’autorisation explicite du 11 octobre à 14:56 UTC couvre l’exécution ciblée des
chercheurs dans un environnement isolé pour extraire leur référence.
**Une référence source complète est obtenue. Aucune référence native admissible,
aucune campagne Rapier et aucun relevage physique de notre mannequin ne sont acquis.**

## Export borné et provenance

Source officielle : [Tao et al., Learning to Get Up](https://github.com/tianxintao/get_up_control),
commit `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`. Le checkout complet contient
les acteurs teacher/student, les modèles XML et leurs paramètres ; aucun fichier
numérique NPY/NPZ de mouvement ni fichier de licence n’y a été trouvé. Le code et
les poids restent dans le checkout privé et ne sont pas inclus dans le livrable.
Aucune publication ou redistribution de ces fichiers n’a lieu.

L’environnement séparé `/workspace/get-up-reference-venv` utilise Python 3.12,
Torch `2.6.0+cpu`, MuJoCo `3.2.7`, dm_control `1.0.27`. Il s’agit d’un port de
compatibilité des dépendances, **pas d’une reproduction exacte du moteur
historique Python 3.8 / MuJoCo 2.1.3**. Les dépendances étaient installées depuis
l’index CPU officiel PyTorch et PyPI. Aucun entraînement, critique, optimiseur,
buffer de rejeu ou interface graphique n’est lancé. Seuls les dictionnaires de
tenseurs des acteurs officiels sont chargés avec `weights_only=True` sur CPU.

Le wrapper conserve les politiques, le calcul des observations, les contrôles
et le modèle physique source. Il supprime les rendus et borne les essais teacher
à deux, chaque export à 600 s murales et 30 s physiques. Il enregistre chaque
sous-pas SLOW de 1,25 ms. Deux exports sont consommés, aucun autre n’est prévu.
La phase complète reste sous le budget d’une heure. Les seeds NumPy/Torch et du
modèle sont zéro ; une seule tentative teacher de 33 pas atteint le déclencheur
de hauteur source. La référence ralentie comporte 66 pas de contrôle, puis
80 pas de maintien source.

Le premier export avait un défaut d’observation : le chemin legacy de
dm_control exécute `mj_step2`, puis `mj_step1` reconstruit les contacts pour la
nouvelle position. Lire ensuite `mj_contactForce` associe parfois les forces de
l’ancien solveur à la nouvelle liste. Le second export capture les contacts et
forces **entre ces deux appels**, puis les poses/vitesses après `mj_step1`.
Les lignes de contact portent le temps de début de résolution, les états son
temps de fin. L’échantillon initial issu du reset ne prétend aucun appui résolu.

La correction ne change aucun résultat cinématique ou moteur : les 2 921
échantillons de temps, qpos/qvel, positions/quaternions de corps, CoM/vitesse et
forces/couples sont exactement identiques entre les deux exports. Le pic
incohérent de 41 297 N à la main droite devient un maximum résolu de 562,27 N.
La première observation rejetée reste uniquement dans les preuves.
Un contrôle indépendant compare la somme des appuis à `masse × (accélération
verticale du CoM + gravité)` : l’écart RMS passe de 1 254,06 N à 6,18 N.
C’est une cohérence numérique par différences finies dans le moteur source,
pas un nouveau seuil de validation ni une preuve native.

| Mesure de la référence source corrigée | Résultat |
|---|---:|
| Durée simulée, maintien inclus | 3,650 s |
| Échantillons à 1,25 ms | 2 921 |
| Corps, monde inclus / masse | 17 / 40,844 kg |
| Hauteur de tête initiale / finale | 0,1819 / 1,4522 m |
| Verticalité du torse finale | 0,99915 |
| Vitesse finale du CoM | 0,01067 m/s |
| Appuis finaux droit / gauche | 211,234 / 189,433 N |
| Couple généralisé maximal | 120 Nm |
| Vitesse articulaire réelle maximale | 15,559 rad/s |
| Accélération du CoM maximale | 52,608 m/s² |
| Lignes de contacts effectivement résolus | 9 569 |

La référence utilise mains, cuisses/tibias et parfois tête/bassin avant les pieds.
Les 115 intervalles d’appui à au moins 3 N sont conservés sans fusion. Cela décrit
la source ; ce seuil d’analyse n’accorde aucune qualification native. L’appui
source ne prouve ni la nécessité des mains chez notre mannequin ni
l’impossibilité d’un relevage avec les pieds seuls.

Le fichier numérique corrigé est
`evidence/recovery-trajectory-preparation-2026-10-11/reference/tao-slow-seed0.npz`,
SHA-256 `2644f160831c096524c20f533046677f7fcf61f66a631f4a417d04c5e29f2347`.
L’auditeur lit exclusivement des tableaux numériques avec `allow_pickle=False`,
contrôle les tailles, le checksum, la finitude, les horloges, quaternions et
index de contacts. Ses régressions refusent objets et archive corrompue.

## Une adaptation géométrique évaluée, puis rejetée

Un unique candidat utilise tout l’intervalle de référence, avec 41 échantillons
de la phase dynamique et un état terminal du maintien, donc une référence
**décimée**, pas chaque échantillon source conservé dans les cibles natives.
Une rotation propre transforme Z-haut en Y-haut ; la permutation explicite des
côtés source suit ce choix de repère. Un ajustement borné des rotations utilise
les véritables cadres articulaires et les limites natives, sans Euler ni
nouvelle articulation. Les segments absents de la source ont une rotation
parent-relative explicite. Une seule rotation rigide du pied source ne crée
pas de degré de liberté dans la cheville native.

Le candidat complet de 18 s inclut une transition géométrique depuis la fixture
exacte `landed-crouch-left`, puis le mouvement source et le maintien terminal.
La verticale du bassin vaut 0,9801 dans notre fixture contre 0,0993 au départ de
la référence déjà au sol. Cette différence nécessite une vraie descente ; une
interpolation de racine **ne la réalise pas physiquement**.

Les poses de racine servent uniquement à placer une géométrie pour inspection.
Elles ne figurent pas dans le programme de commande et ne sont jamais installées
dans une histoire intégrée. Le contrôle géométrique utilise des mondes qui ne
sont jamais avancés, conserve les corps dynamiques, les handles, les meshes,
colliders, exclusions, masses et seuils d’origine.

Résultat des 201 positions à 0° et 60° : **324/402 passent, 78 échouent**.
Parmi les premiers refus par position, 16 sont de sol et 62 d’auto-contact.
Le premier refus aux deux caps est à 0,450 s : avant-pied gauche, pénétration
0,107204 m, limite originale 0,080000 m. Le premier auto-contact observé est
à 1,171 s : torse/bras droit, 0,006505 m contre 0,005000 m.
Ces valeurs sont les observations avant le premier refus, pas des maxima
exhaustifs de toutes les paires d’une pose.

Le programme échoue également à la compilation native : vitesse de commande
à `leftForearm[0]` : **1,004542 rad/s contre la limite de 1 rad/s**. Les knots eux-mêmes ne suffisent
pas ; l’interpréteur inspecte toutes les cibles et vitesses interpolées sur la
grille native. Des points d’appui source correspondent aussi à une main native
à jusqu’à 0,270 m du sol. Le meilleur choix entre surfaces natives hindfoot et
forefoot laisse encore des écarts jusqu’à 0,188 m. Aucune contrainte artificielle
ne remplace ces appuis manquants.

L’ajustement garde un résidu d’orientation jusqu’à 2,166 rad ; les 3 012
évaluations de mapping/géométrie, puis deux fois 402 inspections natives (version finale et diagnostic initial), restent sous
10 000 et une heure. **Ce candidat est rejeté.** Son script et ses données sont
archivés sous `evidence/recovery-trajectory-preparation-2026-10-11/rejected-retarget/` ;
il n’est relié à aucun contrôleur actif. Il ne prouve pas que tout retargeting
possible échouerait, mais il ferme ce candidat et empêche la campagne de partir.

## Interpréteur conservé et vérifications distinctes

Les nouveaux modules sont exclusivement sous `cfg(test)` :

- `trajectory_program.rs` : schéma strict, 25 IDs ordonnés, commandes absolues
  `f32` q/dq provenant du même Hermite, clock 240 Hz, endpoints au repos et
  durée 8–18 s. Rejet de commandes de racine, axes absents, dépassements de
  limites, vitesse ou slew, y compris les intérieurs d’interpolation. Les
  contacts sont des annotations de triangles existants avec barycentriques,
  jamais de nouveaux colliders ou des appuis imposés.
- `reference_mapping.rs` : bases et offsets quaternions explicites, cadres
  natifs et convention `measurement::coordinates`. Contrôles indépendants des
  sources : rotations couplées, représentations antipodales, caps 0°/60°,
  source manquante, quaternion invalide et degré de liberté non modélisé.
- `support_probe/full_trajectory.rs` : un installateur commun à recherche et
  rejeu, utilisant l’allocation native existante. Aucun changement de corps,
  de racine ou de paramètres physiques. Il refuse une horloge différente.
  Son activation est locale au thread de test et absente par défaut.

Le test d’exécution compare deux mondes natifs sur 12 sous-pas, traverse une
frontière de keyframe avec une commande variable et compare tous les corps,
moteurs, données de contacts et mesures. Le test d’installation vérifie aussi
l’absence de mutation des corps. **Ces contrats courts ne valident pas un
relevage complet.** L’inspection géométrique est un test ignoré par défaut,
lancé explicitement ; son statut de test « ok » signifie que l’audit a terminé,
pas que les 402 positions sont acceptées.

Les 19 tests Rust ciblés passent : sept nouveaux contrats, cinq régressions
du prédicteur/horizon et sept de projection conservées. Les deux contrôles
de données numériques passent également. Le wrapper refuse maintenant un
troisième export avant tout import des chercheurs et requiert un répertoire
de sortie neuf ; le reçu de budget est déjà épuisé à deux exports. La suite complète n’est pas déclarée verte : le défaut
hybride d’articulation et les quatre tests TypeScript du témoin ne sont pas
présentés comme résolus. Clippy strict, formatage et `git diff --check` sont
consignés dans le registre de vérification.

Sur 724 fichiers protégés, 722 restent bit pour bit identiques. Les deux
modifications nécessaires sont les déclarations de modules et le hook de test
dans `recovery.rs` et `support_probe.rs`. Les snapshots précédents permettent
leur comparaison. `lib.rs`, `measurement.rs`, le prédicteur natif, ses fixtures,
les six fichiers de projection préservés et tous les autres correctifs protégés
sont conservés. Aucune correction provenant d’un autre environnement n’est
revendiquée.

## Blocage et suite dans le manifeste

Le prérequis restant est une adaptation complète qui respecte l’état initial,
les surfaces de contact et toutes les limites natives, avec une descente et une
remontée réalisables. Il faut traiter explicitement la transition initiale,
les pieds segmentés et les appuis des mains ; un ajustement d’orientations suivi
d’un placement vertical global est insuffisant.

La campagne reste à **zéro histoire** : aucun des budgets de 96/384 histoires
ou quatre heures n’est consommé. Les conditions debout, handoff, maintien et
quatre validations 0°/60° ne sont pas testées comme réussite. Aucun seuil
n’est relâché ; aucune qualification des 24 fixtures/cycles/WASM/navigateur
n’est acquise. Récupération désactivée, aucun push, merge ou publication.

Les médias sont des schémas tirés de poses enregistrées :
`reference/source-milestones.png`, `reference/source-support.png`,
`reference/source-recorded-poses.mp4` et
`rejected-retarget/retarget-geometry-milestones.png`. Ce sont des rendus de données,
**pas des captures du moteur ni une vidéo de réussite Rapier**. La vidéo source
compte 74 images à 20 i/s, 3,70 s de lecture, et inclut exactement le dernier
état physique à 3,65 s ; cette dernière image a été décodée et inspectée.
