# Transfert de charge après deux appuis des mains — essai ciblé refusé

Date : 2026-10-11. Branche locale `investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou déploiement.
Récupération de production désactivée et non qualifiée.

Le candidat corrige une mauvaise initialisation de l’IK et stabilise les appuis,
mais **ne décharge pas la tête**. Deux histoires physiques complètes seulement,
aucune répétition après refus. Les trois fichiers du banc modifiés ont été remis
à leur version précédente ; le nouvel observateur a été retiré du code actif.
Le delta exact du candidat et son binaire restent dans les preuves.

## Comparaison physique

Les deux cas repartent de la chute C dans le même monde, sans reset, assistance
bassin ou modification des contacts, de la friction, des gains, des plafonds
anatomiques et des critères. Ils partagent exactement **2 448 pas / 10,2 s** de
positions, orientations et vitesses des 25 corps. Le premier pas différent est
2449, au début de la montée. La référence compacte reproduit aussi les **3 766
pas** de l’ancien cas02 : états f32, contacts des mains et jalons identiques.

| Mesure | Référence | Initialisation IK mesurée |
|---|---:|---:|
| Arrêt physique | 15,691667 s | 15,875000 s |
| Cause | stagnation originale 3 s | stagnation originale 3 s |
| Charge tête à l’arrêt | 226,483 N | 254,711 N |
| Charge torse à l’arrêt | 0 N | 0 N |
| Mains G / D à l’arrêt | 72,093 / 61,087 N | 14,874 / 13,239 N |
| Deux mains, actuelles + persistantes, durée maximale | 2,708 s | 4,425 s |
| COM/capture dans le hull des membres + 25 corps immobiles, durée maximale | 0,204 s | 3,333 s |
| Charge des membres à l’arrêt | 463,685 N | 448,415 N |
| Gain vertical du centre du torse | 19,787 mm | 18,461 mm |
| Déchargement tête/nuque/torse qualifié | 0 s | 0 s |
| Brace complet qualifié | 0 s | 0 s |

Les arrêts diffèrent : à **temps égal 15,691667 s**, le candidat porte encore
**261,945 N sur la tête**, mains 14,598 / 13,172 N. Son résultat négatif ne dépend
donc pas du seul choix de l’instant final. La tête reste au-dessus de 105 N
pendant toute sa phase de montée. Les membres ne reprennent qu’environ 63,3 % du
poids à l’arrêt, sous le critère original de 80 % ; le torse reste ventral
(`UpY = −0,375`), loin du brace original.

![Comparaison des états intégrés et charges résolues](../evidence/head-load-transfer-2026-10-11/comparison.png)

[Courte vidéo native à temps égal](../evidence/head-load-transfer-2026-10-11/comparison.mp4) :
extrait après chute et préparation, de 9,95 à 15,65 s, vitesse ×1, 20 images/s.
Les poses sont des pas réellement intégrés, sans interpolation ni nouvelle
simulation. La figure montre séparément les deux arrêts effectifs.

## Résultat causal

**1. Une erreur de convergence avait été confondue avec une impossibilité de
portée.** L’IK des mains repartait à chaque pas des angles du plan nominal,
construit avec une racine virtuelle déjà relevée. Or le bassin réel n’exécute
pas cette translation. Sur la même pose finale enregistrée de référence, les
12 itérations donnent des erreurs de 318 / 378 mm depuis le nominal, contre
1,45 / 14,94 mm depuis les articulations mesurées. Ce résultat corrige la portée
de l’interprétation du rapport précédent : une impossibilité géométrique globale
n’était pas établie. Ces projections restent des diagnostics géométriques ;
leurs séparations de skins 7,89 / 6,42 mm ne valident pas la clearance complète
8 mm ni une trajectoire physique. Quatre projections, 30 itérations au total.

Le seul candidat natif change cette initialisation à partir de la phase6.
Cibles, solveur, 12 itérations maximales, amortissement, limitation d’incrément,
limites articulaires et compensation gravitaire restent identiques. À l’arrêt
candidat, les erreurs projetées sont 1,674 / 0,002 mm, mais cela ne prouve pas
que les mains réelles atteignent exactement les cibles ni portent le poids.

**2. Des mains persistantes ne constituent pas à elles seules un appui utile.**
Dans la référence finale, le COM est 123,75 mm devant tout contact de membre
chargé. Les mains reculent, glissent à 0,182 / 0,337 m/s et leurs moments autour
du COM suivant X sont **+5,88 / +7,91 Nm**, dans le sens qui abaisse l’avant.
Une borne de bilan statique coplanaire impose encore environ 130 N à la tête
avec ces points d’appui. C’est une borne conditionnelle sur cette géométrie,
non une preuve d’impossibilité pour d’autres poses ou un mouvement dynamique.

Le candidat maintient les contacts des mains devant le COM de 176 / 180 mm et
réduit le glissement à 0,00544 / 0,000002 m/s. Pourtant les mains ne portent que
28,11 N. Leurs efforts tangentiels vers l’arrière (−16,52 / −12,33 N suivant Z)
compensent presque le moment de leurs forces verticales : moments X **+0,539 /
−0,017 Nm**, total +0,522 Nm. Le corps trouve donc un repos encore soutenu par la
tête. Le hull est revenu au bon endroit, mais la répartition des efforts et
l’orientation du tronc ne réalisent pas le transfert.

**3. La trajectoire mondiale prévue reste une intention non commandée.** Le
plan contient une montée du bassin de 190,76 mm et un recul de 24,75 mm, avec
rotation mondiale. `RiseRow` ne consomme que progression et coordonnées
articulaires : la translation et la rotation mondiales ne sont pas une tâche
exécutée. Le test précédent par réactions de hanche a déjà été refusé ; il n’a
pas été réintroduit ici. Ajouter un déplacement géométrique de racine ne serait
pas une réalisation physique. Il manque une commande des efforts et des modes
de contact du corps entier compatible avec les couples disponibles.

L’accélération RMS du COM pendant la montée baisse, suivant X/Y/Z, de
0,386 / 0,664 / 0,476 à 0,147 / 0,195 / 0,159 m/s² (durées de phase respectives,
non fenêtres identiques). Le maximum du rapport couple réellement résolu / cap
installé reste inférieur à 1 dans les deux histoires ; le rapport de commande
au plafond anatomique atteint 1, sans dépassement. Les plafonds installés varient
avec la commande selon le mécanisme existant : 760 / 562,38 Nm de somme finale,
ce n’est pas un changement des plafonds anatomiques. Tous les efforts et couples
directs au bassin restent nuls. Les gardes originales n’ont pas signalé de
violation d’intégrité ; maxima enregistrés : sol16,37 mm, limite0,00802 rad,
auto-pénétration0. Les contacts et couples détaillés sont conservés aux images
natives et aux deux points de contrôle complets.

## Comparaison avec des mouvements publiés

Les pages et articles ont été consultés. Les tentatives de récupération directe
de vidéos/images depuis cet environnement ont reçu HTTP403 du proxy. **Aucune
vidéo ni image de mouvement externe n’a été visionnée ou décodée ici.** Le schéma
ci-dessous transcrit des descriptions écrites ; ce ne sont pas des observations
image par image. Aucun code tiers exécuté, aucun dataset téléchargé.

![Schéma explicitement tiré des descriptions textuelles](../evidence/head-load-transfer-2026-10-11/reference-phases.png)

- **HumanUP** décompose explicitement le départ ventral en retournement vers le
  dos, puis relevage depuis le dos. Ce n’est pas une validation de notre poussée
  ventrale bilatérale. La symétrie est souple, et la morphologie/motorisation
  diffère. Le texte ne suffit pas à reconstruire les charges de chaque contact
  à chaque phase. [Article, §III](https://arxiv.org/html/2502.12152v1),
  [page du projet](https://humanoid-getup.github.io/).
- **Learning to Get Up** décrit plusieurs stratégies ; certaines changent
  d’abord d’orientation selon la posture préférée. Les auteurs distinguent aussi
  les poses où une pause est possible des phases dynamiques qui ne la supportent
  pas. Cela motive l’examen de plusieurs modes d’appui, sans autoriser une
  relaxation de nos critères. [Article, §8.2–8.3](https://arxiv.org/html/2205.00307v2),
  [vidéo officielle non visionnée](https://tianxintao.github.io/get_up_control/figs/video.mp4).
- **Physitrack** décrit : assis → déplacement latéral → mains et genoux → un pied
  à plat sous la poitrine → mains sur les cuisses → debout. Le transfert des
  mains du sol aux cuisses est une étape distincte ; le départ assis diffère de
  notre départ ventral. [Description primaire](https://na.physitrack.com/home-exercise-video/getting-up-from-the-floor-variation-without-support).

**Décision :** suspendre cette montée ventrale à mains fixes. Une route latérale
ou un retournement est une hypothèse structurée par les références, mais sa
nécessité n’est pas prouvée et aucune nouvelle stratégie n’a été exécutée.
Avant un prochain essai, observer réellement une séquence de référence et poser
un transfert d’appui/efforts vérifiable sous nos limites. La route pieds seuls
reste non validée ; ces essais ne démontrent pas son impossibilité générale.
Les contacts intermédiaires main/jambe sont physiques, mais aucun support
intermédiaire sans tête n’est encore acquis. Aucun pied/genou avancé, relevage
complet ou répétition réussie n’est revendiqué.

## Budget, vérifications et reproduction

Deux histoires, **31,566667 s physiques / 111,342677 s murales natives**, sur le
budget maximal préannoncé de trois cas,18 s physiques/120 s murales par cas,
54/360 s cumulées. La troisième histoire était réservée à un déchargement
stable à répéter : condition absente, donc non lancée. Zéro programme linéaire
supplémentaire, aucune recherche de gains. Compilation, lecture de traces et
rendu hors budget natif. Le résumé hérité affiche encore `wallBudgetS:150` ;
les gardes d’exécution et le timeout externe de cette comparaison sont120 s.

Le candidat et les sources restaurées passent les quatre tests unitaires ciblés
`hand_support_experiment`, Clippy strict et le formatage. Les cinq essais ignorés
ne sont pas des tests passés. Leurs deux intégrations explicites sont les seules
preuves physiques de ce suivi. **Suite complète non déclarée verte** : le test
d’articulation hybride et les quatre échecs TypeScript antérieurement signalés
sur le témoin Linux ne sont pas résolus par ce travail.

Dossier de preuves : `evidence/head-load-transfer-2026-10-11/`.

- `comparison.json`, `recorded-force-decision.json`, `measured-seed-prescreen.json` :
  mesures et portée des diagnostics ; `analyze.py` les exploite sans simulation.
- `case-01/case.json`, `case-02/case.json` : métriques à chaque pas, contacts des
  mains, hash f32 des25corps, états complets20fps et points de contrôle.
- `reference-replay-check.json` : reproduction exacte de l’ancien cas02.
- `PLAN.md`, `native-ledger.json`, `run_case.py` : budget, comptage et invocation
  déterministe. Ne pas relancer dans les dossiers existants : le script refuse.
- `source-delta/`, `source-manifest.json`, `comparison-binary` : quatre fichiers
  du banc et binaire exact SHA256
  `85764fc91973520f0a05dd708f015dbc306661dfdd5905a9092bed179b4cc456`.
  Restaurer les fichiers par leur nom dans
  `rust/crates/sim/src/recovery/support_probe/` sur la base historique indiquée
  dans le manifeste ; ne pas appliquer ce candidat au contrôleur de production.
- Base réutilisée, sans copie nouvelle des288fichiers : archive Library
  `libfile_28c3e675dd78819193626fa3492e60c2`, SHA256
  `c1264597fda226fb2caba036c63a46630756d8a246e957fa85618c7bb5bb75cc`.
  Elle contient `source-final` et les intrants dont le manifeste conserve les
  empreintes. Le petit complément contient également ces deux intrants.
- `active-source-restored.json` atteste la restauration des trois fichiers et
  le retrait de l’observateur. Les sources/binaire historiques restent intacts.
- `build-units.log`, `clippy.log`, `format.log`, `restored-units.log`,
  `restored-clippy.log`, `restored-format.log`, `diff-check.log` : contrôles.

Les archives antérieures ne sont pas remplacées. Le complément de preuves
contient uniquement ce suivi et ses médias, sans copie massive du dépôt.
