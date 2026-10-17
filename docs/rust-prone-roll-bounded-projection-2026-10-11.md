# Projection bornée et suivi physique de la chaîne droite

La cible impossible issue du clamp est corrigée : sur l’état exact fautif à
8,608 s, la nouvelle projection respecte simultanément les bornes articulaires,
les 9 mm de hauteur demandée et les collisions de toute la chaîne droite.
Le rejeu physique libère effectivement la ceinture scapulaire, mais le haut du
bras redevient porteur malgré une référence admissible. **Le relevage et le jalon
latéral restent non acquis.** La correction géométrique et sa régression sont
conservées dans le code de test ; le contrôleur de mouvement refusé est archivé.

## Dépôt et périmètre

HEAD `c597959f46f37715035349c8cd0a699b6822111a`, conforme à la référence fournie,
branche locale `investigate/physical-support`. Aucun push, merge, déploiement ou
publication. Le suivi `origin/main` n'est pas disponible dans cet environnement ;
ce travail ne prétend pas vérifier un nouvel état distant. Les modifications
antérieures de cette branche sont préservées. Aucune correction locale du
comptage du décalage des pieds venant de l'autre environnement n'est revendiquée.

Les histoires natives repartent de C et traversent chute, repos ventral et
préparation de paume gauche dans le même monde. Aucun reset ni déplacement de
racine pendant ces histoires, aucune force ou couple direct au bassin. La fixture
de régression restaure les poses et vitesses enregistrées pour un contrôle
**géométrique sans intégration** ; elle n'est pas une histoire physique.

## Correction et preuve géométrique

L'ancien solveur calcule un pas sans les bornes, puis coupe les coordonnées.
Lorsque la ceinture atteint sa borne, cette coupe annule une partie du mouvement
qui devait dégager le bras. Il renvoie néanmoins la référence avec un résidu.
La nouvelle résolution utilise un sous-problème de moindres carrés avec bornes
actives et conditions KKT, puis vérifie chaque pas dans la géométrie non linéaire.
Les variables sont seulement les coordonnées anatomiques des cinq segments ;
le torse et les autres corps restent leurs poses mesurées.

Le certificat porte sur tous les sommets de la chaîne, les skins natives,
les paires de collision non exclues et la séparation conservatrice torse–bras.
La hauteur de main de 12 cm déjà exigée par la tâche reste une contrainte.
La préparation interpole d'abord sa demande, puis certifie cette commande
réelle ; elle n'interpole pas après certification une extrémité admissible.
La clearance supplémentaire suit la préparation de 0 à 5 mm sur 0,8 s ; les
4 mm de skins restent obligatoires, et la demande finale reste 9 mm. Une marge
numérique supplémentaire de 10 µm protège le certificat ; aucun seuil physique
n'est abaissé.

La recherche est bornée à 32 itérations par départ, 64 pivots par sous-problème
et 10 réductions de pas. Les départs déterministes sont la référence nominale,
le cache vérifié, puis les huit coins de la seule ceinture. Un échec rend
`not-converged`, avec `commandCommitted=false` et `globalImpossibilityProven=false`.
Aucune cible partiellement corrigée n'est copiée vers les moteurs. Une requête
collision non supportée est aussi un refus explicite. Ce solveur local ne
certifie pas une impossibilité globale.

Sur le pas exact 2066 :

| Référence au torse exact enregistré | Ceinture droite, minimum Y | Haut du bras droit, minimum Y |
| --- | ---: | ---: |
| Ancienne cible | −0,474 mm | −5,588 mm |
| Nouvelle cible certifiée | 9,010 mm | 9,010 mm |
| Demande conservée | 9,000 mm | 9,000 mm |

Le test vérifie 282 contraintes, toutes les bornes, l'absence de modification des
corps/vitesses/cibles moteur et le refus sans copie d'une demande non résolue.
Une référence nominale déjà certifiée garde la priorité sur un cache différent.
Le test de sous-problème vérifie que le mouvement est redistribué à une autre
coordonnée lorsqu'une borne est active ; les Jacobiennes de toute la chaîne
sont aussi vérifiées par différences finies. La solution SciPy indépendante
`regression-geometry.json` ne sert que de diagnostic sol seul ; elle n'est pas
une cible codée en dur ni la preuve de réussite physique.

![Ancienne cible et projection certifiée, géométrie uniquement](../evidence/prone-roll-bounded-projection-2026-10-11/projection-regression.png)

## Comparaison native et essai rejeté

Le témoin utilise le binaire précédent exact
`f69f9484d68a02d41af9e1e3450d5419e933c3c2d0df3d7ea54f6b5b8d05950c`.
Ses 2 066 pas sont reproduits, avec états et commandes identiques hors mesure
`projectionWallS`. Le candidat final partage les **1 468 pas de C et préparation
 gauche**, puis reçoit la correction de projection.

| Histoire native | Arrêt réel | Roulis bassin / torse | Contact droit final | Jalon 75° stable |
| --- | ---: | ---: | --- | --- |
| Témoin ancien projecteur | 8,608 s | 42,24° / 51,80° | Ceinture 256,25 N | Non |
| Cache prioritaire, rejeté | 7,975 s | 25,95° / 29,02° | Haut du bras 54,27 N | Non |
| Nominal prioritaire, final | 8,629 s | 42,19° / 51,71° | Haut du bras 35,85 N ; ceinture 0 N | Non |

Le cache prioritaire du premier candidat pouvait accepter une ancienne solution
admissible sans essayer la référence courante interpolée. Ce défaut de priorité
introduit a été isolé et corrigé avant un unique rejeu complémentaire de la même
phase. Il ne s'agit ni d'une répétition réussie ni d'un transfert suivant. Ses
sources, binaire, traces et médias sont conservés séparément comme essai rejeté.
Aucun réglage de pression, gain, plafond, frottement ou seuil ne distingue ces
comparaisons.

Au temps commun **8,608 s**, la correction finale donne 42,02° / 51,35°,
ceinture 0 N, haut du bras 47,25 N, torse 298,40 N et paume gauche 50,72 N.
Le témoin donne 42,24° / 51,80°, ceinture 256,25 N, haut du bras 0 N,
torse 68,40 N et paume gauche 66,37 N. Le support se redistribue réellement.
L'accélération COM XYZ vaut respectivement
`[−0,0730, −0,0674, +0,0917]` et `[−0,0948, −0,0445, −0,0719] m/s²`.
Le moment cinétique Z est `−0,0950` contre `−0,7192 kg·m²/s`, et l'énergie
cinétique `0,1695` contre `1,1365 J`. La projection sûre ne conserve donc pas
le même mouvement global ; enlever un contact ne suffit pas à imposer sa
redistribution dynamique.

## Blocage physique restant

Les **603 commandes** de roulis finales portent un certificat valide au torse
mesuré avant le pas. Leur résidu maximal est inférieur à 0,745 µm, couvert par
la marge supplémentaire de 10 µm. Au dernier pas, les références du haut du bras
et de la ceinture ont leur minimum à 9,010 mm. Recalculées au torse exact après
ce pas, elles sont à 8,859 et 8,942 mm ; les minima physiques sont 3,713 et
7,532 mm. L'écart d'environ **5,15 mm au haut du bras** vient principalement du
suivi articulaire, et non d'une référence sous le sol.

La ceinture Z demandée est 20°, contre 17,39° réel. L'épaule X est demandée à
−35°, contre −32,61° réel ; l'erreur Y est environ −4,82°. La ceinture Y reste
à sa borne anatomique de 15°. Les couples droits ne prouvent pas une saturation
anatomique : les ratios maximaux à leurs caps alloués sont 0,657 à la ceinture et
0,417 à l'épaule. La ceinture gauche atteint 0,992 de son cap, notamment sur
l'axe Y plafonné à 28 Nm. Aucun plafond n'est relevé.

Toute la chaîne droite est physiquement libre au maximum **2,038 s**. La
ceinture reprend transitoirement jusqu’à **307,20 N à 8,200 s**, sur un épisode
de charge réelle d’au plus 0,083 s au-dessus de 3 N ; elle se dégage ensuite et
le haut du bras devient un support. Le pic cinétique est 3,211 J à 8,192 s. À 8,629 s, ceinture/avant-bras/twist/main sont
à 0 N, le bras supérieur porte 35,85 N. Tête et nuque sont à 0 N, déchargement
stable verrouillé depuis 7,988 s, sans recharge après ce verrouillage. COM et
capture restent dans le support réel ; les 25 corps ne sont pas tous stables.
Le maximum de persistance du jalon final reste **zéro**.

Le bras droit produit un moment normal Z de +7,05 Nm, opposé à celui de la paume
gauche −35,68 Nm ; le torse et les cuisses participent aussi au bilan. Les moments
normaux sont calculés sur les points effectivement résolus. La contribution de
frottement estimée au centroïde des forces normales est explicitement une
estimation, pas un couple exact par point.

L'arrêt final est la garde originale de **recontact 0,20 s du mode déclaré
chaîne dégagée**. Il n'est ni un échec au terme de l'horizon de 16 s ni une
stagnation : un progrès réel d'au moins 1° a encore été enregistré à 8,546 s.
Le témoin de 8,608 s était lui aussi en mouvement. Une borne de temps atteinte
sans qualification serait rapportée séparément comme horizon expérimental ;
aucun seuil final ne serait assoupli pour prolonger.

## Pivot chargé et transfert à vérifier

Le dégagement temporaire est physiquement observé ; une impossibilité générale
de libérer la chaîne n'est pas démontrée. Le problème ouvert est de conserver
ce dégagement sous le suivi moteur et les contacts. Une correction suivante
doit prévoir l'accélération normale des sommets réels et les efforts délivrés,
ou déclarer explicitement un pivot pendant le transfert.

Un **pivot huméral chargé est déjà réalisé physiquement** au dernier état :
bras supérieur droit porteur, chaîne distale dégagée, torse, paume gauche et
cuisses porteurs, tête/nuque déchargées. Le témoin réalise également un pivot
scapulaire à 256 N. Cela ne prouve pas leur stabilité finale ni la faisabilité
du transfert complet.

Pour un mode de pivot cohérent, conserver le contact réellement chargé et ses
points natifs dans les contraintes de support, plutôt que lui demander en même
temps 9 mm de dégagement. Les autres liens doivent rester non pénétrants et
libres. Transférer ensuite cette charge vers torse et cuisses, avec la paume
gauche, au moyen des couples articulaires existants : contraintes de frottement,
COM/capture dans le polygone réellement chargé, limites de couples délivrés et
tête/nuque déchargées. Le moment positif du pivot doit entrer dans le bilan ;
supprimer la garde actuelle sans traiter ce moment n'est pas un correctif.
Le mode de dégagement complet ne peut commencer qu'après déchargement réel du
pivot avec la persistance originale. Le jalon 75–105° et sa stabilité de 0,55 s
restent ensuite requis, puis sa répétition avant la phase suivante. Ce transfert
est **défini mais non exécuté ni validé** ici ; aucune cible impossible ne lui
est substituée. Aucun nouvel appui de genou ni trajet pieds seuls n'est qualifié.
La paume gauche intermédiaire est effectivement utilisée dans ces histoires.

## Budget, code conservé et vérifications

Trois histoires au total : **25,2125 s physiques / 74,150 s murales natives**,
sous le budget maximal 68 s / 480 s, avec plafonds de 120 s par appel.
Aucune quatrième histoire, répétition ou phase suivante après les refus.
Le plan et son amendement causal sont consignés avant le troisième appel.

Le binaire final exact est
`2d927527c1b9da070d01359b8e5dca2d7c0cfb80c92edcc9f90e6e5b059c312b`.
`source-delta/` contient ses cinq fichiers à appliquer sur la base précédente.
`source-final/` contient seulement la correction pure conservée et sa fixture
exacte. Les trois autres fichiers expérimentaux du début de session sont remis
à l'identique ; `prone_roll_experiment.rs` est retiré du code actif. Le code de
projection conservé est dans un module de tests : récupération de production
toujours désactivée et non qualifiée.

Sept tests ciblés passent sur le candidat release, six essais explicites sont
ignorés. Après retrait du contrôleur, sept tests ciblés passent, cinq sont
ignorés ; Clippy strict, format et `git diff --check` passent. Le `ok` du test
ignoré exécuté pour produire les traces ne compte pas comme réussite physique.
**Aucune suite complète verte annoncée** : le test d'articulation hybride et
quatre tests TypeScript préexistants ne sont pas corrigés par ce travail.

## Livrables locaux et livraison

Preuves : `evidence/prone-roll-bounded-projection-2026-10-11/`, notamment
`PLAN.md`, `native-ledger.json`, `reference-replay.json`, `regression-certificate.json`,
`command-certification-audit.json`, `analysis.json`, les deux nouvelles histoires,
les binaires exacts, sources différentielles et logs de vérification.

![Comparaison native des commandes et contacts](../evidence/prone-roll-bounded-projection-2026-10-11/comparison.png)

![Étapes natives enregistrées](../evidence/prone-roll-bounded-projection-2026-10-11/milestones.png)

La vidéo locale `native.mp4` couvre le candidat final entier, poses natives à
20 images/s sans interpolation ; `projection-regression.png` demeure séparée
et explicitement géométrique. Les médias du cache rejeté sont conservés dans
`rejected-case-02-media/`. Les fichiers et médias des sessions précédentes sont
intacts.

Le complément réutilise les deux archives précédentes et leurs intrants :
`LaboratoireHumain-prone-roll-2026-10-11.zip` et
`LaboratoireHumain-prone-roll-arm-release-2026-10-11.zip`. Il ne duplique pas
le dépôt, les intrants historiques ou le binaire témoin. Les dépendances,
contenu et sommes de contrôle sont consignés dans le paquet.

**Library : aucun nouvel envoi ni nouvel identifiant.** Le blocage officiel connu
reste `library upload failed: hosted apps tools/list request failed: network`.
Conformément à l'instruction reçue, aucune tentative identique n'est répétée
tant que ce transport reste inchangé ; les livrables restent disponibles localement.
