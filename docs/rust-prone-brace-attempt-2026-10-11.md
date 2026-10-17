# C → repos ventral → préparation des bras : premier refus natif

**Suivi causal du même témoin :** l’[audit de commande et la comparaison
appariée](rust-left-hand-command-audit-2026-10-11.md) confirment les repères
et révèlent un contact réel torse–bras participant à l’équilibre. Une seule
correction gravitaire articulaire reprend la levée : 99,95 → 24,35 mm, sous
le seuil inchangé de 50 mm. La tête reste chargée ; aucun brace/relevage
n’est qualifié. L’essai, les coûts et les archives ci-dessous restent le
témoin historique refusé. Seul un diagnostic de première levée est désormais
conservé dans le banc vivant, sans les phases suivantes rejetées/non vérifiées.

Le candidat ne sort pas du repos ventral. Dans le **même monde**, après
reproduction exacte des 484 pas de C, la première levée de main atteint un
équilibre à environ **100 mm de sa cible**. La garde de stagnation de 3 s
arrête l’essai à **5,283333 s**. Tête et torse restent chargés. Aucun passage
latéral, appui préparé ou mouvement de montée du torse n’est commandé.
Un seul cas est exécuté ; aucune répétition, transition suivante ou campagne
de réglages. La récupération reste désactivée par défaut et non qualifiée.

Le dépôt reste sur la branche locale `investigate/physical-support`, à
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou publication.
Le prototype rejeté a été retiré du code vivant ; le banc préexistant est
restauré octet pour octet. Le prototype compilé, ses 285 sources et son
binaire sont conservés dans les preuves.

## Ce qui a été réellement exécuté

Le banc reconstruit uniquement le départ original C, rejoue sa commande
inchangée jusqu’au repos mesuré à 2,016667 s, puis continue le même assemblage
avec des cibles moteur articulaires. Il ne recharge jamais le snapshot de
repos, ne clone aucun monde vivant, ne remet à zéro ni poses, ni vitesses,
ni horloge, et n’exerce aucune assistance au bassin. Les handles de corps,
articulations et colliders ainsi que la génération sont contrôlés à chaque
pas. Le préfixe de 484 pas correspond exactement au témoin pour les poses,
vitesses, contacts, impulsions, commandes, accélération du centre de masse et
compteurs ; le premier refus expérimental C sur cuisse reste enregistré.

Le programme prévu préparait les mains une à une : levée, passage extérieur,
descente sur une surface de main ; puis poussée coordonnée jambes/bras,
rachis dans son extension capturée, vers un brace. Le transfert genoux/pied
avancé ne devait être exécuté qu’après réussite et répétition fraîche du
brace. **Seule la première levée gauche est effectivement exécutée.**
Les cibles non liées aux bras de C sont conservées pendant la préparation
pour maintenir les efforts de repos existants ; leur biais ne devait être
fondu que pendant la montée. Gains, solveur, anatomie, caps et contrats
physiques restent ceux du témoin.

| Mesure native | Repos à 2,017 s | Arrêt à 5,283 s |
|---|---:|---:|
| Hauteur du centre de main gauche | 27,35 mm | 160,84 mm |
| Charge réelle tête | 109,88 N | 138,56 N |
| Charge réelle torse | 254,45 N | 218,73 N |
| Charge réelle main gauche | 9,49 N | 0 N |
| Charge réelle main droite | 10,11 N | 10,71 N |
| Charge réelle cuisses gauche/droite | 157,36 / 163,05 N | 217,32 / 123,25 N |
| Torso UpY | −0,1548 | −0,1655 |
| Gain de hauteur du torse depuis l’entrée | 0 | 0,52 mm |
| Accélération COM avant +Z | −0,0771 m/s² | −0,00161 m/s² |

La cible de main était à 257,35 mm de hauteur. La main se lève puis s’affaisse
vers 160,84 mm ; le résidu spatial final est 99,939 mm. Le seuil de suivi de
**50 mm est un critère ajouté à cette tâche du banc**, pas une nouvelle
limite physique ou un critère de qualification debout. La durée de
stagnation reste celle du contrat original. Les 25 corps sont sous les
vitesses de stabilité à l’arrêt, mais les deux mains ne portent pas, le
COM/capture ne sont pas dans le polygone des membres et tête/torse ne sont
pas déchargés : aucun temps de brace qualifié n’est acquis.

## Premier mécanisme utile

Au dernier pas, le moteur X de la ceinture scapulaire gauche livre environ
12,77 Nm pour 19,16 Nm installés, avec un écart de 0,0670 rad (3,84°).
Le moteur X du bras supérieur livre 12,51 Nm pour 24,06 Nm installés,
avec un écart de −0,04215 rad (−2,41°). L’utilisation maximale finale des
caps installés du bras gauche est **66,64 %**. Les caps anatomiques et le
budget global ne sont donc pas saturés à cet équilibre.

Les écarts de ressort articulaires correspondent au déficit spatial : un
calcul sur les données enregistrées, remplaçant algébriquement les seuls
angles scapulaire et huméral par leurs cibles, réduit le résidu de 99,94 à
22,81 mm. Ce calcul n’est jamais envoyé au moteur physique : c’est une
attribution cinématique, pas une intervention physique ni une réussite.
Ce constat suggérait un suivi avec déflexion sous charge, mais ne démontrait
pas sa cause : les contacts entre corps et les paramètres moteurs réellement
installés n’étaient pas encore audités. La garde terminale est la stagnation,
sans violation anatomique ; cela n’exclut pas un contact participant au
blocage. **Aucun correctif n’est essayé dans ce témoin historique.** Le
suivi lié ci-dessus apporte ensuite le bilan gravité + moteur + contact
et la comparaison physique courte ; il ne change aucun résultat de cet essai.

Ce refus ne prouve ni l’impossibilité d’une préparation de main, ni la
nécessité universelle d’un appui supplémentaire. Le déchargement de tête,
le brace mains/genoux, le pied avancé et la station debout restent ouverts.
Un appui transitoire cuisse/torse n’est pas traité comme une faute générale
de récupération ; la tête est observée sans autoriser la progression.

## Présélections et coûts — distincts du résultat physique

La préparation géométrique a consommé **1 302 évaluations de résidu en
90,176 s**. Le budget de 90 s est dépassé de 0,176 s dans le dernier callback ;
ce dépassement n’est pas masqué. Aucun appel d’optimisation supplémentaire.
Les refus successifs sont conservés avec leurs scripts : orientation
absolue de main inutile, retour intérieur non convergé, chevauchement
bras/torse de 12,3 mm, puis réparation collision dont les critères
numériques ajoutés restent refusés (normale 0,069 rad, chevauchement
0,18 mm). Ce dernier état respecte toutefois les vraies gardes physiques.

Les constructions analytiques rejetées sont également conservées : bassin
haut épuisant la portée, puis inversion de hauteur non encadrée produisant
un rachis hors limites. La correction finale conserve le rachis atteint et
déduit le pitch de bassin du seuil UpY 0,40. Le terminal géométrique proposé
a un bassin à 0,4055 m et un torse relevé de 0,4132 m ; **il n’a jamais été
atteint physiquement**. Les 201 positions contrôlées en 2,80 s respectent
les gardes originales. Les axes de face donnent une borne conservatrice
de pénétration ; les axes croisés SAT sont évalués lorsque cette borne ne
suffit pas à certifier la garde de 5 mm. Aucun seuil n’est relâché.

Deux LP statiques stricts passent : préparation 61,01 Nm cumulés ; terminal
315,26 Nm et utilisation anatomique maximale 0,4967. Contacts, friction,
caps anatomiques et plafond 760 Nm sont inchangés. Le LP terminal propose
427,31 N sur les mains et 280,98 N sur les tibias. Ce sont des forces
**prospectives**, jamais des contacts résolus natifs ou une réussite.
Aucun LP relâché n’est exécuté.

L’unique essai natif consomme **5,283333 s physiques et 26,422 s de calcul**,
sous les budgets déclarés 15 s par cas et 180 s cumulés. La compilation
optimisée réussie coûte 62,994 s, hors budget natif ; deux erreurs de
compilation préparatoires restent archivées. Aucun budget de répétition
n’est consommé après le refus.

## Vérifications et livrables

- Préfixe témoin exact sur 484 pas, entrées C et sources gelées par SHA-256.
- Assistance directe bassin : force et couple nuls sur toute la trace.
- Maxima, incluant l’état final : ancrages 4,03 µm ; sol 16,37 mm ; erreur
  de limite 0,00802 rad ; auto-pénétration non exclue 0. Aucun défaut réel
  d’intégrité. Plafond installé maximal 760 Nm, ratio anatomique ≤1 et
  aucune livraison moteur au-dessus du cap installé.
- Deux tests unitaires préexistants du banc passent sur le prototype gelé ;
  quatre expériences sont ignorées. Ils vérifient la propriété des corps et
  les cibles moteur, **pas le relevage physique**. Clippy strict et format
  passent sur cette même source avant retrait du prototype.
- La suite complète n’est pas annoncée verte : le test d’articulation hybride
  et les quatre échecs TypeScript Linux signalés sur le témoin ne sont pas
  corrigés ni retestés ici.

[Trace native et résumé](../evidence/prone-brace-2026-10-11/native/summary.json),
[comparaison physique](../evidence/prone-brace-2026-10-11/native-analysis.json),
[premier mécanisme et couples](../evidence/prone-brace-2026-10-11/native-stop-causality.json),
[budget préalable et refus](../evidence/prone-brace-2026-10-11/PLAN.md),
[sources compilées](../evidence/prone-brace-2026-10-11/source-frozen-manifest.json),
[binaire gelé](../evidence/prone-brace-2026-10-11/prone-release-binary),
[retrait du prototype](../evidence/prone-brace-2026-10-11/final-source-scope.json).

[Capture des états réels](../evidence/prone-brace-2026-10-11/prone-brace-actual.png)
et [vidéo native ralentie ×2](../evidence/prone-brace-2026-10-11/prone-brace-actual.mp4).
La vidéo n’interpole ni ne remplace les poses ; elle montre le refus de
première levée et aucun relevage. Le cercle bleu désigne une cible de tâche,
jamais une pose imposée. La vidéo et l’image sont livrées séparément dans la
Library privée ; aucune nouvelle publication.

Les [reçus Library séparés](../evidence/prone-brace-2026-10-11/library-media.json)
confirment le dépôt privé de l’image `libfile_ce0dc81bd7788191bc1d64d07abddad3`
et de la vidéo `libfile_67e4221eaa308191a6758996d8c2fbe6`.
