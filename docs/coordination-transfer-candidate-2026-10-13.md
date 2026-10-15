# Candidat de transfert depuis l'état accroupi observé

Le correctif fournit une entrée physique archivée, une politique commune
d'acquisition des talons, un raccord de commandes continu vers la référence
debout et un adaptateur expérimental d'admission native. **Le premier problème
commun sur A36 produit quatre commandes admissibles numériquement. Aucun
nouveau mouvement physique n'a été calculé.** Les talons ne sont toujours pas
acquis dans la preuve disponible et aucun relevage n'est qualifié.

Cette étape prolonge la [reformulation statique](coordination-support-reformulation-2026-10-13.md).
Elle ne ferme pas la préparation globale et ne rouvre aucune campagne close.
Le contrôle ordinaire et la récupération par défaut restent inchangés ; le
nouveau chemin est expérimental et exige une admission explicite.

## Entrée retenue et limites

L'[entrée A36](evidence/coordination-transfer-candidate-2026-10-13/observed-entry-review.md)
est le premier tick complet archivé où les quatre segments de pieds sont
chargés et persistants et où CoM et capture sont dans leur support. Elle
provient de 36 intégrations réellement effectuées dans l'histoire A, à 0,15 s.
Le bassin est à 0,9477 m et les talons postérieurs portent zéro newton. Les
vitesses maximales atteignent 0,409 m/s et 0,998 rad/s : cet état est dynamique,
peu fléchi et non stabilisé. Le genou à 0,66 rad est une **consigne**, pas un
angle physique mesuré.

Le préfixe commence dans la fixture analytique historique ; il ne comprend
pas une chute provoquée et un impact qualifié. Un futur résultat depuis A36
sera donc un résultat de transfert/continuation, distinct d'un relevage après
impact en moins de 25 s et de la qualification sur 24 poses/cycles/impacts.

Les 25 poses, vitesses, contacts, charges, commandes antérieures et mémoires
accessibles sont archivés. Les CoM corporels utilisés pour les tâches sont
une dérivation algébrique explicite des poses et des propriétés massiques
natives archivées ; leur somme reproduit le CoM observé. Aucun monde ni FK
n'est construit. Il n'existe pas de checkpoint natif complet au pas 36 :
une reprise exacte devra rejouer les 36 pas du préfixe puis vérifier les
champs archivés. Transplanter les poses dans un monde neuf n'est pas admis.

## Décision de commande

Le candidat conserve d'abord les deux avant-pieds matériels (segments 19 et
24), abaisse les points de talon (18 et 23) vers le support observé et recentre
le CoM/capture. Les points antérieurs chargés des arrière-pieds restent dans
l'observation physique ; ils ne deviennent pas de fausses ancres de talon.
Une intention de pose ne crée jamais un contact. Les talons ne sont ajoutés
au tracker qu'après douze observations consécutives du point matériel
correspondant, avec charge et persistance vérifiées.

Les tâches partagent les 46 coordonnées : déplacement du CoM, orientation
des pieds, abaissement des talons, posture et retour vers les ancres. Gains,
poids et limites sont fixes. Les caps et l'installation `ForceBased` restent
ceux du contrat ; aucun couple idéal ne remplace cette action. Les guides
matériels sont des heuristiques de commande, tandis que perte d'appui,
charge, persistance, dérive et intégrité restent des obligations natives.

Le candidat bilatéral n'a pas besoin d'un retrait de pied pour sa première
question causale. La branche de retrait existante garde sa garde de transfert
avant libération : soutien restant mesuré, membre allégé, CoM/capture
intérieurs, puis disparition observée avant nouvelle acquisition. Elle n'est
pas confondue avec le candidat bilatéral et aucune variante n'est explorée.

Le jalon local nécessite les conditions du diagnostic original pendant 132
observations. Il est mémorisé pour permettre ensuite le mouvement de montée,
qui n'est évidemment plus immobile. L'absence d'appui non plantaire, la
conservation des ancres et la dérive restent surveillées à chaque pas après
ce jalon. Le délai du candidat exige ce jalon **achevé** au plus tard à A36+480 inclus ; c'est
une limite de cette expérience, plus stricte qu'un simple début de maintien
avant deux secondes, et non une réécriture du critère historique.

La [revue de provenance](evidence/coordination-transfer-candidate-2026-10-13/support-threshold-provenance.md)
retrouve le seuil de 80 % dans les gardes du contrôleur puis dans le protocole
figé. Il ne figure pas comme invariant moteur universel dans le contrat et
n'est pas une loi valable pour toute dynamique. C'est une garde conservatrice
de transfert issue d'un raisonnement quasi statique. Elle est **conservée**,
ainsi que les 25 %, 3 mm et autres seuils ; aucune attribution à une demande
numérique explicite du propriétaire n'est inventée.

## Raccord et continuation finis

`observed_raccord` part des dernières q/dq **installées**, non des angles
physiques ou d'une posture supposée. Un unique polynôme cubique rejoint la
coupe stationnaire 432 de la référence historique. Sa durée est déterminée
par une borne analytique puis vérifiée sur les extrema continus et sur chaque
commande f32. Aucun balayage de durées ni saturation réparatrice n'est permis.
Les vitesses émises sont exactement les différences f32 des positions.

La référence locale aide à acquérir les talons. Lorsque le jalon local est
observé, un second raccord est préparé depuis les q/dq alors installées,
jamais depuis les anciennes q/dq d'A36. Il continue avec la partie 433–1920,
dont une seconde de référence debout. Toute la référence finie doit tenir
sous l'échéance absolue. Cette couverture des commandes ne prouve pas que
le corps suivra la trajectoire ou que ses contacts seront admissibles.

Le contrôleur demande `Stand` une fois la référence debout atteinte, conserve
les temporisations originales et laisse l'observateur original décider du
relais et du succès. Après le relais, le contrôleur ordinaire possède seul
les moteurs ; l'adaptateur conserve seulement la surveillance et l'admission.
La récupération n'est jamais redémarrée après un échec.

## Admission native et corrections de contre-revue

Une proposition numérique seule ne permet aucune installation. Le wrapper
natif construit un reçu seulement après un horizon effectivement terminé,
budgété, avec le même compilateur, les mêmes moteurs et les mêmes observateurs.
Les copies contiennent toutes les mémoires de décision. Le reçu n'est pas
désérialisable depuis les fixtures JSON et son constructeur est privé au
wrapper natif.

Chaque commande réelle doit correspondre exactement à la commande et à
l'identité native prédites. Les identités comprennent le monde et ses caches,
les bits corps/moteurs, l'observateur de récupération et les mémoires de
commande. Seuls les reçus d'autorisation et les journaux de validation sont
exclus des équations et de cette identité.

La contre-revue a fait corriger des défauts concrets :

- séparation entre état extérieur de tick et état avant configuration, car le
  noyau efface des accumulateurs de livraison entre ces deux frontières ;
- comparaison du checkpoint extérieur après chaque tranche réelle de 24 pas,
  y compris avant copie ou renouvellement ;
- refus des observations différentes au même horodatage, des commandes
  étrangères au lot et de leur double consommation ;
- surveillance des contacts après le jalon local et après le relais ;
- échéance absolue du succès original et maintien terminal borné, sans
  réinitialisation des compteurs ;
- conservation du reçu complet jusqu'au terminal, pour éviter qu'une nouvelle
  prévision minimale de 120 pas rende impossible la dernière fraction de la
  seconde de stabilité réelle ;
- couverture exhaustive des mémoires privées de pas, ancres, swing et
  temporisations de mouvement, des paramètres du modèle/profil, des filtres,
  handles et accumulateurs ; les interactions actives sont refusées dans ce
  protocole statique ;
- identité exacte à champs balisés et longueurs explicites : les octets sont
  conservés sans les convertir en tableaux JSON décimaux. Ce changement réduit
  le coût de représentation, sans remplacer la comparaison par un hash ;
- contrôle du dernier checkpoint extérieur avant de déclarer l'histoire finie.

Les tests purs de ces gardes ne constituent pas une preuve de parité native.
La parité et le terminal de ce nouveau chemin restent à mesurer dans le
protocole suivant ; les reçus des anciennes sources ne sont pas réutilisables.

## Résultat numérique et vérifications

L'[assemblage A36](evidence/coordination-transfer-candidate-2026-10-13/archived-a36-command-result.json)
est unique. La cible horizontale est à l'intérieur du support réel avec la
marge inchangée. Les points de talon se trouvent à 25,35 et 26,01 mm au-dessus
du support et reçoivent des intentions verticales de −0,0507 et −0,0520 m/s.
Ce sont des objectifs, pas des vitesses physiques obtenues.

Le QP commun respecte ses contraintes avec une erreur continue nulle et
produit quatre q/dq f32 vérifiées. Le résidu maximal hors ancienne bande
matérielle vaut **0,03229946 m/s** sous l'heuristique cinématique non validée.
Il reste archivé et ne sert pas de preuve de glissement réel ou de soutien.
Les deux talons portent toujours le statut « non observé de façon persistante ».

Les [journaux](evidence/coordination-transfer-candidate-2026-10-13/) distinguent
les échecs de compilation préparatoires, le test géométrique réussi et les
tests purement algébriques. La suite globale Rust/TypeScript et tous les tests
qui construisent un monde sont exclus de cette étape. Aucun échec historique
n'est déclaré résolu par ces vérifications ciblées.

## Budget, conservation et suite

La [réservation du 13 octobre](evidence/coordination-transfer-candidate-2026-10-13/preparation-reservation.json)
a engagé 10 800 s de préparation, total 46 489/144 000 s, sans remboursement.
Sa fenêtre a expiré à 15:01:54 UTC ; elle reste inchangée. Après l'interruption
du service et la demande explicite de reprise, la
[réservation du 15 octobre](evidence/coordination-transfer-resume-2026-10-15/preparation-reservation.json)
engage 10 800 s supplémentaires de 09:27:54 à 12:27:54 UTC. Le total engagé est
**57 289/144 000 s**, et 86 711 s restent non engagées. Il s'agit d'engagements
de fenêtres, pas d'une durée de calcul réellement exécutée. L'interruption
n'a déclenché aucun travail supplémentaire ni remboursement.

La sous-réservation géométrique de trente minutes a débité douze inspections.
Une seule a été exécutée, le 13 octobre à 12:44:48 UTC ; les onze autres ne
sont pas remboursées. Les compteurs sont **un assemblage de données archivées,
zéro FK, zéro monde et zéro intégration native**. Les empreintes sources
avant/après cet essai correspondent. Les modifications ultérieures des tests
et de l'admission sont distinguées du code de cet assemblage.

Les cinq registres physiques, quatre binaires et cinq manifests historiques
restent protégés ; leurs sources figées sont vérifiées dans leurs commits.
La famille physique a consommé ses quatre emplacements et demeure close.

Le [protocole futur distinct](evidence/coordination-transfer-native-proposal-2026-10-13/future-native-protocol.md)
prévoit une seule histoire complète, avec une porte locale et un arrêt causal,
une admission du nouveau code, tous les préfixes et toutes les prévisions
comptés. Le pilote dédié est compilé en release avec `--no-run` ; ses sept
tests purs passent. Le gel du superviseur et du manifeste termine cette
préparation. Ce document n'ouvre pas le budget : le budget distinct doit être
autorisé par le parent avant toute exécution. Aucun succès local ne vaudra
relevage ni qualification générale.

Le blocage Library déjà constaté reste signalé : aucune nouvelle version
Library n'est revendiquée. Les fichiers suivis et les SHA Git constituent
les livrables vérifiables de cette étape.
