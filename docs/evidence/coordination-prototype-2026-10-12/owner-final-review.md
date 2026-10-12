# Revue finale du propriétaire expérimental — 12 octobre 2026

Relecture contradictoire de `whole_body_controller.rs`, des raccordements dans
`lib.rs` et `recovery.rs`, de l'installateur réutilisé et de l'observateur original.
Aucun code modifié, aucun test, pas physique, FK ou inspection géométrique
exécuté par ce réviseur. Cette note décrit l'arbre de travail avant le gel ; le
gel doit ensuite identifier ses fichiers et son binaire exacts.

**Aucun nouveau défaut bloquant identifié dans ce périmètre après les
corrections. Ce constat logiciel n'est pas une admission physique du transfert
ni une qualification du relevage.** Les restrictions et les vérifications
natives prévues restent nécessaires.

## Corrections vérifiées

- Le préfixe sans patch utilise maintenant la même branche pour A et B : même
  position nominale, vitesse dérivée de la différence effective des positions
  f32 et même mise à jour de mémoire. Les taux calculés dans le bloc de préfixe
  ne sont pas émis tant que les patches sont absents. Le premier solveur commun
  intervient à une frontière de quatre sous-pas, après acquisition observée.
- Les nouvelles ancres sont en mode `Hold`. Cette famille ne prévoit aucun
  remplacement matériel : la disparition de la correspondance provoque un
  refus. Le changement automatique d'ancre qui pouvait masquer un déplacement
  sous l'étiquette pivot a été supprimé de ce propriétaire.
- Les références doivent couvrir la grille complète, avec deux côtés
  déterminés, des quaternions finis/unitaires et les quatre segments matériels
  attendus. La valeur du retour choisi doit appartenir aux quatre propositions
  gelées. Les sommes d'horloge des commandes utilisent des additions vérifiées.

## Propriété des commandes et actionnement

`Simulation::configure_motors` retire temporairement le propriétaire, le fait
configurer les moteurs, le réinsère et retourne immédiatement. Le hook
`offline_trajectory_before_step` constate ensuite la présence de ce propriétaire
et n'installe aucun autre programme. La logique de décision ne lance ni
prévision ni intégration ; la sélection des prévisions reste externe.

Le problème comporte les 46 taux articulaires. Les points retenus produisent
trois contraintes dures par point ; aucune pondération de posture ne peut les
acheter. Le twist mesuré du bassin est une donnée du modèle géométrique, pas
une commande de racine. Les quatre commandes f32 sont vérifiées avec leurs
taux réellement émis avant de les installer. Une correction matérielle
irréalisable retourne un refus ; sa bande ne doit pas être élargie après coup.

A et B utilisent le même installateur existant `OwnedRequests`. La diff de cet
installateur change sa visibilité, pas son calcul. Il conserve le modèle moteur
`ForceBased`, les gains du profil, les plafonds anatomiques et la normalisation
au budget agrégé, avec contrôle de la somme effectivement installée. La formule
de demande ressort/amortisseur ne devient pas une preuve de couple réellement
livré. Aucune modification de masse, friction, collision, segment ou force de
bassin n'apparaît dans les changements lus.

## Observateurs et arrêt

L'intégration continue d'employer `advance_tick_observed` et ses inspections
originales après chaque sous-pas. L'observateur de récupération, le test de
station debout, le settle, le handoff et la seconde de stabilité ne sont pas
remplacés. Le propriétaire met en place ses commandes et son statut de phase,
mais ne modifie ni `settle_substeps`, ni `stable_substeps`, ni le compteur de
récupérations. Un observateur ayant supprimé la récupération ne provoque pas
un redémarrage : la configuration suivante refuse de continuer.

Au handoff observé, le propriétaire est retiré et l'ancien chemin reprend.
Un statut Handoff artificiel sans les données mesurées correspondantes échoue
dans la garde originale. Le relevage automatique et les pas correctifs restent
désactivés par défaut ; le raccordement est limité à la compilation
expérimentale de test.

La garde de perte d'appui conserve sa durée originale. La garde de stagnation
emploie ici la progression articulaire mesurée du nouveau programme, avec un
arrêt au plafond temporel ; elle n'accorde aucune récupération sur ce progrès.
Le diagnostic de talon reste séparé, avec sa persistance propre et les appuis
réellement présents. La persistance du segment permet au contrôleur de retenir
une tâche, mais n'accorde pas rétroactivement douze sous-pas à un talon nouveau.

## Preuves disponibles et limites

Le reçu `whole_body_controller-tests.log`, lu sans relancer les tests, indique
trois tests réussis : moteurs identiques au premier pas de commande A/B sans
intégration, refus d'une référence malformée, et préservation de la garde du
handoff. Cela ne prouve pas l'identité de deux histoires physiques, la capacité
du solveur à satisfaire la première situation chargée, ni l'acquisition d'un
talon. Le test de parité natif réservé et l'histoire bornée sont nécessaires
pour ces questions.

Les constantes `0.005 m/s` de bande de tâche et `0.012 m` de correspondance
matérielle appartiennent à la proposition géométrique gelée. Elles ne remplacent
aucun seuil physique du diagnostic local ou du terminal original. Toutes les
dérives matérielles doivent rester visibles dans les traces, même lorsqu'une
correspondance locale est encore acceptée. Les 46 références bornées ne
garantissent pas que les moteurs suivront ces vitesses.

La construction `scene` installe uniquement la fixture initiale avant toute
intégration. La lecture du chemin ne révèle aucun settling caché ni reset
ultérieur. Cette revue ne couvre pas à elle seule la validité de toute la
continuation géométrique, le registre global, les critères de choix des
prévisions ou leurs reçus de parité : leurs preuves séparées restent à joindre.
