# Contrat physique de la reformulation des appuis — 13 octobre 2026

Revue et implémentation statiques uniquement. Aucun monde ni `Simulation` créé,
aucun modèle canonique construit, aucune cinématique directe et aucun sous-pas
Rapier exécuté dans ce travail. Les observations natives déjà archivées sont des
entrées ; les observations synthétiques des tests ne sont pas des résultats
physiques. Cette note ne rouvre aucun registre de calcul clos.

## Décision

Séparer trois domaines : les limites des commandes autorisées, les obligations
d'appui matériel réellement observées, et les objectifs de mouvement désirés.
Une commande représentable dans l'enveloppe autorisée n'est pas une preuve que
les moteurs `ForceBased`, les contacts et la base flottante réaliseront la
cinématique souhaitée. La nouvelle préparation de commandes reste donc
`NativePreviewRequired` ; l'ordonnanceur d'appuis reste `PreviewRequired`.
Aucun de ces états ne fournit une admission d'exécution réelle.

La contradiction exacte des quatre QP archivés prouve l'incompatibilité de ces
problèmes particuliers, avec leurs hypothèses et bornes. Elle ne prouve ni une
impossibilité anatomique ni la nécessité universelle de mains ou de genoux.
Les huit sous-ensembles incompatibles minimaux par inclusion et leurs témoins
sont documentés dans `minimal-conflicts-review.md`. Pour trois échelles,
deux composantes matérielles suffisent déjà à une contradiction : supprimer
seulement la composante tangentielle x n'est pas une réparation générale.
L'assimilation de la vitesse de référence à la vitesse physique et le traitement
de la vitesse de racine observée comme donnée figée sont des hypothèses à
corriger ou valider ; ces certificats ne démontrent pas une cause unique.

## Invariants et observations conservés

Les 25 corps, 46 coordonnées, masses, inerties, géométrie, collisions, friction,
limites anatomiques et contrats de couple restent inchangés. La sortie admissible
reste constituée de références articulaires et de vitesses cohérentes avec leurs
différences f32. Aucun bassin assisté, déplacement de racine demandé comme un
actionneur, reset, verrouillage ni changement de gain/cap n'est introduit.
Le point d'installation des moteurs et les gardes originales demeurent sous la
responsabilité du contrôleur unique existant.

Les contacts courants du solveur, les charges persistantes par segment et les
impulsions CCD intégrées ont des significations distinctes. L'ordonnanceur ne
crée jamais un contact à partir d'une cible géométrique ou d'une impulsion CCD
ancienne. Il utilise `SupportObservation`, `MaterialTracker` et le contrôle
`supported_transfer` existants. Les seuils numériques de ce contrôle demeurent
inchangés : au moins 3 N et 12 sous-pas pour un segment persistant, au moins
80 % du poids sur les appuis restants, au plus 25 % sur le groupe retiré, CoM et
capture dans l'enveloppe restante avec marge de 3 mm. Les membres retirés sont
exclus de la charge **et** de l'enveloppe.

Les seuils du diagnostic local restent distincts et inchangés : charge propre
des deux régions de talon (`local_z < -0.02`), 12 observations chacune,
132 sous-pas stables soit 0,55 s, début de l'intervalle avant 2 s, au moins 80 %
du poids sur les pieds, calme réel des 25 corps à 0,22 m/s et 0,65 rad/s,
absence d'appui persistant non pédestre. L'ordonnanceur ne remplace pas ce
diagnostic. Le relevage complet en au plus 25 s après impact, sa stabilité
réelle de 1 s et la qualification des 24 poses/cycles/impacts ne sont pas
obtenus par ce jalon ; la récupération reste désactivée hors qualification.

## Pourquoi les contraintes précédentes ne certifiaient pas la physique

`geometric_tasks::assemble_observed` construit des lignes de Jacobien de
vitesses physiques, avec la vitesse de racine observée. Pour chaque point
retenu, le retour vers son ancre initiale est demandé sur un tick de quatre
sous-pas. Le QP est ensuite utilisé pour produire des vitesses de **référence**
moteur. Sous `ForceBased`, saturation, inertie, contacts et erreur de suivi
empêchent d'identifier ces deux vitesses sans modèle de réponse validé.

Remplacer les lignes matérielles par des objectifs souples permet uniquement
de construire une proposition inspectable. `command_proposal.rs` conserve
explicitement les obligations, les lignes originales, leurs bandes et les
résidus calculés pour chaque commande f32. Les autres contraintes du QP et les
limites de commande restent dures. Une résolution réussie ne doit modifier ni
l'état natif, ni l'horloge du propriétaire, ni son autorisation d'exécuter.
Les quatre échelles déjà essayées ne constituent pas une identification de la
réponse native sur les 46 axes.

De même, `Hold` à durée indéfinie n'est pas une loi universelle du contact :
un pied peut changer d'appui pendant une trajectoire. Mais remplacer un point
de contact courant par une nouvelle ancre à chaque observation efface la
dérive au lieu d'établir un pivot ou un transfert. Le passage entre ces deux
situations doit être explicite et documenté.

## Machine pure livrée

`rust/crates/sim/src/recovery/planner/support_schedule.rs` ne dépend ni de
`Simulation`, ni de `Model`, ni d'un moteur. Ses champs sont privés ; seule une
référence en lecture au journal matériel est exposée. Le module sérialise ses
décisions et son état, sans permettre de construire un état admis natif.

1. `from_retained(observation, tracker)` importe les points, identités,
   ancres et horloges archivés sans les rafraîchir. `bootstrap_observed`
   choisit seulement des contacts actuels chargés sur des segments réellement
   persistants. Cette initialisation ne prétend pas que le point choisi a lui-
   même déjà été chargé pendant douze sous-pas ; le diagnostic talon reste
   chargé d'établir sa propre histoire.
2. `request_unload(ReleaseGroup)` garde toutes les obligations et ancres.
   `LeftFoot` exclut obligatoirement les segments 18 et 19 ; `RightFoot`
   exclut 23 et 24. Aucun appel ne peut retirer seulement l'avant-pied sous
   l'étiquette « pied entier » et compter son talon comme appui restant.
3. `try_release` recalcule `supported_transfer` depuis la dernière observation
   validée. Les appuis restants doivent aussi avoir une obligation matérielle
   courante suivie. Cette première version ne s'attribue donc pas discrètement
   l'aide d'une main, d'un tibia ou d'un nouveau segment non suivi. En cas de
   refus, l'état `Unload`, les obligations et le journal restent identiques.
   En cas de passage du contrôle, les anciens points du groupe sont archivés
   et la **proposition** passe à `Released`, toujours `PreviewRequired`.
4. `begin_reposition` exige une cible explicite `RepositionGoal` : segment du
   groupe retiré, point local, cible monde et échéance en sous-pas. Cette cible
   est disponible pour un futur préparateur de commandes ; elle ne crée aucun
   point observé et n'a pas encore d'adaptateur d'exécution.
5. Pour cette route de retrait complet, `begin_acquire` exige d'abord au moins
   une observation postérieure au retrait où aucun contact courant porteur ne
   subsiste sur **les deux** segments du groupe. Il s'agit de la définition
   originale d'un point courant porteur : charge strictement positive et
   normale verticale au moins 0,65. Sans cette observation, un simple tour de
   phases pourrait réancrer le contact resté en place et masquer son glissement.
6. `acquire_observed` exige un contact actuel d'au moins 3 N correspondant au
   point local demandé. La tolérance de correspondance de 0,012 m est celle
   du propriétaire existant, pas un nouveau seuil de dérive admissible. Le
   candidat garde son premier point local fixe ; sa persistance propre est
   remise à zéro si le point disparaît ou change. Il faut douze observations
   consécutives, avant l'échéance, puis une nouvelle identité est créée avec
   l'ancre effectivement observée. L'erreur vers la cible monde est publiée
   comme résidu, jamais utilisée comme preuve de contact ou de réussite.

`request_pivot` marque une intention, mais garde le point local, l'ancre monde,
l'identité et l'heure d'acquisition. Il n'existe aucune méthode de remplacement
automatique d'ancre dans cette machine. Une route de roulement/pivot conservant
un contact tout en changeant de point matériel demandera une autre formulation
explicite, une correspondance de patches et une validation native ; elle n'est
pas remplacée ici par une hypothèse de retrait complet obligatoire pour tout
mouvement humain.

Les observations doivent suivre exactement les sous-pas natifs et l'intervalle
de 1/240 s. Les erreurs de validation et d'horloge ne mutent pas la machine.
Un point perdu reste journalisé comme perdu : `observe` ne transforme pas à lui
seul le journal en contrôleur d'arrêt. Le futur driver devra appliquer à chaque
pas les gardes originales, les obligations retenues et la continuation ; le
contrôle ponctuel de retrait ne vaut pas preuve sur toute la phase suivante.

## Contre-exemples couverts par les tests purs

- Les quatre archives B16 refusent chacun le retrait du pied gauche et du
  pied droit : huit refus conservent les deux obligations et leurs ancres.
- Un cas synthétique autorise le retrait apparent du seul avant-pied parce
  que son talon fournit encore l'essentiel de l'appui ; le groupe pied entier
  refuse correctement ce même retrait.
- Une translation observée de 48 cm suivie d'une intention de pivot garde
  la dérive de 48 cm, l'ancienne ancre et l'obligation ; elle ne recrée pas un
  contact matériel valide.
- Douze observations d'un pied resté en contact, ou la disparition du seul
  avant-pied avec talon encore porteur, n'autorisent pas une ancre neuve par un
  simple tour de phases.
- Une cible monde exactement atteinte et un segment historiquement chargé
  ne suffisent pas lorsqu'aucun point matériel correspondant n'est observé.
  Une réapparition repart avec sa propre persistance de douze observations.
- Un appui restant sans correspondance matérielle, ou une main non suivie
  dont le contrôle géométrique pourrait compter la charge, refuse le retrait.
- Un chemin synthétique positif conserve l'ancien journal, exige une
  observation de retrait puis douze contacts nouveaux, et produit une nouvelle
  identité toujours marquée `PreviewRequired`.
- Observations répétées, sautées, non finies et échéance dépassée ne produisent
  aucune acquisition ni mutation indue.

Les neuf tests du module utilisent uniquement tableaux, sérialisation, les
observations archivées et l'algèbre d'observation existante. La vérification
centrale a passé **9 tests sur 9**, zéro échec, 204 tests hors filtre, en 0,03 s ;
le reçu est `support-schedule-tests.log`. Aucun test natif n'a été lancé par
cette revue. Ces tests ne construisent aucun `Model` ni aucune `Simulation`.

Une relance instrumentale du même filtre passe également **9/9** en 0,03 s,
sans réécrire le reçu précédent : `support-schedule-tests-evidence.log`.
L'option `LH_SUPPORT_SCHEDULE_STATIC_EXPORT` écrit exclusivement un nouveau
fichier (`create_new`), ici `static-support-release-results.json`. Les huit
décisions y conservent `TransferEvidence`, le refus, les obligations et le
journal matériel avant/après. Une lecture indépendante du JSON confirme les
huit états inchangés et les obligations `[19,24]` conservées.

Quantitativement, retirer un pied dans ces archives laisserait seulement
**42,534–43,157 %** du poids sur les contacts retenus, contre les **80 %**
requis. Le groupe à retirer porte encore **42,534–43,157 %** du poids, au-delà
des **25 %** autorisés. CoM et capture sont hors de l'enveloppe restante dans
les huit cas. Ces refus concernent les états archivés à 16 sous-pas, pas toutes
les trajectoires de transfert possibles. Le reçu déclare zéro sous-pas natif,
zéro construction Model/Simulation, zéro FK et `PreviewRequired` ; aucune
admission physique n'est produite.

## Suite nécessaire et limite actuelle

Le jalon rend la distinction construisible et les refus inspectables. Il reste
à raccorder les buts explicites à un préparateur commun de commandes, puis à
une admission fondée sur des prévisions natives complètes et budgétées. Ce
driver devra préserver copies, mémoire/horloge, contacts, impulsions, modèle,
gains/caps, diagnostic et gardes originales, comparer la copie au réel au
préfixe utile et arrêter sur perte d'intégrité ou de diagnostic. L'accès de
lecture aux commandes candidates n'est pas à lui seul une barrière universelle
contre leur installation ; le futur adaptateur devra exiger une preuve
d'admission plutôt qu'un simple tableau de commandes.

La route de huit secondes existante sert de guide depuis son seed peu fléchi.
Une branche avec retrait, repositionnement ou réacquisition ne peut hériter
des preuves de l'ancienne référence, ni être annoncée comme continuation
qualifiée depuis l'accroupi réel. Ce travail ne produit aucune nouvelle
trajectoire dynamique, aucun succès local physique et aucun relevage. Les
conditions de continuation et de nouvelle autorisation sont précisées dans
`continuation-and-protocol.md`.
