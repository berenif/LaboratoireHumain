# Contre-revue de la séparation commande/appui

Le nouveau chemin est un correctif statique cohérent de la **catégorie de faisabilité**. Il permet de préparer une commande bornée sans présenter une relation cinématique non identifiée comme une loi physique du contact. Il ne répare pas, à lui seul, la capacité physique de transfert ou de relevage. La famille précédente reste close.

## Commandes et obligations

La lecture de `command_proposal.rs`, du raccord `Controller::prepare_command_proposal_observed` et des fonctions de compilation existantes confirme les points suivants :

- Les bandes matérielles sont identifiées explicitement à partir de chaque patch retenu, une fois par composante. Une omission, un doublon, une identité inconnue ou une obligation absente/périmée est refusé. Les autres contraintes dures et les bornes de commande sont conservées.
- Les coefficients matériels originaux servent désormais de guides heuristiques à poids unitaire. `MaterialGuide` conserve leur rangée complète et leurs bandes ; les résidus des quatre commandes f32 restent visibles. Cette conservation a été ajoutée après une première revue où les rangées n’étaient pas présentes dans la preuve de sortie.
- Le compilateur f32 reste celui d’origine : horloge 60 Hz, limites anatomiques, slew et cohérence exacte entre positions et vitesses émises. La certification porte sur le problème de **commande séparé**, pas sur les anciennes bandes matérielles devenues guides.
- Le propriétaire est lu par `&self`. La fonction ne modifie ni sa mémoire, ni ses patches, ni son horloge, et n’installe aucun moteur. Le chemin `configure` historique n’est pas remplacé et aucun appel de production à cette nouvelle préparation n’est ajouté.
- Le seul statut est `NativePreviewRequired`. Même un résidu géométrique nul ne fait réussir `require_real_execution_admission`, qui retourne toujours une erreur.

Ce n’est donc pas une augmentation d’une tolérance physique pour annoncer un succès. C’est une nouvelle formulation de génération de candidats, dont l’admission physique demeure fermée. La faisabilité attendue après retrait des six rangées de l’ensemble dur est précisément une faisabilité de commande ; elle ne constitue pas une validation de support et ne réfute pas les [huit IIS archivés](minimal-conflicts-review.md).

La substitution `J·u` demeure dans la direction heuristique choisie, où `u` est la dérivée de la cible moteur. Le code ne prétend plus qu’elle soit la vitesse physique `J·v`. L’actionnement `ForceBased` calcule une force à partir de l’erreur et de la différence entre vitesse cible et vitesse mesurée : il n’impose pas `u=v`. Aucune carte commande→état à 46 dimensions n’est identifiée par ce correctif. Cette limite concerne aussi les guides de CoM et d’orientation, pas seulement les points retenus.

## Transfert préparatoire

`support_schedule` sépare l’intention de retrait de l’observation. Un groupe pied exclut l’arrière-pied **et** l’avant-pied du calcul de soutien restant. `try_release` conserve le calcul original de charge, CoM et capture, puis exige que les appuis comptés aient des obligations matérielles suivies. Les patches retirés sont archivés ; le pivot conserve son point local, son ancre et sa dérive. Aucun déplacement du but mondial ne crée un contact.

Une acquisition exige sa propre séquence de douze observations chargées, avec correspondance au point local demandé et au premier point du candidat. Le compteur historique du segment ne remplace pas cette séquence. L’erreur au but mondial reste un résidu et toutes les décisions restent `PreviewRequired`.

La revue a découvert puis fermé une boucle Retrait→Repositionnement→Acquisition qui pouvait réancrer un pied n’ayant jamais perdu son appui. Le code exige désormais une observation strictement postérieure au retrait où les deux segments n’ont plus aucun contact courant porteur selon la définition originale (`normal ≥ 0,65`, charge `> 0 N`). `begin_acquire` refuse tant que cette observation manque. Le nouveau test négatif couvre douze observations du même appui et la disparition du seul avant-pied tandis que le talon reste porteur. La correction a été relue ; aucune réserve bloquante ne subsiste pour ce module statique.

Cette disparition est celle du **support porteur** ; elle ne prouve pas une séparation géométrique du sol ou une trajectoire de dégagement. Le pivot conserve un point matériel et n’implémente pas à lui seul un roulement changeant de point. Ces mouvements exigent une formulation et une admission ultérieures explicites.

## Limites avant tout raccord natif

`commands_for_native_preview` expose une tranche de `Command` copiable. L’absence d’admission est garantie aujourd’hui par le chemin isolé et le refus explicite ; ce n’est pas un verrou de type universel sur tous les installateurs de la crate. Un futur adaptateur devra réclamer une preuve d’admission liée à l’état, aux commandes et au programme exacts, et ne pas contourner ce contrôle en recopiant la tranche.

De même, `observe` conserve les pertes de correspondance dans les patches ; il ne ferme pas, à lui seul, toutes les phases sur toute perte. Un futur driver devra réévaluer à chaque pas les gardes de support, d’intégrité, de délai et de continuité. Le succès du seul `try_release` certifie une condition présente nécessaire ; il ne certifie ni la phase entière de repositionnement/acquisition, ni le mouvement flottant futur, ni le terminal de récupération.

Ces lacunes sont explicitement hors de l’admission actuelle, qui demeure impossible. Elles sont des prérequis de tout futur raccord, pas des autorisations d’élargir les caps, gains, seuils ou budgets. Aucun nouveau sous-pas n’est demandé par cette revue.

## Portée de la vérification

Cette contre-revue lit le code et les coefficients archivés. Mes [sept tests Python](minimal-conflicts-tests.log) ne construisent aucun `Model`, monde ou `Simulation`, n’importent aucune bibliothèque native et n’effectuent aucune FK ; ils vérifient uniquement des preuves rationnelles exactes. Les tests Rust du propriétaire utilisent par ailleurs une valeur `Model` désérialisée comme métadonnées anatomiques : ce n’est ni `Model::canonical`, ni une construction de monde, ni un calcul de géométrie. Leur exécution et leurs journaux relèvent du parent ; cette note ne convertit pas leur résultat en qualification native.
