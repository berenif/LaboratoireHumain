# Coordination : les quatre refus numériques corrigés sans nouvelle physique

Les quatre problèmes archivés au sous-pas 12 produisent maintenant des commandes
admissibles. **30 tests ciblés passent**, ainsi que la compilation de production.
Aucun nouveau sous-pas Rapier n'a été exécuté. Le protocole physique initial
reste clos : B n'a toujours pas été appliqué à une histoire réelle, les talons
ne sont pas acquis et aucun relevage n'est qualifié.

Le [lot natif distinct du 13 octobre](coordination-native-parity-2026-10-13.md)
a été clos avant tout démarrage du worker, sur une interface `/proc` absente.
Il n’ajoute aucune validation physique : 0 pas exécuté, 44 réservés sans
remboursement, aucune relance.

Cette correction poursuit la préparation de 40 h autorisée par le propriétaire
et par le message du parent après le premier résultat. Elle traite un défaut
logiciel réparable ; elle ne transforme pas ce refus en preuve d'infaisabilité
physique. Les caps, gains, masses, friction, collisions et anatomie sont
inchangés. Récupération et pas correctifs restent désactivés par défaut.

## Diagnostic reproduit

L'[extrait archivé](evidence/coordination-numeric-correction-2026-10-12/archived-step12.json)
contient les poses, vitesses, propriétés de masse et mémoires nécessaires aux
équations du premier appui. Il ne constitue pas un checkpoint dynamique complet.
L'adaptateur statique reconstruit les tâches sans créer de monde physique ni
propager les poses. Le CoM natif est retrouvé bit à bit, en conservant l'ordre
des opérations f32. Les quatre problèmes possèdent 46 variables, six contraintes
dures et 61 tâches souples.

Le solveur figé au commit `b6e711d3dc58909979295df81437e6a5579c41db` est conservé
séparément pour reproduire les refus. Sa copie de rejeu ne change que les messages
de diagnostic et enlève les anciens tests ; l'algorithme est inchangé.

| Facteur | Refus de l'ancienne implémentation reproduit |
|---:|---|
| 0 | Solution f64 retrouvée bit à bit en 391 itérations ; les commandes f32 dépassent les deux contraintes matérielles sur l'axe mondial 2 |
| 0,5 | 512 itérations, résidu projeté `5,2678802462e-5` |
| 1 | 512 itérations, résidu projeté `4,6927519467e-4` |
| 1,5 | 512 itérations, résidu projeté `1,3577859368e-3` |

Pour le facteur 0, dès la commande du pas 12 :

- `material-patch-0-world-2` : valeur `0,2769597274148982`, borne haute
  `0,2769587450839477`, dépassement `9,8233095053e-7` ;
- `material-patch-1-world-2` : valeur `0,2739359927415980`, borne haute
  `0,27393510533429255`, dépassement `8,8740730547e-7`.

Le seuil numérique est `1e-8`. Il n'a pas été assoupli. Le défaut de représentation
est distinct des trois défauts de convergence. Les quatre commandes des pas 12
à 15 sont conservées dans le [rejeu des refus](evidence/coordination-numeric-correction-2026-10-12/reproduced-refusals.json).
Ces valeurs sont calculées sur les archives, sans intégration physique.

## Correction implémentée

Le solveur utilise maintenant Newton sur les contraintes actives, QR pour les
normales indépendantes et Cholesky dans le sous-espace libre. Il conserve
l'objectif et la régularisation. Les seuils restent `1e-8` pour la faisabilité
et `1e-6` pour le résidu projeté ; plafonds de 512 itérations et 512 cycles par
projection. Le certificat porte sur le point effectivement retourné.
La précision interne de projection est resserrée lorsque la courbure amplifie
son erreur. Les régressions couvrent notamment les égalités redondantes,
les axes fixés, la forte courbure et les contraintes presque parallèles.

Le compilateur essaie d'abord le problème original et vérifie ses quatre
commandes f32 exactes. Une solution déjà représentable est conservée, y compris
l'immobilité et les égalités. Si la conversion est refusée, une seule seconde
résolution réserve une marge ULP vers l'intérieur des contraintes. Chaque
commande est ensuite vérifiée contre le **problème original**. Au plus deux
résolutions de 512 itérations sont possibles, sans boucle de réglage. Un défaut
de convergence directe ou de l'enveloppe d'arrondi arrête le calcul.

L'ordre de calcul de q/dq reste celui du prototype : position f64 convertie en
f32, éventuel recul d'un ULP à la borne de déplacement, puis différence et
division en f32. La [revue historique](evidence/coordination-numeric-correction-2026-10-12/f32-checkpoint-review.md)
compare explicitement le précédent `dcbcdf...` : reconstruire exactement la
commande est transférable ; restaurer le dq Hermite ancien changerait le témoin
A. Celui-ci n'est donc pas modifié.

Les diagnostics enregistrent désormais le problème complet, les tentatives
directe et resserrée, les commandes essayées et la contrainte fautive avant un
refus. La mémoire de commande n'est installée qu'après certification. Une
absence d'intérieur de l'enveloppe conservatrice n'est jamais présentée comme
une preuve d'impossibilité de toute commande f32.

## Preuves obtenues

| Facteur | Itérations, résolution resserrée | Résidu projeté | Commandes f32 conformes |
|---:|---:|---:|---:|
| 0 | 8 | `6,489e-9` | 4/4 |
| 0,5 | 8 | `9,363e-9` | 4/4 |
| 1 | 12 | `1,733e-7` | 4/4 |
| 1,5 | 12 | `1,779e-7` | 4/4 |

Les positions et vitesses de ces 16 commandes sont identiques **bit à bit**
au rejeu (`to_bits`, zéros signés compris). Leur dq correspond exactement à
la différence f32 des positions divisée par dt. Ce sont des preuves du
compilateur et des contraintes linéaires assemblées, pas des preuves de
charges réelles, de parité dynamique ou de stabilité.

Les [résultats certifiés](evidence/coordination-numeric-correction-2026-10-12/certified-commands.json),
la [revue indépendante](evidence/coordination-numeric-correction-2026-10-12/numerical-independent-review.md)
et le [reçu de validation](evidence/coordination-numeric-correction-2026-10-12/validation/receipt.json)
sont versionnés. La vérification finale exécute 11 tests solveur, 7 compilateur,
6 géométrie et 6 propriétaire, puis `cargo check --lib`. Les trois exports
reconstruits sont identiques octet par octet aux exports précédents. Les
échecs globaux Rust/TypeScript historiques n'ont pas été retestés ni déclarés
corrigés.

Reproduction, Rust 1.89.0 disponible :

```sh
python3 docs/evidence/coordination-numeric-correction-2026-10-12/verify-numeric-preparation.py --output /tmp/lh-numeric-verification
```

Le dossier de sortie doit être neuf. Le script fixe les quatre modules revus,
ne lance ni worker ni test d'intégration et vérifie avant/après le SHA du
registre clos. `--cargo /chemin/vers/cargo` permet d'utiliser un toolchain local.
Les tests de rejeu archivé ne construisent aucun monde ; les anciens tests du
propriétaire construisent des scènes et installent des moteurs, sans sous-pas.
Aucun binaire release du protocole ancien n'a été remplacé.

## Budget, décision et prochaine frontière

Le [supplément de préparation](evidence/coordination-numeric-correction-2026-10-12/preparation-correction-budget.json)
a réservé avant travail 7 200 s et 1 000 inspections supplémentaires, sans
remboursement : total conservateur 14 989 s / 144 000 s et 7 923 / 20 000
inspections. Les événements de clôture distinguent réservation et observation.
Il autorise et enregistre **zéro sous-pas natif**. Le SHA du registre physique
reste `2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07`.
Le lot précédent conserve 1 024 pas exécutés et 3 856 réservés ; aucun reliquat
n'est réutilisé.

**Décision : le blocage numérique identifié est corrigé sur les quatre états
archivés. La continuation complète de 8 s reste non admise physiquement.**
La correction ne prouve ni acquisition des talons, ni recentrage stable, ni
relevage avant 25 s, ni la seconde de stabilité et les 24 qualifications.

La prochaine preuve minimale requiert une autorisation distincte, avant tout
calcul : **44 sous-pas maximum**, soit un préfixe commun de 12 puis quatre
paires de 4 pas prédits et 4 pas exécutés ; plafond mural proposé **2 minutes**,
un processus et un thread. Vérifier l'identité du préfixe avec les archives,
puis commandes, états, contacts, impulsions, mémoire et horloges à chaque pas.
Arrêter avant le pas 13 si le préfixe diverge, sans rembourser les pas consommés.
Un nouveau registre, un manifeste figé et une réservation durable sont requis.
Ce contrôle technique ne comporte aucune histoire de 4 s ni prévision longue.
Voir l'[audit de ce coût](evidence/coordination-numeric-correction-2026-10-12/static-boundary-and-native-parity-budget-review.md).

Les [captures physiques](evidence/coordination-prototype-2026-10-12/continuation-physical-results.png)
et la [vidéo A](evidence/coordination-prototype-2026-10-12/continuation-A-actual-history.mp4)
restent celles des états archivés du lot clos : aucun nouveau rendu ne masque
une intégration. La voie officielle Library a précédemment échoué sur
`hosted apps tools/list ... network` ; aucun ID/version n'a été obtenu et aucune
voie alternative n'a été utilisée. Les livrables et preuves sont disponibles
dans Git ; cette correction ne prétend pas avoir été déposée en Library.
