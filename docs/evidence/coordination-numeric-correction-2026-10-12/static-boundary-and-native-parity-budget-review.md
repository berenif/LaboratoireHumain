# Frontière statique et coût d'une future parité native

Audit en lecture seule du 12 octobre 2026. Aucun test, compilation, FK ou pas
physique exécuté par cet audit ; aucun code ou registre modifié. La seule
écriture est cette note. Les conclusions portent sur les trois tests
`archived_step12_` examinés et leurs dépendances, pas sur une suite Rust entière.

## Les trois tests restent sans intégration

La chaîne examinée est `archived_problems()` → `problem_observed()` →
`assemble_observed()` → solveur/compilateur de commandes. Dans
`whole_body_controller.rs`, lignes 544–628 de la version examinée :

- Le modèle est désérialisé directement depuis `CANONICAL_JSON`, ligne 559.
  `Model::canonical()` et `Model::validate()` ne sont pas appelés. C'est
  déterminant : `model/src/lib.rs:248–272` montre que cette autre voie ferait
  appel à `integrate_mass` sur les maillages.
- Les 25 poses/vitesses et `MassProperties` sont désérialisées depuis
  `archived-step12.json`. Aucun `RigidBody`, `PhysicsWorld` ou `Simulation`
  n'est construit par cette fonction.
- `MassProperties::world_com(&pose)` ne fait que `pose * local_com` ; `mass()`
  appelle l'inverse scalaire de `inv_mass`. Vérification directe dans Parry
  0.30.2, `src/mass_properties/mass_properties.rs:234–250` et
  `src/utils/inv.rs:3–8`. La dérivation serde des propriétés est déclarée
  lignes 11–12. Il n'y a ni réintégration des maillages, ni progression d'un
  état dynamique dans cette reconstruction du CoM.
- `Program::compile()` vérifie les commandes interpolées et leurs limites ;
  ce parcours de la grille temporelle n'appelle pas l'intégrateur.

`geometric_tasks.rs:19–123` transporte des tableaux de valeurs observées.
L'adaptateur vivant `from_simulation` ne copie que des champs existants et
n'est pas utilisé par ces trois tests. Le noyau `assemble_observed`, lignes
185–303, assemble des équations à partir de ces valeurs. `measured_geometry`,
lignes 370–443, utilise les rotations déjà observées, les frames articulaires
et des Jacobiennes analytiques. `measurement::coordinates`, lignes 152–161,
effectue uniquement de l'algèbre de rotations. Aucune pose n'est propagée par
FK de chaîne et aucun corps n'est écrit.

Les tests de reconstruction, des refus historiques et des commandes corrigées
appellent ensuite des solveurs algébriques, `runtime::command` et la validation
des contraintes. Les variables nommées `real` et `preview` dans le troisième
test sont des **mémoires de commandes**, pas des simulations. Les reçus
`reproduced-refusals.json` et `certified-commands.json` indiquent
`nativeIntegrations: 0`, ce qui concorde avec ce chemin source.

Portée de la preuve : zéro sous-pas Rapier dans ces tests, avec calculs
géométriques et numériques statiques. Elle ne certifie pas la parité dynamique
du nouveau compilateur. À l'instant de lecture, les assertions de commandes
des lignes 756–759 utilisaient l'égalité `f32`, qui ne distingue pas les zéros
signés ; le libellé `replayedCommandBitsEqual` réclame des comparaisons
`to_bits()` pour constituer une preuve littérale des bits. Ce point a été
signalé au parent, sans modification de code par cet audit.

Empreintes des sources examinées avant éventuelles modifications concurrentes :

| Source | SHA-256 |
| --- | --- |
| `whole_body_controller.rs` | `f6dbedf59113ea08ad10d221c82c81ff6498df2dc45650aaddcc1214ad007a3a` |
| `geometric_tasks.rs` | `1c018f4d20ff09921e640aa2c45b2ec04dfc860452a18e816a05e2b9ebf2ee68` |

## Registres

Le SHA-256 du registre physique a été recalculé en lecture seule :
`2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07`.
Il est **identique à celui de la clôture précédente** : statut
`closed-native-failure`, 3 856 sous-pas réservés, 1 024 exécutés connus,
clôture `2026-10-12T20:53:20.625705+00:00`. Aucune réouverture ni crédit rendu.

Le supplément `preparation-correction-budget.json`, empreinte observée
`92f3229a4fa2e91e44d3a3335a62f98635e4a5dbdb36bce0d49b7b789836a8bd`,
réserve 7 200 s de préparation et 1 000 inspections supplémentaires : totaux
14 989 s / 144 000 s et 7 923 / 20 000 inspections. L'échéance originale du
17 octobre à 20:14 UTC est conservée. Il déclare explicitement zéro sous-pas
autorisé, zéro exécuté et `newPhysicalTrialAuthorized: false`. Son tableau
`events` était vide au moment de lecture : les consommations effectives de ce
supplément restent à y documenter ; les réservations ne sont pas des mesures
d'exécution.

## Minimum proposé, sans exécution ni autorisation implicite

Pour les quatre échelles figées `0`, `0,5`, `1`, `1,5`, une comparaison de
quatre pas prédits et quatre pas exécutés par le chemin réel coûte :

**4 × (4 + 4) = 32 sous-pas natifs à 240 Hz.**

C'est le coût du noyau de comparaison à partir d'un état natif complet et
identique au pas 12. L'extrait numérique actuel ne fournit pas à lui seul cet
état : poses, moteurs et observations ne remplacent pas les caches de contact,
îles, CCD, solveur et l'ensemble de la mémoire du contrôleur. Les mondes natifs
mentionnés par l'extraction concernent les quatre premiers pas, pas un
checkpoint complet admis au pas 12. Reconstituer un corps depuis ses seules
poses ne doit pas être présenté comme une reprise exacte de ce dernier.

Proposition minimale concrète depuis la fixture initiale : **44 sous-pas**,
soit un préfixe commun de 12 pas réellement intégrés, puis les 32 pas de
comparaison. Le partage du préfixe exige de vérifier qu'il reste identique pour
les quatre échelles ; l'archive précédente montre cette identité et le
contrôleur utilise la référence commune avant acquisition des patches. Au
pas 12, copier l'état complet en mémoire pour les branches de diagnostic,
utiliser le compilateur et l'installateur communs, puis comparer après chacun
des quatre pas commandes, corps, impulsions, contacts, mémoire et horloges.
Aucun état prédit n'est transplanté dans une histoire réelle. Si un préfixe
distinct s'avère nécessaire, les 44 pas ne suffisent plus : arrêter et faire
approuver le coût révisé avant toute exécution supplémentaire.

Ces 44 pas ne couvrent aucune prévision longue, identification, histoire de
4 s, stabilisation ou qualification de relevage. Une éventuelle autorisation
doit nommer ce coût complet, son plafond mural, le binaire et les entrées
gelés, avec réservation durable avant intégration dans un **nouveau** registre.
Le présent audit n'autorise et n'exécute rien de cela.

## Addendum — contrôle final des bits et proposition bornée

La relecture finale confirme la correction des assertions :
`whole_body_controller.rs:759–763` applique maintenant
`q.map(|row| row.map(f32::to_bits))` aux positions et vitesses des deux mémoires
et du certificat. Le défaut de preuve concernant les zéros signés est ainsi
traité dans le code examiné, SHA-256
`286498411e8fd3dde6e8de3f094c03c8289ab3b2608e7cc9c6610231820cebe3`.
Le parent rapporte les trois tests archivés repassés avec succès ; cet audit
ne les a pas relancés. L'empreinte du registre physique reste identique.

La proposition de **44 sous-pas maximum** ajoute un plafond mural de
**deux minutes**, un seul processus natif et un seul thread, sans renouvellement
automatique. Les 12 premiers pas doivent reproduire les archives du préfixe
commun. Tout écart arrête le contrôle avant le pas 13 : **zéro pas supplémentaire
de comparaison autorisé**, aucun des 32 pas copie/réel ne commence. Les pas de
préfixe déjà intégrés restent comptés et la réservation n'est pas remboursée.
Une divergence, un diagnostic manquant ou le plafond mural ferme ce futur
contrôle, sans variante ni essai supplémentaire. L'exécution exige toujours une
autorisation distincte et un nouveau registre ; les anciennes échéances et la
clôture physique ne changent pas.

Pour la préparation statique, le parent annonce une clôture du supplément avec
les **1 000 inspections réservées entièrement chargées**, sans remboursement,
et les mesures effectives connues consignées séparément. À cette relecture,
`events` est encore vide : cette déclaration est une action annoncée, pas une
clôture de registre constatée par l'audit.
