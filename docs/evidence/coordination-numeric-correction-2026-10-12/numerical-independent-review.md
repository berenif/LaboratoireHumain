# Revue indépendante de la correction numérique — 12 octobre 2026

La correction traite les défauts numériques reproduits au pas archivé 12. **Elle ne qualifie aucune dynamique native** : aucun pas Rapier supplémentaire, aucune réouverture du protocole physique clos. La présente passe relit le code final et les preuves ; elle ajoute uniquement cette note. Les tests de solveur mentionnés ci-dessous ont été exécutés auparavant sur des tableaux numériques, sans `Simulation` ni monde physique.

Le [rejeu de l'ancien solveur](reproduced-refusals.json) retrouve les trois épuisements d'itérations et reproduit bit à bit le vecteur f64 refusé à l'échelle 0. Les [quatre problèmes reconstruits](reconstructed-problems.json) conservent les 46 variables, six contraintes dures et 61 tâches souples ; la reconstruction du CoM natif concorde bit à bit avec l'archive.

Le nouveau solveur utilise Newton dans le sous-espace des contraintes actives, avec QR des normales et Cholesky de la Hessienne réduite. Les égalités sont permanentes, les lignes redondantes restent vérifiées dans le problème original et les multiplicateurs d'égalité ne sont pas soumis à une condition de signe. L'objectif, la régularisation et les seuils d'acceptation sont conservés : faisabilité `1e-8`, résidu projeté `1e-6`. Chaque résolution reste limitée à 512 itérations ; chaque projection reste limitée à 512 cycles. Le certificat est recalculé au point effectivement retourné.

Onze tests algébriques ont passé, notamment les quatre QP archivés, un optimum KKT dense construit, des égalités redondantes, des axes fixés et des contraintes presque parallèles. Les QP originaux convergent en **8/8/12/12 itérations**. Le [rapport des commandes certifiées](certified-commands.json) établit également, pour les problèmes resserrés utilisés après refus de représentation :

| Échelle | Itérations | Résidu projeté | Erreur dure f64 maximale | Commandes f32 vérifiées |
|---:|---:|---:|---:|---:|
| 0 | 8 | 6,489 × 10⁻⁹ | 1,735 × 10⁻¹⁸ | 4 |
| 0,5 | 8 | 9,363 × 10⁻⁹ | 5,551 × 10⁻¹⁷ | 4 |
| 1 | 12 | 1,733 × 10⁻⁷ | 5,551 × 10⁻¹⁷ | 4 |
| 1,5 | 12 | 1,779 × 10⁻⁷ | 1,110 × 10⁻¹⁶ | 4 |

Ces quatre commandes par cas sont des valeurs produites par le compilateur, **pas des intégrations**. Chacune est revérifiée contre les contraintes du problème **original**, avec le même seuil. Le rejeu pur retrouve les bits des positions et vitesses de commande. Cela ne constitue pas une parité de trajectoires physiques.

La revue a identifié et suivi les corrections suivantes :

- **Résidus d'égalité figés** : une direction seulement tangente conservait l'erreur de la projection initiale. La restauration affine impose désormais `A_W p = b_W − A_W x`. Une régression à forte courbure couvre ce défaut.
- **Projection insuffisamment précise pour le certificat** : le facteur de courbure `L` amplifie son erreur numérique. La précision interne est maintenant resserrée à `min(1e-8, 1e-6/(8L))`, sans modifier les seuils d'acceptation. Cette marge ne constitue pas une borne universelle démontrée de l'erreur Dykstra pour toutes les configurations presque parallèles.
- **Immobilité exacte perturbée par un resserrement systématique** : le compilateur final résout d'abord le problème original et teste ses quatre commandes f32. Si elles passent, il les conserve, notamment pour un optimum stationnaire exactement représentable. Un unique recours à l'intérieur resserré est permis après refus de représentation ; il n'y a pas de boucle de recherche. Au maximum, cela représente deux résolutions de 512 itérations chacune. Un refus du premier solveur ou de l'enveloppe d'arrondi arrête le traitement.
- **Certificat perdu lors de la conversion f32** : l'enveloppe couvre l'arrondi de position, l'éventuel recul d'un ULP, puis la soustraction et la division f32. Elle resserre les contraintes ; elle ne remplace jamais la vérification des quatre commandes exactes. Aucune relaxation des forces, limites anatomiques ou critères d'appui n'est introduite.
- **Preuve insuffisante au refus** : les erreurs identifient maintenant la ligne ou borne et son résidu. Les tentatives directe et resserrée, les commandes essayées et les bornes gardées sont conservées dans l'audit, y compris avant une validation d'intérieur impossible. La mémoire d'entrée est empruntée sans mutation ; le travail s'effectue sur des copies.

L'enveloppe ULP reste conservatrice. Une égalité ou une bande très étroite peut ne posséder aucun intérieur compatible avec cette enveloppe, même lorsqu'une commande f32 admissible existe. Le chemin direct préserve les cas déjà représentables ; un échec ultérieur est correctement formulé comme absence d'intérieur de l'enveloppe, **pas comme preuve d'impossibilité de représentation**. Les cas numériques difficiles restent bornés et peuvent être refusés.

Avis : correction numérique acceptable pour cette étape de préparation et ces fixtures, avec le chemin « original, puis au plus un recours resserré » approuvé. Le [budget complémentaire](preparation-correction-budget.json) autorise zéro sous-pas natif. Les preuves disponibles ne démontrent ni acquisition des talons, ni stabilité d'appui, ni réussite de B ou de la continuation de 8 s ; ces qualifications restent absentes.
