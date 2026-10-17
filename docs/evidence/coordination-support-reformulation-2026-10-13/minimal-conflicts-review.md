# Conflits minimaux et sens des contraintes de support

Huit sous-systèmes incompatibles minimaux par inclusion sont identifiés dans les quatre QP archivés au pas 16 : un par pied et par échelle. Chacun contient huit inégalités scalaires sur sept coordonnées. Un certificat dual rationnel exact prouve l’incompatibilité ; huit témoins rationnels, un après retrait de chaque inégalité, prouvent la minimalité. Les 64 témoins sont vérifiés exactement.

Cette analyse n’établit ni une cardinalité minimale globale ni une énumération de tous les conflits. L’unité comptée est une face scalaire de bande ou une borne de coordonnée, pas un patch complet ni une contrainte 3D regroupée. Les témoins satisfont le sous-système minimal après retrait, **pas le QP complet après retrait** : l’autre pied y contient toujours son propre conflit indépendant. Ils ne sont ni des consignes candidates ni des trajectoires physiques.

## Contenu des huit sous-systèmes

Le même ensemble d’indices convient aux quatre échelles. Pour le patch gauche, on conserve :

- la borne inférieure de `material-patch-0-world-1` (y) ;
- la borne supérieure de `material-patch-0-world-2` (z) ;
- la borne inférieure de `material-patch-0-world-0` (x) ;
- les bornes supérieures de `leftThigh.x` (32), `leftThigh.y` (33), `leftAnkle.x` (36) et `leftForefoot.x` (38) ;
- la borne inférieure de `leftFoot.z` (37).

À droite, le patch est 1 et les cinq indices sont respectivement 39, 40, 43, 45 et 44. Le nom anatomique provient seulement de la lecture du JSON canonique et de l’ordre d’énumération des axes ; aucun `Model` n’est construit.

| Échelle | Marge de contradiction gauche | Marge droite |
|---:|---:|---:|
| 0 | 0.035184763692771036 | 0.031092837384892630 |
| 0.5 | 0.048063855940865150 | 0.044780389404780976 |
| 1 | 0.051766185794813890 | 0.050224698202059930 |
| 1.5 | 0.052651109407174350 | 0.048874266006705730 |

Ces marges incluent déjà l’élargissement de **chaque** inégalité retenue par la tolérance originale `1e−8`. Les chiffres affichés sont arrondis ; les calculs et vérifications utilisent les fractions exactes des coefficients f64 archivés.

La construction combine les trois faces matérielles puis annule les coefficients `leftThigh.z`/`leftShin.x` (34/35), ou 41/42 à droite. Les cinq coefficients restants donnent les bornes signées ci-dessus. Pour `A v ≤ b`, les huit multiplicateurs sont strictement positifs, `wᵀA=0` exactement et `wᵀb<0`. Après chaque retrait, les sept rangées restantes sont indépendantes ; une élimination de Gauss rationnelle résout leurs sept égalités et produit un témoin qui satisfait toutes les inégalités restantes. L’inégalité retirée est strictement violée, ce qui est enregistré.

Les témoins peuvent dépasser une borne retirée : leur maximum absolu reste ici inférieur à 21 rad/s, ce qui n’en fait aucunement des commandes admissibles. Une autre combinaison des anciens certificats aurait produit des coefficients d’environ `1e−19` et des témoins d’environ `1e18` ; elle n’a pas été retenue pour cette présentation. Aucun coefficient n’a été arrondi à zéro ou modifié pour obtenir les preuves.

## Ce que cela dit du maintien 3D

Chaque IIS présenté inclut trois faces matérielles, mais cela ne démontre pas que supprimer une seule composante du maintien 3D suffirait. Un contrôle arithmétique supplémentaire combine seulement `y.lower` et `z.upper` avec les bornes de commande : il prouve déjà une contradiction aux deux pieds pour les échelles 0.5, 1 et 1.5. Les marges après tolérances vont de 0.005511829588 à 0.012020772315 m/s. L’absence de contradiction de cette combinaison particulière à l’échelle 0 ne prouve pas sa faisabilité.

La limite de `leftAnkle.x` contribue à ces conflits, mais aucune « cause unique axe 36 » n’en découle. Les trois dimensions du point, le mouvement de base utilisé dans l’assemblage et plusieurs bornes de vitesse de consigne interviennent ensemble. Un simple passage de la tangente x en objectif souple n’est donc pas une réparation générale prouvée.

## Hypothèse de vitesse à séparer

Dans `geometric_tasks`, les colonnes du Jacobien géométrique proviennent des poses mesurées. Pour une vitesse articulaire physique `v` et le mouvement de base `b`, `Jq·v + Jbase·b` décrit une vitesse instantanée du point matériel. La cible de maintien vise le retour à l’ancre retenue sur un tick.

Dans `coordinated_runtime`, la variable résolue est au contraire une **vitesse de consigne** `u` : elle fait évoluer la position cible, puis donne `target_velocity` à un moteur `ForceBased`. L’installateur calcule explicitement une autre grandeur `rate` depuis les vitesses angulaires observées, et utilise `stiffness*error + damping*(target_velocity-rate)`. Le code distingue donc déjà les deux quantités au niveau moteur. Aucune identité instantanée `u=v` n’est établie par les certificats de commandes f32, par la limite de slew ou par la parité réelle/copie.

Substituer `u` à `v` dans une égalité dure de vitesse de contact ajoute une hypothèse d’effet du moteur. La parité prouve que les deux exécutions reproduisent cette formulation ; elle ne prouve pas cette hypothèse physique. Les incompatibilités archivées sont prouvées pour cette formulation, mais leur attribution exclusive à cette substitution n’est pas démontrée : la définition du maintien, les états de base et les bornes doivent également être examinés.

Une séparation cohérente consiste à garder dures les limites des **commandes** et à conserver les identités/ancres comme obligations de **support mesuré ou prédit**. Un étage purement géométrique peut produire une proposition et ses résidus. Il ne doit pas transformer ces résidus en certificat de charge, de non-glissement ou de réussite, ni admettre automatiquement la proposition en physique. Pour promouvoir une obligation en contrainte prédictive, il faut expliciter le lien commande→mouvement prédit, ou refuser cette admission quand ce lien manque. La racine ne devient pas une action optimisable libre, et aucune ancre ne doit être actualisée implicitement pour masquer un glissement.

## Reproduction et tests

- [Extraits des quatre problèmes](archived-support-problems.json), issus exclusivement de la décompression contrôlée par SHA du rapport clos `5ed160d4c9c7d4361ec5d285268a9aa694bc797a419768cfaac0370d46313711`.
- [Script exact](minimal_support_conflicts.py) et [huit IIS / 64 témoins](minimal-support-conflicts.json).
- [Contrôle à deux composantes](two-component-conflicts.json).
- [Sept tests purs](test_minimal_support_conflicts.py), [journal](minimal-conflicts-tests.log) : reproduction depuis les coefficients, validation duale et des témoins, corruption refusée, tolérance d’origine conservée, portée des témoins explicitement limitée, contre-exemples à deux composantes.

SciPy et NumPy sont présents dans l’environnement mais n’ont été ni importés ni utilisés. Aucun ajout de dépendance, aucun appel LP ou QP, aucun `Model`/`Simulation`, aucune FK ni intégration native. Les seules résolutions effectuées sont les 64 petits systèmes linéaires rationnels de preuve ; les tests les reproduisent. Aucun registre clos, manifeste ou source Rust n’a été modifié par cette analyse.
