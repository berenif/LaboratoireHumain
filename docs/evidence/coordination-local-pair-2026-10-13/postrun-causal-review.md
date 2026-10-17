# Revue causale après fermeture de la paire locale

Le lot clos ne fournit aucune amélioration A/B mesurée : A a exécuté ses 960 pas sans succès local, les quatre copies B ont exécuté 16 pas chacune, puis toutes ont refusé la commande suivante ; B réel a exécuté zéro pas. Le refus est celui des contraintes numériques du contrôleur. Il ne démontre ni une défaillance anatomique ni une impossibilité physique générale.

Cette revue utilise exclusivement les archives, le code figé et l’arithmétique rationnelle exacte. Aucun worker, solveur, modèle, FK ou pas natif supplémentaire n’a été exécuté. Sources principales : [index des archives : rapport des quatre copies et traces B](archive-index.json), [rapport B](native-run/002-B.json), [protocole](native-run/protocol.ndjson). SHA256 du rapport brut de planification : `5ed160d4c9c7d4361ec5d285268a9aa694bc797a419768cfaac0370d46313711`.

## Refus observé et axe 36

Les quatre `terminalOwner.numericAttempt` indiquent `nativeSubstep=16`, `status="refused before installation"`, `stage="direct continuous solve"`. Le refus se produit au tick 4, sous-pas 0, avant le dix-septième appel Rapier. La dernière commande installée est celle d’indice 15. Les 64 admissions/completions des copies sont exactes ; 80 places locales ont été débitées, dont 16 places du tick refusé abandonnées sans remboursement. La réservation parent de 1 920 demeure intégralement chargée.

Il s’agit chaque fois d’un refus de **projection initiale de faisabilité après 512 cycles**, avant les pivots Newton d’optimisation et avant une nouvelle compilation f32. Les matrices originales, les bornes et `previousQ` sont archivés. Les champs `directAttempt` et `guardedAttempt` sont absents à cette tentative : le repli conservateur d’arrondi n’a pas été atteint.

L’indice zéro-base 36 désigne **`leftAnkle.x`, segment 17**, dans le vecteur des 46 vitesses de commande. Ce n’est pas l’indice d’une contrainte matérielle. L’axe 43 est `rightAnkle.x`, segment 22. Les boîtes sont valides : aucune borne inversée, non finie ou extérieure à `[-1,1]` n’est présente.

| Échelle | `previousQ` gauche (rad) | Borne supérieure axe 36 (rad/s) | Violation finale rapportée (rad/s) |
|---:|---:|---:|---:|
| 0 | 0.336176335811615 | 0.773370225626293 | 0.147508948492261 |
| 0.5 | 0.336472928524017 | 0.755574663810263 | 0.187358589266892 |
| 1 | 0.336666405200958 | 0.743966063799244 | 0.197825151528070 |
| 1.5 | 0.336666405200958 | 0.743966063799244 | 0.198096713144863 |

Chaque borne est exactement reproduite par `(q_max − previousQ)/(4×dt)`, avec le maximum anatomique f32 `q_max=0.3490658402442932 rad` et `4×dt=0.01666666753590107 s`. Elle assure que les quatre commandes suivantes restent dans la limite anatomique. Le résidu désigne la violation maximale de l’itéré projeté final ; cet itéré rejeté n’est pas une commande appliquée. L’axe 36 n’est donc pas désigné comme une cause unique : la contradiction porte sur une combinaison de tâches et de bornes.

Les deux patches matériels, acquis au pas 12 et encore en mode `Hold`, sont conservés aux avant-pieds (segments 19 et 24). Leurs trois bandes imposent une vitesse compatible avec le retour à l’ancre d’origine sur un tick, compte tenu du mouvement de base observé, à ±0.005 m/s. Ces bandes et les limites des commandes doivent être satisfaites ensemble. Une très faible variation finale de l’itéré (`1.42e−14` à `2.75e−14`) ne constitue pas un certificat : les corrections Dykstra changent encore d’environ 0.155 à 0.208 et la violation demeure de 0.148 à 0.198.

## Incompatibilité démontrée indépendamment du solveur

Le message générique « infeasible or not converged » ne permettrait, à lui seul, de trancher. Les coefficients archivés permettent ici une preuve plus forte : **les quatre systèmes linéaires instantanés sont incompatibles avec leurs boîtes**, même en accordant partout leur tolérance originale de `1e−8`.

Pour un patch, notons `x,y,z` ses trois rangées, et `Lx,Ly,Uz` les bornes correspondantes. Pour les indices `(i,j)=(32,34)` à gauche, `(39,41)` à droite, posons :

```
t = (y[i]*x[j] - y[j]*x[i]) / (z[i]*x[j] - z[j]*x[i])
u = (t*z[j] - y[j]) / x[j]
r = y - t*z + u*x
```

Dans les huit cas, `t>0`, `u>0`, et `r[i]=r[j]=0` exactement. Toute vitesse admissible `v` devrait vérifier simultanément :

```
r·v ≥ Ly - t*Uz + u*Lx
r·v ≤ Σ r[k] * (upper[k] si r[k]≥0, sinon lower[k])
```

La première borne dépasse pourtant la seconde. Les valeurs ci-dessous sont arrondies pour lecture ; les signes et opérations du certificat utilisent les fractions exactes des coefficients f64 archivés.

| Échelle | Écart contradictoire gauche (m/s) | Écart contradictoire droit (m/s) |
|---:|---:|---:|
| 0 | 0.047612272929207382 | 0.043873812724341793 |
| 0.5 | 0.060477372329590169 | 0.057595604428008186 |
| 1 | 0.063852371497323382 | 0.062678491655579469 |
| 1.5 | 0.063934769369670214 | 0.060499417545024040 |

Élargir **chaque** bande et **chaque** borne de coordonnée de `ε=1e−8` réduit cet écart d’au plus `ε(1+t+u+Σ|r[k]|)`, soit moins de `1.50870e−8 m/s`. Les huit contradictions restent strictement positives. Cette preuve ne dépend ni des 512 cycles ni de l’objectif souple. Augmenter seulement le nombre de cycles ne rendrait donc pas ces mêmes systèmes admissibles. Un repli f32 qui resserre les contraintes ne peut pas non plus restaurer leur faisabilité.

Reproduction : [script purement arithmétique](postrun_qp_certificate.py), [huit certificats avec numérateurs/dénominateurs exacts](postrun-qp-certificates.json). Le rapport de planification est archivé en plusieurs parties gzip : décompresser chaque partie et concaténer leurs contenus dans l’ordre de l’[index](archive-index.json), vérifier le SHA256 brut indiqué, puis fournir le JSON reconstitué à `--input` ; `--output` exige un nouveau fichier. Deux relectures indépendantes ont reproduit les huit contradictions depuis le rapport brut, sans reconstruction géométrique ni résolution d’un QP.

## Pourquoi les preuves précédentes restent compatibles

Les seize pas de vérification de ce lot sont quatre pas sur chacune des instances A-copie, A-réel, B-copie et B-réel. Ils établissent la parité de ces fenêtres ; ils ne parcourent pas seize pas successifs de décision B. Le lot numérique antérieur de 44 pas avait, lui, vérifié le préfixe puis les commandes d’indices 12 à 15 sur chaque échelle. Il s’arrêtait à l’état physique complété 16, avant la décision suivante.

Les nouvelles traces confirment précisément cette limite : la décision d’indice 12 est certifiée, avec 8/8/12/12 itérations et quatre commandes f32 par cas, puis les pas physiques 13 à 16 sont réalisés. Au prochain tick, les états observés et la marge anatomique restante donnent un **nouveau** problème, incompatible. Il n’y a donc pas de contradiction entre la parité vérifiée et l’échec de cette prévision de 480 pas : chaque copie atteint seulement `16/240 ≈ 0.0667 s`, puis refuse avant le pas 17. Aucun des quatre horizons de deux secondes n’est disponible à la sélection.

Les valeurs booléennes toutes fausses du résultat de planification agrègent ce refus. Elles ne signifient pas que quatre accidents physiques se sont produits. Les échantillons finaux existent et portent notamment des charges de talon nulles et `passed=false`, mais ces copies s’arrêtent très avant le délai d’acquisition de deux secondes. L’histoire B n’a pas été appliquée ; aucun effet causal favorable ou défavorable de B par rapport à A ne peut être mesuré avec ce lot. La continuation complète de huit secondes reste refusée, sans nouvelle autorisation implicite.

## Préparation possible d’un autre protocole

Conserver ces quatre problèmes comme fixtures numériques d’infaisabilité, avec un diagnostic qui distingue explicitement contradiction certifiée et absence de convergence. Auditer hors physique la provenance des bandes matérielles, la compensation du mouvement de base, le lien entre vitesses de cibles commandées et vitesses articulaires observées, ainsi que la consommation de marge anatomique entre les décisions 12 et 16. Il faut ensuite justifier une formulation ou une famille de commandes viable avant de proposer un autre protocole.

Cette recommandation n’autorise ni changement de gains, forces, tolérances ou ancres, ni essai supplémentaire. Elle ne transforme pas une route géométrique statique en preuve dynamique. Les sources, le manifeste et les registres du lot clos demeurent inchangés.
