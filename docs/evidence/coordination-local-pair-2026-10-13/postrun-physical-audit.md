# Audit physique après clôture de la paire locale — 13 octobre 2026

**Le témoin A échoue au diagnostic local ; aucune histoire réelle coordonnée B n'a commencé.** A termine ses 960 pas autorisés, mais tombe et reste au sol. Les quatre copies de B terminent chacune 16 pas, puis leur problème de tâches contraintes est refusé avant l'intégration suivante. Ce résultat ne démontre pas l'impossibilité physique ou anatomique du relevage.

Audit indépendant par lecture des JSON/JSONL archivés et de leurs octets sérialisés, comparaison de nombres/bits, hachage et arithmétique rationnelle des matrices déjà enregistrées. **Zéro `Model`, `Simulation`, monde, FK, worker, rendu ou sous-pas physique supplémentaire.** Aucun fichier gelé ni registre n'est modifié. Les grands fichiers sont référencés par leur nom logique et leur SHA brut ; leur stockage compressé est décrit dans [archive-index.json](archive-index.json).

## Étendue effectivement observée

| Opération | Intégrations documentées | Portée |
| --- | ---: | --- |
| Admission native A et B, copie/réel | 16 | Deux paires de 4 + 4 pas ; 16,67 ms par instance. |
| Histoire réelle A | 960 | 4 s depuis la fixture `landed-crouch-left`. |
| Quatre prévisions de B | 64 | 16 pas chacune, soit 66,67 ms ; aucune n'atteint l'horizon demandé de 480 pas/2 s. |
| Histoire réelle B | 0 | État initial conservé ; aucun facteur retenu, aucun ACK de reprise. |

Les traces contiennent exactement **1040 observations après intégration** : 16 de vérification, 960 de A, 64 des copies. Le reçu B et son journal d'admission déclarent séparément 0 admis/0 achevé. Le registre parent clos conserve néanmoins `actualNativeStepsExact=false` et la borne 1040–2000, car le reçu final B n'a pas été accepté avant fermeture. Cette reconstruction documentaire ne réécrit pas ce registre et ne rembourse pas les 3856 pas réservés. Voir [postrun-budget-audit.md](postrun-budget-audit.md).

La fixture est identique pour les deux histoires, sans reset en cours d'histoire : bassin initial `(0 ; 0,976323664 ; 0)` m, genoux proches de 0,66 rad. Il s'agit de cette pose historique peu accroupie, pas d'un nouvel impact initial ni d'un accroupissement profond reconstruit.

## A : absence de talons porteurs et chute

Sur les **960 observations de A**, les charges postérieures des deux hindfeet restent nulles, ainsi que leurs compteurs propres de persistance. Aucun échantillon ne satisfait simultanément les critères locaux ; le maximum de stabilité qualifiante vaut **0 pas sur les 132 requis**. Les valeurs `−0.0` archivées sont des zéros de sommation, pas une charge négative.

Le tableau reprend les champs `support` et `localDiagnostic` des observations natives, sans recalcul géométrique. La fraction est la charge courante des pieds persistants divisée par le poids observé, 708,281982 N.

| Temps | Hauteur bassin | CoM `(x ; y ; z)` en m | Charge pieds / poids | Max. vitesse des 25 corps | Max. vitesse angulaire |
| ---: | ---: | --- | ---: | ---: | ---: |
| 0,05 s | 0,968148 m | `(−0,000044 ; 1,001964 ; 0,036138)` | 65,54 % | 0,331947 m/s | 2,349132 rad/s |
| 0,50 s | 0,945947 m | `(−0,004203 ; 0,967288 ; 0,068220)` | 101,70 % | 0,579803 m/s | 1,088319 rad/s |
| 1,00 s | 0,912366 m | `(−0,008616 ; 0,910907 ; 0,280572)` | 87,31 % | 1,333979 m/s | 0,988174 rad/s |
| 2,00 s | 0,136897 m | `(−0,033401 ; 0,110645 ; 1,127182)` | 29,02 % | 0,027706 m/s | 0,266114 rad/s |
| 4,00 s | 0,133282 m | `(−0,033651 ; 0,116246 ; 1,123794)` | 30,90 % | 0,025183 m/s | 0,146333 rad/s |

À 4 s, le bassin est à `(−0,032493 ; 0,133282 ; 1,089686)` m et le torse à `(−0,046109 ; 0,102018 ; 1,487102)` m. La vitesse du CoM est `(−0,010059 ; 0,003345 ; −0,001438)` m/s. Le calme final est celui d'un corps couché, pas une stabilité debout.

Les charges natives instantanées de segment à ce même pas valent environ **425,78 N au torse, 58,94 N à la tête, 111,58 N à l'avant-pied gauche et 107,29 N à l'avant-pied droit**. La main gauche porte alors 3,56 N mais n'a que 9 pas de persistance ; le diagnostic non-pied persistant final identifie torse et tête. La charge non-pied courante totale rapportée atteint 489,08 N. Ces données proviennent de la frontière de sous-pas ; les contacts du snapshot final de tick utilisent une autre agrégation et ne doivent pas être mélangés avec elles.

Le premier appui non-pied persistant apparaît au pas **402, soit 1,675 s**, sur le lumbar. Au cours de l'histoire, les segments non-pieds persistants comprennent lumbar, torse, tête et les deux mains. Les pics de mouvement sont **5,528247 m/s pour la tête à 1,583333 s** et **91,631919 rad/s pour le cou à 1,5875 s**. Le minimum de hauteur du bassin vaut 0,104904 m. Aucun refus d'intégrité original n'est enregistré dans A ; cela ne fait pas réussir les critères plus stricts de stabilité locale.

À titre de décomposition, A présente 243 échantillons avec charge pieds ≥80 % du poids, 57 avec CoM dans le support, 21 avec capture dans le support, et 522 avec calme des 25 corps. Aucun de ces sous-critères isolés ne compense l'absence totale de talons porteurs ; `passed=false`, aucune récupération et aucun tick debout qualifié.

## B : refus des quatre prévisions, avant toute reprise réelle

Les quatre facteurs figés produisent les observations suivantes au pas 16. Tous ont encore **0 N sur chaque talon**, aucune persistance de talon, CoM et capture hors du support admis, et aucun appui non-pied persistant. Les deux patches retenus sont ceux des avant-pieds, acquis au pas 12 et encore présents au pas 16.

| Facteur | Charge pieds / poids | Max. vitesse au pas 16 | Max. vitesse angulaire au pas 16 | Dérive tangentielle maximale des deux patches |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 85,672 % | 0,435815 m/s | 3,966398 rad/s | 0,3502 mm |
| 0,5 | 85,480 % | 0,453821 m/s | 3,520738 rad/s | 0,4331 mm |
| 1 | 85,494 % | 0,452102 m/s | 3,427422 rad/s | 0,4410 mm |
| 1,5 | 85,886 % | 0,449988 m/s | 3,619262 rad/s | 0,4491 mm |

Le bassin se trouve encore vers 0,9624–0,9626 m et le CoM vers 0,9962–0,9963 m. Ces 66,67 ms ne démontrent ni stabilisation ni impossibilité de stabilisation. Les charges suffisantes des seuls avant-pieds ne satisfont pas le diagnostic de talons et de calme.

Le refus survient à `nativeSubstep=16`, avant installation de la commande suivante et avant un dix-septième pas physique : `Direct common solve: Initial feasibility`, 512 cycles, violation maximale de borne environ 0,147509 à 0,198097 pour une tolérance de `1e−8`. L'archive marque `status="refused before installation"`, `stage="direct continuous solve"`. Ce n'est pas le précédent défaut de compilation des commandes f32 : cette fois le problème continu lui-même est contradictoire.

### Contradiction des problèmes archivés, contre-vérifiée exactement

La contre-revue numérique a fourni une combinaison des trois contraintes matérielles d'un pied et des bornes de coordonnées. Elle a été vérifiée indépendamment ici avec `fractions.Fraction` appliqué à chaque f64 archivé, sans reconstruire de géométrie ni relancer le solveur. Pour les lignes `x`, `y`, `z` d'un patch, les indices annulés sont `i=32+7·side`, `j=34+7·side` :

```
t = (y_i*x_j - y_j*x_i) / (z_i*x_j - z_j*x_i) > 0
u = (t*z_j - y_j) / x_j > 0
r = y - t*z + u*x                  # r_i = r_j = 0 exactement
minimum requis = lower_y - t*upper_z + u*lower_x
maximum dans les bornes = Σ r_k * (upper_k si r_k >= 0, sinon lower_k)
```

Dans les huit combinaisons, le minimum requis excède strictement le maximum possible :

| Facteur | Écart pied gauche | Écart pied droit |
| ---: | ---: | ---: |
| 0 | 0,047612273 m/s | 0,043873813 m/s |
| 0,5 | 0,060477372 m/s | 0,057595604 m/s |
| 1 | 0,063852371 m/s | 0,062678492 m/s |
| 1,5 | 0,063934769 m/s | 0,060499418 m/s |

Même en accordant la tolérance de `1e−8` à chaque contrainte et borne, sa contribution amplifiée `1e−8·(1+t+u+Σ|r_k|)` reste ≤`1,508696e−8` m/s, très inférieure aux huit écarts. **Les quatre QP instantanés archivés sont donc infaisables sous leurs tâches et bornes gelées.** Davantage d'itérations ne résoudrait pas cette contradiction. Ce certificat porte sur cette formulation cinématique, cet instant et ces bornes ; il ne prouve aucune impossibilité anatomique, aucune impossibilité d'une autre trajectoire ou nécessité universelle de mains/genoux.

Les booléens génériques de sortie doivent être interprétés avec prudence : `support_feasible=true` pour A signifie absence de refus technique, pas réussite de son support local. Pour les prévisions, `diagnostic_present=false` et `integrity_ok=false` viennent de l'échec global `outcome(false, false)` ; les 16 échantillons par branche existent bien. La cause précise est le refus `ownership` du QP, pas un diagnostic absent ou une collision invalide démontrée.

## Parité native et contrat d'actionnement

Les deux cas de vérification A/B comportent chacun un état initial, quatre frames après intégration et une frontière posttick. Les **12 paires** ont été comparées intégralement : tableaux d'octets du monde, de l'état et de la mémoire externe, bits des corps et moteurs, horloges, contacts, cibles et persistance. **Aucun écart.** La mémoire inclut Recovery complète, `modeSignature`, et `DiagnosticMemory` avec diagnostic, dernier échantillon et vitesse du CoM. Au posttick, `tick=1`, `substep=4` et l'échantillon est bien celui du pas 4. Le monde conserve le masque exact `170141183460469231731687303715884105728 = 2^127`, sans conversion intermédiaire destructrice en `Value`.

Cette admission concerne le préfixe commun de 16,67 ms, avant acquisition de patch au pas 12 ; elle ne qualifie pas à elle seule un tick contraint à 60 Hz. Les copies exploratoires atteignent ensuite le premier tick contraint puis refusent le suivant, sans histoire réelle B correspondante.

Sur les **1040 frames et 47840 installations d'axes** archivées (46 axes × 1040), la lecture indépendante constate :

- tous les axes anatomiques actifs emploient `ForceBased` ; les gains stiffness/damping correspondent bit pour bit aux formules du profil gelé et à la somme anatomique de 2800 Nm ;
- positions et vitesses effectivement installées correspondent bit pour bit aux commandes archivées ; les vitesses sont les différences finies exactes f32 entre deux références consécutives, divisées par le pas natif f32 ;
- les `max_force` installés, lus directement dans les moteurs, restent ≤ leur cap anatomique ; ratio maximal 1,0 et somme comprise entre 280,000000358 et **760,0 Nm**, jamais supérieure au plafond de 760 Nm ;
- aucune force ou aucun couple direct de bassin non nul dans les diagnostics ; aucune récupération enregistrée ;
- les empreintes courantes des rôles gelés modèle, profil et fixture correspondent toutes au manifeste.

La vérification porte sur les caps et gains installés, pas sur une assimilation de l'action `ForceBased` à un couple idéal. Les charges de contacts courantes et les journaux CCD restent des observations distinctes ; aucune impulsion ancienne n'a été ajoutée aux charges de talon pour satisfaire un seuil.

## Identité des preuves lues

| Fichier logique brut | SHA-256 |
| --- | --- |
| `000-native-verification.json` | `66d4e802e3b474d9065fe32fc49f6915cae64713eb34476227d2f20ffda2d02e` |
| `001-A.json` | `e10a1cdd539bdec5e0d445a63ef27ace3fa42143b43f22c26a194f080b13538c` |
| `001-A.states.jsonl` | `8338425db8c92038b98b89ae22c4e6f366a6ebd71b000aa615700acbd530bdd8` |
| `002-B-planning-000.json` | `5ed160d4c9c7d4361ec5d285268a9aa694bc797a419768cfaac0370d46313711` |
| `002-B.json` | `72ec39bf64e1f637cc1ed1099eff42cf8836ade2c21bb99dc12c843f502aa723` |
| `002-B.states.jsonl` | `9f152e6402331eda7edfa97a76cf7b352064ed26f632af91b4ed1dcd46d6d667` |

Le protocole reste clos. Aucun résultat local positif, relevage ≤25 s, stabilité debout d'une seconde ou qualification des 24 cas n'est établi. La décision sur la continuation complète reste documentaire ; aucune nouvelle histoire n'est autorisée par cet audit.
