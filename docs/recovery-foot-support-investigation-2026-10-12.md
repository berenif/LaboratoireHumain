# Appuis du pied friction et segmentation du pied

**Priorité à la pose et à la commande du pied existant dans le modèle Rust A/B.**
Les talons sont présents
dans la géométrie, le pied est déjà articulé et le frottement pied-sol vaut 4.
Dans les premiers états A/B, les avant-pieds pivotent avec peu de glissement,
tandis que les talons restent en l'air. Le frottement approche ensuite sa borne
dans certains contacts de B ; son rôle dans la suite reste à isoler. Un segment
supplémentaire n'est pas encore justifié par un passage anatomique nécessaire.

Base locale et distante vérifiée : `e8205c8665fec3eaab8dc6a71c554b8fdc6defd1`.
Cette investigation ajoute de la documentation uniquement. Aucun code physique,
paramètre, contact ou seuil changé ; zéro intégration native supplémentaire.
Les registres A/B, P1 et 96 histoires restent fermés et conservent leurs SHA,
consignés dans le [reçu numérique](recovery-foot-support-investigation-2026-10-12.json).

## Articulations et géométrie existantes

Chaque côté possède la chaîne `Shin → Ankle → Foot → Forefoot`, avec trois axes
commandés après le tibia. `Foot` est l'arrière-pied rigide, talon compris ;
`Forefoot` est un corps articulé distinct. Le nom `multi-axis` du profil Foot
ne signifie pas trois axes libres : seul Z est déclaré, les autres sont verrouillés.

| Liaison | Axe dans le repère articulaire | Limites | Cap anatomique |
|---|---|---|---:|
| Shin vers Ankle | X sagittal | −45° à +20°, deux côtés | 110 Nm |
| Ankle vers Foot | Z, roulis | Gauche −10° à +25° ; droite −25° à +10° | 50 Nm |
| Foot vers Forefoot | X sagittal | −20° à +45°, deux côtés | 35 Nm |

Les repères parent/enfant de l'Ankle et du Forefoot tournent de 180° autour de Y.
L'axe X local correspond donc à l'opposé de X du parent dans la pose alignée.
Les coordonnées natives utilisent `2 atan2(q_i,q_w)`, pas une décomposition Euler.
Dans le repère corporel neutre, +Y pointe vers le haut et +Z vers l'avant.
[Modèle](../rust/crates/model/data/canonical-v1.json),
[convention de coordonnées](../rust/crates/sim/src/measurement.rs).

L'arrière-pied mesure 106 × 90 × 150 mm. Ses sommets locaux s'étendent de
Z=−75 à +75 mm ; toute la face inférieure est à Y=−45 mm. Sa partie postérieure
et sa semelle existent donc réellement. L'avant-pied mesure 108 × 80,1 × 120 mm,
avec sa face inférieure également à Y=−45 mm. Chaque maillage contient 12 sommets
et 20 triangles et devient un collider convexe. À la pose neutre, l'ensemble
s'étend de 40 mm derrière l'axe de cheville à 230 mm devant, soit 270 mm.
La masse est 0,55 kg pour Foot, 0,25 kg pour Forefoot et 0,20 kg pour Ankle.
Ces dimensions décrivent le modèle ; aucun étalon externe ne permet ici de
qualifier ses volumes de disproportionnés.

Toutes les paires parent/enfant directes sont exclues des collisions. Les
exclusions pied/tibia incluent aussi les contacts de la propre chaîne nécessaires
au modèle. Les pieds gauche/droit restent en collision ; aucune exclusion nouvelle
ne leur est appliquée. [Construction et filtre](../rust/crates/sim/src/lib.rs).

Le marqueur `localZ < −20 mm` utilisé pour le talon est une région diagnostique
de Foot, pas un nouveau critère de récupération. La qualification originale
n'exige pas ce marqueur : une route physiquement stable sur les avant-pieds
pourrait rester admissible. Aucun talon n'est crédité simplement parce qu'un
contact porte le nom Foot.

## Friction réellement utilisée

Le sol plat vaut μ=4, avec la combinaison Average par défaut. Les corps utilisent
Min, qui a priorité sur Average dans ce fork. Pour des coefficients positifs,
le coefficient effectif est donc le minimum des deux matériaux.

| Paire | Coefficient effectif dans A/B |
|---|---:|
| Sol et Foot ou Forefoot | 4 |
| Sol et main, avant-bras, tibia ou cheville | 1,2 |
| Sol et autres régions corporelles | 0,45 |
| Foot gauche et Foot droit | 4 |

Les surfaces d'environnement utilisent le coefficient de leur définition, avec
Average par défaut : le pied impose encore Min. A/B exclut environnement,
striker et interaction. Le filtre des contacts ne modifie pas leur friction.
[Matériaux](../rust/crates/sim/src/lib.rs),
[combinaison](../rust/vendor/rapier3d/src/dynamics/coefficient_combine_rule.rs),
[affectation au manifold](../rust/vendor/rapier3d/src/geometry/narrow_phase/pair_update.rs).

Le profil épinglé est Coulomb. Le solveur borne, pour chaque point, la norme
des deux impulsions tangentielles à `μ × impulsion normale`, sans adhésion ni
traction quand l'impulsion normale est nulle. Il ne s'agit pas de la friction
avec torsion simplifiée. [Résolution Coulomb](../rust/vendor/rapier3d/src/dynamics/solver/contact_constraint/contact_with_coulomb_friction.rs),
[borne du vecteur tangent](../rust/vendor/rapier3d/src/dynamics/solver/contact_constraint/contact_constraint_element.rs).

Augmenter uniquement μ du pied au-delà de 4 laisse le coefficient pied-sol à 4,
mais peut changer celui entre les deux pieds. La hausse du coefficient du sol
seul laisse également cette paire à 4. Une comparaison pied-sol exige donc
d'isoler son coefficient effectif, sans changer simultanément pied-pied ou
les autres surfaces. Un coefficient supérieur ne place pas un talon en l'air
dans le sol ; il peut seulement modifier les efforts et donc le mouvement ultérieur.

## Rotation glissement et hauteur du talon

À 50 ms, A et B sont encore identiques. Le centre des avant-pieds se déplace
horizontalement à environ 0,113–0,114 m/s, mais les points porteurs ont une vitesse
reconstruite d'environ 0,005 m/s. Leur rotation est proche de 2,4 rad/s. Ce contraste
indique surtout une rotation autour de l'appui, avec un faible glissement résiduel,
plutôt qu'un pied qui patine librement. Seuls les avant-pieds portent ; les
arrière-pieds sont inclinés de 10,7° et leurs sommets postérieurs restent à 46,1 mm.

| État de B | Talon gauche au-dessus du sol | Talon droit | Inclinaison arrière-pied G/D |
|---|---:|---:|---|
| 50 ms | 46,1 mm | 46,1 mm | 10,7° / 10,7° |
| 150 ms | 23,9 mm | 24,2 mm | 9,25° / 9,35° |
| 550 ms | 15,4 mm | 20,3 mm | 5,85° / 7,08° |

Les chevilles atteignent environ 20° à 100–150 ms, pendant que les arrière-pieds
restent inclinés. À 300 ms, les avant-pieds sont proches de l'horizontale, mais
les arrière-pieds restent autour de 7°. L'articulation distale joue donc déjà
son rôle sans poser le talon.

[Profils du pied gauche à 50 et 550 ms](media/foot-support-investigation-2026-10-12/recorded-foot-side-views.png)
présente les maillages et points porteurs archivés. Le rendu est géométrique,
sans nouvelle intégration et sans déplacement d'une pose enregistrée.

Une reconstruction purement géométrique depuis le tibia mesuré à 50 ms, avec
la cheville portée à +20°, laisse au mieux le talon gauche à 35,4 mm en variant
le roulis. Même un minimum optimiste qui libère le roulis au-delà de ses limites
ne descend pas plus bas. À 150 ms, cette borne est encore près de 14 mm. Il faut
changer la pose du tibia/de la chaîne entière, et non seulement ajouter une petite
correction locale au pied. Cette comparaison de poses ne démontre aucune
trajectoire dynamique. La peau de contact de 4 mm et la prédiction de 2 mm du
profil ne rendent pas porteur le talon observé à 46 mm.

Le journal fournit des impulsions tangentielles non nulles. À 50 ms, le ratio
agrégé `||ΣJt|| / (4 ||ΣJn||)` des avant-pieds vaut environ 0,024–0,028. À 300 ms,
B atteint environ 0,915 sur Foot gauche et 0,982 sur Foot droit ; le maximum
Foot droit entre 0 et 600 ms est 0,9924. A atteint au maximum 0,793 sur ce corps
sur la même période. B sollicite donc davantage le frottement, sans le modifier.
Cela rend plausible un rôle du frottement dans la suite, sans établir qu'il est
insuffisant : plusieurs points restent presque immobiles et les talons sont hauts.

Les vitesses sont reconstruites depuis les poses et vitesses de corps après le
pas, avec leur centre de masse local, pas depuis un suivi du même point matériel
avant/après résolution. Les ratios sont par corps et intervalle CCD ; des efforts
opposés peuvent s'annuler, donc un petit ratio ne prouve pas que tous les points
sont loin de leur limite. Il manque un journal par point de μ effectif, impulsions
normale/tangente, bras de levier et vitesses relatives avant/après résolution pour
trancher glissement persistant, adhérence et basculement. Cette instrumentation
ciblée reste distincte de l'audit exhaustif du solveur différé.

## Collisions observées et passage géométrique

Dans B, les poses archivées présentent un chevauchement pied-pied vers
7,55–7,7 s. Le maximum natif cumulé d'interpénétration est 4,286 mm, sous le
seuil conservé de 5 mm. À 10 s les pieds sont séparés. Le bassin avait déjà
passé sous 0,56 m à 1,396 s ; cette collision est donc ultérieure à la perte
initiale de hauteur. Elle peut gêner la suite, mais ne déclenche pas cette perte.
Le refus final de B reste celui de la pose debout à 20,8875 s, distinct du refus
pied-pied historique à 4,883 s. Le passage géométrique adapté avait été admissible
sur 402 positions à 0°/60° ; cela ne certifie ni son suivi physique ni les charges.
[Résultats A/B](rust-getup-ab-results-2026-10-12.md),
[géométrie et référence adaptées](rust-recovery-coordinated-retarget-2026-10-11.md).

## Options et tests discriminants proposés

Les options ci-dessous sont des alternatives à décider, pas trois campagnes
autorisées. **Le premier test choisi serait limité à deux histoires de 26 s,
12 480 pas natifs et cinq minutes murales**, un processus mono-thread. Les tests
avec intégration, témoins et copies consomment tous ce plafond. Manifeste,
paramètres et critère de sélection figés avant le premier lancement. Préparation
proposée plafonnée à une journée de travail, soit huit heures : sans exécuteur,
instrumentation et candidat complet prêts à cette limite, arrêt sans histoire.
Aucun réglage entre histoires, répétition d'un échec ou renouvellement pour un talon
acquis. Le relevage complet avec terminal original reste le but physique.
Sans succès complet, clôture et décision nouvelle ; aucun budget créé ici.

1. **Commande du pied existant — priorité.** Comparer B à une seule correction
   coordonnée tibia/cheville/arrière-pied/avant-pied, préparée sur un parcours
   entier. Mesurer acquisition réelle de la région arrière et transfert de
   charge, sans imposer une pause basse ou qualifier le talon comme réussite.
   Même géométrie, friction, caps, horloge et terminal. Une demande de talon est
   une cible articulaire ; aucun verrouillage ou contact fabriqué. Refus si la
   correction exige hors-limites ou assistance au bassin. Le témoin vérifie
   également la neutralité de la nouvelle instrumentation, dans les deux histoires.

2. **Friction isolée — priorité conditionnelle.** Choisir avant lancement une
   comparaison du μ effectif pied-sol 4 contre 3, pour tester si l'adhérence
   excessive gêne le pivot/la décharge. Tous les autres paramètres et coefficients
   de paire, notamment pied-pied, restent identiques. Enregistrer les données par
   point citées ci-dessus dès le premier pas. Une augmentation n'est pas proposée
   sans données de matériau et glissement persistant à la borne ; une valeur
   supérieure arbitraire ne serait pas une démonstration de force humaine.
   Le changement de matériau est un modèle expérimental, pas une qualification
   automatique du profil original. Refus immédiat aux seuils d'intégrité originaux ;
   aucune deuxième valeur tentée après échec.

3. **Segment supplémentaire — prérequis géométrique seulement pour l'instant.**
   Au plus 500 inspections cinématiques hors monde et 30 minutes, zéro intégration : comparer
   une configuration nécessaire au transfert pour le modèle actuel et pour un
   segment précisément identifié, avec dimensions et amplitude justifiées.
   Si la séparation ne change que l'identification des contacts, elle n'ajoute
   aucune capacité mécanique. Un corps réellement articulé apporte une déformation,
   une inertie et un degré de liberté supplémentaires ; il peut changer la
   répartition de pression, mais aussi compliquer la commande. Ajouter des orteils
   en aval ne déplace pas géométriquement le talon quand la chaîne amont reste fixe.
   Une compliance de semelle ou un mouvement d'arche doivent être justifiés, pas
   utilisés comme une seconde cheville contournant 20°.

   Une segmentation physique doit conserver masse totale, centre/inertie de
   référence et enveloppe d'effort, déclarer les énergies élastiques, et définir
   de nouveaux contacts/limites sans soutien artificiel. Aucun moteur ni énergie
   initiale gratuits. Le programme et plusieurs tableaux sont explicitement limités
   à 25 corps : ajouter un corps exige une migration de modèle/fixtures/contrats,
   puis une qualification distincte. Aucune comparaison native de cette option
   n'est admissible avant cette décision. Si elle devient justifiée, son protocole
   serait présenté séparément avec le même petit plafond de comparaison, sans
   réutiliser les soldes des registres clos.

Une correction de forme du collider existant reste également une modification
de modèle. Elle ne se justifie que par un défaut de forme ou de passage identifié,
avec conservation des propriétés de masse et contrôles articulaires/sol/collision.
Les constats actuels n'identifient pas un talon absent ou un obstacle de volume
qui imposerait cette correction.

## Sauvegarde documentaire

Le reçu numérique contient les articulations, repères, coordonnées, mesures aux
instants cités, définitions des calculs, empreintes des sources et registres clos.
Les nouvelles données sont dérivées des traces existantes. Aucun test physique,
modification de paramètres ou nouvelle qualification dans ce jalon.
La récupération reste désactivée par défaut et non qualifiée ; P1 reste non certifiée.
