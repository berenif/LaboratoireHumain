# Horizon natif : équivalence des commandes et verdict physique

Travail local sur `investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ni publication.
La récupération de production reste désactivée et non qualifiée. Les limites
anatomiques, contacts, couples et critères d'acceptation sont conservés.

L'écart mesuré à quatre pas vient de deux définitions différentes de la commande.
Le prédicteur existant `predict_horizon` maintient la commande installée ; son
contrat le dit explicitement. Le contrôleur hors ligne réinstalle chaque pas la
base retenue, puis ajoute le delta cumulé. Au pas de décision, l'addition est
`(base + ancien_delta) + incrément`. Les pas suivants exécutent
`base + (ancien_delta + incrément)`. En `f32`, ces deux associations diffèrent ici
sur quatre vitesses de 1 ou 2 unités du dernier bit. Aucun défaut du moteur
Rapier n'est démontré : une même séquence de commandes donne les mêmes résultats.

Un même état préparé en mémoire à 10,016667 s est copié en trois branches. Toutes
utilisent le même intégrateur, filtre anatomique, PGS, état de solveur, forces,
cibles de position, gains, caps et modèles. Les branches maintiennent les
commandes, réémettent leurs valeurs identiques, ou reconstruisent les valeurs
réellement exécutées. Le premier pas à 10,020834 s est exact dans les trois.
À 10,025001 s, les deux premières divergent. La reconstruction reproduit bit à
bit les quatre pas : 25 corps, impulsions moteur, 25 impulsions normales et
sérialisation complète des paires de contacts. Réémettre les valeurs figées ne
résout pas l'écart. Au terminal à 10,033334 s, la définition figée surestime le
bras de 0,735990 N et sous-estime le torse de 4,243744 N.

![Même état préparé, trois séquences comparées](../evidence/prone-roll-horizon-equivalence-2026-10-11/first-divergence.png)

La correction ciblée ajoute `predict_reprepared_horizon` au module de diagnostic
natif, sous `cfg(test)`. Le premier pas applique les deltas habituels ; chaque
pas ultérieur réinstalle les commandes absolues fournies. Le code refuse toute
modification de position, gain, cap, modèle ou masque moteur, et conserve les
impulsions vivantes. L'appelant fournit la même reconstruction en `f32` que
l'exécution. Le contrat de maintien constant reste disponible. Le contrôleur
expérimental conserve ses objectifs, probes, cadence de quatre pas, borne de
0,25 rad/s, QP et règles d'acceptation ; seule sa prévision est alignée.

La régression embarque l'état chargé et les commandes du premier bloc divergent.
Elle compare chaque intégration, y compris les impulsions moteur et contacts,
et exige que l'ancien horizon figé diverge effectivement. Ce test est une preuve
de prévision dynamique locale ; ce n'est ni une preuve géométrique de corridor,
ni une réussite du relevage. Le certificat QP concerne toujours la proposition
locale avant clipping cumulé ; aucune optimalité globale après clipping n'est
revendiquée.

Le premier export de diagnostic a échoué après 26,232 s sur la conversion de
l'état Rapier en `serde_json::Value` (entier hors plage). Une reprise utilise
la copie native en mémoire et une sérialisation structurée directe, sans
paramétrage physique différent. Une invocation a été refusée immédiatement car
son dossier de sortie vide avait été créé avant le test : aucun pas exécuté.
Le préfixe valide termine après 33,876 s murales / 10,033333 s physiques sur
l'arrêt administratif déclaré `horizon-audit-complete`, sans verdict physique.
Les logs, versions de source, empreintes binaires et budgets sont archivés.

Les livrables se trouvent dans
`evidence/prone-roll-horizon-equivalence-2026-10-11/` : `causal-summary.json`,
`diagnostic-audit-memory/same-state-audit.json`, état chargé et commandes,
manifestes, journaux de calcul et médias locaux. Les archives précédentes restent
inchangées. Les corrections d'un autre environnement ne sont pas revendiquées.

Le rejeu physique unique avec l'horizon aligné ferme la question de prévision,
mais échoue au jalon : 3 215 pas / 13,395833 s physiques, 115,521 s murales,
arrêt `roll-stage-stall` sur le seuil original de 3 s. Aucun timeout ni build
concurrent. Le bassin finit à 44,875° et le torse à 54,266° ; leurs maxima sont
44,935° et 54,808°. Le bras porte encore 48,256 N, le torse 290,682 N,
la main gauche 75,444 N et les cuisses 53,665 / 248,915 N. Les 25 corps sont
immobiles et la capture COM est dans le support réel non céphalique. Aucune
remontée ni qualification n'est obtenue.

| Mesure | Cadence quatre pas, horizon figé | Horizon aligné |
|---|---:|---:|
| Arrêt physique (s) | 13,270833 | 13,395833 |
| Roulis final bassin / torse (°) | 45,068 / 54,388 | 44,875 / 54,266 |
| Charge finale bras droit (N) | 53,207 | 48,256 |
| Décharge continue sous 3 N (ms) | 20,833 | 20,833 |
| Dégagement complet de chaîne après 10 s (pas) | 0 | 0 |
| Contrôles exacts à un pas | 785 | 815 |
| Contrôles exacts corps, impulsions, charges, contacts de l'horizon | Non garanti | 815 / 815 |

Les 2 404 premiers états corporels sont identiques au précédent essai de cadence
quatre pas ; les commandes changent ensuite parce que la décision utilise enfin
la bonne prévision. Les 203 blocs terminaux complets et les trois pas du dernier
bloc vérifient la séquence réellement exécutée. Les charges natives sont exactes
bit à bit ; la recomposition numérique des sorties normalisées donne au maximum
7,1e-15 N d'écart bras et zéro torse. L'ancien écart de plusieurs newtons est
supprimé. Les 242 snapshots communs conservent exactement les 24 caps et cibles
de position. Le ratio vectoriel maximum de couple/cap est 0,984844 ; tête et cou
restent sans charge après 10 s, force et couple directs au bassin sont nuls.

La correction n'allonge pas la décharge du bras et n'acquiert pas de dégagement
complet. Sur la fenêtre commune 10–12,045834 s, contre le maintien natif inchangé,
le bras reçoit en moyenne 3,577 N de moins, mais le torse également 9,515 N de
moins. La main gauche reçoit 16,829 N de plus et la cuisse droite 16,566 N de plus.
La différence moyenne d'accélération COM X est −0,000545 m/s² et celle du moment
global déduit de dH/dt en Z est +0,008845 N·m. Ces mesures montrent surtout une
redistribution des autres appuis ; elles ne démontrent pas le transfert durable
bras→torse demandé ni un mouvement utile jusqu'à 75°. Le sous-test historique de
recontact demeure échoué. Cette expérience utilise déjà des appuis intermédiaires
réels ; elle ne valide pas une séquence de relevage pieds seuls.

204 décisions dont 149 acceptées, 0,368 s murale par décision en moyenne pour
16,7 ms simulées, au maximum 19 évaluations natives et 0,25 rad/s de delta cumulé.
C'est un outil hors ligne. Une prévision exacte ne fournit aucune garantie
d'optimalité globale, de cadence temps réel ou de réussite physique. Le budget
est fermé ; l'échec n'est pas répété, aucune campagne de réglages ni phase suivante.
Les six fichiers antérieurs sont restaurés à l'octet près et les quatre modules
expérimentaux retirés. Seuls l'API de prévision de commandes réémises et la
régression chargée restent dans le code final sous `cfg(test)`.

![Comparaison physique : prévision exacte, stagnation persistante](../evidence/prone-roll-horizon-equivalence-2026-10-11/comparison.png)

![Poses natives avant action, après action et à l'arrêt exact](../evidence/prone-roll-horizon-equivalence-2026-10-11/milestones.png)

[Courte vidéo native](../evidence/prone-roll-horizon-equivalence-2026-10-11/native.mp4)
— 20 images/s, sans interpolation ; la pose exacte de l'arrêt est ajoutée à la fin.
Les horodatages physiques figurent à l'image. Aucun média ne présente cet essai
comme une réussite du relevage.

Vérifications finales : cinq tests de prévision native passent (dont l'état
chargé, ses quatre hash corporels et charges enregistrés indépendamment), un
replay explicite est ignoré ; sept tests de projection/commandes passent, cinq
expériences explicites sont ignorées. Clippy strict `-D warnings`, format et
`git diff --check` passent. La vidéo contient 73 images natives, dure 3,65 s et
sa dernière image décodée a été inspectée. Aucune assertion ne transforme ces
unitaires en succès physique. La suite complète n'est pas déclarée verte : le
test d'articulation hybride et quatre tests TypeScript échouaient déjà sur le
témoin Linux et ne sont pas revendiqués comme corrigés.

Binaire du diagnostic valide :
`663bf7f7ef28ad655efb3c895c8197335818e47a256eec89b4cf72c425da76d0`.
Binaire du rejeu physique :
`2971f3ff4403fa07ecd87dc4e587951a5a0543f27c2120bc0e5d1e34371da951`.
Les versions exécutées restent dans `source-diagnostic-memory/` et
`source-aligned/` ; `source-final/` contient le correctif retenu et la régression
renforcée. Les quatre prototypes retirés sont conservés uniquement comme pièces
d'expérience dans l'archive. Le blocage restant est physique : obtenir un appui
receveur et un dégagement durables jusqu'au jalon, sans aide au bassin ni reset.
