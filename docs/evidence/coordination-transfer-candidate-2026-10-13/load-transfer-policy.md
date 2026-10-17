# Politique de transfert de charge avant retrait — préparation pure

La politique `load_transfer.rs` prépare des intentions communes pour les 46 axes
à partir d'observations archivées ou synthétiques. Elle ne retire aucun appui,
ne déplace aucun corps, ne construit ni Model ni Simulation et n'exécute ni FK
ni intégration native. Ses propositions restent `PreviewRequired`. Le résultat
est un candidat de commande inspectable, pas une trajectoire dynamique prouvée.

## Séquence proposée

Le candidat prioritaire conserve les points matériels d'avant-pied et demande
l'acquisition des deux talons. Il rapproche simultanément la capture du centre
des contacts pédestres **actuellement observés**, sans ajouter les talons désirés
à l'enveloppe d'appui. Lorsque les talons puis le diagnostic local complet sont
acquis, il rend la main aux objectifs de continuation bilatérale. Aucun retrait
de pied n'est imposé pour poursuivre la montée.

Une intention optionnelle `RecenterForWithdrawal` prépare ensuite le recentrage
vers le support restant. Elle ne vaut jamais retrait : `supported_transfer`
est appelé sur l'observation réelle, avec exclusion du pied entier 18/19 ou
23/24, et ses résultats/écarts de charge sont publiés. La machine distincte
`support_schedule` garde la responsabilité de la demande explicite de retrait,
des refus, des archives d'ancres et d'une éventuelle réacquisition. Une charge
désirée ou un objectif de CoM ne remplace aucun de ces contrôles.

L'audit `support-threshold-provenance.md` rappelle que les seuils 80 %, 25 % et
3 mm ont été introduits dans le transfert Rust puis figés par le protocole ;
ils ne constituent ni une entrée du contrat JSON général ni une loi universelle
du mouvement humain. Ce jalon ne les change pas. Les charges persistantes du
pied retiré et les charges courantes des points restants ne sont pas une
partition exacte d'un bilan d'impulsions CCD. Les déficits publiés conservent
leurs définitions originales.

## Entrées et provenance des points

L'API utilise `SupportObservation` (25 corps, contacts courants, charge,
persistance, CoM et vitesse), `MaterialTracker` et un `TransferIntent` explicite.
Les observations doivent suivre l'horloge native de 240 Hz ; les rejets de
validation/horloge ne mutent pas la mémoire.
La politique mémorise aussi les identités et ancres initiales : même si le
propriétaire fournit un tracker mutable, retirer ou réancrer ces obligations
fait refuser la proposition. Les seuls nouveaux patches admis dans cette
politique sont des talons ayant déjà établi leurs douze observations propres.

Les deux points de talon proviennent de `materialTargets` dans
`docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json`, SHA256
`a96d03d736ca185a0457ae29c33846b69af28ecd5eb81609f60d281463756dd6` : segments 18
et 23, chacun au point local
`[-0.04399999976158142, -0.04500000178813934, -0.07500000298023224]`.
Il s'agit du sommet géométrique archivé du talon, pas d'un contact porteur déjà
prouvé. Les anciennes ancres monde planifiées ne sont pas réutilisées. Le point
monde courant est obtenu uniquement par transformation de ce point local avec
la pose mesurée du corps ; il n'y a pas de reconstruction de pose articulaire.

À l'entrée A36 archivée, les quatre segments de pied peuvent déjà toucher le
sol mais les régions postérieures des talons ne portent pas de charge. Les
contacts antérieurs de hindfoot 18/23 restent des contacts observés : ils ne
sont pas automatiquement figés comme pivots de talon. L'initialisation du
candidat suit les ancres d'avant-pied 19/24. `Policy::new` refuse de recevoir
un point antérieur de hindfoot présenté comme obligation de talon ; il ne
supprime pas une obligation existante en silence.

## Intentions communes, sans nouveaux actionneurs

`TransferProposal::apply_to_goals` modifie seulement les `TaskGoals` du solveur
commun. Nominal q/dq, coordonnées, bornes, caps, posture faible, régularisation
et tolérance matérielle restent ceux du propriétaire. Il n'existe pas de
commande directe réservée à la cheville, de déplacement du bassin ou de
supposition que la racine est actionnable.

- Le guide horizontal est `2 × (centre_support − capture)`, de norme au plus
  0,15 m/s. Les nombres 2/s et 0,15 m/s sont ceux du propriétaire existant.
  La composante verticale du guide de recentrage est nulle pendant acquisition
  et stabilisation ; la référence de montée reprend après le jalon mémorisé.
- Le centre est la moyenne des sommets de l'enveloppe convexe des contacts
  pédestres courants, ou des points disponibles si cette enveloppe dégénère.
  Le test de marge originale de 3 mm est publié séparément. Un centre choisi
  hors marge reste un guide non certifiant ; cela ne prouve pas à lui seul
  l'absence de tout autre centre admissible pour une enveloppe générale.
- Chaque talon non encore acquis reçoit un objectif de descente du point
  local archivé vers le niveau d'appui observé, à gain 2/s et vitesse au plus
  0,15 m/s, plus un objectif de pitch à gain d'orientation 2/s. L'axe de pitch
  et l'inclinaison sont calculés depuis le quaternion mesuré. Yaw et roll ne
  reçoivent aucun nouvel objectif correctif. Un pitch ne pouvant pas abaisser
  le point est signalé comme obstacle ; cette algèbre ne prouve jamais sa
  réalisation par les moteurs `ForceBased`.
- Les intentions matérielles d'avant-pied sont marquées `Pivot`, mais les
  patches observés sont copiés sans changement : local, ancre, identité et
  horloges sont conservés. Ce mot n'autorise aucun réancrage et ne crée pas
  une cinématique idéale de pivot.

La différence référence/vitesse physique reste traitée par `command_proposal` :
ces tâches guident une proposition, elles ne certifient pas la tenue d'un
contact. Les gains moteur, la saturation et l'allocation du couple restent
inchangés. La politique ne choisit ni modèle de friction ni segments nouveaux.

## Événements mesurés et mémoires

La politique tient le `LocalSupportDiagnostic` existant et une persistance
propre à chaque point de talon recherché. Il faut un contact actuel de normale
admissible, d'au moins 3 N, dans la région `local_z < -0.02`, correspondant au
point demandé selon la tolérance matérielle existante de 0,012 m. Le premier
point local du candidat reste fixe : une succession de petits déplacements ne
peut pas déplacer progressivement son origine. Une disparition ou un changement
de point remet cette persistance à zéro, sans héritage de l'ancien compteur du
segment. Douze observations sont nécessaires pour annoncer cette acquisition.

Le relais bilatéral attend en plus les 132 sous-pas du diagnostic local complet,
avec son intervalle commençant avant 2 s, ses charges, son enveloppe et le calme
des 25 corps. L'événement est mémorisé une fois, avec son sous-pas exact et celui
du début de l'intervalle stable. Les accesseurs `continuation_event()` et
`first_stable_substep()` permettent au propriétaire de raccorder la nouvelle
référence à sa dernière commande et vitesse réellement installées.

Ce relais reste mémorisé lorsque la montée produit un mouvement non calme ;
sinon la politique imposerait à nouveau un recentrage immobile et empêcherait
sa propre continuation. Les obligations matérielles courantes restent contrôlées
séparément : `apply_to_goals` refuse avant mutation si un point retenu a perdu
sa correspondance réelle. Le jalon local n'autorise ni à ignorer les gardes
originales pendant la montée, ni à annoncer un relevage ou la qualification.

Les mains et genoux ne sont pas imposés. Les dépendances non pédestres réellement
observées sont publiées ; aucun point de main/genou désiré ne peut remplir le
support. Un éventuel candidat les utilisant devrait disposer de points proches
observables, d'une acquisition propre et d'un transfert explicitement vérifié.
Ce module ne fabrique pas cette branche manquante.

## Obstacles mesurés dans les archives et tests purs

Les quatre archives B16 ne portent que sur des contacts d'avant-pied presque
colinéaires. Leur étendue sagittale est de l'ordre de quelques dizaines de
micromètres, bien inférieure à une marge de 3 mm. La politique publie donc
`InteriorTargetUnavailable`, des talons non acquis et les déficits du retrait
optionnel, tout en produisant des intentions de descente/recentrage. Elle ne
retire pas le pied pour rendre le problème plus facile et ne remplace pas le
support courant par le futur support talon espéré.

Neuf tests purs couvrent les quatre états B16, l'entrée A36, la provenance des
points, le jalon synthétique et son maintien pendant la montée, la persistance
propre, l'absence de retrait non demandé, les erreurs d'horloge/perte de point,
les ancres initiales et les nouveaux patches, et les enveloppes dégénérées/
impulsions CCD insuffisantes. Les tests utilisent
des observations et de l'algèbre de tableaux ; zéro construction Model ou
Simulation, zéro FK, zéro pas natif. **9/9 tests passent**, zéro échec, 216 tests
hors filtre, en 0,05 s : `load-transfer-tests-rerun.log`. Le premier journal
`load-transfer-tests.log` conserve un échec de compilation pendant que le
raccord du propriétaire était incomplet (module/méthode/champ encore absents) ;
aucun test n'avait alors été exécuté. Ce journal n'a pas été réécrit.

L'export exclusif `static-load-transfer-results.json` contient les quatre
propositions B16. Une relecture du JSON confirme : deux obligations conservées
dans chaque cas, aucun retrait, aucun relais de continuation, marge du centre
non satisfaite et gate de transfert refusé. Les hauteurs de talon calculées
depuis les poses observées vont de 40,893 à 42,670 mm ; les vitesses verticales
**désirées** de −0,08179 à −0,08534 m/s et les composantes de pitch x de −0,3540
à −0,3658 rad/s. Ces valeurs décrivent des intentions, pas des vitesses délivrées
par les moteurs ni une acquisition physique.

SHA256 du reçu de propositions :
`e4e7f0c7f5deb65f11ab2eee30099ccf777902e2cbaeedf13ed12a9c93b9bdcc`.
SHA256 du journal passant :
`2cdd0c7cb9ff4d00ade901a9b2ad16658d0510c4e01b3552b0c0f63fcaea6d15`.

## Ce qui reste à valider

Le propriétaire doit appliquer les intentions via le compilateur commun,
archiver les quatre commandes f32 et leurs résidus, installer les mêmes moteurs
dans copie et réel, préserver toutes les mémoires de politique/diagnostic et
contrôler les admissions/budgets avant chaque intégration. Les talons doivent
être effectivement observés avant d'ajouter leurs nouveaux patches retenus.
La continuation doit se raccorder au q/dq effectif au sous-pas mémorisé, pas à
une pose transplantée ou à une vitesse annulée.

La préparation statique rend le candidat applicable aux observations ; elle
ne démontre pas la possibilité dynamique du transfert, la stabilité de la
montée, le terminal original, ni la performance depuis un véritable accroupi
après impact. Ces faits demandent un protocole natif distinct explicitement
autorisé et borné. Aucun tel calcul n'est exécuté par cette note.
