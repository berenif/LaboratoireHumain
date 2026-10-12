# Current status

**2026-10-12 revue de coordination : aucun nouveau relevage ni essai physique.**

L'audit du vrai contrôleur SLOW distingue la trajectoire exportée de son retour
global appris. L'A/B local suivait presque tout le chemin articulaire sans
redresser la racine ; le dernier programme numérique présente encore une rupture
de support. Une revue contradictoire recommande une décision commune sur les
membres et les changements d'appui, avec anticipation et validation native des
commandes. Un prototype court d'acquisition des talons depuis l'accroupi est
proposé, sans lancement ni ouverture de budget. Aucun code moteur changé ;
récupération inactive par défaut et non qualifiée, critères conservés.
[Recommandation, sources, objections, interfaces et budget proposé](coordination-architecture-review-2026-10-12.md).

Le résultat du 11 octobre ci-dessous est historique ; les essais A/B et pieds B/C
du 12 octobre restent eux aussi en échec. Le dernier candidat est
[rejeté avant intégration](intermediate-transfer-numeric-audit-2026-10-12.md).

**2026-10-11 verdict actuel : géométrie admissible, zéro relevage sur 96 histoires.**

Le support utilise les points persistants réellement chargés. Le programme entier
respecte les limites et passe 402/402 inspections à 0°/60°, mais échoue en physique :
première collision des pieds à 4,883 s, absence d’appui arrière de talon dans les
captures initiales et recentrage transitoire. Le rejeu est exact sur les 1 171
sous-pas enregistrés. La campagne est arrêtée à 96 ; aucune validation finale,
récupération désactivée par défaut et non qualifiée. Les 24 tests Rust ciblés et
les deux tests Python passent ; aucune affirmation de suite complète verte.
Checkpoint autorisé sur `investigate/physical-support`, sans merge ni force push.
Les données de mouvement, poids et environnements restent locaux.
[Résultats, traces, médias et limites diagnostiques](rust-recovery-coordinated-retarget-2026-10-11.md).

Les sections suivantes décrivent les jalons précédents, pas l’état actuel.

**2026-10-11 référence source extraite ; adaptation native encore bloquée.**

SLOW réalise le relevage dans son moteur source : 2 921 échantillons, 3,65 s,
appuis finaux aux deux pieds, tête à 1,452 m. Le second export corrige une
association erronée de forces/contacts après `mj_step1` ; toutes les poses,
vitesses et couples restent exactement identiques. Une adaptation de 18 s
échoue aux contrôles natifs : 78/402 positions invalides, commande trop rapide
à l’avant-bras gauche et mains/pieds sans appui correspondant. Aucune campagne
Rapier n’est lancée. L’interpréteur partagé et ses tests courts sont conservés
sous `cfg(test)` ; ils ne prouvent pas un relevage. Récupération désactivée et
non qualifiée, seuils conservés, aucune publication.
[Référence, adaptation rejetée, preuves et médias](rust-recovery-reference-preparation-2026-10-11.md).


**2026-10-11 décision d'architecture : mouvement complet avant nouvelle recherche.**

La prochaine voie est une référence numérique complète issue du contrôleur SLOW
publié, adaptée au mannequin, avec contacts explicites et optimisation hors
ligne de programmes articulaires entiers dans Rapier. Le contrôle précédent
conservait la posture et ne recherchait que 12 vitesses du tronc/bras à 16,7 ms ;
son modèle exact ne lui donne pas les variables nécessaires à la remontée.
Le prérequis absent est une référence entière adaptée. Son export CPU paraît
possible depuis les poids publics, mais la compatibilité et le retargeting ne
sont pas validés. Si cette préparation échoue, aucune recherche native ne part.
La campagne prévue est fermée : 384 histoires ou 4 h, arrêt dès 96 sans atteindre
les conditions debout originales ; gel du premier succès puis quatre validations
sans retouche. Une étape locale réussie ne renouvelle aucun budget.
Ce tour n'exécute ni physique ni apprentissage, ne change aucun code physique et
préserve les correctifs. Récupération désactivée/non qualifiée, aucun push,
merge ou publication. [Plan exécutable et critères complets](rust-recovery-architecture-decision-2026-10-11.md).


**2026-10-11 horizon natif : séquence exacte, stagnation physique persistante.**

La cause de l'écart à quatre pas est démontrée : le contrôleur reconstruit ses
vitesses absolues avec une association `f32` différente de celle du maintien
prévu. Depuis un même état, réémettre les valeurs figées reste divergent ; la
reconstruction réelle donne quatre intégrations exactes, corps, impulsions,
charges et contacts compris. Une API et une régression chargée sous `cfg(test)`
conservent ce contrat sans toucher aux positions, gains ou caps.
Le rejeu unique avec l'horizon aligné vérifie 815 pas et horizons exacts, puis
stagne sur les 3 s originales à 13,395833 s : 44,875° bassin / 54,266° torse,
bras encore porteur à 48,256 N. Décharge de 20,833 ms seulement, aucun dégagement
complet après 10 s et aucun jalon 75°. Budget fermé, échec non répété, prototypes
retirés, récupération désactivée et non qualifiée. Les appuis intermédiaires
réels de cet essai ne suffisent pas au roulis ; pieds seuls non validés.
Aucun push, merge ni publication. Suite complète non déclarée verte.
[Preuves, traces et médias](rust-prone-roll-horizon-equivalence-2026-10-11.md).


**2026-10-11 cadence16,7 ms : verdict physique acquis, jalon non acquis.**

La définition exacte du troisième essai est relancée une fois avec600 s natives
et60 s de sauvegarde, sans compilation concurrente. Elle s'arrête après118,109 s
murales sur la stagnation originale3 s à13,27083 s physiques :45,068° bassin /
54,388° torse, bras53,207 N. La décharge sous3 N dure20,833 ms, aucun dégagement
complet de chaîne après10 s. Les785 prochains pas et impulsions sont exacts ;
les horizons à quatre pas présentent des écarts de charge mesurés, sans garantie
terminale complète.197 décisions,371 ms en moyenne pour16,7 ms simulées : outil
hors ligne, aucun contrôleur temps réel ni relevage qualifié. Aucun succès à
répéter. Prototypes retirés ; quatre + sept tests, Clippy strict et format passent.
Suite complète non déclarée verte, récupération désactivée/non qualifiée,
aucun push/merge/publication. [Verdict, traces et médias](rust-prone-roll-cadence-resolution-2026-10-11.md).

**2026-10-11 contrat natif : neutralité et impulsions exactes, relevage ouvert.**

L'inversion d'une impulsion moyenne en vitesse moteur change la commande sous
contacts ; le pas moteur réel vaut 0,167 ms, et corriger seulement ce pas ne
restaure pas la neutralité. Le delta nul natif reproduit les 2 891 pas du maintien.
Après 10 s, les 25 corps et toutes les impulsions moteur sont prédits bit à bit
sur 491 pas témoin et 829 pas transfert. Ce dernier décharge brièvement le bras,
puis recharge avant la décision suivante et stagne à 13,45417 s : 45,269° bassin /
54,729° torse, aucun jalon de 75°. Le troisième essai de cadence s'arrête sur la
limite murale, sans résultat physique terminal ; sa compilation concurrente
empêche une conclusion de débit intrinsèque. Trois tentatives, 223,108 s murales,
budget clos. Prototypes retirés, régression du contrat nul conservée ; quatre
et sept unitaires, replay chargé, Clippy strict et format passent. Suite complète
non déclarée verte. Récupération désactivée/non qualifiée ; aucune publication.
[Rapport causal, traces et médias](rust-prone-roll-native-command-contract-2026-10-11.md).

**2026-10-11 transfert bras→torse : redistribution partielle, aucune phase acquise.**

Le signe utile et les24 budgets vectoriels sont vérifiés ; marge initiale2,274 N
après contraction native, limite d'allocation ceinture X. Le maintien reproduit
2 891 hashes exacts. Un contrôle nul révèle une conversion moteur incorrecte ;
le termeD+dt×K la corrige. Le contrôle nul corrigé et le transfert restent stables
jusqu'à la stagnation originale3 s à12,04583 s. Transfert43,81°/55,36° :
+0,11°/+0,22° face au contrôle nul. Maximum admis2,325 N pour4 N demandés ;
delta moyen bras−7,38 N, torse+14,26 N entre11 et12 s, autres contacts modifiés.
Les couples réellement livrés diffèrent du motif demandé ; modèle statique
insuffisant, suivi dynamique des couples et charges ouvert. Aucun jalon75°,
aucune répétition de réussite ni phase suivante. Quatre histoires46,89583 s
physiques/144,20681 s natives, budget clos. Prototypes retirés, projection pure
conservée ; onze unitaires candidat corrigé, sept après retrait, Clippy strict
et format final passent. Suite complète non déclarée verte. Récupération
désactivée/non qualifiée, aucun push/merge/publication, aucune tentative Library.
[Rapport, bilan dynamique et médias](rust-prone-roll-dynamic-transfer-2026-10-11.md).

**2026-10-11 continuation native : le recontact porte, puis le roulis stagne.**

Le verdict original reste faux. La continuation reproduit2 071 pas historiques,
ajoute huit pas puis refuse une référence géométrique à8,6625 s. Une seconde
histoire conserve ensuite exactement la dernière commande installée, sans reset
des corps ni des impulsions. Roulis final43,60°/54,83°, bras50,96 N,
tête/nuque0 N ; stabilité partielle3,3125 s. Arrêt physique à12,04583 s sur
stagnation3 s, jalon75° non acquis. Cela établit l'équilibre sous commande
maintenue, pas l'échec de toutes les trajectoires. Le moment global moyen Z
passe de+0,240 N·m pendant le freinage à−0,00126 N·m sur la dernière seconde.
Un transfert de charge bras→torse est proposé depuis les contacts mesurés ;
budget vectoriel et réponse dynamique encore non vérifiés. Deux histoires,
20,70833 s physiques/53,64531 s murales natives ; aucun troisième essai.
Prototypes retirés, projection pure conservée. Sept unitaires finaux, Clippy
strict et format passent ; suite complète non déclarée verte. Récupération
désactivée et non qualifiée, aucun push/merge/publication, médias locaux.
[Rapport, moments, captures et vidéo](rust-prone-roll-contact-continuation-2026-10-11.md).

**2026-10-11 projection sûre : ceinture libérée, suivi du haut du bras insuffisant.**

Le défaut clamp après résolution est corrigé avec bornes actives, contrôle de
la chaîne entière et refus explicite des références non certifiées. La régression
sur l'état exact2066 trouve9 mm sans déplacer le torse ni les corps. Trois
histoires comparent le témoin, un cache prioritaire rejeté et la correction
nominale finale ;1 468 pas historiques communs. Au dernier cas,603 références
sont certifiées, mais le haut du bras réel est≈5,15 mm sous sa référence :
ceinture0 N, bras35,85 N, tête/nuque0 N, roulis42,19°/51,71°. Arrêt sur recontact
0,20 s à8,629 s, encore en mouvement ; jalon75° non acquis, aucune répétition
ni phase suivante. Définition d'un pivot/transfert explicite, non validé.
Correction pure conservée, prototype physique retiré. Sept tests ciblés après
retrait, Clippy/format passent ; aucune suite complète verte déclarée.
Trois histoires25,2125 s physiques/74,150 s natives. Récupération désactivée et
non qualifiée ; aucune publication. Médias locaux, aucun nouvel essai Library
sur son transport inchangé en erreur réseau.
[Rapport, régression et médias](rust-prone-roll-bounded-projection-2026-10-11.md).

**2026-10-11 chaîne droite : dégagement distal réel, recontact scapulaire refusé.**

Le témoin reproduit ses 3 186 pas et le candidat partage 1 469 pas initiaux.
La tâche coordonne la chaîne depuis le torse mesuré, sans racine libre ni force
au bassin : haut du bras/avant-bras/main droits à 0 N, tête/nuque déchargées.
Toute la chaîne reste libre 2,254 s, puis la ceinture droite porte 256,25 N.
Arrêt sur recontact 0,20 s à 8,608 s, roulis 42,24°/51,80°, encore en mouvement.
Jalon 75° non acquis ; aucune répétition ni phase suivante. La projection
laisse une référence sous le sol à une borne articulaire active ; aucune
impossibilité anatomique générale n'est démontrée. Coordonner les bornes actives
ou définir le pivot/transfert scapulaire avant de poursuivre. Prototype retiré,
deux nouveaux fichiers et binaire archivés ; anciens témoins/médias intacts.
Deux cas 21,883 s physiques / 71,879 s natives. Cinq unitaires candidat puis
quatre après restauration, Clippy strict et format passent ; suite complète
non déclarée verte, récupération désactivée/non qualifiée, aucune publication.
[Comparaison, diagnostic de projection et étapes natives](rust-prone-roll-arm-release-2026-10-11.md).

**2026-10-11 nouvelle route latérale : mouvement réel, retournement incomplet.**

Deux histoires reprennent exactement1 468pas de C+préparation gauche et partagent
1 469pas avant la pression unilatérale. Par couples articulaires et vrais appuis,
le candidat parcourt46,17° au bassin/62,34° au torse et décharge tête/nuque ;
sous-critères d'appui stable3,229s. Jalon latéral75° non acquis, arrêt sur3s de
stagnation ; aucune répétition ni transfert suivant. La mainD est dégagée mais
le bras supérieurD porte390N et freine le roulis, sous un cap scapulaireG28Nm.
Pas de pose/reset/effort imposé au bassin ; la racine mondiale reste mesurée.
Prototype retiré du code vivant, cinq fichiers et binaire exacts conservés.
Deux cas20,217s physiques/58,329s natives, aucun gain sweep. Référence HumanUP
fondée sur texte/légendes ; vidéos403 non visionnées, aucune tentative de
contournement. Quatre unitaires ciblés,Clippy,format passent après restauration ;
suite complète non déclarée verte,récupération désactivée et non qualifiée.
[Comparaison causale, route complète et média natif](rust-prone-roll-2026-10-11.md).

**2026-10-11 transfert de poids : stabilité accrue, tête non déchargée.**

Deux histoires complètes,2 448pas/10,2s identiques avant un changement unique
initialisant l’IK depuis les articulations mesurées. La référence reproduit
exactement les3 766pas du cas02 historique. Le candidat stabilise COM/capture
et les25corps pendant3,333s, mais les mains ne portent que28,11N et la tête254,71N.
Déchargement0s,brace0s ; même refus original de stagnation3s. La divergence IK
ne prouvait pas une impossibilité de portée ; corriger l’initialisation ne règle
pas la répartition des efforts. Candidat retiré du code vivant, sources/traces
et médias préservés. Budget31,567s physiques/111,343s natives,aucune répétition.
Références primaires comparées sur texte, vidéos externes bloquées HTTP403 ;
aucune nouvelle route latérale exécutée. Quatre unitaires ciblés,Clippy et format
passent ; aucune suite complète verte ni récupération de production activée.
[Rapport, comparaison native et schéma sourcé](rust-head-load-transfer-2026-10-11.md).

**2026-10-11 chaîne physique C → deux appuis des mains : progression réelle,
redressement refusé.** Trois cas repartent de la chute complète et continuent
le même monde,484pas C identiques. La correction des références de mains depuis
le torse mesuré,avec clearance des skins natives,franchit les six tâches et
exécute la montée. Appui bilatéral actuel+persistant2,708s au cas02 ; tête226,48N,
torse0N,gain torse19,79mm,brace0s. L’ajout de référence mondiale via réactions
de hanche réduit l’erreur de racine mais régresse physiquement ; rejeté et
retiré. Code actif opt-in testé :4unitaires,Clippy strict,format ; aucune suite
complète verte,aucun genou/pied avancé ou relevage qualifié. Récupération de
production désactivée. Trois cas44,108s physiques/198,024s natives ; quatrième
réservé à une répétition réussie et non utilisé.
[Résultats causaux,sources exactes et vidéos](rust-prone-chain-gravity-2026-10-11.md).

**2026-10-11 levée gauche : cause et intervention courte vérifiées.**

L’audit confirme le mapping IK→coordonnées→moteurs et inclut un contact
torse–bras réel dans l’équilibre gravité/moteurs (résidu <0,00486 Nm).
Deux intégrations ont 1 268 pas strictement identiques avant la différence
de commande. La compensation gravitaire calculée par les moteurs existants
ramène l’erreur de 99,95 à 24,35 mm ; passage sous 50 mm en 187,5 ms,
sans nouvelle cible/gain/plafond. Le contact torse–bras se décharge, mais
tête 143,5 N et torse 212,0 N restent porteurs. Aucun brace/relevage qualifié.
Coût : deux cas, 11,57 s physiques / 38,29 s natives. Diagnostic opt-in seulement ;
phases suivantes non vérifiées retirées du code vivant. Deux unitaires,
Clippy strict et format passent ; aucune suite complète verte revendiquée.
[Bilan causal, traces appariées et vidéo réelle](rust-left-hand-command-audit-2026-10-11.md).

**2026-10-11 sortie du repos ventral : verdict natif négatif.**

Un seul essai reproduit les 484 pas C exactement puis continue le même monde.
La main gauche se lève mais reste à 100 mm de sa cible ; arrêt à 5,283 s sur
stagnation 3 s. Tête 139 N, torse 219 N, aucun défaut physique d’intégrité et
moteurs du bras sous 67 % des caps au dernier pas. La préparation suivante et
la montée du torse ne sont pas exécutées. Le prototype rejeté est archivé
hors code vivant ; aucune répétition ou nouvelle candidature. Récupération
par défaut désactivée et non qualifiée. Géométrie/LP/unitaires restent des
preuves séparées, aucun succès physique ou suite complète verte annoncé.
[Rapport, trace et vidéo réelle](rust-prone-brace-attempt-2026-10-11.md).

**2026-10-11 C scope audit and continuation:** thigh support is an experimental
corridor refusal, not a general terminal recovery failure.373 archived steps
replay exactly; the same commands continue to prone rest at 2.016667 s with no
numerical guard violation. K=0.00166 J, torso/thighs/hands loaded, head 109.88 N,
no feet/shins loaded. This is a candidate ground start, not a qualified
intermediate support or get-up. The next task is arm-support preparation and
head unloading from measured state; no new command is executed. Two invocations
are retained: an invalid initial-rest stop and one complete continuation,
total 2.433 s physical/6.154 s native wall. Targeted bench tests, strict Clippy and
format pass; production stays disabled/unqualified, no publication.
[Criteria audit, physical rest and next decision](rust-c-framing-continuation-2026-10-11.md).

**2026-10-11 coordinated tibia-first descent:** a single native C acquires
tibia contacts37.5ms before archived B, but matched-time energy remains221J
against218J at1.35s. Tibia contact force accelerates COM forward (+293.29N),
consistent with native momentum; COM/capture remains in front of the knee
contacts. C refuses on persistent thigh support at1.554s, stability0s.
159/200 geometry evaluations, one algebraic contact-branch construction,
one native trial5.37s wall; no repeats or transfers after refusal.31 recovery
tests,4 measurement tests, strict Clippy and formatting pass. No complete-suite
or recovery qualification claim; production remains disabled/unchanged.
[Energy balance and first causal obstacle](rust-controlled-descent-2026-10-11.md).

**2026-10-11 native hand-support experiment:** matched leg/trunk commands
with reference versus prepared arms both fail on head support at 1.596/1.617 s.
Prepared hands carry real load and provide −98.05 N·s of forward braking, but
loaded elbows collapse after a roughly 2 m/s downward arrival. Stable
intermediate support remains zero. Geometry and terminal statics pass after one
measured geometric correction; they do not certify the transition. Two native
trials only, no fresh/60-degree repeats or onward transfer. Preparation used
400 total geometric evaluations, exceeding the earlier 200-evaluation limit;
physical thresholds/caps remain unchanged. Generic opt-in test bench retained,
no production-controller adoption. Recovery stays disabled and unqualified.
[Results, causal evidence and video](rust-hand-support-experiment-2026-10-11.md).

**2026-10-11 strategic decision:** the 25 failed programs leave planned
hand/shin support unexplored. Arms were fixed by the trajectory family, and
Stand prepares no new hand contact. The next experiment is one controlled
descent corridor with matched leg/trunk commands and an A/B of reference
versus anticipatory arm preparation. Prove actual intermediate support first,
then hands/shins → half-kneel → feet → rise; no claim that hands are necessary
or sufficient. Source/archived-state review only, no new native rollout or Rust
change. [Decision and bounded Sol experiment](rust-recovery-strategy-review-2026-10-11.md).

**2026-10-11 offline global planning:** ten seeds and two bounded coordinate
rounds evaluate 25 distinct programs in 48.772 s. All fail on persistent
unplanned head/torso support before any stable transfer or rise; a fresh
score-selected replay is exact. Owner allocation corrects measured command
clipping within 760 Nm, but does not solve support. Command generation is
31 microseconds median; physical recovery remains disabled and unqualified.
A prior metric incorrectly accepted bilateral toes after body collapse, now
conservatively refused. Neither feet-only impossibility nor mandatory hand/knee
support is proven. [Method/results/evidence](rust-offline-trajectory-2026-10-11.md).

**2026-10-11 native coupled control:** direct native clones reproduce all 25
bodies exactly, including a four-step fixed-command continuation. Bounded
coupled commands produce actual multistep forefoot braking. Unique ownership
of position/velocity targets moves the pelvis backward, but capture still
lies outside measured support at 0.5 s. The final owned-target full trial
refuses at 1.3917 s after the original 0.2 s support-loss persistence.
Transfer, rise and recovery qualification remain open; the rejected controller and its hooks are archived and removed from live
source. Full identification costs about 464 ms per 4.17 ms
substep and is not interactive. [Implementation/evidence](rust-native-control-2026-10-11.md).

**2026-10-11 native actuation audit:** identical native prefixes isolate the
right forefoot command. Turning off only that motor yields 9.164 rad/s, near
the QP prediction 9.163, instead of 10.488 with the original command. The
raw constraint impulse sign had been misread as child braking; vector
impulses show acceleration. The motor has braking authority under its
unchanged cap. Full native clones predict all 25 body states exactly; a
locally identified command produces real braking. Native prediction and
validated local command search are the chosen architecture for Sol. No
new long rollout or recovery qualification. See [causal audit](rust-native-actuation-audit-2026-10-11.md).

**2026-10-11 general QP consolidation:** 458 distinct exact archived matrices
(651 occurrences) are independently certified: 42 optimal, 416 infeasible in
their archived formulations, zero unknown. Native/WASM numeric results match
exactly; repeated native vectors match exactly. A general two-stage backend
handles the additional feasible refusal without relaxing original tolerances.
Two final 100 ms physical reports are byte-identical. Full rollout stops before
stability at 279.17 ms: the right toe rapidly closes on its anatomical limit.
A terminal Farkas certificate is valid for the inverse-dynamics predictor,
but native toe speed increases despite predicted braking. Predictor/actuation
fidelity remains the blocker; feet-only impossibility and need for intermediate
support are undetermined. Recovery stays disabled and unqualified.
Experimental controllers/hooks are archived and removed from live source.
24 recovery tests, targeted clippy, formatting and diff checks pass; no complete
suite claim. See [solver evidence](rust-recovery-general-qp-consolidation-2026-10-11.md).

**2026-10-11 general hybrid-contact experiment:** two byte-identical native
100 ms trials acquire hindfoot support with unchanged 32 passes and caps
<=760 Nm. The extension stops at 183.33 ms after acquiring a left heel contact
for one step; the exact refused QP is feasible but the optimizer reaches its
iteration limit. No right heel, recentering, stability or rise qualifies.
The experimental controllers/hooks are archived outside the final source.
Recovery remains disabled and unqualified. See [causal results, images and
reproduction](rust-recovery-hybrid-contact-modes-2026-10-11.md).

**2026-10-11 native constraint comparison:** the bounded 100 ms proof fails.
Both original and independent/deduplicated formulations stop at 66.67 ms in
the actual 32-pass pipeline, with repeatable traces and every installed cap sum
≤760 Nm. The new state combines incompatible sticking tasks, native joint
velocity residuals and local torque envelopes. A different frozen feet-only
mode can carry body weight within 474.6 Nm; its dynamic transition is untested.
No heel pivot, rise or recovery qualifies. See the
[native result and next structural decision](rust-recovery-native-constraints-2026-10-11.md).

**2026-10-11 bounded QP audit:** the historical 37.5 ms refusal is caused by
two redundant contact directions amplified from rounding noise. The same Rust
solver resolves the identical frozen problem with 16 independent rigid-contact
rows, preserving every torque/contact/joint inequality. Strict-budget replay
preserves the recorded body states. This is a mathematical correction validated
at a fixed state, not physical recovery. The next bounded native comparison and
remaining limits are in the [causal audit](rust-recovery-constraint-audit-2026-10-11.md).

**2026-10-10 local recovery system diagnosis:** pristine crouch failure reproduced;
resolved toe contacts contradict an early geometric “inside support” result.
A sampled flat-sole static branch satisfies original joint/torque limits, but
a complete physical recovery remains unproven. Measured-contact balance is corrected with 24 focused tests passing.
Coupled control holds three prepared poses for 7 s; actual crouch attempts still
fail. The stricter 52-coordinate/contact model refuses at 37.5 ms; its contact
formulation defect is now isolated; dynamic integration remains unverified. No anatomy change, gate relaxation,
or publication. See the
[mechanical investigation and next discriminating tests](rust-recovery-system-diagnosis-2026-10-10.md).

**2026-10-09 strict corrective stepping repaired:** all **9/9** frozen strict
fixtures now pass and finish physically stable, preserving the four previously
stable fixtures. Direction reversal completes two verified steps at ticks 237
and 339 and remains stable through tick 510. Each other fixture completes one
verified step. Two final full replays are byte-identical.

All **88 workspace tests**, formatting and native/WASM Clippy pass. The 13
tooling/release checks and four engine tests pass with unchanged inputs.
Measured takeoff rebasing, native effort allocation, loaded-sole preparation,
measured-pelvis foot anchoring and coordinated touchdown/stance handoff repair
the physical failures. A raw-pointer displacement guard prevents held-target
filter overshoot from selecting a reversed transfer.

Canonical anatomy, scenarios, measurements, the frozen v24 input profile and
every physical step criterion are unchanged. Final source snapshots and
receipts are in `evidence/strict-stepping-2026-10-09/code-checks/`, `verified-a/`
and `verified-b/`; `comparison.json` confirms parity and preserved inputs.
See the [repair and fixture results](rust-strict-stepping-2026-10-09.md).
Production stepping/recovery remains disabled by default. Broader recovery,
cycles, terrain, browser and hardware release qualification remains open;
Pages promotion is still gated.

**2026-10-05 swing reach diagnosis:** read-only substep traces identify an
outdated takeoff anchor and an infeasible second right-foot path. At admission,
the foot has moved 132.56 mm from its saved anchor; the landing target exceeds
the current two-link IK reach by 76.06 mm. At the requested lift peak, measured
raise is 9.55 mm and the foot still carries 144.90 N. Hip roll uses at least 99%
of its allocated impulse cap during all 109 nominal-trajectory substeps.
First unloading occurs at 0.721 s; verified liftoff follows at 0.829 s, beyond
the unchanged 0.45-second trajectory. Installed and intended FK endpoints agree
within numerical precision. The fixture still falls at tick 260.

All **86 workspace tests**, formatting and native/WASM Clippy pass. Two full
strict replays are byte-identical; every pre-existing report field matches the
retained later-transfer baseline exactly. Strict stepping remains **1/9**,
with **4/9** physically upright. All 1,473 live-swing substep records, source
snapshots, executable and receipts are retained under
`evidence/migration-swing-diagnosis-2026-10-05/final/`. The initial disk-full
linking attempt is preserved. See the [diagnosis and plotted paths](rust-swing-diagnosis-2026-10-05.md).
Next is planning from measured takeoff state and qualifying landing reach with
the retained stance, then resolving unloading and posture dynamics. Production
stepping/recovery stays disabled; release admission remains false.

**2026-10-05 later-transfer reserve:** after a verified landing, an uncommitted
transfer can gradually redistribute the existing motor budget toward legs
carrying persistent measured sole loads. The leg priority is
`0.55 + 0.45 * load_share`; upper-body priority approaches 0.12 over the
0.6-second transfer clock. Native gains, anatomical ceilings, the 760 N m
aggregate ceiling and all physical step criteria are unchanged. First transfers
receive no new allocation; support loss clears it and committed swing uses its
original allocation immediately.

The right-foot-heading fixture now admits a second swing at tick 180 after the
unchanged 0.10-second readiness proof, then verifies liftoff at 229. It still
falls at tick 260 before a second landing. This late liftoff follows the nominal
0.45-second trajectory endpoint, so it does not establish a stable second step.
Held-target remains strictly qualified through tick 601; the other previously
upright fixtures and direction-reversal report are unchanged. Five fixtures
still land once. Strict stepping remains **1/9**, with **4/9** physically upright.

All **85 workspace tests**, formatting and native/WASM Clippy pass. A regression
checks landing/phase prerequisites, persistent versus transient load, empty
support, swing precedence and immutable body state; the native allocation test
also exercises the full later-transfer reserve with unchanged gains and caps.
Two fresh dedicated native replays are identical and match the retained probe.
Source stayed fixed and matches the tested snapshots. The final receipt and
comparison are in `evidence/migration-transfer-stance-2026-10-05/final-qualified/`;
it references the same-source code checks in `final-verified/`. A test-only borrow
error and a rejected attempt to reuse a rebuilt binary remain recorded. The
final physical reports were rerun on the dedicated acceptance build.

Directly coordinating stance IK with the final transfer reference, both alone
and with this reserve, regressed held-target and was removed. Sources, receipts
and losslessly compressed reports are preserved in the same evidence root.
Next work is measuring requested versus reachable swing paths and the delayed
physical lift, then resolving transfer/stance dynamics. Continuous stepping,
recovery and release during an admitted step remain unqualified. Production
stepping/recovery stays disabled; release admission remains false.

**2026-10-05 post-landing stability:** held-target now passes its strict
corrective-step fixture: one verified landing at tick 201, no fall through
tick 601 (10.02 seconds), and minimum pelvis height 0.942 m. Strict stepping
improves from **0/9 to 1/9**, with four physically upright fixtures versus three.
Five fixtures still complete a verified step. The other four landings still
lead to falls; direction reversal completes no step, and the three previously
upright fixtures still complete none. Their full reports are unchanged.

The stance blend now supplies anatomical position and velocity targets from
the complete trajectory, including its moving virtual pelvis reference. After
verified landing, the prior support leg's motor allocation fades to the ordinary
allocation over the same 0.6-second blend. This removes the abrupt loaded-hip
capacity drop while preserving native gains, every anatomical ceiling, the
760 N m aggregate ceiling, and the original full-swing allocation. Support loss
clears the reserve; a new committed swing takes priority immediately. Physical
readiness, liftoff, touchdown and step-count requirements remain unchanged.

Two regressions cover complete blend rates at two headings, native distal rate
installation, bounded targets, allocation handoff and opposite-side swing
precedence, unchanged gains/caps, support-loss clearing, one counted landing
and unchanged body snapshots. All **84 workspace tests**, formatting and
native/WASM Clippy pass. Two full strict replays are identical; sources stayed
fixed during verification and match the tested snapshots. Receipts, comparisons
and intermediate experiments are in
`evidence/migration-stance-tracking-2026-10-05/final/` and its parent directory.

Next work is coordinating post-landing stance goals with subsequent transfer
references and achieving feasible lateral unloading. The release-during-step
fixture still never admits a swing, so its requested release remains unexercised.
Production defaults still disable stepping/recovery and release admission
remains false. Earlier checkpoints below retain their recorded source scope.

**2026-10-05 swing tracking:** admitted swing now supplies anatomical velocity
targets alongside its position targets. Both IK samples use the same measured
pelvis pose; their coordinate difference over one substep feeds the native
motor's canonical angular-rate row. Existing pelvis-torque feedforward remains
additive, and requested-effort accounting includes the desired rate. This lets
damping follow the requested lift instead of always opposing its motion.
The original 4 cm sinusoidal lift, 0.45-second trajectory, gains, effort ceilings
and physical readiness/liftoff/touchdown requirements are retained.

The regression checks velocity-predicted forward kinematics at two headings
through start, mid-swing and completion, native distal motor rate installation,
hard ceilings, zero rates after support loss and unchanged body snapshots.
All **82 workspace tests**, formatting and native/WASM Clippy pass. Two full
strict reports repeat identically and match the retained candidate; sources
stayed unchanged throughout verification and match the tested snapshots.

Five fixtures now complete a verified step, including backward and held-target
which previously completed none. All five subsequently fall: backward at tick
208 versus 227 before, held-target at 474 versus 540. The three upright fixtures
match the prior physical reports exactly and still complete no step. Strict
stepping remains **0/9**. This advances swing tracking, without qualifying stable
stepping or recovery.

Five transfer/coordination policies failed physical admission and were removed.
A squared-sine lift envelope reduced verified-step fixtures from five to three
and was also removed. Their source, receipts and decisions are under
`evidence/migration-support-reserve-2026-10-05/`; five rejected reports are
losslessly compressed with verified hashes in `compressed-rejected.json`.
The final receipt and comparisons are in `swing-final/`. Next work is measured
post-landing stance tracking and feasible lateral unloading. Production defaults
still disable stepping/recovery, and release admission remains false.

**2026-10-05 landing target bounds:** landing now clips measured leg coordinates
to their declared anatomical limits before storing the posture blend's starting
targets, and zeros locked axes. Solver compliance permits small measured joint
overshoot; that allowance must not expand the controller's target limits.
The regression covers both boundaries on all eight leg joints at two headings,
checks capture through the landing path and confirms unchanged body snapshots.
A second regression preserves abort anchors, stance policy and step counts.

All **81 workspace tests**, formatting and native/WASM Clippy pass. Two fresh
strict replays are identical and remain **0/9**: three upright fixtures and
three one-step-then-fall fixtures. Every measured body, contact, motor and failure
result matches the prior verified source. The only requested-target differences
are 49 locked-axis values becoming zero. Sources stayed unchanged throughout
verification, and the final core files match the saved tested snapshots.

Two measured-posture abort handoff trials regressed upright fixtures from three
to zero and one respectively; both were removed. The existing abort policy
remains. Evidence and decisions are retained in
`evidence/migration-handoff-2026-10-05/decisions.json`; the verified receipt and
comparison are under `clamp-final/`. Feasible unloading, abort posture continuity
and full physical stepping remain unresolved. Production defaults still disable
stepping/recovery, and release admission remains false.

**2026-10-05 swing clock repair:** an admitted swing now has its own bounded
two-second phase clock. Previously, a late admission consumed the transfer's
remaining time and could abort before the 0.45-second trajectory finished.
In the preserved central-reference diagnostic, right-foot swing started at
tick 133 and aborted at 151; the repaired clock allowed verified liftoff and
loaded landing at tick 169. That fixture still fell later, so this is timing
evidence rather than step qualification.

The regression proves a late swing survives the old request deadline, still
rejects a continuously loaded slide, times out its own phase and leaves live
body snapshots unchanged. All **79 workspace tests**, formatting and native/WASM
Clippy pass. Two fresh strict replays are identical, remain **0/9**, preserve
three upright fixtures and three one-step-then-fall fixtures, and retain all
frozen readiness/liftoff/touchdown limits. Backward/reversal/held-target traces
change when a late swing is allowed to continue; reversal now falls at tick 224
instead of 233. Full stepping is still unqualified.

Central-reference, coordinated-stance, load-feedback and damping trials were
rejected and removed. Their receipts and decisions remain in
`evidence/migration-step-coordination-2026-10-05/phase-decisions.json`.
The restored controller retains the constrained IK repair and this clock fix;
`lib.rs` matches its prior baseline byte for byte. The build guard refreshed
both restored inputs, while subsequent compilation reused unchanged sources.
The verification receipt is
`evidence/migration-step-coordination-2026-10-05/phase-final/execution.json`.
Sources stayed unchanged throughout verification; release admission is false.
Next physical work needs a feasible unloading trajectory and a smooth posture
handoff when transfer aborts, without counting that handoff as a step.

**2026-10-05 build freshness repair:** repository native checks, disturbance
probes, worker builds and Trunk builds now compare local Rust input contents
with a ledger under the build cache before compiling. Changed/restored inputs
receive fresh timestamps; unchanged inputs preserve their cache. Missing or
invalid history refreshes all local inputs. The guard includes vendored files
and records its input fingerprint in native evidence.

All **13 tooling/release checks** pass, including a real pinned-Cargo regression:
A builds, B builds, timestamp-preserved A incorrectly runs B without the guard,
and guarded restored A rebuilds and runs A. A fresh repository disturbance build
refreshed 277 inputs and reproduced the verified lateral fixture byte for byte
(upright, zero completed steps). Sources stayed unchanged during that replay.
Evidence is under `evidence/migration-build-freshness-2026-10-05/`; the executable
regression preserves its separate `rust-freshness-integration-*` receipts.
JavaScript lint reports no errors and one existing native-runner warning.
This repairs the restored-source gap described below; direct Cargo invocations
must also call the guard when restoring checkpoints.

**2026-10-05 stepping continuation:** constrained leg inverse kinematics now
refines the virtual foot endpoint after anatomical angle clamping. An
independent forward-kinematics regression covers 36 shifted stance/swing goals
at three headings, requires less than 0.1 mm endpoint error and every anatomical
bound, and confirms that solving does not change live physics. The former
analytic-only solution missed the first tested swing goal by 1.28 mm.

Disturbance traces now include measured leg coordinates, configured axis
ceilings and delivered motor impulses. These observations leave the physical
reports unchanged. All **78 workspace tests**, formatting and native/WASM
Clippy pass. The fresh strict stepping replay remains **0/9**: three fixtures
stay upright without completing a step; three others complete one step and
subsequently fall. Transfer/stance/unloading experiments did not qualify and
were removed, retaining the existing controller policy and default disabled
stepping/recovery flags. No acceptance limit or motor ceiling was relaxed.

The authoritative receipt is
`evidence/migration-step-coordination-2026-10-05/final-fresh/execution.json`.
Its sources stayed unchanged, and all nine physical reports match the earlier
constrained-IK replay after removing the added observation fields. The earlier
`final/` receipt is rejected: restoring checkpoint files preserved older
timestamps, allowing Cargo to reuse an experimental executable. Updating those
timestamps and rebuilding produced the verified receipt. The subsequent build
freshness repair above now guards repository compilation commands.
Full migration and production promotion remain unaccepted.

**2026-10-05 native actuator momentum repair:** all **432/432** anatomical
motor probes now pass, covering 1,728 physical substeps at the unchanged
1e-6 N m s tolerance. Two standalone acceptance runs produce byte-identical
reports. Maximum delivered/body readback error is 1.2158e-7 N m s, down from
3.7956e-6; maximum paired body momentum residual is 9.0641e-8 N m s, and
reported opposite motor rows cancel exactly.

The isolated regression reproduced torque-free spherical spin drift in the
gyroscopic principal-frame round trip. Rapier now preserves angular velocity
exactly when all three principal inertias are equal, where gyroscopic torque
is identically zero. Unequal-inertia behavior retains its existing calculation.
The anatomical matrix test now requires physical delivery passes as well as
complete coverage. All **77 workspace tests**, three gyroscopic regressions,
three cached-row equivalence tests, native/WASM Clippy and formatting pass.
The 216-case contact stress matrix, 36 floor-disabled controls and three
striker cases also pass. Evidence and tested source hashes are retained in
`evidence/migration-actuator-gyro-2026-10-04/verification.json`; sources stayed
unchanged throughout verification.

This qualifies the native actuator probes. Full native/WASM equivalence,
strict stepping, physical recovery/cycles, terrain traversal, production
worker/browser qualification, hardware timing/touch and rollback qualification
remain open. Production promotion remains unaccepted.

**2026-10-03 migration continuation:** the v24 standing candidate increases
supported COM restoration from 3 to 5 rad/m without changing acceptance limits
or motor ceilings. Preliminary native standing passes all six public-mode
cases; the freshly built WASM artifact now passes all six with unchanged source
and artifact hashes. Selected pulls, terrain and striker checks also pass
natively. The
gain-4 trial is rejected for native drift and a terrain structural failure.
Fresh verification passes all **73 Rust tests**, native/WASM Clippy and
formatting. All **15 native stages pass**, with byte-identical repeated
foundation, six-case public-mode standing, terrain and striker reports and
unchanged source hashes. All **10 final browser stages pass**, including six standing
cases, 42 terrain cases, physical striker, worker lifecycle and both renderers
at root/subpath origins. The normal preview build is byte-identical to the
verified root artifact. Final evidence is under
`evidence/migration-remaining-final-{native,browser}`.

Native quiet diagnostics now select protocol, playground or both modes, and
foundation verification repeats the six-case public-mode matrix. The old v23
native playground +60° trial fails foot drift at tick 1871, exposing the gap
in the previous protocol-only native diagnostic. Scoped disturbance exit
codes now reflect their actual results while release admission stays false;
invalid profiles leave no evidence directory. Windows browser builds preserve
the inherited Cargo/Node paths even when the environment spells `Path`.

Fresh v24 strict stepping is **0/9**, crouch-left recovery records zero
recoveries in 25 seconds, and isolated-box contact stress passes **200/216**.
The full retained-app suite reports **375 pass, four fail, seven skip**: the
same slow-pull, planted-reversal and two strong-pull/fall failures remain.
Fresh idle browser timing fails on both renderers: WebGL2/WebGPU record
197/483 core tick deadline misses at simulation/wall ratios 1.000/0.986.
Source and artifact hashes remain unchanged; full performance admission stays
open (`evidence/migration-remaining-performance/report.json`).
Strict stepping, automatic recovery/cycles, contact stress, full disturbance
and terrain traversal coverage, the retained-app failures and hardware timing
still block full migration and production promotion. See [the continuation
record](rust-rework.md#migration-continuation--2026-10-03).

**2026-10-02 migration integration:** this source checkpoint includes the Rust
preview, audited engine, native/WASM tooling, renderer, UI, terrain and striker.
Production remains on the existing application while
physics and performance acceptance are open. Cargo output from the standalone
vendor build is now ignored along with workspace builds and local evidence.

The native acceptance CLI rejects unknown/misplaced diagnostics, duplicate or
missing options and nonfinite headings before creating evidence. Four new
argument tests bring the passing Rust suite to **71 tests**. An executable-level
typo check confirms exit 2 without an evidence directory. Native/WASM Clippy,
formatting, TypeScript, the application build, Pages export/asset verification
and four JavaScript tooling tests pass; JavaScript lint has no errors and eight
existing warnings. The full app
suite reports **374 pass, four fail, seven skip**: slow pull falls at tick 304,
planted reversal at tick 228, and two strong-pull/fall assertions fail. Evidence:
`evidence/migration-final/` and `evidence/migration-final-native/`.

The final native runner passes all 13 stages. Both foundation runs pass gates
A–C and all eight selected pulls; both 21-case terrain runs and both three-case
striker runs pass. Every repeated report is byte-identical, and source hashes
remain unchanged (`implementedChecksPassed: true`, `repeatable: true`,
`releaseAccepted: false`). Gate D and full release admission remain incomplete.

Fresh v23 WASM standing passes **five of six cases**. The +60° playground case
fails left-forefoot drift at tick 1902, substep 3: 0.010000321 m against the
unchanged 0.010 m limit. All three protocol headings and the other two playground
headings pass the full two-second warmup plus 30-second observation. Source and
artifact hashes remain unchanged. This supersedes the older all-six passing
v22 result for the current source. Evidence:
`evidence/migration-final-browser/quiet/report.json`.

The combined browser verifier completes all ten stages: **nine pass**, with
quiet standing the sole failed stage. Root/subpath builds, 39 UI checks at each
origin, both fault suites, worker lifecycle, all 42 terrain cases and all six
striker cases pass. Desktop and mobile screenshots were inspected. See
`evidence/migration-final-browser/execution.json` and its per-stage reports.

Native foundation verification now repeats terrain and striker cases as well
as gates A–C and selected pulls. Browser verification includes the existing
42 terrain cases and physical striker checks against the freshly built root
artifact, plus root/subpath UI, faults, worker lifecycle and quiet standing.
Neither command claims recovery, terrain traversal or hardware-performance
admission. The browser standing failure, strict stepping, automatic
recovery/cycles, contact stress coverage,
the four retained-app regressions and performance remain release blockers.

**2026-10-02 articulation feasibility and further motion diagnostics:** a new
native-only diagnostic preserves all 25 canonical bodies, colliders and joint
definitions while testing reduced-coordinate articulation. Full conversion
cannot integrate: Rapier's two-axis wrist branches are unimplemented, and its
spherical coordinates do not match the anatomical motor/limit coordinates.
A hinge-only hybrid keeps all anatomy and integrates. Read-only final
generalized velocities expose stale rigid-body velocity reporting at the last
solver substep: the floorless hybrid passes the unchanged momentum threshold
over 240 substeps when measured from those generalized velocities. This is
diagnostic evidence, not permission to ignore the ordinary body-state failure.
With real floor contact, the hybrid exceeds the unchanged right-thigh angular
limit tolerance. The new vector contact-momentum diagnostic also fails for
the impulse baseline, so that comparison is not a qualified contact gate.
No controller migration, vendor change or performance promotion follows.
See the detailed results in [the Rust record](rust-rework.md).

Further capture, contact-moment and stance-frame trials do not fix the four
application failures. The stance-frame variant passes the two overload/input
tests but makes the two gentle-pull falls earlier; it was rejected. Recovery
trials using a virtual upright pelvis, canonical resting targets or measured
gravity/support feedforward each still produce zero recoveries and stall in
bracing. All these runtime experiments were restored byte-for-byte to the
previous checkpoint. The app's 374 pass / four fail / seven skip result and
disabled recovery default remain current. Commit, push and merge are pending.

**2026-10-02 recovery and solver diagnostics:** Rust now has an opt-in,
contact-gated recovery prototype and 24 exported landed-pose seeds, constructed
before integration. Recovery changes bounded joint motor intent; it never moves
dynamic bodies directly. The default `recovery_enabled` is false. No dynamic
recovery trial has passed, so this is unfinished implementation, not admission.
Repeated crouch trials still tip forward; the foot-reserve trial
also trips the unchanged 5 mm self-contact limit at 5.113 mm
(`evidence/finish-rust-recovery-crouch-21`). A subsequent trial returns a failed
rise to passive falling and resumes bracing after settling, without a structural
failure, but remains in bracing at 25 seconds (`finish-rust-recovery-crouch-22`).
The full Rust suite passes 64 tests (`finish-rust-tests-26.log`), including six
recovery checks for seed validation, read-only planning, coupled interpolation,
absent support, torque ceilings, input lockout, impact interruption and passive
fallback after a failed rise. Native/WASM Clippy passes on the retained source
(`finish-rust-lint-29.log`), as do TypeScript and the changed scripts' lint
(`finish-recovery-typecheck-28.log`, `finish-recovery-scripts-lint-28.log`).
An arm-extension experiment did not advance beyond bracing and was reverted.
The final replay (`finish-rust-recovery-crouch-24`) preserves the earlier
initial/final snapshots and every sampled physical snapshot exactly, still with
zero recoveries and no structural failure.
The diagnostic now requires a further second of measured standing after motor
handoff, in addition to the second required before completion. Actual repeated
strike/recovery cycles and browser recovery remain unqualified.

The 10 outer/32 internal/64 falling-pass diagnostic passes all three native
quiet-standing cases, all three impacts, 21 terrain checks and eight selected
pulls. Reports are `evidence/finish-rust-{quiet,impact,terrain,pulls}-outer10-pgs32-21`.
It still fails actual browser timing: 278/1,819 charged update misses on WebGL2
and 192/1,804 on WebGPU, with maxima 50.7/27.5 ms against 16.667 ms. Round-trip
p99 is 130.1/48.5 ms against 20 ms. The run uses the frozen `rust/dist/striker-10`
artifact with an explicit worker profile override; served/local asset hashes,
the profile and observed source hashes remain unchanged. See
`evidence/finish-rust-performance-outer10-pgs32-21/report.json`. This candidate
is not promoted. The performance probe now records optional profile/asset
provenance; its targeted lint passes (`finish-profile-probe-lint-24.log`).

The four retained-app motion failures, automatic recovery, strict stepping,
terrain traversal and performance remain release blockers. The default physics
profile and production entrypoint remain unchanged. No commit, push or merge
has occurred.

**2026-10-02 retained-app impact repair:** the left-hand chest-drag regression
is fixed by increasing internal solver passes during recovery from 32 to 64.
Standing keeps its configured budget, restored immediately after each recovery
integration. The measured peak joint-anchor separation falls from 27.4 mm to
8.4 mm under the unchanged 10 mm assertion; both chest-drag cases retain their
contact, penetration and ownership checks. The contact/motor selection passes
28/28 (`evidence/finish-chest-regressions-11.log`). No test deadline, body pose,
motor ceiling or contact threshold was changed.

The complete build/test run in `evidence/finish-npm-test-11.log` confirms the
contact repair but reports 372 pass, five fail and seven skip. Four failures are
the existing slow-pull, planted-reversal and two physical-overload/lockout
cases. The fifth is a concurrent-test timing failure in the H77 selector test.
That selector now uses a controlled clock; a separate boundary regression
admits exactly 8 ms and requires a timeout transition at 8.001 ms. All four H77
tests pass in `evidence/finish-implicit-clock-12.log`. The final complete
application test replay reports **374/385 passing, four failures and seven
skips** in `evidence/finish-node-test-12.log`; only the four existing motion
failures remain. Real runtime/browser timing limits remain intact. Typecheck
passes; lint has no errors and eight unchanged warnings
(`finish-typecheck-11.log`, `finish-lint-11.log`); the final test edit also passes
targeted lint (`finish-implicit-lint-12.log`).

Two native solver-budget diagnostics remain unpromoted. Four outer/32 internal
passes fail all three quiet-standing cases; eight outer/32 internal passes
pass the three 30-second quiet cases and all three impacts, but fail gentle
rubble at 5.188 mm pelvis/hand overlap (5 mm limit) and the rotated right-hand
pull at 0.55958 m pelvis height (0.56 m limit). Evidence is in
`evidence/finish-rust-{quiet,impact,terrain,pulls}-outer8-pgs32-12` and
`evidence/finish-rust-quiet-outer4-pgs32-12`. The v23 source/default profile and
browser artifact remain unchanged; these diagnostics do not qualify timing.

The balance geometry, trigger-margin, recentering and pelvis-gain experiments
did not repair the motion failures and were reverted; their failing logs remain
under `evidence/finish-balance-*-11.*`. `BalanceController.ts` and the balance
diagnostic script retain their pre-experiment hashes. Automatic Rust recovery,
strict steps, terrain traversal and performance remain open; no commit, push,
merge or deployment has occurred.

**2026-10-02 physical striker integration:** Rust now owns the protocol room
and its kinematic impact head. Positioning checks the complete machine against
current and predicted body geometry; retraction checks clearance and refuses
new/deeper overlap. Only a measured solver impulse increments the strike count.
The button and P shortcut reject paused/busy requests, snapshots expose the
integrated apparatus, and falls cancel body input without replacing bodies.
The separate protocol geometry export preserves the frozen anatomy/course data.

The current candidate is `rust-physics-v23-passive-fall`. Falling uses canonical
passive damping and cubic resistance near joint limits, under the existing
individual/aggregate torque ceilings. Standing retains 20 internal solver
passes; falling uses 32, explicitly recorded in the profile. The three native
heading/impact cases and all 21 native terrain cases pass with unchanged
structural thresholds. All 58 Rust tests and native/WASM Clippy pass. Evidence:
`evidence/finish-rust-striker-passive-impact-10`,
`evidence/finish-rust-striker-passive-terrain-10`,
`evidence/finish-rust-striker-tests-10.log`, and
`evidence/finish-rust-striker-lint-10.log`.
The frozen final browser artifact passes six impact cases and a second strike
in each backend, 42 terrain cases, and 39 existing interaction checks across
WebGPU/WebGL2. The mobile machine framing is corrected and visually checked.
See the reports in `finish-rust-striker-browser-10`,
`finish-rust-striker-terrain-10`, and `finish-rust-striker-ui-10` under `evidence/`;
`finish-rust-striker-frozen-10` preserves the source and verification index.

The earlier `finish-rust-striker-foundation-06` passes the complete scoped
foundation twice with identical reports, but predates v23 passive fall changes;
it is not full v23 admission. The five retained-app failures, automatic recovery,
strict stepping, terrain traversal and performance remain open. No commit,
push, merge or production entrypoint change has been made.

**2026-10-02 terrain integration:** the Rust playground now creates the canonical
course solids, spawns the connected anatomy at the selected station, measures
upward support from actual terrain contacts, and integrates the wobble decks.
Station/difficulty controls create a fresh generation while preserving pause;
the renderer uses the same canonical triangles and integrated environment poses.
All 21 native two-second structural/contact cases and all 42 actual WASM cases
(21 each on WebGL2/WebGPU) pass. Browser checks also verify deck pause/resume,
station changes while paused, and removal of course geometry on returning to
the protocol. Evidence: `evidence/finish-rust-terrain-native-04/terrain.json`
and `evidence/finish-rust-terrain-browser-06/report.json`.
All 50 Rust tests pass; the native foundation and eight selected pulls repeat
with byte-identical output. Native/WASM formatting and Clippy pass after a
test-module ordering repair, whose unchanged implementation and test content
are checked in `evidence/finish-rust-terrain-checkpoint-06.json`. The existing
browser interaction matrix also passes 39 checks across both backends
(`evidence/finish-rust-terrain-ui-06/report.json`).

This is scoped terrain integration evidence. Five slope/rubble setups fall
within the two-second native observation; successful balance, terrain traversal
and recovery are not qualified. Flat and hurdles start on the base floor, so
their idle cases do not claim obstacle impact coverage. The five retained-app
failures below remain open. A longer unchanged-input diagnostic observes the
two strong-pull falls at ticks 193 and 250, after their existing 180-tick test
deadline (`evidence/finish-overload-observation-03.log`); the tests and thresholds
have not been relaxed. Striker, recovery, strict stepping and performance still
block release, commit/push/merge remain pending, and production is unchanged.

**2026-10-02 regression repair in progress:** six of the eleven recorded
retained-app failures have been repaired in focused checks. Recovery foot
planning now requires an exact, level floor plant and includes the anatomical
hip line in its candidate search. Full-extension arm reconstruction is stable
across headings, mutable rest poses no longer alias canonical anatomy, and the
balanced half-kneel fixture carries its projected mass inside actual loaded
contacts. Native stream initial snapshots were refreshed without changing
historical input commands or recorded transfers. The forward-plant and arm
tests now use the actual fixture geometry and legal elbow reach.

The current focused recovery/geometry matrix passes 63/63. The dynamic matrix
still fails five tests: slow pull/reversal, planted reversal, left-hand chest
drag joint separation, and the two overpowering-pull fall/lockout cases.
Two experimental step changes were rejected and removed after broader checks
exposed transfer regressions. The final full suite records **372 passes,
5 failures, 7 skips** (384 tests, including two added regressions) in
`evidence/finish-npm-test-02.log`. Build and typecheck pass; lint reports no
errors and the same eight warnings. Focused results are preserved in
`evidence/finish-recovery-geometry-05.log` and
`evidence/finish-dynamic-independent-poses.log`. The previous full run's two
additional controller-support failures are resolved: exact foot search now
includes the anatomical hip line, and movement fixtures supply a reachable
crouch instead of relying on shared rest-pose mutation.
Rust striker/terrain, automatic recovery, strict stepping and performance
qualification remain open. Work is on `codex/finish-rust-rework`; no commit,
push, merge or deployment has been performed in this continuation.

**2026-10-02 migration continuation:** Rust now observes physical falls,
updates the fall counter and cancels active/future body input. Reset opens a
fresh trial; automatic recovery is still absent. Pinned tool bootstrap and
root/subpath browser verification are reproducible package commands.
The unchanged v22 Coulomb-contact profile passes native gates A–C and eight
selected pull probes twice with byte-identical results. All 46 Rust tests and
all six WASM standing trials pass. Root/subpath browser checks pass 78 assertions
across WebGPU/WebGL2, plus 35 worker lifecycle checks and 24 fault checks.
The final preview matches the tested root artifact byte for byte.
The last nine-fixture step diagnostic had zero passes and was not rerun in this
continuation. Physical
striker/stations, recovery and full performance/release qualification remain
incomplete; production stays on TypeScript.
The current browser performance probe also fails update/IPC timing in both
backends (worker round-trip p99 73.8/94.1 ms; required maximum 20 ms).

The full retained application suite records **364 passes, 11 failures,
7 skips**. These 11 failures also occurred in the preceding run; its H77 selector
failure did not reproduce. Build and typecheck pass; lint has eight existing
warnings and no errors. These failures remain release blockers.
See the [current verification record](rust-rework.md#migration-continuation--2026-10-02)
for scope, commands and local evidence. Earlier checkpoints below are historical.

**2026-10-01 Rust checkpoint:** the isolated [Rust rework](rust-rework.md)
passes its implemented native gates A–C (geometry/contact/joints, loaded feet and
chains, and 2 s settle + 30 s quiet standing at three headings).
`rust-migration-frozen-01` records two identical native executions on preserved
source and passes all eight selected pull probes, repairing the three v13
failures. Native lifecycle controls are tested. Gate D remains incomplete because
corrective-step counts, swing-release and full coverage are not qualified;
E–I and full browser/release qualification remain incomplete. Production still
uses the legacy TypeScript entrypoint. Native quiet-standing success does not
resolve the browser candidates' failures below.

The opt-in corrective-step follow-up records measured liftoff and touchdown in
four fixtures, but none of the nine strict recoverable fixtures pass. Post-step
falls, final foot orientation, lateral transfer, reversal and swing-release
remain unresolved. The candidate stays disabled; the narrower native pull
passes above do not admit completed steps. See the [step evidence](rust-rework.md#corrective-step-follow-up).

**2026-10-01 renderer checkpoint:** both browser modes now expose Auto/Low/High
quality, shared presentation metrics, and redraws on demand while paused.
The local `evidence/sandbox-acceptance/report.json` records 16 before/after replay
combinations, four UI combinations, and four 60 s live trials without reported
errors. Its 18 recorded scene/runtime/core hashes still match the reviewed tree;
this is a partial source fingerprint. Live simulation/wall-time ratios are
0.416–0.746, so these trials do not establish real-time physics performance.
The report does not qualify reference hardware or GPU execution. See
[rendering results and reproduction](rendering.md) and [local evidence](evidence.md#rust-and-renderer-local-evidence).

**H78–H80 bounded result, 2026-09-30:** the [coordinated contact-force
investigation](standing-h78-h80-contract.md) has exhausted all three frozen,
opt-in candidates. H78 and H79 fail the 8 ms controller deadline. H80 fails two
references on that deadline (8.0155/8.468 ms); the -5 mm reference reaches tick
120 but fails entry/hold with 74.49 cm pelvis error, 11.96 cm foot error,
2.972 m/s linear speed and 2.774 rad/s angular speed. At its first admissible
allocation, requested horizontal restoration already changes direction; native
command readback remains within tolerance. This identifies the earliest observed
discrepancy, not the complete physical cause. All native first-failure replays
match 25 bodies exactly. No standing or transfer operating range is validated.

The implementation includes constrained allocation under combined motor ceilings,
one serializable support state shared with a non-mutating forecast, complete
controller-state snapshots, explicit H78–H80 selectors, and the previously
unfinished two-renderer browser performance stage. Forecast admission stays
disabled. Local-return, prediction validation, slow-pull/reversal acceptance,
short screens, official standing, sustained/held-out and actual browser gates
remain incomplete because feasibility failed. There is no fourth attempt.

Frozen checks: typecheck, lint and build exit 0 (lint: seven warnings); regressions are
80/82 pass, one failure and one skip. The strong-pull physical-fall failure is
reproduced on the exact saved pre-change source. Archive compatibility passes
inside acceptance (and skips without its archive in the standalone suite).
Source and manifest fingerprints remain unchanged during each completed run.
Subsequent shared-workspace renderer/runtime edits are preserved and are outside
this result's source scope. A verified `evidence/standing-h80-v1/frozen-workspace`
matches all 354 frozen source/config fingerprints for reproduction.
The dirty starting tree and historical evidence are preserved; normal startup
stays legacy. See [evidence storage and run paths](evidence.md#h78h80-local-evidence)
before copying the workspace: these new evidence directories use verified
junctions to available storage. Recovery, terrain and release acceptance remain
open.

**H76/H77 standing result, final verification 2026-09-28:** [H76](standing-h76-contract.md) now
freezes a successful six-snapshot scalar native build/read diagnostic with zero
physics steps; its rows appear coherent but do not explain within-step forefoot
motion. [H77](standing-h77-contract.md) implements opt-in implicit native posture
feedback and passes current-state torque/readback equivalence, but improves only
one of three preserved references. The official acceptance operation fails all
three reference entries (ticks 132/120/120), before local return or later gates.
Output-neutral actual-substep traces show responsive, uncapped joint rows and
select the contact-wrench/load-distribution evidence branch. H77 is not promoted;
normal startup remains legacy, and H74/H75 remain immutable failed experiments.
Final checks: typecheck and production build pass; lint exits 0 with 6 warnings;
the complete unit run is 325/339 pass with 13 failures and 1 skip; the focused
balance/controller run is 25/28 pass, with both active-step commitment tests now
passing and the separate slow-pull/reversal tests still failing at ticks 304 and
228. The full physics harness is 6/63 pass. The H77 standing operation itself
fails feasibility and leaves every later acceptance stage incomplete.

**Recovery performance investigation, 2026-09-27:** repeated foot-placement searches during rolling are skipped until a placement phase needs them. A 10-second strike replay fell from 146 seconds to 20–27 seconds on this host; it still does not run in real time or complete get-up. Lower solver counts and rolling motor trials were rejected. Per-tick diagnostics expose a transient joint-limit violation missed by coarse sampling. See the [measurement, retained changes and limitations](recovery-responsiveness-profile.md).

**Recovery launch correction, 2026-09-27:** the application now integrates recovery motors with native contact/joint constraints instead of applying frame-sized torque impulses first. A real strike reproduced the old floor-to-ceiling launch; the corrected 10-second replay remains near the floor (maximum pelvis height after landing 0.1222 m). The character still stalls while rolling toward a brace, so get-up acceptance remains open. See [cause, comparison and validation](recovery-native-actuation-fix.md). The standing investigations below retain their recorded source scope; this correction does not establish quiet standing.

Updated 2026-09-27 through H77-v1. **Quiet standing remains the first unfinished TODO.** H74/H75 remain rejected and frozen. H76 closes only the static native build/read gap. H77's native-PD request identity passes, but its reference feasibility and promotion gates fail. No physical acceptance checkbox closes.

One confirmed mistake was rejecting a helpful correction because it missed a preferred extra margin, then keeping an old command that caused much more wobbling. The [plain-language explanation and replay evidence](standing-h71-contract.md#plain-language-explanation) describe why. The experimental repair retains a tested correction within the actual speed limits when the preferred margin is unreachable; it does not establish lasting balance.

Some short tests improved, but H66's longer test has a recorded speed failure. H73's first three concurrent simulations exhausted memory without final reports. Any H73 retry must preserve their partial results and run **one simulation at a time**, following the [H73 restart procedure](standing-h73-contract.md#first-attempt--memory-failures-no-complete-screen). These remain experimental fixes. Weight transfer, stepping, recovery, interactive performance, and full application verification remain open. Dated reports retain their own evidence and limitations.

## Coordinated investigation — bounded negative result

The [coordinated stabilization plan](standing-stabilization-plan.md) now has an
explicit shared experimental controller and serial acceptance operation:
`node scripts/standing-acceptance.mjs`. [H74-v1](standing-h74-contract.md) freezes
numeric limits, preserves nine exactly replayed failure captures, exercises six
failure transitions and verifies unchanged default behavior against archived
source for thirty steps. The three reference trials fail at ticks 133, 120 and
123. Later physical stages are incomplete; the command exits 1. No validated
operating/backup region, MPC result or interactive-performance claim follows.
The H42 rigid comparison reproduces its representation difference. H75's new
frozen diagnostic verifies all 176 listed H74 artifacts and 315 retained source
hashes, then completes six exact controls and 168 independently repeated signed
motor commands. Zero native position stiffness excludes direct excitation by
the motor's arcsine position-coordinate formula in these states. Substantial
relative forefoot motion in locked directions remains unexplained; unchanged
contact counts do not isolate contact or constraint effects. Compiled raw/finalized
row reconstruction is incomplete after the three-build budget is exhausted.
Next freeze the [six-snapshot build-and-read diagnostic](standing-h75-contract.md#concrete-next-diagnostic),
using zero new physical steps. All failures and sources are retained.

H73 remains a **pending comparison candidate**, not an accepted baseline or a
prerequisite for this investigation. Its documented serial restart remains the
procedure for evaluating H73. H66's 1.72–1.88 s timing is average preview time
under concurrent diagnostic runs, not an isolated application measurement.
H74 changes neither physical thresholds nor historical results. It remains
opt-in; standing, transfer, stepping, recovery and release gates stay open.

## Source and evidence scope

The reviewed working tree starts at `44fc25437d841151c704895c1fb92de4c2e468d0` and includes uncommitted physics, test, and workflow changes. The [documentation checkpoint manifest](checkpoint-2026-09-26.json) records a partial source fingerprint and missing artifact inventory. It is not a physics test report or a complete dependency fingerprint.

The default physical controller retains [H51](standing-h51-contract.md); H74 adds an opt-in candidate and shared application/harness frame function. Its tests and thirty-step disabled comparison do not refresh H51's longer physical histories. [H44](standing-h44-contract.md), [H41](standing-h41-contract.md), [H40](standing-h40-contract.md) and the earlier [geometric-inertia report](inertia-correction-2026-09-27.md) retain their source-scoped results. Earlier results below retain their dated source scope. The [September 26 investigation](physics-standing-contract-2026-09-26.md) still has missing original reports; recovered artifacts and limitations are listed in [evidence availability](evidence.md). Missing reports cannot be treated as independently verified results merely because their counts appear below.

## Recorded gates

[H52](standing-h52-contract.md) verifies native motor inspection and added knee
authority. [H53](standing-h53-contract.md) exposes scaling and search limitations;
[H54](standing-h54-contract.md) reaches the local foot-motion deadband using all
fourteen existing leg axes. Continuous [H55](standing-h55-contract.md) then fails
all three short screens: foot drift stays below 1 cm, but hand and forefoot
speeds exceed the frozen limits. All 2,160 live predictions are exact. These
diagnostic changes do not modify the H51 production candidate or close any gate.

[H56](standing-h56-contract.md) adds measured pelvis pose rates to the local
objective. Its bounded variant meets the declared foot/pelvis task targets at
all three saved states; the zero-target variant does not. Five mathematical
tests and targeted lint pass. [H57](standing-h57-contract.md)'s completed short
screens fail speeds at all headings despite foot drift below 1 cm and 2,160 exact
predictions. [H58](standing-h58-contract.md) corrects two early isolated states
with full calculated updates under the original motor ceilings; later +π/3
peak states remain uncorrected. This is limited local search evidence;
[H59](standing-h59-contract.md)'s completed short screens pass at 0 and +π/3,
but retain eight hand-speed failures at −π/4. All 2,160 predictions are exact.
[H60](standing-h60-contract.md) uses the twenty existing arm motor axes to
correct two distinct retained hand-speed failures in one local iteration each.
[H61](standing-h61-contract.md)'s completed continuous arm-stage evaluation
bounds all observed speeds but fails +π/3 foot drift at 10.095 mm. All 2,160
live predictions match exactly; the full archive reconstructs all original files.
[H62](standing-h62-contract.md) reduces foot position error in three saved
states using the existing leg motors, but only one meets the complete local
tracking/pelvis deadband. [H63](standing-h63-contract.md)'s continuous position
feedback fails all three speed screens and falls at +π/3; its full verified
archive preserves 417,945 queries and 1,906 exact live predictions, with 254
guarded skips after the fall. [H64](standing-h64-contract.md)'s coupled local
speed search reduces the violations but does not meet all search guards.
[H65](standing-h65-contract.md) expands improving probe directions and reaches
the stricter speed reserve at all three first-failure states. Its 947 copied
queries and earlier comparator failures are archived. [H66](standing-h66-contract.md)
reproduces all 947 local queries through its extracted helper and passes all
three short standing screens. Its 489,616 queries produce 2,160 exact live matches
with bounded speeds, drift, support and state. The unchanged 2+30-second
diagnostic was launched and later recorded a speed failure at tick 978. Preview cost remains 1.72–1.88 seconds per step;
interactive performance and production adoption remain unresolved.
[H67](standing-h67-contract.md) finds physically bounded previous-command seeds
at three saved states. [H68](standing-h68-contract.md)'s cached response matrices
improve those seeds but beat the original selected correction at only one
heading; neither local comparison establishes continuous performance.
[H69](standing-h69-contract.md)'s continuous cache candidate passes heading 0
but fails speeds at −π/4 and falls at +π/3. Its 2,147 exact live predictions and
thirteen guarded skips are preserved in a complete verified recursive archive.
Reduced query counts do not establish an acceptable replacement.
[H70](standing-h70-contract.md) passes its completed −π/4 short screen; the
other two headings remain pending while H73 tests the separate fallback repair.
The H66 long history has a known heading-0 speed failure at tick 978;
[H71](standing-h71-contract.md) confirms that physically admissible commands were
discarded because they missed an internal search margin. Complete long results
remain unreported in that checkpoint; this known failure already prevents a pass.
[H72](standing-h72-contract.md) finds corrections within the actual speed limits
at all three saved failure states by extending the copied search and retaining
valid fallbacks. This is local evidence, not continuous standing success.
[H73](standing-h73-contract.md) verifies local replay and disabled compatibility,
but its first concurrent enabled runs end in memory errors without final reports.
Repeat one simulation at a time under the documented runtime bound, preserving
all partial results. No completed serial screen is established by this update.
All 67 production/package files remain unchanged. No standing or production
gate closes before complete independent acceptance results.

[H45](standing-h45-contract.md) rejects a forefoot target-frame correction.
[H46](standing-h46-contract.md) verifies exact two-motor predictions but leaves
one local angular-speed failure. [H47](standing-h47-contract.md) uses the six
existing distal motors to reduce all three sampled peaks and keep the next
five steps bounded. Every live output matches its copied prediction for all
25 bodies. Earlier peaks remain in the full histories; no standing or P1
prediction gate follows from these local results. [H48](standing-h48-contract.md)
bounds every observed speed with exact predictions on all 2,160 steps but
fails the +π/3 foot drift bound (11.208 mm in the short screen). No 30 s
acceptance run follows. [H49](standing-h49-contract.md) adds horizontal motion
correction through the same native motors and passes all three short screens,
then fails all three 2+30 s foot-drift bounds (26.035/29.916/23.336 mm).
All 5,760 long-run steps match their exact previews. All 67 production/package
files matched H44 during those evaluations; the results do not replace
production acceptance.

[H50](standing-h50-contract.md) checks search convergence on isolated saved
states while H49's longer evaluation ran unchanged. One full calculated
update improves the local residual more than four 1 Nm-limited updates at
each heading, within the existing motor ceilings and speed guards. This is
an opportunity to reduce calculation cost, not a measured online speedup.

| Gate | Latest recorded result | Scope and qualification |
| --- | --- | --- |
| Current geometric-inertia candidate | Mathematical defect corrected; candidate unaccepted | [H38 audit](standing-h38-contract.md) identifies 13 incorrect automatic segment tensors. Explicit geometric mass properties are now supplied at construction. Solver, geometry, collisions and actuator ceilings are unchanged. |
| Current focused validation | 69/69 selected tests pass; typecheck and modified-file lint pass | [H51](standing-h51-contract.md): original swing-foot assertion now passes, and all twelve test files remain unchanged. The earlier unused `RIGHT` warning in unchanged `EmbodiedCharacter.ts` is outside this lint selection. This count is not the physics harness scenario count. |
| Swing foot target | Original anatomy failure repaired within unchanged anatomical limits | [H51 evidence](checkpoints/2026-09-27/h51-swing-range-production/manifest.json): target utilization follows the existing lift envelope, releasing the planted reserve while raised and restoring it at touchdown. |
| Finite physical state | Mass and center of mass now checked | [H44 evidence](checkpoints/2026-09-27/h44-finite-mass/manifest.json): the identical failed multibody trajectory is rejected at its first non-finite COM, one tick before the previous inertia exception. |
| Relaxed elbow clearance | Original lean/twist/heading cases pass | [H41 evidence](checkpoints/2026-09-27/h41-elbow-clearance/manifest.json): 25 mm minimum lateral clearance, four geometric regression tests, original joint bounds and physical contact tests preserved. |
| Active hand reach | Chest contact repaired within unchanged limits | [H40 evidence](checkpoints/2026-09-27/h40-bounded-reach/manifest.json): both original contact fixtures pass. Left-hand contact occurs in `reacting` before falling; the test now requires this. The later falls remain unaccepted balance/recovery behavior. |
| Current 2+30 s standing | All three headings fail | [Fresh H51 capture](checkpoints/2026-09-27/h51-swing-range-production/manifest.json) exactly replays H44/H41/H40 and the inertia candidate: 0.04451–0.05895 m foot drift, 0.61833–0.71482 m/s peak linear speed. Upright, double support and zero steps do not establish acceptance. |
| Corrected-inertia constraint calibration | Rigid impulse trees still topple; hinge-forest experiment rejected | [H42](standing-h42-contract.md) preserves the rigid representation difference. [H43](standing-h43-contract.md) retains anatomical motion but leaves upright at tick 109 and becomes non-finite at tick 133. No controller/runtime change is adopted. |
| Fresh baseline tooling | Capture succeeded; five CLI regression tests passed | [New baseline and source-scoped verification](checkpoints/2026-09-26/fresh-baseline-validation.json). Four historical artifacts are recorded missing; no inherited focused result and no physics executed. |
| Regenerated allocator replay | Exact replay of both ticks; three identical control histories | [New capture and verified archive](standing-h8-evaluation.md); the missing original is not recovered. |
| H8 hypothesis | Rejected after all six 2+10 s screens failed | Static and local checks pass; prolonged standing does not. No production adoption. [Results and event counts](standing-h8-evaluation.md). |
| H12–H21 continuation | No accepted repair | [Declared experiments and outcomes](standing-h12-contract.md). H13 and H20 combinations pass short-screen speed bounds but fail foot drift; holding references still fails. Production physics was unchanged at that checkpoint. |
| H22–H24 continuation | Rejected | [Idle forcing/stance-task screens](standing-h22-contract.md): H22 passes speeds but fails 0.01378–0.01777 m foot drift. H23/H24 also fall at −π/4. A trace boundary fix now separates recovery commands from standing contributions; no production repair adopted. |
| H25–H26 continuation | Rejected | [Displacement feedback and fixed local leg targets](standing-h25-contract.md) still fail drift or stability. Six control histories replay exactly. |
| Native constraint calibration | Constraint representation isolated in the rigid fixture | [Fixed multibody comparison](rapier-constraint-calibration.md#fixed-multibody-comparison) keeps 25 separate dynamic bodies and limits foot endpoint drift to 0.45 mm, while fixed impulse joints collapse. Both f32/f64 impulse variants fail. Anatomical motion is locked in these fixtures; no articulated standing repair is established. All 65 production source/package files matched the regenerated capture at that checkpoint. |
| Focused physical suite | 140/142, exit 1 | September 24 retained-source report; slow-pull sequence falls at tick 216, planted reversal at tick 343. Original focused report/log unavailable at their documented paths. |
| Historical quiet standing | Failed at all three headings | Earlier 2 s settling plus 30 s observation. Bodies remain upright with zero steps, but drift and speed limits fail. An upright state is insufficient. |
| Balance probes | Both fail | Recorded capture/force feasibility and pelvis-height failures remain unresolved. |
| Small-impulse diagnostic | Precision-sensitive residual identified | [Matched native f32/f64 comparison](small-impulse-diagnostic.md#result--numerical-precision-sensitivity-isolated): all twelve f64 cases are smooth and within 1.71%; f32 remains irregular. Exact state transfer, matching solver features, unchanged contact/limit regimes and exact hooked WASM replay are verified. The application outlier still exists; no tolerance or runtime changed. |
| Historical diagnostic validation | 25 tests pass; lint passes without warnings | [Post-H26 verification](checkpoints/2026-09-27/h25-h26-validation-complete/manifest.json) covers original motor/continuity assertions, baseline capture and corruption detection for binary snapshot archives. The native calibration executable builds successfully. Full typecheck last passed in [prior validation](checkpoints/2026-09-26/standing-tooling-validation.json); no TypeScript production file had changed at that checkpoint. These checks do not establish a complete application/physics gate. |
| Complete application and physics suites | Failed on the final H77 source | The complete unit suite is 325/339 pass (13 fail, 1 skip); all 63 physics scenarios ran, with 6 pass and 57 fail. Typecheck/build pass and lint has 0 errors/6 warnings. These checks do not override failed H77 feasibility. |
| Five-cycle protocol, recovery, browser review, both Pages configurations | Acceptance open | No later complete integration result establishes acceptance. |

The [physics acceptance contract](physics-acceptance.md) retains independent numerical limits. The [merge ledger](merge-acceptance.md) maps requirements to tests, but its older table results are historical. The local Pages workflow keeps required verification failures blocking deployment; this document does not establish remote CI or deployment status.

## What is currently missing

The [prioritized TODO](../TODO.md) is the actionable backlog. The protocol room, playground, renderers, and controller infrastructure exist; their presence does not close the physical gates.

| Missing milestone | Current gap | Completion evidence |
| --- | --- | --- |
| Quiet standing | Recorded failure at all three headings; no accepted repair | Unchanged official standing contract plus structural regressions |
| Transfer, prediction, and stepping | Complete repeated physical sequence is unaccepted; two focused failures and both probes remain open | Repeated measured transfers, held-out prediction comparisons, then steps/release/reversal |
| Recovery, terrain, and browser integration | No complete result on the retained candidate; short/synthetic replays cover only part of the requirements | Full recovery/five-cycle and inspected browser results in both renderers |
| Release verification and durable evidence | No complete candidate-wide gate bundle; CI artifacts expire | Full suites, both Pages configurations, durable manifests/artifacts, and verified deployment gating |

The small-impulse outlier remains a separate diagnostic task. See [fresh capture instructions](evidence.md#fresh-baseline-capture) and [missing original artifacts](evidence.md#still-unavailable). Native touch and hardware GPU performance have no certification from synthetic replay.

## Investigation and next milestone

The September 26 report records a local causal intervention at ticks 175–176: changing the previous lower-chain load request reduced a forefoot angular-speed spike. Its offline replay isolates sensitivity to measured hull membership. This is evidence for a specific allocation discontinuity, not an accepted stationary controller repair.

H1–H4 did not meet their declared repair predictions. H6 and H7 failed their static load-jump bounds. The [new H8 evaluation](standing-h8-evaluation.md) passed static/local checks but failed every prolonged screen, including the upper-limb damping combination. H8 is rejected and remains diagnostic-only. New traces, exact replay, matching controls, explicit reset/infeasibility counts, and failed screens are preserved with verified digests. A passing local intervention does not establish successful standing, transfer, stepping, or recovery.

The next milestone remains quiet standing under the unchanged official three-heading contract, followed by repeatable measured transfer, individual steps, sequences, and integration. Preserve failed runs and thresholds. Restore the original source-scoped evidence or generate a fresh complete run before claiming a gate passes. See [evidence preservation](evidence.md#preserving-future-results).

The [H9–H11 continuation](standing-h9-contract.md) also produced no accepted repair. Soft projection of prior references and its combination with held posture references fail prolonged standing; hindfoot-preferred allocation fails the static discontinuity bound. These rejected results and selected peak traces are archived. A remaining forefoot spike occurs with continuous commanded loads, so load continuity alone does not explain every failure. The production controller and frozen limits remain unchanged.

The [post-H11 baseline](checkpoints/2026-09-26/standing-post-h11-baseline.json) records 255 source/configuration files, installed dependencies, executable digest, and preserved uncommitted work. All 65 production/package files still match the regenerated capture. The baseline itself executes no physical scenarios; its full worktree snapshot remains local-only under the named ignored evidence directory.

The [H12–H21 continuation](standing-h12-contract.md) preserves 174 additional
compressed artifacts in two verified archives. H20 reduces the local forefoot
response by 92.05% and passes both short-screen speed bounds at all headings,
but foot drift remains 0.01310–0.02018 m. H21's held-reference combination is
also rejected. Contact calibration shows that this installed engine reports
zero tangent impulse even when momentum balance proves friction acted; raw
tangent fields therefore cannot diagnose friction absence or saturation.
All 65 production/package files still match the regenerated capture.

The [H22–H24 archive](checkpoints/2026-09-27/h22-h24-evaluation/manifest.json)
adds 32 verified artifacts, including rejected idle-forcing/stance-task runs
and the observer's controller-phase repair. H23's large neck peak is post-fall;
it does not establish the initiating standing fault. The next physical milestone
remains the unchanged official quiet-standing gate.

The [H25–H26 and calibration archive](checkpoints/2026-09-27/h25-h26-calibration/manifest.json)
adds 55 verified artifacts, including rejected screens, full bounded event
traces, native snapshots, upstream-source receipts and source snapshots.
The [native calibration](rapier-constraint-calibration.md) qualifies the
position/velocity discrepancy and the separately confirmed upstream contact-
impulse reporting defect. No source-backed engine finding yet closes standing.

The subsequent [precision archive](checkpoints/2026-09-27/impulse-precision/manifest.json)
closes the separate small-impulse diagnosis through a controlled precision
comparison. All 65 production/package hashes remain unchanged. The fixed
assembly still collapses in f64, so its instability and quiet standing remain
open. [Nine original joint-motor tests and targeted lint pass](checkpoints/2026-09-27/impulse-precision-validation/manifest.json);
the original impulse assertion is unchanged.

The [H27–H29 archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json)
contains 27 verified artifacts. [Hybrid translational constraints](standing-h27-contract.md)
lose support; the [upstream runtime comparison](standing-h28-contract.md) fails
two unchanged motor/inertia assertions and the standing speed bounds; the
[centered neutral target](standing-h29-contract.md) falls at all three headings.
All three H22 controls replay exactly, as does the pinned loader's heading-zero
control. All 65 production/package hashes remain unchanged. The original
runtime passes the 21 focused adapter/coordinate/motor tests through the same
loader. [Nine archive tests and targeted lint pass](checkpoints/2026-09-27/h27-h29-validation/manifest.json).
These rejected interventions do not change the open physical gates.

The [H30 transition](standing-h30-contract.md) and
[H31 centered initial posture](standing-h31-contract.md) also fail. H31 verifies
the centered construction and keeps all three headings upright, but foot drift
is 0.02550–0.03240 m and all speed bounds fail. H30 and H31 preserve six further
exact H22 control replays; no production change follows.

## Documentation verification

The 2026-10-01 refresh aligns the overview, architecture, index, standing
selection, browser/rendering guides, TODO and evidence summary with the existing
working tree and local reports. Link/anchor checks cover 374 local references in
ten updated documents; 123 historical checkpoint links remain unavailable as
documented in the evidence guide. All other checked references resolve, and the
documentation diff has no whitespace errors. No physics or browser scenarios
were rerun for this documentation-only refresh.

The documentation review checked local links, implementation settings, and harness discovery, which returned 63 scenarios. Discovery executes no physical scenarios. The repair also verified all six recovered artifact digests and the Playwright module-resolution recipe with the installed 1.62.1 runtime; the documented tooling install uses the 1.63.0 version pinned in CI and was not installed during this repair. Documentation repairs do not change controller behavior, refresh fixtures, or establish physics/browser acceptance.

The subsequent TODO review confirmed that all 12 source/package digests in the partial documentation checkpoint still match the working tree. It inspected the baseline script's prerequisites, H8 call sites, and local workflow coverage. This does not extend that partial fingerprint or establish a fresh test, browser, or remote CI result.

The first P0 implementation subsequently added explicit `--fresh` capture and verified it against the local checkout. Its [portable ledger](checkpoints/2026-09-26/fresh-baseline.json) includes 252 source/configuration hashes, installed direct dependency versions, executable information, and uncommitted-work digests. Five CLI regression tests cover absent and invalid history, staged/unstaged/deleted/untracked files, dependency failures, and overwrite prevention. The [validation manifest](checkpoints/2026-09-26/fresh-baseline-validation.json) records this tooling-only scope. These checks do not change the recorded physical failures above.
