# Retournement latéral natif — progression réelle, jalon non acquis

2026-10-11, branche locale `investigate/physical-support`, HEAD correspondant à la référence distante fournie
`c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou déploiement.
La récupération reste désactivée en production et non qualifiée. Le tracking
`origin/main` est absent de ce checkout ; aucun nouvel état distant n’est affirmé.

Le mouvement change effectivement de route : une commande asymétrique produit
un roulis maximal de **46,17° au bassin / 62,34° au torse**, puis un repos sans
charge tête/nuque. Le jalon préannoncé de75° pour les deux corps n’est pas atteint.
Le banc arrête le candidat après3s sans progression. Deux histoires seulement ;
aucune répétition ni transfert suivant après ce refus. Le prototype est conservé
hors du code actif, avec son binaire exact et ses traces déterministes.

## Comparaison physique et causalité

Le témoin et le candidat reproduisent exactement **1 468 pas / 6,116667s** de
l’ancien cas02 : chute C, repos ventral, levée/déplacement/appui de la paume gauche.
Ils partagent aussi **1 469 pas** entre eux. Ensuite mêmes références asymétriques,
mêmes cibles, gains, limites et contacts : seule la demande de pression unilatérale
sur la paume gauche diffère. Le bassin et les25corps continuent dans le même monde ;
aucun reset, remplacement de corps, translation ou rotation imposée à la racine.

| Mesure | Témoin sans pression | Candidat avec pression |
|---|---:|---:|
| Durée physique totale | 6,941667s | 13,275000s |
| Arrêt | paume perdue pendant0,20s | stagnation de roulis3s |
| Roulis final bassin / torse | +1,56° / +1,75°, sens opposé | −46,10° / −61,87° |
| Maximum vers la droite bassin / torse | ≈0° | 46,17° / 62,34° |
| Charge tête instantanée finale | 23,08N | 0N |
| Charge paume gauche instantanée finale | 0N | 118,67N |
| Charge bras supérieur droit instantanée finale | 0N | 390,42N |
| Stabilité des sous-critères sans tête/nuque, sans le critère d’angle | 0s | 3,229s |
| Jalon latéral complet,75° + support + stabilité | 0s | 0s |
| Relevage qualifié | non | non |

À temps égal6,941667s, le candidat garde24,33N à la paume et parcourt
−0,42° / −2,37°, contre+1,56° / +1,75° au témoin. Le témoin ne continue
pas au-delà de son refus d’appui : aucune comparaison à13,275s n’est inventée.

Le signe négatif désigne le roulis vers le côté droit. L’angle est mesuré depuis
le front réel du bassin/torse, projeté autour de l’axe longitudinal horizontal
capturé à l’entrée ; ce n’est pas un angle relatif de hanche ou une pose de plan.
Le jalon exige75–105° aux DEUX corps, tête/nuque<3N actuels+persistants, appui du
côté droit,≥80%du poids sur des contacts non-tête/nuque réellement chargés,
COM/capture dans leurs points, vitesses originales des25corps et0,55s de tenue.
Les3,229s satisfont les sous-critères de support mais pas l’angle : elles ne
requalifient pas le retournement. Le torse au sol est un appui intermédiaire
explicite ; les critères Brace/Stand ne sont ni remplacés ni abaissés.

![États réels et comparaison des charges/angles](../evidence/prone-roll-2026-10-11/roll-comparison.png)

[Vidéo des pas natifs du candidat](../evidence/prone-roll-2026-10-11/roll-native.mp4) :
histoire complète,20images/s,vitesse nominale×1, sans interpolation. Les images
d’entrée et d’arrêt ajoutées peuvent décaler la durée de lecture de moins de0,1s.
Les légendes distinguent mouvement réellement exécuté, appuis et refus du jalon.

**La pression est une intervention causale utile.** Sans elle, la paume perd
l’appui et les références asymétriques seules n’amorcent pas le roulis demandé.
Avec elle, la paume reprend le sol, le moment cinétique global suivantZ descend
jusqu’à−0,830kg·m²/s et les angles progressent. Le COM se déplace réellement de
(−0,0209;0,1539;1,1718)m à(0,1266;0,1748;1,1272)m. Il n’est pas déplacé par un
optimiseur. L’énergie mécanique entre premier pas de roulis et arrêt passe de
109,02 à123,81J ; l’énergie cinétique finale est0,00238J. Les séries d’énergie,
de moment cinétique, de forces du sol et d’accélération sont conservées.

**La libération de la main droite n’a pas libéré le bras droit.** Le bras
supérieur devient porteur actuel+persistant à8,329s, alors que la main reste
levée. À l’arrêt, sa réaction normale produit **+70,19Nm autour du COM suivantZ**,
contre **−66,49Nm pour la paume gauche** ; les cuisses et la friction participent
au bilan. Sur les0,55s finales, les charges moyennes sont : paumeG115,39N,
bras supérieurD308,84N,cuisseG31,78N,cuisseD253,13N ; tête/nuque/torse0N.
La somme moyenne709,13N est proche du poids708,28N. Les valeurs finales du tableau
sont instantanées et fluctuent avec le solveur ; elles ne sont pas un bilan
statique unique. La butée porteuse du bras droit explique un frein au roulis,
sans démontrer l’impossibilité d’une autre organisation des membres.

**La commande atteint aussi une limite locale.** L’axeY de la ceinture
scapulaire gauche reçoit27,33Nm pour un cap28Nm à l’arrêt. La demande nominale
paume150N est ramenée à73,70N à cet instant (maximum demandé borné113,74N), sans
augmenter aucun plafond. La force réelle de la paume n’est pas identifiée à cette
demande : elle résulte de tous les moteurs et contacts. La cible de l’épaule
droite atteint sa borne d’extension−35° pour tenter de conserver la main en
hauteur. Le suivi d’un seul point terminal contraint donc mal le dégagement de
la chaîne entière. Monter des gains ou supprimer les contacts ne traite pas
ce problème de mode d’appui.

Le diagnostic isole ainsi le prochain changement structurel : contrôler le
dégagement **bras supérieur + avant-bras + main** du côté de roulis, ou assumer
explicitement leur rôle de pivot avec un transfert d’efforts compatible. Le
critère actuel `rightHandClear` n’est pas une preuve de côté droit libre. Aucune
nouvelle candidature ou correction de gain n’a été lancée après cette observation.

## Référence publiée et adaptation complète

HumanUP §III sépare retournement ventral→dorsal et relevage dorsal ; §V-D5
explique la difficulté du retournement sous symétrie stricte. La légendeFig.2
présente apprentissage puis raffinement ; Fig.3 décrit des essais réels sur
plusieurs surfaces. Fig.5 compare HumanUP avec le contrôleur G1 manuel, dont le
texte distingue pose dorsale canonique, propulsion/glissement par les mains et
flexion des jambes, puis redressement depuis le squat. Ce découpage manuel n’est
pas la séquence détaillée HumanUP. [Article primaire](https://arxiv.org/html/2502.12152v1),
[page du projet](https://humanoid-getup.github.io/).

Seuls les textes et légendes accessibles ont été lus. Les images de mouvement
et vidéos403 **n’ont pas été visionnées** ; aucune tentative de contournement,
aucun code tiers exécuté ou dataset téléchargé. Les choix de côté, de paume,
de hanches et les charges ci-dessus sont propres à notre mannequin.

![Phases publiées et route adaptée, avec états exécutés indiqués](../evidence/prone-roll-2026-10-11/route-phases.png)

| Phase de notre référence | Actionneurs et appuis prévus | État et passage à vérifier |
|---|---|---|
| C→repos ventral | commandes C existantes ; contacts tête/torse/cuisses | reproduit jusqu’au repos physique ; aucune pose importée |
| Préparer paumeG | trois tâches de bras déjà validées ; compensation gravitaire et IK depuis le torse mesuré | paume actuelle+persistante ; reproduit |
| Dégager côtéD | brasD relevé par articulations, sans modifier bassin mondial | main levée, mais bras supérieur devient porteur : tâche incomplète |
| Amorcer roulisD | réaction de paumeG via−JᵀF ; abduction/rotation de hancheG, extension partielle de jambeD, rotation relative du rachis | réalisé jusqu’à46°/62° ; jalon75° refusé |
| Capturer le latéral | réduire la pression et tenir la forme articulaire mesurée, appuis explicites côtéD | prévu seulement après75° ; non exécuté |
| Passer au dorsal | dégagement effectif des membres, déplacement du COM relatif aux contacts par leurs couples ; gravité autour du pivot réel | définir/mesurer le moment à chaque appui ; aucune rotation globale imposée |
| Dorsal→appui mains/pieds | flexion hanches/genoux, réaction des bras et pied réellement planté ; tête déchargée | contacts et forces à vérifier depuis le vrai état dorsal |
| Appui→accroupi | appuis pied/main porteurs, transfert du COM par couples articulaires | pelvis/tronc doivent s’élever physiquement ; aucune racine virtuelle |
| Accroupi→Stand | extension coordonnée sous caps, talons réellement porteurs, contrôle COM/moment cinétique | critères existants, répétitions fraîches/60° et qualification complète restent requis |

Ce tableau définit une route complète et ses dépendances. Les phases après le
roulis n’ont pas de trajectoire numérique ou de réussite physique validée. Le
prochain transfert n’est pas commandé avant acquisition et répétition du jalon
latéral/dorsal. Aucun succès d’un autre modèle ne valide nos efforts ou contacts.

## Mécanisme de commande et preuves

Avant la posture, le plan fixe une réaction sol→paumeG vers+Y àX<COM, donc un
momentZ<0. Elle est traduite par **−JᵀF** en biais de vitesse des moteurs
anatomiques, avec jacobiens/ancres mesurés et compensation gravitaire existante.
L’amplitude est réduite uniformément selon les caps anatomiques ; l’allocateur
agrégé760Nm reste identique. Les deux IK suivent des points de mains depuis les
racines natives MESURÉES. Aucune liberté de translation/rotation mondiale du
bassin n’entre dans la commande ou un solveur de posture. `pelvisGlobalTarget`
est explicitement nul et les efforts/couples directs mesurés au bassin sont0.

Le moment normal du sol est calculé avec chaque point résolu. La friction est
comptée une seule fois par manifold pour la force totale ; son moment utilise
le centroïde pondéré par les normales et reste **une estimation** lorsqu’il y a
plusieurs points. Il ne faut pas l’annoncer comme un équilibre exact de couples.
Le moment cinétique global et les couples effectivement livrés par les moteurs
sont enregistrés indépendamment. Les contacts à cette étape sont des appuis
réels, pas les simples positions d’un corridor géométrique.

Un seul contrôle FK descriptif a rejeté une posture du brasD avec main sous le
sol ; aucune optimisation ni root libre. La tâche de levée historique a ensuite
été réutilisée. Le défaut de dégagement de toute la chaîne n’a été identifié
qu’en exécution. Les limites de couples, articulation, géométrie, friction,
profils et gardes natives n’ont pas changé. Rapport couple livré/cap<1 sur tous
les pas ; commande/anatomie≤1. Aucune garde d’intégrité ne refuse : maxima
candidat sol16,37mm, erreur limite0,00802rad, auto-pénétration0, séparation
ancre4,03µm ; les gardes originales restent la source de décision.

## Budget, vérifications, livraison

Budget fixé avant essai : quatre histoires possibles,14s physiques/120s murales
chacune,56/480s cumulées. **Deux histoires exécutées :20,216667s physiques /
58,329471s natives murales.** Les cas3/4 étaient réservés à répétition/transfert
après succès ; non lancés. Un lancement initial refusé parce que la copie du
binaire n’avait pas le bit exécutable est consigné séparément : aucun processus
natif ni pas physique lancé. Compilation, lecture et rendu hors budget natif.

Quatre tests unitaires ciblés passent avant les intégrations et après retrait
du prototype ; six essais ignorés avant retrait,cinq après ne comptent pas comme
passés. Clippy strict,formatage et contrôle du diff passent sur les sources
restaurées. Les histoires natives explicites établissent seulement les résultats
ci-dessus. **Suite complète non déclarée verte** : le test d’articulation hybride
et les quatre échecs TypeScript signalés précédemment sur le témoin Linux restent
hors de ce correctif. La récupération automatique n’est pas activée.

Preuves dans `evidence/prone-roll-2026-10-11/` :

- `PLAN.md`, `native-ledger.json`, `run_case.py` : route gelée,budget et appels
  explicites ; aucun gain sweep ou répétition automatique après refus.
- `case-01/case.json`, `case-02/case.json` : métriques chaque pas, contacts,
  angles,énergie,moment cinétique et hash f32 des25corps ; poses/couples20fps.
- `analysis.json`, `analyze.py` : comparaison déterministe, moments par corps,
  appuis réels, sous-critères de stabilité et portée des estimations.
- `source-delta/` et `comparison-binary` : expérience exacte, SHA256 binaire
  `aa48c8b5a55690d5ceed9715aa9f4868e91c06b6a7092665ee48de894bb9474e`.
  Restaurer ces cinq fichiers par leur nom dans
  `rust/crates/sim/src/recovery/support_probe/` sur la base historique indiquée
  dans `source-manifest.json`. Les deux intrants sont dans `inputs/`.
- `active-source-restored.json` : quatre fichiers antérieurs remis à l’identique
  et nouveau module retiré ; le prototype refusé demeure hors code actif.
- `build-units-final.log`, `clippy-final.log`, `format-final.log`,
  `restored-units.log`, `restored-clippy.log`, `restored-format.log`,
  `diff-check.log` : contrôles ciblés et leurs périmètres.
- `render.py`, `video-metadata.json`, `media-validation.json` : poses réellement
  intégrées, indices natifs, métadonnées du média et vérification.

La base historique n’est pas dupliquée : archive Library
`libfile_28c3e675dd78819193626fa3492e60c2`, SHA256
`c1264597fda226fb2caba036c63a46630756d8a246e957fa85618c7bb5bb75cc`.
Le nouveau complément ne contient que ce suivi utile ; les archives antérieures
restent intactes. Les livrables locaux sont complets ; **aucun nouvel identifiant
Library n'a été créé**. Une première demande réseau a été refusée par le contrôle
automatique faute de preuve de propriété de la destination. La lecture de la
Library privée a ensuite permis une nouvelle tentative autorisée, mais le
helper échoue à `tools/list` avant toute préparation ou mutation. Aucun envoi
direct ni contournement n'a été effectué. Le reçu `library-blocker.json` et la
vérification `library-authorization-check.json` distinguent ces deux obstacles.

La compétence [Library](skill://plugin_connector_1p_1b8ff8edfc1481918b252c8277e23125/library/SKILL.md)
impose ici « Use the bundled prepared-upload helper below. » : les deux outils
préparés sont disponibles et cinq fichiers doivent être déposés. Le helper
prescrit étant inaccessible par le réseau de cet environnement, le dépôt reste
à réaliser depuis un environnement disposant de ce transport.
