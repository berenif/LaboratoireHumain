# LaboratoireHumain — coordination, prototype et résultat décisif

Rapport consolidé du 13 octobre 2026. **Le prototype est implémenté et testé ; cette famille est close sans acquisition des talons ni relevage.** Les quatre problèmes prédictifs refusés sont contradictoires sous leurs contraintes archivées, selon deux vérifications arithmétiques indépendantes. Ce résultat porte sur la formulation locale, sans démontrer une impossibilité anatomique.

Preuves publiées : [commit cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c](https://github.com/berenif/LaboratoireHumain/commit/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c). Branche `investigate/physical-support` ; `main` conservé à `c597959f46f37715035349c8cd0a699b6822111a`.

Les sections historiques décrivent la préparation et ses hypothèses. Le résultat et les décisions après clôture ci-dessous font autorité pour la suite. Le rapport, la vidéo et les mesures sont disponibles dans le dépôt. **Publication Library bloquée : erreur réseau du flux officiel avant toute création confirmée ; aucun nouvel identifiant ni version ne sont disponibles.** Les liens vers des traces volumineuses renvoient à leur archive compressée ou à son index de restitution sans perte.


---

# Résultat décisif et décision

# Coordination : dernière paire close, prévisions refusées

**Décision : fermer cette famille et ne pas exécuter la continuation complète.**
La vérification native passe ; A tombe sans acquérir les talons ; les quatre
prévisions de B refusent la cinquième commande avant leur dix-septième sous-pas.
B ne produit aucune histoire réelle. Le prototype est construit et testable,
mais ne démontre ni transfert local ni relevage. Aucun nouveau calcul physique
n’est autorisé par les crédits restants de ces registres clos.

Objectif causal : depuis la même fixture et la même référence, déterminer si
la coordination avec choix prédictif acquiert les talons et stabilise le
recentrage là où la référence seule échoue. Les deux dernières places de la
famille initiale sont A puis B ; elles ne permettent aucune répétition après
ce lot. Un succès local ne qualifie ni la continuation debout ni un relevage.

## Contrat avant lancement

- Nouvelle paire : 16 pas de vérification + 1 920 réels + 76 800 prédictifs,
  soit **78 736 pas maximum**. Toutes les copies sont comptées.
- Famille : 3 988 pas antérieurement réservés + cette paire = **82 724/300 000** ;
  quatre emplacements d’histoire consommés au maximum, chacun borné à 4 s.
- Temps : **7 100 s** natifs, enveloppe supervisée démarrage/arrêt compris
  **7 137 s**. La borne familiale conservatrice reste sous 7 200 s.
- Préparation réservée : **28 489/144 000 s**, inspections **9 173/20 000**.
- Modèle, anatomie, masses, friction, collisions, gains, caps et seuils locaux
  conservés. Récupération désactivée par défaut. Aucun appui artificiel du bassin.
- Référence géométrique documentée jusqu’à 8 s, mais aucune continuation réelle
  au-delà de 4 s autorisée dans ce lot. La fixture ne suit pas un impact simulé.

Le [contrat figé](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/launch-contract.json),
l’[audit cumulatif](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/cumulative-budget-audit.md)
et le [préflight de continuation](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/continuation-preflight.md)
précisent les consommations, limites et critères de décision.

## TODO et preuves

- [x] Revue cumulative des quatre anciens registres, sans réouverture.
- [x] Critères physiques inchangés et continuation géométrique archivée relue.
- [x] Registre strict de la dernière paire et comptage incomplet explicite.
- [x] Supervision avec arrêt de groupe, signaux et limite murale externe.
- [x] Worker dédié reconstruit, tests purs et contrôles de production terminés.
- [x] Neuf prérequis appuyés par les preuves actuelles ; manifeste et binaire gelés.
- [x] Gel publié et SHA distant vérifié, puis unique lancement du registre distinct.
- [x] Registre clos ; décision causale A/B, premier refus et consommations archivés.
- [x] Mesures et vidéo issues exclusivement des états enregistrés.

Les modifications de ce jalon portent sur l’admission, la mémoire comparée,
le comptage durable et l’arrêt des processus. Elles ne changent pas les actions
physiques. Le gel `65925ef032197639f6ebd6c8e897084a5735cacf` a été publié et
vérifié sur le distant avant l’unique lancement du 13 octobre à 11:10:50 UTC.
Le registre est `closed-native-failure` depuis 11:11:02 UTC. Aucun worker ne reste actif.

Préflight statique réussi sans worker. Manifeste `2fbcb3db0cd1bccf14082e772b1efba63e567fe7fce35ed8cccbbbf2ac2cdde9`.
Binaire dédié `c1c8a12825428758643a7f6e2fafb0a2710f7dca787c1daee07c27ec59849e9c`.
Les 14 tests Rust purs, le contrôle de production, la compilation release sans lancement,
89 tests de registre, 43 du lanceur, 47 du superviseur réutilisé, sept du builder
et six essais OS inertes passent. La suite globale Rust/TypeScript n’est pas relancée.


## Résultats et portée causale

| Opération | Sous-pas natifs exécutés | Résultat |
| --- | ---: | --- |
| Vérification copie/exécution A et B | 16 | Parité complète des états exposés et mémoires testées |
| A, référence commune seule | 960, soit 4 s | Aucun talon porteur, aucun intervalle stable ; chute |
| B, quatre prévisions figées | 64, soit 4 × 16 | Refus du solveur au tick 4, avant le sous-pas 17 |
| B, réalisation | 0 selon reçu et journaux indépendants | Aucune proposition admissible |

A termine avec le bassin à **0,1333 m** et le torse à **0,1020 m** du sol.
Ses 960 diagnostics restent négatifs. Les critères locaux et l’intégrité du
modèle restent inchangés ; l’échec de A est physique, pas un succès articulaire
assimilé à un relevage.

Les quatre facteurs 0, 0,5, 1 et 1,5 échouent au même stade de faisabilité
initiale des tâches dures. Après 512 cycles, la violation maximale de la borne
supérieure de l’axe 36 (`leftAnkle.x`, vitesse en rad/s) vaut respectivement **0,14751 ; 0,18736 ; 0,19783 ;
0,19810**, pour une tolérance fixée à **1e−8**. Ce sont les valeurs du solveur,
pas des déplacements de pied ni des couples mesurés. Le diagnostic brut indique
« infaisable ou non convergé dans le budget fixé ». L’audit arithmétique
postérieur tranche pour ces problèmes enregistrés : **les contraintes sont
effectivement incompatibles**. Deux vérifications indépendantes, utilisant
des fractions exactes des nombres f64 archivés, établissent huit contradictions
(quatre facteurs, deux pieds). La marge de contradiction est de **0,04387 à
0,06393 m/s**, après combinaison des lignes matérielles et des bornes, pour
une allowance de tolérance au plus **1,51e−8 m/s**. Aucun solveur, monde, FK
ou sous-pas supplémentaire n’est nécessaire à ce certificat. Davantage
d’itérations ne rendrait pas ces problèmes admissibles.

Le drapeau agrégé `integrity_ok=false` ferme l’admission dès ce refus logiciel.
Il ne prouve ni une casse anatomique native, ni l’impossibilité physique des
appuis. Le champ agrégé `diagnostic_present=false` accompagne le même refus :
les seize échantillons de chacune des quatre copies existent bien. À l’inverse,
`support_feasible=true` dans A ne signifie pas que son diagnostic local passe. Les quatre prévisions n’atteignent que 66,67 ms sur les deux secondes
requises. La parité vérifiée sur un préfixe court n’établit donc pas la
faisabilité du contrôleur au tick suivant. La comparaison A/B n’obtient aucune
réalisation de B et ne mesure pas un bénéfice causal de sa correction.

## Mesures, vidéo et archives

![A tombe ; B reste dans sa pose initiale, sans histoire réelle](https://raw.githubusercontent.com/berenif/LaboratoireHumain/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/rendered/milestones.png)

- [Vidéo A, durée exacte de quatre secondes](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/rendered/A-actual-history.mp4).
- [Mesures A, tous les sous-pas réels](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/rendered/A-measurements.csv).
- [Reçu du rendu et politique d’échantillonnage](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/rendered/render-receipt.json).
- [Index des archives compressées sans perte](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/archive-index.json).
- [Résultat du lanceur](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/run-result.json) et [registre clos](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/budget/ledger.json).

La vidéo maintient les poses enregistrées sur leurs intervalles temporels,
sans interpolation ni prolongation de la pose finale. Le CSV conserve chaque
ligne mesurée ; les poses issues seulement du rapport ont leurs mesures
absentes laissées vides. B ne possède aucune vidéo d’histoire réelle. Le rendu
n’exécute aucune simulation, FK ou requête de collision. Les neuf sources
volumineuses sont conservées et compressées en seize fragments vérifiés par
hash après décompression ; leur restitution concatène les fragments dans
l’ordre, sans séparateur.

## Comptabilité conservée

Ce lot réserve **3 856 pas**, dont **1 040 exécutés** selon les journaux durables
et les reçus indépendants : 16 + 960 + 64 + 0. Chaque intégration journalisée a
une admission et une terminaison. Le registre parent conserve volontairement
`actualNativeStepsExact=false` et **[1 040, 2 000]** : la réservation B de 960
est restée non rapportée lorsque le premier échec de prévision l’a clos. La
réconciliation postérieure ne réécrit pas cette pièce ni ne rembourse de pas.

La fenêtre de première réservation à clôture vaut **12,321321 s**. Elle ne
mesure pas exactement tout le démarrage et l’arrêt ; la borne murale familiale
conservatrice et la supervision restent conformes au plafond de deux heures.

La famille cumule **7 844 pas réservés sur 300 000**, et ses quatre emplacements
d’histoire sont débités. Les anciens registres clos et les trois anciens
binaires ont les mêmes empreintes. Les 347 entrées du manifeste sont également
inchangées après calcul. Les sous-pas historiquement inconnus gardent leur
statut ; les 1 040 de ce lot ne doivent pas être présentés comme un total exact
pour toute la famille.

## Décision sur l’histoire complète

La continuation géométrique jusqu’à huit secondes a été examinée avant le
calcul ; elle ne constitue pas une exécution dynamique. Aucun état réel de B
stabilisé ne rejoint cette continuation. **Ne pas lancer l’histoire complète** :
le motif d’arrêt « prévisions inutilisables » est atteint. Les quatre places
initiales sont consommées ; ni nouvelle variante, ni répétition, ni changement
de cap/friction/anatomie n’est ajouté.

Le prochain travail utile est l’audit hors physique de **l’origine de cette
incompatibilité** : unités, assemblage, ancres matérielles, compensation de
la racine libre, bornes de vitesse et validité de la linéarisation instantanée.
L’existence d’une solution à ces quatre problèmes précis est désormais
tranchée négativement ; il reste à expliquer quel choix de formulation les
rend contradictoires et quelle correction reste fidèle au contrat physique.
Ce résultat ne justifie ni davantage d’itérations ni un relâchement des seuils.
Toute nouvelle expérience physique nécessite un protocole et un budget
distincts approuvés par le propriétaire via le parent.

La fixture `landed-crouch-left` est une pose native stockée, peu accroupie, avec
vitesses initiales nulles ; ce lot ne crée pas d’impact ni de phase de settle.
Le relevage ≤25 s après impact, la seconde stable réelle, le passage final au
contrôleur normal et la qualification des 24 poses/cycles/impacts restent
non démontrés. Récupération et pas correctifs restent désactivés par défaut.


## Revues indépendantes après exécution

- [Certificat causal des contraintes incompatibles](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-causal-review.md).
- [Audit physique, moteurs et parité](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-physical-audit.md).
- [Réconciliation des budgets sans réécriture](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-budget-audit.md).
- [Décision sur la continuation complète](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-continuation-decision.md).
- [Validation indépendante des fichiers livrables](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-artifact-validation.json).


---

# Revue du plan et architecture implémentée — historique du 12 octobre

# Prototype de coordination : préparation et décision

Mise à jour après clôture : [correction numérique testée des quatre refus](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/coordination-numeric-correction-2026-10-12.md), sans nouveau sous-pas physique. Les résultats physiques ci-dessous restent ceux du lot initial.

**Protocole clos : prototype implémenté, refus numérique de B.** Voir le [rapport de résultat](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/coordination-prototype-result-2026-10-12.md). Total exécuté : 1 024 sous-pas ; aucune récupération qualifiée.

Base inspectée : `e3b4ed3b2ddaac5450c2846ec12c736a1287082e` sur
`investigate/physical-support`. `main` reste à `c597959`.
Autorisation : « Yes, review, corrige implement dans un contexte neuf ».
Début de préparation : 2026-10-12 20:14 UTC. Les anciens registres restent clos.

## Corrections de la proposition

Le problème comporte déjà une synchronisation articulaire, un installateur
ForceBased et des gardes de transfert. La nouvelle hypothèse concerne la décision
commune de tâches, la conservation des appuis matériels et leur remplacement.
Les caps, gains, masses, friction, collisions et anatomie ne changent pas.

1. **Préparation et admission sont distinctes.** Une référence articulaire valide
   ne prouve ni contact ni soutien. Une continuation chiffrée vers debout, avec
   les patches/pivots, les limites, le retrait d'appuis et le maintien de 1 s, doit
   être admise avant de lancer l'histoire locale. Des inconnues restent des
   refus ; elles ne deviennent pas des cases cochées.
2. **Commandes totales bornées.** Le solveur agit sur 46 vitesses de cibles,
   intégrées en positions. Le budget de vitesse de 1 rad/s porte sur référence
   plus correction, avec bornes anatomiques. Une tâche d'appui contradictoire
   ou un solveur non convergé refuse la commande ; aucune pénalité de posture
   ne peut acheter l'abandon d'un appui.
3. **Sondes correspondant à l'action réelle.** La différence finie doit compiler
   les positions et vitesses cohérentes puis recalculer l'allocation native à
   chaque pas. Modifier seulement `target_vel` à caps fixes mesure une autre
   action. Une sonde à une borne devient unilatérale et conserve son amplitude
   effective dans le reçu.
4. **Prévision complète.** Copier le monde physique seul n'est pas copier le
   contrôleur. Mémoire, horloge, ancres, allocation et observation doivent suivre
   la même fonction de décision future. Les journaux `thread_local` des anciens
   probes doivent être isolés ; leurs copies ne peuvent alimenter l'histoire
   réelle. Une interface de prévision ne vaut pas une parité native vérifiée.
5. **Horizon dérivé.** La stabilité locale exige déjà 132 pas (0,55 s), auxquels
   s'ajoutent acquisition et persistance de charge. Un horizon de 120 pas ne
   peut donc certifier toute la transition. Maximum inchangé : 480 pas. Le
   compilateur de programme complet garde son minimum de 8 s : un horizon court
   doit extraire une fenêtre d'un programme complet, pas relâcher ce contrat.
6. **Mesures distinctes.** Les impulsions CCD du sous-pas servent aux bilans.
   Les contacts persistants présents servent au soutien actuel. L'ancienneté
   de charge d'un segment n'est pas celle de son talon nouvellement posé.
   Une tâche matérielle conserve son point local et son ancre mondiale ; un
   pivot autorise des patches nouveaux observés, jamais une fausse conservation.

## Budget et arrêts

Préparation : au plus 40 h actives sur cinq jours et 20 000 inspections statiques.
Physique : au plus A, B, répétition B et B à 60°, chacun 960 sous-pas (4 s).
Les deux dernières histoires sont conditionnées au diagnostic local de B.
A et B partagent référence et allocation. Aucun réglage entre histoires.

Budget total : `3840 + 3*40*(4*480 + 93*4) + 21120 = 300000` sous-pas.
Les 21 120 pas de vérification ne constituent pas des histoires supplémentaires.
Chaque intégration, y compris copie, préfixe ou test technique, doit être réservée
durablement avant exécution. Un crash ne rembourse rien. Au plus 40 appels
d'identification/prévision par B (<=10 Hz), quatre prévisions longues et 93 sondes
courtes par appel. Décision commune à 60 Hz, Rapier à 240 Hz. Plafond mural 2 h,
un processus mono-thread, aucune compilation simultanée aux calculs physiques.

Arrêt : soutien irréalisable, prédiction inutilisable, intégrité violée, absence
du diagnostic avant son échéance, prérequis absent ou plafond atteint. Les outils
historiques `run-getup-ab.py` et `whole_history` ne sont pas les exécutants de ce
budget : ils conservent leurs anciennes durées. Aucun renouvellement automatique.

## Livrable logiciel visé et TODO

- [x] Lire rapport et deux revues, vérifier branche/base et absence d'instructions locales.
- [x] Ouvrir uniquement la préparation ; zéro intégration physique.
- [x] Implémenter observation des patches, preuve de retrait et diagnostic local.
- [x] Implémenter solveur commun borné, compilation q/dq et refus explicites.
- [x] Implémenter registre durable et tester ses plafonds sans physique.
- [x] Préparer et vérifier la continuation numérique complète.
- [x] Compiler et tester les propriétés pures avant toute admission physique.
- [x] Vérifier la parité native puis clôturer au refus prévu, avec preuves et publication.
- [x] Décider sur la continuation entière sans la lancer hors budget.

Le checkout neuf ne contient pas le répertoire historique non versionné
`evidence/`. Les rapports et les reçus versionnés sont lisibles, mais ne fournissent
pas tous les programmes/états natifs référencés par ces chemins. La préparation
doit rester reproductible avec les fichiers effectivement présents.

## Qualification inchangée

Le diagnostic local n'est pas un relevage. Le terminal original exige debout
avant 25 s après impact, stabilité réelle de 1 s, puis une qualification distincte
des 24 poses/cycles/impacts. Récupération et pas correctifs restent désactivés par
défaut. Une réussite locale n'autorise pas une histoire complète hors budget.
La comparaison SLOW reste une étude de port/réentraînement, sans exécution de
son logiciel ou de sa politique ni ouverture d'un service externe.

## Architecture exécutable corrigée avant le gel

La carte de réponse native proposée expire après 24 sous-pas, alors qu'une
prévision dure jusqu'à 480 sous-pas. La réidentifier récursivement dans chaque
branche rendrait le coût annoncé faux. Le prototype reconstruit donc les tâches
géométriques communes à 60 Hz, avec la vitesse observée de la racine flottante,
et évalue à au plus 10 Hz quatre facteurs de retour globaux figés : 0, 0,5, 1 et
1,5. Il n'emploie aucune sonde courte : les 93 sondes étaient un plafond, pas une
obligation. L'infrastructure d'identification reste testée séparément et ne sert
pas de carte supposée valide pendant deux secondes.

Les 46 références q/dq restent l'unique commande. Le solveur borne l'anatomie,
la vitesse et l'écart à la référence (0,12 rad), puis impose les contraintes des
points matériels observés. La bande de vitesse de tâche de 0,005 m/s est une
contrainte du contrôleur, pas un assouplissement des critères physiques. Les
patches acquis ne changent jamais d'identité ou d'ancre pour masquer un glissement.
A et B partagent exactement les commandes initiales avant acquisition des patches,
le programme, les caps et l'allocation ForceBased existante. Les gardes originales
de stagnation, perte d'appui et handoff mesuré sont conservées.

La prévision copie état physique et mémoire explicite du contrôleur et appelle
le même noyau natif. L'admission physique exige d'abord une comparaison complète
de quatre sous-pas réels et quatre copiés pour A puis B (16 sous-pas réservés).
Ce reçu doit confirmer états, compteurs, cibles et sémantique des moteurs.
Les réservations du registre précèdent toute intégration ; après une prévision,
le worker attend l'accusé du registre avant de reprendre l'histoire réelle.

La nouvelle continuation contient six clés et dure huit secondes. Les 1 921
états à 240 Hz ont été inspectés statiquement dans le modèle natif, à 0° et 60°.
Tous passent les bornes de coordonnées ; la dérive des points retenus reste
inférieure à 0,4 micromètre dans cette construction géométrique. Ceci prouve
une cible cohérente, aucunement la faisabilité des charges, des couples ou du
transfert en dynamique. Les images et la vidéo portent cette distinction.

## Comparaison SLOW

La source épinglée contient une politique résiduelle entraînée utilisant l'état
global et la référence, pas seulement une succession d'angles
([environnement](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/env.py),
[apprentissage SAC](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/SAC.py)).
Ses 21 axes, son état d'entrée et son actionnement ne correspondent pas aux 46
coordonnées et moteurs ForceBased de ce dépôt. Réutiliser le principe du retour
global est faisable ; porter directement les poids n'est pas établi. Un
réentraînement demanderait un environnement natif fidèle, le contrat d'action,
une distribution d'impacts et une évaluation distincte. Aucun logiciel tiers,
entraînement ou service externe n'a été exécuté pour cette comparaison.

## Vérifications et frontière des preuves

40 tests Rust ciblés passent (solveur, compilation q/dq, appuis, continuation,
observations natives sans intégration, tâches géométriques, copie de mémoire et
propriétaire). Le chemin de production passe `cargo check --lib --locked`.
La suite globale Rust/TypeScript, avec ses échecs historiques, n'a pas été
relancée ni déclarée réparée. Le binaire autonome est compilé sans exécuter
les tests physiques historiques.

Les journaux CCD sont collectés dans l'exécution réelle, isolés des prévisions.
La parité vérifie néanmoins les contacts, états et moteurs natifs complets.
Un succès local mémorise un intervalle de 132 pas admissible débutant avant 2 s ;
il ne certifie pas que la posture terminale de l'histoire reste stable. Le
raccord physique à la continuation, le handoff et le relevage entier demeurent
des preuves distinctes à produire, jamais déduites du rendu géométrique.


---

# Certificat causal après le dernier essai

# Revue causale après fermeture de la paire locale

Le lot clos ne fournit aucune amélioration A/B mesurée : A a exécuté ses 960 pas sans succès local, les quatre copies B ont exécuté 16 pas chacune, puis toutes ont refusé la commande suivante ; B réel a exécuté zéro pas. Le refus est celui des contraintes numériques du contrôleur. Il ne démontre ni une défaillance anatomique ni une impossibilité physique générale.

Cette revue utilise exclusivement les archives, le code figé et l’arithmétique rationnelle exacte. Aucun worker, solveur, modèle, FK ou pas natif supplémentaire n’a été exécuté. Sources principales : [index des archives : rapport des quatre copies et traces B](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/archive-index.json), [rapport B](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/002-B.json), [protocole](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/protocol.ndjson). SHA256 du rapport brut de planification : `5ed160d4c9c7d4361ec5d285268a9aa694bc797a419768cfaac0370d46313711`.

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

Reproduction : [script purement arithmétique](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun_qp_certificate.py), [huit certificats avec numérateurs/dénominateurs exacts](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-qp-certificates.json). Le rapport de planification est archivé en plusieurs parties gzip : décompresser chaque partie et concaténer leurs contenus dans l’ordre de l’[index](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/archive-index.json), vérifier le SHA256 brut indiqué, puis fournir le JSON reconstitué à `--input` ; `--output` exige un nouveau fichier. Deux relectures indépendantes ont reproduit les huit contradictions depuis le rapport brut, sans reconstruction géométrique ni résolution d’un QP.

## Pourquoi les preuves précédentes restent compatibles

Les seize pas de vérification de ce lot sont quatre pas sur chacune des instances A-copie, A-réel, B-copie et B-réel. Ils établissent la parité de ces fenêtres ; ils ne parcourent pas seize pas successifs de décision B. Le lot numérique antérieur de 44 pas avait, lui, vérifié le préfixe puis les commandes d’indices 12 à 15 sur chaque échelle. Il s’arrêtait à l’état physique complété 16, avant la décision suivante.

Les nouvelles traces confirment précisément cette limite : la décision d’indice 12 est certifiée, avec 8/8/12/12 itérations et quatre commandes f32 par cas, puis les pas physiques 13 à 16 sont réalisés. Au prochain tick, les états observés et la marge anatomique restante donnent un **nouveau** problème, incompatible. Il n’y a donc pas de contradiction entre la parité vérifiée et l’échec de cette prévision de 480 pas : chaque copie atteint seulement `16/240 ≈ 0.0667 s`, puis refuse avant le pas 17. Aucun des quatre horizons de deux secondes n’est disponible à la sélection.

Les valeurs booléennes toutes fausses du résultat de planification agrègent ce refus. Elles ne signifient pas que quatre accidents physiques se sont produits. Les échantillons finaux existent et portent notamment des charges de talon nulles et `passed=false`, mais ces copies s’arrêtent très avant le délai d’acquisition de deux secondes. L’histoire B n’a pas été appliquée ; aucun effet causal favorable ou défavorable de B par rapport à A ne peut être mesuré avec ce lot. La continuation complète de huit secondes reste refusée, sans nouvelle autorisation implicite.

## Préparation possible d’un autre protocole

Conserver ces quatre problèmes comme fixtures numériques d’infaisabilité, avec un diagnostic qui distingue explicitement contradiction certifiée et absence de convergence. Auditer hors physique la provenance des bandes matérielles, la compensation du mouvement de base, le lien entre vitesses de cibles commandées et vitesses articulaires observées, ainsi que la consommation de marge anatomique entre les décisions 12 et 16. Il faut ensuite justifier une formulation ou une famille de commandes viable avant de proposer un autre protocole.

Cette recommandation n’autorise ni changement de gains, forces, tolérances ou ancres, ni essai supplémentaire. Elle ne transforme pas une route géométrique statique en preuve dynamique. Les sources, le manifeste et les registres du lot clos demeurent inchangés.


---

# Audit indépendant des mesures, moteurs et états

# Audit physique après clôture de la paire locale — 13 octobre 2026

**Le témoin A échoue au diagnostic local ; aucune histoire réelle coordonnée B n'a commencé.** A termine ses 960 pas autorisés, mais tombe et reste au sol. Les quatre copies de B terminent chacune 16 pas, puis leur problème de tâches contraintes est refusé avant l'intégration suivante. Ce résultat ne démontre pas l'impossibilité physique ou anatomique du relevage.

Audit indépendant par lecture des JSON/JSONL archivés et de leurs octets sérialisés, comparaison de nombres/bits, hachage et arithmétique rationnelle des matrices déjà enregistrées. **Zéro `Model`, `Simulation`, monde, FK, worker, rendu ou sous-pas physique supplémentaire.** Aucun fichier gelé ni registre n'est modifié. Les grands fichiers sont référencés par leur nom logique et leur SHA brut ; leur stockage compressé est décrit dans [archive-index.json](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/archive-index.json).

## Étendue effectivement observée

| Opération | Intégrations documentées | Portée |
| --- | ---: | --- |
| Admission native A et B, copie/réel | 16 | Deux paires de 4 + 4 pas ; 16,67 ms par instance. |
| Histoire réelle A | 960 | 4 s depuis la fixture `landed-crouch-left`. |
| Quatre prévisions de B | 64 | 16 pas chacune, soit 66,67 ms ; aucune n'atteint l'horizon demandé de 480 pas/2 s. |
| Histoire réelle B | 0 | État initial conservé ; aucun facteur retenu, aucun ACK de reprise. |

Les traces contiennent exactement **1040 observations après intégration** : 16 de vérification, 960 de A, 64 des copies. Le reçu B et son journal d'admission déclarent séparément 0 admis/0 achevé. Le registre parent clos conserve néanmoins `actualNativeStepsExact=false` et la borne 1040–2000, car le reçu final B n'a pas été accepté avant fermeture. Cette reconstruction documentaire ne réécrit pas ce registre et ne rembourse pas les 3856 pas réservés. Voir [postrun-budget-audit.md](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-budget-audit.md).

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


---

# Comptabilité et conservation des registres

# Audit budgétaire après clôture de la paire A/B — 13 octobre 2026

Audit indépendant réalisé par lecture des registres, reçus, journaux durables avant/après intégration, traces et code gelé. Aucun calcul physique, FK, renderer, compression ni archivage exécuté pour cet audit. Seul ce nouveau document est écrit ; aucun registre n’est modifié.

**Le lot est clos en échec après exactement 1 040 intégrations documentées : vérification 16 + A 960 + prévisions B 64 + B réel 0. Les 3 856 pas réservés restent entièrement débités.** Le registre parent conserve intentionnellement une borne moins précise, **1 040–2 000**, car il n’a pas admis le reçu final de B après l’échec de planification. Cet audit rapproche les preuves externes sans modifier cette borne officielle.

## Réconciliation de chaque opération

| Opération | Réservés au parent | Réalisés inscrits au parent | Preuve native indépendante | État au parent |
| --- | ---: | ---: | ---: | --- |
| Vérification A/B : copies et originaux | 16 | 16 | 16 | `completed` |
| Histoire A | 960 | 960 | 960 | `completed` |
| Histoire B réelle | 960 | Non renseigné | 0 | `reserved` dans un lot clos |
| Premier appel de planification B | 1 920 | 64 | 4 × 16 = 64 | `failed-reported` |
| **Total** | **3 856** | **1 040 connus** | **1 040 exacts documentés** | **`closed-native-failure`** |

Le registre `33c9dd5f-73f2-472a-a2eb-b866ae7c9f2b` annonce `actualNativeStepsExact=false`, `unreportedReservedNativeSteps=960`, `actualNativeStepsLowerBound=1040`, `actualNativeStepsUpperBound=2000` et `authorizesIntegration=false`. Ces champs sont cohérents avec ses quatre réservations ; le zéro de B ne lui a pas été ajouté rétroactivement.

La précision supplémentaire est établie par plusieurs preuves concordantes :

- Les journaux `*.json.accounting.jsonl` contiennent respectivement **16, 960, 64 et 0** couples admission/fin pour vérification, A, planification et B. Chaque admission est suivie de sa fin, sans compteur manquant ; les compteurs de fin sont continus depuis 1 jusqu’au total de leur permis.
- Le code gelé place `before_integration` immédiatement avant `world.step_with_events` et `after_integration` immédiatement après. Chaque checkpoint est écrit, vidé puis synchronisé par `sync_data`. Les quatre journaux se terminent avec admission = complétion, `actualNativeStepsExact=true` et aucun emplacement natif en attente.
- Le journal réel B n’a qu’une ligne `permit-activated`, avec 0 admis et 0 terminé. Son [rapport final](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/002-B.json) confirme `actualNativeSteps=0`, `localReservedRealSteps=0`, `workerAccountingExact=true`, `workerActualAllOperations=1040` et `workerReservedAllOperations=3856`. Ses poses initiale/finale sont identiques, avec horloge 0.
- Les traces contiennent **16** états de vérification (quatre groupes de quatre), **960** lignes `actual-history` de A aux pas 1–960, et **64** lignes B exclusivement prédictives (quatre groupes de pas 1–16). Aucune ligne réelle B n’apparaît.
- Le protocole reçoit la planification en échec, puis ferme l’admission. Il ne contient aucun `continue-history`, second appel de planification, nouvelle histoire ou nouveau permis.

Les **80** réservations locales du rapport prédictif sont des emplacements de ticks : 64 intégrés et **16 abandonnés avant intégration**, quatre par branche. Elles sont incluses dans les 1 920 pas du permis parent ; elles ne sont ni 80 intégrations ni un débit supplémentaire. Au total, les **2 816 pas réservés mais non intégrés documentés** — 960 pour B réel et 1 856 pour ses copies — restent consommés.

## Arrêt et portée du résultat

A termine 4 s sans refus d’intégrité, mais ne franchit pas le diagnostic local. Les quatre prévisions B sont refusées au tick 4, avant le pas 17, par la projection des contraintes du solveur dans son budget fixe. Chacune a donc exécuté **16/480 pas** ; aucune n’est utilisable, aucune échelle n’est sélectionnée et aucun pas réel de B ne suit.

Le champ parent `integrity_ok=false` est un motif de fermeture du protocole ; il ne prouve pas un dépassement physique de couple ou une impossibilité anatomique. L’échec observé est l’absence de prévision admissible dans cette famille figée. Le succès de parité courte antérieur ne qualifiait pas les 480 pas demandés. Aucune comparaison causale A/B réelle complète, acquisition des talons ou récupération complète n’est établie.

## Gel, preuves et processus

**347/347 entrées gelées** concordent avec leurs empreintes, ainsi que les **7 anciennes pièces protégées** — quatre registres clos et trois binaires historiques. Les trois reçus admis par le parent (vérification, A, planification) correspondent à leurs empreintes enregistrées. L’enveloppe SHA-256 du registre est valide ; le résultat du runner reproduit ses compteurs et son statut clos.

Le manifeste sur disque et sa version normalisée dans le registre ont des empreintes différentes, conformément au code : la normalisation conserve fichiers/prérequis/mode mais retire `workerInputs`. Le contenu normalisé recalcule exactement l’empreinte stockée ; les 347 entrées et prérequis sont identiques dans les deux représentations. Le manifeste sur disque correspond aussi au hash du préflight. Aucune anomalie de gel n’en découle.

Un seul PID **12156**, un thread, binaire `c1c8a12825428758643a7f6e2fafb0a2710f7dca787c1daee07c27ec59849e9c`. Tous les permis portent ce même PID et la même échéance native. Le PID a disparu ; le recensement en lecture de `/proc/*/exe` après clôture ne trouve aucun worker natif de coordination ou de parité actif. Les six essais de supervision avec programme C inerte restent des tests sans physique et ne sont pas ajoutés au compteur natif.

## Délais : mesures et bornes distinctes

- Première réservation native : **11:10:50.135208 UTC** ; clôture durable : **11:11:02.456529 UTC**. Fenêtre mesurée par ces dates : **12,321321 s**.
- Le début monotone de supervision précède cette réservation de **0,079525382 s**. Du début de supervision à la clôture du registre : environ **12,400846382 s**, en combinant ce décalage monotone avec les dates UTC.
- Le reçu de supervision ne donne pas l’instant monotone final de récupération du processus : ces nombres ne sont donc pas présentés comme une durée exacte de tout le lancement/arrêt. L’échéance externe est bornée à **7 137 s** (30 + 7 100 + 7), et l’échéance native commune à **13:09:10.135208 UTC**. La clôture intervient bien avant ces plafonds ; aucun délai n’a été renouvelé.
- La somme des fenêtres documentées de la famille est désormais **21,951356994 s** : 9,630035994 s antérieures + 12,321321 s présentes. Les démarrages/arrêts historiques incomplets empêchent d’en faire une durée totale exacte. La borne conservatrice avant exécution de **7 183,630036 s**, démarrages/arrêts inclus selon les phases bornées, demeure inférieure aux 7 200 s familiales ; ce résultat effectif ne requiert aucune extension.

## Cumul familial et préparation

| Poste | Débit après clôture |
| --- | ---: |
| Ancien prototype et trois lots techniques | 3 988 |
| Nouvelle paire : permis effectivement réservés | 3 856 |
| **Famille** | **7 844 / 300 000** |
| Histoires réelles réservées, quatre emplacements | 3 840 |
| Prévisions longues réservées, deux appels B | 3 840 |
| Vérifications techniques, tous lots compris | 164 / 21 120 |

Les **82 724** pas de la préparation étaient la projection si les 40 appels B du nouveau lot étaient tous réservés. Le compteur effectif est **7 844** : les appels jamais admis ne constituent pas un remboursement. Les **quatre emplacements d’histoire sont utilisés**, en comptant les deux B réservés même lorsqu’ils n’intègrent aucun pas réel. Ni les sous-pas inutilisés ni l’arrêt précoce n’ouvrent une cinquième histoire, un B60 ou un renouvellement automatique.

La somme des quantités connues dans les registres familiaux devient **2 108** ; le total familial exact reste non certifié à cause des anciens compteurs inconnus, notamment le renouvellement technique clos après erreur de sérialisation. La certitude propre au présent lot n’efface pas cette incertitude historique.

Préparation : **28 489 s réservées sur 144 000 s**, **9 173 inspections sur 20 000**, échéance générale inchangée au **17 octobre à 20:14 UTC**. Le plafond de préparation supplémentaire de 3 600 s n’est pas renouvelé et aucun débit n’est remboursé. Les deux campagnes historiques hors famille et les tests factices restent exclus pour les raisons détaillées dans [l’audit cumulatif avant lancement](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/cumulative-budget-audit.md).

Les reçus de post-traitement produits séparément par le parent ont été lus : `archive-index.json` déclare zéro intégration ; `rendered/render-receipt.json` déclare zéro intégration, FK et requête collision. Hachage, compression, transformation visuelle des poses stockées et arithmétique sur matrices QP archivées consomment la préparation déjà réservée ; ils ne constituent pas de nouvelles inspections géométriques. Le compteur reste donc 9 173 et la charge conservatrice 28 489 s. Cette distinction suppose de rester sur les données archivées, sans reconstruire un monde ni recalculer la cinématique ; elle ne certifie pas le résultat du contrôle rationnel encore en cours au moment de cette note.

## Empreintes principales

| Pièce | SHA-256 |
| --- | --- |
| Registre clos `budget/ledger.json` | `e7c6831159ac1ae913a05f2af545a0521181d7142dbabd967908dfe7958fba97` |
| Manifeste sur disque | `2fbcb3db0cd1bccf14082e772b1efba63e567fe7fce35ed8cccbbbf2ac2cdde9` |
| Manifeste normalisé du registre | `2e6a9fe308763d940b0076f8b172bc88557bfaa8d06f30e9c7090043b6470604` |
| Rapport final B, non admis par le parent | `72ec39bf64e1f637cc1ed1099eff42cf8836ade2c21bb99dc12c843f502aa723` |
| Journal durable de vérification | `fcde8d9825d21d5892fb7a31adf382adb40225f0d0fd7d0e01bff5a2477ee8fa` |
| Journal durable A | `a66c893eb4d89d7a127ad04b999e4deb0fa9239e74485d0a42b385517156d406` |
| Journal durable des prévisions B | `8f277ded9d8a2c0226698e30561d78420d4438f9b59c21da5e4f6e82f97536c9` |
| Journal durable réel B | `d0449e433e8a0ac8eb05fff998e45c93fc2f95ddfa4f95839a58a4cd425a8965` |
| `native-run/run-result.json` | `ade23ba9efb1462f96780b86ecd66d910385a341952ab78535b5d8d00033b846` |
| `native-run/protocol.ndjson` | `f6af31c19c1064b7bea4f574797859eaefe6ec44e80cde5c51a00c1254b924aa` |

**Conclusion : clôture cohérente, comptabilité conservatrice intacte, aucun dépassement identifié. Aucun nouveau calcul physique ou relancement n’est autorisé par cette note.**


---

# Décision de continuation complète

# Décision après clôture : arrêter cette famille, ne pas continuer debout

**La famille expérimentale actuelle est abandonnée pour l'exécution physique. Aucun prolongement de A ou B, nouvelle variante, répétition ou renouvellement automatique n'est admis.** Le lot est clos en `closed-native-failure` ; le runner conclut `failed-closed`, `fullRecoveryQualified=false` et `fullHistoryAuthorized=false`. Cette décision suit les [conditions d'arrêt annoncées](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/launch-contract.json), pas une modification des seuils après lecture du résultat.

Cette note nouvelle est extérieure au [manifeste gelé](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/manifest.json), SHA `2fbcb3db0cd1bccf14082e772b1efba63e567fe7fce35ed8cccbbbf2ac2cdde9`. Elle repose sur la lecture des sources et reçus, la décompression en mémoire des archives, leurs empreintes et les certificats arithmétiques indépendants produits après clôture. Aucun fichier gelé n'a été modifié ; aucune simulation, construction de monde, FK, inspection géométrique nouvelle ou exécution de solveur n'a été effectuée pour cette décision.

| Partie exécutée | Constat | Portée |
| --- | --- | --- |
| Admission technique | 16 pas ; comparaisons copie/réel passées | Preuve technique courte, incluant la mémoire du diagnostic ; aucune stabilité locale démontrée |
| Histoire A | 960 pas, 4 s, diagnostic négatif, aucune récupération | La référence seule n'acquiert pas les talons et ne réalise pas le transfert demandé |
| Premier appel prédictif de B | Quatre échelles 0 ; 0,5 ; 1 ; 1,5 ; chaque copie termine 16 pas puis refuse au tick 4 avant le 17e | Aucune copie n'atteint les 480 pas requis ; aucune proposition admissible |
| Histoire réelle B | 0 pas, une tentative de planification, aucune échelle sélectionnée | Aucune commande de B évaluée dans une histoire réelle de transfert |

Le [reçu A](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/001-A.json) confirme `first_local_conditions_s=null`, zéro échantillon stable consécutif et `recoveriesObserved=0`. Les [mesures archivées de A](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/rendered/A-measurements.csv) montrent des charges postérieures de talon nulles pendant les 960 états observés. À 4 s, la fraction de support pied est de 0,3090, les charges non-pieds atteignent 489,08 N et le bassin est à 0,13328 m. Le faible mouvement terminal ne transforme pas cet appui au sol en stance debout. Le [reçu B](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/native-run/002-B.json) donne explicitement `actualNativeSteps=0`, `predictionNativeSteps=64` et « All four native predictions refused; no substitute proposal ».

Les reçus d'opération décrivent donc **16 + 960 + 64 = 1 040 pas nouveaux**. Le [registre clos](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/budget/ledger.json) conserve toutefois l'histoire B réservée et non complétée : `actualNativeStepsExact=false`, borne enregistrée 1 040–2 000. Le reçu terminal B précise son zéro réel sans réécrire cette incertitude comptable. Les 3 856 pas effectivement réservés dans ce lot restent débités, même si les copies ont refusé tôt. Les quatre emplacements d'histoire de la famille sont consommés ; le solde arithmétique du plafond de 300 000 pas ne permet aucune cinquième histoire.

**Le refus est localisé avant la continuation.** Le rapport de planification, archivé par morceaux gzip dans [l'index](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/archive-index.json), conserve les quatre problèmes numériques complets, les commandes précédentes et le stade `direct continuous solve` au sous-pas 16. Son contenu restauré a pour SHA `5ed160d4c9c7d4361ec5d285268a9aa694bc797a419768cfaac0370d46313711`. Chaque cas échoue à la recherche initiale de faisabilité des contraintes dures après 512 cycles. Le diagnostic signale la borne supérieure de la coordonnée de décision d'indice 36 : violation d'environ 0,1475 ; 0,1874 ; 0,1978 ; 0,1981 selon l'échelle, pour une tolérance de `1e-8`.

L'indice 36 désigne ici une composante du vecteur de vitesses articulaires bornées ; il n'identifie pas à lui seul la cause d'un conflit entre toutes les contraintes. Le message source distingue expressément **infaisabilité ou absence de convergence dans le budget fixé**. Un point projeté qui reste hors borne ne constitue pas, à lui seul, un certificat que l'intersection des contraintes est vide. Le présent refus est en amont de la certification des commandes f32 ; il ne suffit pas de l'attribuer au problème d'arrondi antérieur au sous-pas 12, ni de supprimer la borne qui a correctement arrêté l'exécution.

Les [certificats arithmétiques après clôture](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-qp-certificates.json), [contre-vérifiés indépendamment](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun-physical-audit.md), apportent une preuve plus forte : **huit contradictions exactes ont été vérifiées, une par pied dans chacun des quatre problèmes archivés**. Le [calcul rationnel exact](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/postrun_qp_certificate.py) n'appelle aucun solveur, monde natif ou FK. Les certificats combinent les trois lignes matérielles du pied et les bornes des coordonnées ; les marges de contradiction valent environ **0,0438738 à 0,06393477 m/s**, bien au-delà de l'allocation de tolérance maximale de **1,508696×10⁻⁸ m/s**. Cette preuve porte uniquement sur les coefficients et bornes des systèmes instantanés linéarisés enregistrés. Elle établit leur incompatibilité ; augmenter les cycles ne pourrait donc pas rendre ces mêmes systèmes admissibles. Elle ne prouve ni l'impossibilité d'un autre ensemble de contraintes correctement justifié, ni celle d'un mouvement physique, ni une impossibilité anatomique générale.

Les copies possèdent encore 85–86 % du poids sur les pieds à leur dernier état, mais zéro charge postérieure de talon, CoM/capture hors du support et aucun intervalle stable. Ce support partiel ne satisfait pas le diagnostic conjoint. Aucune copie ne constitue une continuation admissible simplement parce qu'elle a intégré ses seize premiers pas. Le refus de toutes les propositions active exactement l'arrêt prévu ; ni la moins mauvaise échelle, ni la référence seule, ni une extrapolation de la dernière commande ne doivent remplacer la proposition manquante.

**La portée expérimentale reste étroite.** A et B partent de `landed-crouch-left`, pose analytique peu fléchie de 25 segments à vitesses nulles, chargée directement par le moteur. Le bassin initial est à environ 0,9763 m et le CoM géométrique à 1,0103 m. Aucun impact physique ni settle préalable n'a produit cette entrée ; le temps zéro n'est donc pas un instant de chute mesuré. La [revue avant lancement](https://github.com/berenif/LaboratoireHumain/blob/cba6b74a74234ee1da4d4e0a6863dce9b1e9ef9c/docs/evidence/coordination-local-pair-2026-10-13/continuation-preflight.md) expose cette convention. Le résultat n'évalue ni le relèvement depuis une chute réelle, ni l'entrée accroupie profonde du logiciel source.

Le seul bras réel de la comparaison de transfert est A. **L'efficacité causale de B n'a pas été mesurée contre A**, puisque B a été arrêté avant son premier pas réel. Les quatre copies refusées constituent une preuve d'échec du planificateur figé à produire une proposition admissible sur cet état. Elles ne démontrent ni que la coordination globale est inefficace dans toute architecture, ni que les moteurs sont globalement trop faibles, ni qu'un relevage est anatomiquement impossible. L'abandon porte sur cette famille exécutée, ses paramètres et son allocation close.

**La référence de huit secondes ne donne aucun droit de poursuivre.** Ses six clés et ses deux grilles géométriques archivées restent des preuves de préparation statique. Le talon prévu à 0,70 s, le recentrage à 1,25 s et le maintien jusqu'à 1,80 s n'ont pas été réalisés par une histoire réelle réussie. A s'arrête à 4 s dans la phase d'extension ; B reste au début de sa propre histoire. Aucune exécution ne rejoint l'objectif articulaire debout à 7 s ou le maintien de cible jusqu'à 8 s. L'appui chargé, le handoff original, `recoveries == 1`, `Motion::Upright` avant 25 s et les 240 pas réels supplémentaires de stabilité demeurent non démontrés. Une pose géométrique finale et une seconde de cible constante ne remplacent aucun de ces critères.

Le prochain travail utile est **un audit hors simulation de l'assemblage des contraintes, des appuis et des bornes**, à cadrer dans la préparation restante et sans rouvrir cette famille. L'incompatibilité des quatre systèmes archivés étant certifiée, il ne s'agit plus de rechercher leur faisabilité en augmentant le nombre de cycles :

1. Conserver quatre cas autonomes au sous-pas 16 : matrices et bornes enregistrées, ordre des 46 coordonnées, mémoire précédente, patches retenus, refus et certificats. Conserver hashes et unités ; ne reconstruire aucun monde ni géométrie pour combler silencieusement une donnée manquante. La preuve exacte déjà obtenue est un point de départ à préserver, pas un motif de relancer les copies.
2. Relier les lignes contradictoires à leurs sources : origine des vitesses matérielles demandées, prise en compte de la racine libre, sélection et persistance des patches, commandes précédentes et calcul des bornes. Vérifier les conventions et distinguer les appuis effectivement observés des cibles géométriques prévues. L'existence du conflit est démontrée ; sa cause dans l'assemblage ou dans les hypothèses de contrôle reste à expliquer.
3. Produire un diagnostic contrôlable de cette provenance. Déterminer si le conflit révèle une erreur d'assemblage, une hypothèse de conservation matérielle incompatible avec cet état, une limite du modèle instantané ou une référence incompatible avec les commandes autorisées. Toute correction proposée doit être justifiée et testée par calculs purs, sans effacer les contraintes gênantes, modifier les critères de succès ou augmenter les caps pour obtenir artificiellement une solution. Un résultat numérique sur un problème modifié ne devient pas une commande physique et ne prouve pas le transfert.
4. **S'arrêter au diagnostic.** Présenter au parent la cause étayée, la correction éventuellement proposée, ses tests purs et le périmètre d'un éventuel protocole distinct. Celui-ci devrait de nouveau expliciter hypothèse, référence complète, admission technique, nombre d'histoires/copies, plafonds et arrêt. Aucune nouvelle physique n'est autorisée par cette note ni par l'espace restant sous un ancien plafond ; aucune demande de relance n'est implicite.

Tant que ce diagnostic n'est pas établi et qu'un nouveau protocole n'est pas défini par le parent, il n'y a ni essai supplémentaire, ni prolongement debout, ni qualification de récupération. Les traces présentes suffisent à conserver un résultat négatif honnête et une frontière d'arrêt vérifiable.
