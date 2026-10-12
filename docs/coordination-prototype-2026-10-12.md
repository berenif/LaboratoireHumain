# Prototype de coordination : préparation et décision

**Protocole clos : prototype implémenté, refus numérique de B.** Voir le [rapport de résultat](coordination-prototype-result-2026-10-12.md). Total exécuté : 1 024 sous-pas ; aucune récupération qualifiée.

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
