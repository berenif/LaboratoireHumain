# Validation native distincte de la correction numérique — 13 octobre 2026

Le propriétaire a autorisé ce lot par « Go » le 13 octobre à 06:44 UTC,
après la proposition explicite de 44 sous-pas maximum et deux minutes.
Base vérifiée localement et à distance :
`8c050515d7fd9d8cb28f92742057e03cdc2dd9f5`, branche
`investigate/physical-support`. `main` reste à `c597959f46f37715035349c8cd0a699b6822111a`.

Ce lot vérifie la correction numérique des quatre facteurs figés
`0 / 0,5 / 1 / 1,5` : commandes compilées f32, installation native et égalité
entre prévision et exécution. Il ne comporte aucune histoire de relevage,
aucune continuation longue ni réglage. Les anciens registres restent clos.

## Protocole figé avant lancement

1. Compiler et tester sans intégration le worker dédié et son superviseur.
   Figer code, binaire, référence, tâches, attendu archivé et critères avant
   toute admission native. Ne pas remplacer le binaire du lot précédent.
2. Créer un registre distinct et une réclamation durable de l'autorisation.
   Réserver les **44 pas entiers**, sans remboursement ni reprise. Le
   processus unique possède un seul thread ; le plafond mural global de
   **120 s** inclut démarrage, vérifications et résultats. Un arrêt forcé
   par le superviseur complète les gardes du worker.
3. Construire la fixture initiale inchangée, sans settle. Exécuter **12 pas**
   communs, en comparant chaque état attendu à l'archive B. Au moindre écart,
   arrêter ; aucune correction au pas 13 n'est alors autorisée.
4. À partir du checkpoint complet au pas 12, pour chacune des quatre échelles,
   comparer une prévision de **4 pas** à une exécution de **4 pas** par le
   chemin natif réel, sur des instances distinctes : **4 × (4 + 4) = 32 pas**.
   Les copies servent uniquement à la vérification technique ; aucun état
   prévu n'est transplanté dans une histoire réelle.
5. Vérifier chaque commande contre les valeurs f32 certifiées lors du rejeu
   purement numérique ; relire son application sur les moteurs natifs.
   Comparer corps, contacts, impulsions, mémoire et horloges après chaque
   sous-pas, puis l'état final après les mises à jour de fin de tick.
6. Exiger la convergence connue du premier tick contraint : résolutions
   resserrées en **8 / 8 / 12 / 12 itérations**, seuils `1e-8` et `1e-6`
   inchangés. Conserver les deux tentatives numériques et leurs diagnostics.
7. Fermer le registre au premier refus, plafond ou verdict complet. Ne pas
   lancer de variante ni renouveler ce lot. Publier les preuves et le SHA
   distant exact.

L'archive ne contient pas de trajectoire physique antérieure après le pas 12.
Les attentes des pas 13–16 portent donc sur les commandes certifiées ; la
preuve d'état repose sur la comparaison indépendante des deux chemins natifs.
Les journaux CCD dédiés au chemin réel et les contacts/impulsions du monde
natif sont distingués. Aucune sérialisation ne sera présentée comme une
preuve universelle de tous les caches internes non exposés.

## Préparation et TODO

- [x] Vérifier checkout, distant, instructions locales et absence de calcul actif.
- [x] Préserver les empreintes des six registres antérieurs et de l'ancien binaire.
- [x] Extraire les attentes depuis les archives, sans intégration.
- [x] Implémenter puis tester statiquement le worker et le superviseur bornés.
- [x] Revue contradictoire du budget et des preuves d'application/parité.
- [x] Compiler, geler manifeste et autorisation ; publier le jalon préparatoire.
- [x] Soumettre l’unique lancement borné : refus de supervision avant tout worker.
- [x] Examiner et fermer le registre ; archiver les preuves du refus.

Le fichier [de préparation](evidence/coordination-native-parity-2026-10-13/preparation.json)
distingue les réservations de préparation du futur compteur physique. Les
anciennes vérifications de compilation restent des preuves historiques ; les
contrôles propres au nouveau worker seront archivés dans ce dossier distinct.

## Verdict : blocage de supervision, aucune nouvelle physique

L’unique lancement a eu lieu le **13 octobre 2026 à 07:15:05,374 UTC**, après
publication et vérification distante du jalon
`08aa8684911a87e0c3d6565855a0bb2c84340024`. Le registre est fermé en
`closed-failure` après **0,02845 s**. Les **44 sous-pas sont réservés et non
remboursés** ; l’autorisation est close, sans nouvelle tentative.

**Zéro worker créé, zéro permis envoyé, zéro intégration native exécutée.**
À l’entrée de la garde de délai, `child_pids()` ne trouve pas
`/proc/3658/task/3658/children`. Cette lecture précède le `yield` de la garde
et l’appel à `worker_factory`. Le protocole est vide, le champ worker est nul
et aucun fichier stderr du worker n’a été créé. Le registre conserve
`actualNativeStepsKnown: null`, puisqu’aucun reçu natif n’existe ; l’absence
d’intégration est établie séparément par ce chemin de contrôle et l’
[audit de clôture](evidence/coordination-native-parity-2026-10-13/post-run-root-audit.json).
Aucun compteur ni registre antérieur n’a été réécrit pour changer ce constat.

Ce défaut vient d’un préalable système omis : les 41 tests synthétiques
couvraient les gardes avec une inspection de processus simulée, et le contrôle
préliminaire vérifiait les fichiers sans tester cette interface `/proc` réelle.
Les cinq tests Rust purs, la compilation de production et le build release
réussis ne prouvaient donc pas que le superviseur pouvait démarrer ici.
Le défaut est dans la préparation du test, avant l’évaluation du contrôleur.

Les 294 fichiers figés, les six anciens registres et l’ancien binaire ont été
vérifiés inchangés après l’arrêt. Le [manifeste](evidence/coordination-native-parity-2026-10-13/manifest.json)
porte SHA-256 `40e492a29d723118cb26990b9dd2866d498d9a1a5d5ae815d7aae1d9d92a407a` ;
le binaire dédié porte `9e125785fad85b0befc9373c3116d66aa32e8050a998ce08974b0da29d2760cc`.
Les [tests et compilations](evidence/coordination-native-parity-2026-10-13/build-validation.json),
la [réclamation unique](evidence/coordination-native-parity-2026-10-13/authorization-claim.json),
le [registre clos](evidence/coordination-native-parity-2026-10-13/native-run/physical-ledger.json)
et le [résultat du superviseur](evidence/coordination-native-parity-2026-10-13/native-run/run-result.json)
constituent les preuves de ce lot. Il n’existe aucun nouvel état physique à
rendre en vidéo ; l’index d’archive indique explicitement zéro trace native.

## Décision et suite nécessaire

La correction numérique conserve ses certificats statiques ; **son application
native et la parité prévision/exécution restent non vérifiées**. Ce refus ne
permet aucune conclusion sur les talons, la stabilité ou la faisabilité
anatomique. La continuation de huit secondes, le relevage en moins de
25 secondes, la seconde stable et les 24 qualifications restent inconnus.
Récupération et pas correctifs restent désactivés par défaut.

Avant toute proposition d’un nouveau lot, le contrôle préliminaire devra
vérifier en lecture seule les interfaces de processus réellement disponibles
*avant* de réclamer l’autorisation. Une supervision compatible devra ensuite
être revue et testée avec des processus inertes, sans moteur physique : identité,
thread unique, descendance, interruption pendant le démarrage et arrêt au délai.
Il ne suffit pas d’ignorer l’exception ou de retirer le contrôle de descendance.
Les fichiers figés de ce lot restent intacts. Une nouvelle exécution native
nécessite une autorisation distincte via le parent ; rien n’est relancé ici.
