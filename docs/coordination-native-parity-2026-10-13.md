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
- [ ] Compiler, geler manifeste et autorisation ; publier le jalon préparatoire.
- [ ] Exécuter exactement une admission sous 44 pas / 120 s.
- [ ] Examiner les preuves, fermer et publier le verdict.

Le fichier [de préparation](evidence/coordination-native-parity-2026-10-13/preparation.json)
distingue les réservations de préparation du futur compteur physique. Les
anciennes vérifications de compilation restent des preuves historiques ; les
contrôles propres au nouveau worker seront archivés dans ce dossier distinct.

## Verdict

Préparation terminée : 5 tests Rust purs, 41 tests synthétiques du superviseur,
compilation de production et compilation release dédiée réussis. Le manifeste
fige les sources, entrées et le binaire. Aucun sous-pas de ce nouveau lot n'est
encore exécuté.
Un succès éventuel signifiera uniquement « commandes corrigées appliquées et
prévisibles sur les quatre pas vérifiés depuis cet état ». L'acquisition des
talons, la stabilité, la continuation de huit secondes, le relevage en moins
de 25 secondes, la seconde stable et les 24 qualifications restent inconnus.
