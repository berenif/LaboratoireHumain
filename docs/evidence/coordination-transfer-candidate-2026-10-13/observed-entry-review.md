# Entrée native observée au pas 36 : candidat dynamique, aucun transfert acquis

**Un état réellement intégré est disponible à 0,15 s de A : pas natif 36, avant la commande 36.** Ce n'est plus la pose analytique à t=0. Les quatre segments de pieds portent une charge persistante, et les prédicats archivés placent CoM et capture dans leur support. L'état reste dynamique, peu fléchi et sans charge postérieure de talon ; il n'est ni une stance stable ni une récupération.

La [fixture autonome](observed-entry-step36.json) contient le relevé natif complet de cette frontière, son propriétaire de commandes, les 36 commandes du préfixe et une fenêtre d'observations 24–36. Elle sert à préparer l'adaptateur et le raccord hors physique. **Elle ne constitue pas un checkpoint de monde restaurable.** Une future histoire fidèle devrait rejouer les 36 pas originaux puis vérifier leur état, dans un protocole distinct ; aucun préfixe n'est exécuté ici.

## Sélection et inventaire des preuves

Le parent a fixé le critère après l'identification de la fenêtre 35–39 : prendre le **premier multiple de quatre positif** ayant les quatre segments pieds actuellement chargés ≥3 N et persistants ≥12 pas, avec les deux prédicats archivés CoM/capture intérieurs. Les points courants retenus par ce filtre ont une normale verticale ≥0,65. Il ne s'agit pas de choisir le meilleur état selon le succès futur, ni de considérer ces conditions comme un certificat de transfert. Les seules frontières correspondantes dans A sont 36 et 40 ; 36 est donc retenu. Le pas 35 satisfait déjà la persistance, mais n'est pas une frontière complète du contrôle à 60 Hz.

L'[inventaire numérique](observed-entry-scan.json) examine les archives compressées suivies, vérifie les SHA des fichiers gzip, de leurs parties et du flux brut reconstitué, puis lit les valeurs JSON sans moteur ni géométrie nouvelle.

| Archive inspectée | Relevés | Dernier pas natif | Frontières satisfaisant le critère | Portée |
| --- | ---: | ---: | --- | --- |
| A, paire locale du 13 octobre | 960 | 960 | 36, 40 | Histoire réelle sélectionnée |
| A, prototype du 12 octobre | 960 | 960 | 36, 40 | Autre histoire réelle, même famille de référence |
| Copies B, paire locale | 64 | 16 | Aucune | Quatre prévisions refusées, zéro B réel |
| Copies B, prototype | 48 | 12 | Aucune | Quatre prévisions refusées, zéro B réel |
| Admission locale | 16 | 4 | Aucune | Vérification technique courte |
| Admission prototype | 16 | 4 | Aucune | Vérification technique courte |

Les 2 064 lignes ne sont pas 2 064 nouveaux pas : elles sont uniquement relues. Les traces de parité numérique couvrent les états initiaux jusqu'au pas 16, sans état accroupi ultérieur. Les traces factices de portabilité du lanceur sont exclues. Les grilles géométriques de la référence ne sont pas des observations physiques.

Les rapports getup A/B historiques, l'enquête sur les pieds et les deux rapports foot-coordination ont également été inspectés. Leurs snapshots accessibles sont soit initiaux à t=0, soit terminaux avec le corps au sol ; les rapports de synthèse fournissent des mesures intermédiaires mais pas le relevé complet nécessaire à une reprise. Le répertoire historique brut `evidence/` est absent de ce checkout. Cette recherche ne prouve donc pas qu'aucun autre état utile n'a jamais existé ; elle identifie le candidat traçable dans les archives actuellement disponibles.

## Mesures copiées, limites du candidat

| Grandeur au pas 36 | Valeur archivée |
| --- | ---: |
| Temps physique | 0,150000 s |
| Hauteur du bassin | 0,947701216 m |
| Vitesse verticale du bassin | −0,001506162 m/s |
| Hauteur du CoM | 0,979662061 m |
| Vitesse verticale du CoM | −0,021733573 m/s |
| Charge gauche Foot / Forefoot | 150,696671 / 224,105682 N |
| Charge droite Foot / Forefoot | 175,822205 / 195,473923 N |
| Persistance Foot / Forefoot, chaque côté | 13 / 36 pas |
| Fraction de support pieds | 1,053391848 |
| CoM et capture dans le support | vrai / vrai |
| Charge postérieure des talons | 0 / 0 N |
| Vitesse linéaire maximale des 25 corps | 0,408759775 m/s |
| Vitesse angulaire maximale des 25 corps | 0,997766119 rad/s |
| Diagnostic local acquis / intervalle stable | faux / 0 pas |

Une charge d'un segment `Foot` ne prouve pas la charge de sa région postérieure : le diagnostic talon reste nul. La fraction supérieure à un est un résultat instantané de la convention d'observation des impulsions, pas une stabilité supplémentaire. Les deux vitesses maximales dépassent les limites locales de calme 0,22 m/s et 0,65 rad/s. Les appuis se sont organisés physiquement, mais la transition talons/recentrage reste à réaliser.

Les poses mesurées des cuisses et tibias sont conservées intégralement. Par exemple, les composantes x/w des quaternions gauche valent environ −0,071178/0,997459 pour `leftThigh` et 0,258105/0,966112 pour `leftShin`. Le genou reçoit alors une **cible** de 0,660000026 rad ; ce nombre n'est pas un angle articulaire mesuré. Aucun calcul de coordonnées articulaires physiques ou FK n'a été ajouté à l'extraction. Les 25 poses et vitesses donnent les données nécessaires à un éventuel calcul purement algébrique séparé et explicitement nommé.

L'histoire enregistrée ultérieure ne stabilise pas cette configuration. À 0,20 s, la capture est déjà extérieure malgré une fraction de support de 0,988809 ; à 1,25 s, le bassin est à 0,812881 m avec une vitesse verticale de −0,655869 m/s, support pieds 0,645840, CoM/capture extérieurs. À 1,50 s, le bassin est à 0,463614 m, sa vitesse verticale vaut −2,455810 m/s et le support pieds tombe à 0,058916. Une pose plus basse à cet instant est une phase de chute, pas un meilleur accroupi stable. Ces mesures justifient le choix d'une entrée précoce traçable ; elles ne prédisent pas ce que ferait le nouveau contrôleur.

## Frontière de commande et provenance du préfixe

`commandEntry.native_substep=36`, `last_installed_command_substep=35`. Les champs `previous_q` et `previous_dq` sont copiés de `owner.last.positions/velocities`, donc de la dernière commande installée. `owner.nextSubstep=36`, `previousStep=35` et les corps sont ceux observés après la trente-sixième intégration. Le prochain adaptateur doit commencer par `configure(36)`, sans réappliquer ou sauter la commande 35.

Le snapshot possède encore `stamp.tick=8` et `contact_time_s=0,13333333333333333`, tandis que `support.native_substep=36`, `support.time_s=0,15` et `snapshot.simulation_time_s=0,15`. La frontière fine doit utiliser ces derniers champs. Le tick plus grossier ne permet pas de ramener artificiellement l'état au pas 32.

Le propriétaire est celui de **A**, `enabled=false` ; ses patches actifs sont vides. Il ne faut pas inventer un historique de B ni créer rétrospectivement des ancres persistantes à partir du seul relevé. Les charges, contacts et compteurs de persistance observés existent indépendamment des tâches du propriétaire. Une future acquisition par le nouveau contrôleur doit être explicite et conserver cette distinction.

Le préfixe commence avec le seed analytique `landed-crouch-left` et suit la référence A durant 36 pas. Les collisions pied-sol qui y apparaissent sont natives, mais il ne s'agit pas d'une chute préalable provoquée puis récupérée ; les compteurs ne prouvent aucun impact externe ni relevage. Réexécuter ce préfixe donnerait une entrée accroupie effectivement observée pour un test de transfert, pas un test complet de relevage après chute. Les vitesses présentes ne doivent pas être annulées à la frontière.

## État de tâches : dérivation massique explicite

Le fichier [observed-entry-task-state.json](observed-entry-task-state.json) expose `observed_task_state={native_substep:36,bodies:[…25…]}` au format `geometric_tasks::ObservedState`. Positions, rotations et vitesses sont des copies des observations. Les champs `mass_kg` et `world_center_of_mass_m` ont une autre provenance : ils sont **dérivés**, sans construction de modèle ou de monde, des propriétés massiques natives déjà archivées.

Le [script de dérivation](observed-entry-derive-tasks.py) lit [archived-initial-world.json](../coordination-native-serialization-2026-10-13/archived-initial-world.json), SHA `7660dda75bd7b8edf567a3e804f452457208e3a5cfd35cd9478704ba473feb55`, et vérifie son reçu d'extraction. Les 25 associations sont uniques : identité canonique fournie par les snapshots, correspondance exacte des bits f32 de pose/vitesses initiales entre le rapport A et l'arène archivée, puis correspondance indépendante de l'indice d'arène, de `inv_mass` et de `local_com` avec [l'archive massique précédente](../coordination-numeric-correction-2026-10-12/archived-step12.json). Aucune masse n'est recalculée depuis des maillages ou depuis `Model::canonical`.

La masse corporelle est `f32(1 / inv_mass_archivée)`. Le CoM corporel dérivé est la translation observée plus la rotation, par son quaternion observé, du `local_com` natif archivé. Chaque opération élémentaire est arrondie en f32, sans renormaliser les quaternions. La masse totale, sommée dans l'ordre canonique, vaut **72,19999694824219 kg**. Le CoM global agrégé vaut exactement, dans cette implémentation, les trois valeurs déjà archivées : `[-0,00037677420186810195, 0,9796620607376099, 0,034044403582811356]` m. La même formule reproduit également tous les CoM corporels du monde initial archivé avec un écart nul.

Ces vérifications confortent l'adaptation pure ; elles ne transforment pas les **caches corporels au pas 36**, absents de la trace, en mesures directes. L'identité bit à bit de ces caches individuels n'est pas certifiée. Le fichier porte expressément `worldComIsDirectlyArchivedAtStep36=false` et `bodyWorldComCacheIdentityAtStep36Certified=false`. Il est utilisable pour les tests de formulation hors physique ; une reprise native exacte repose sur le rejeu vérifié du préfixe, pas sur l'installation des valeurs dérivées dans un monde neuf.

## Reproductibilité et suites possibles

L'[extracteur](observed-entry-extract.py) ne lit que JSON/gzip et copie des champs ; il refuse d'écraser un résultat différent. Son [reçu](observed-entry-receipt.json) lie la fixture et l'inventaire aux empreintes des archives. Les scripts peuvent être rejoués pour contrôler l'extraction et la dérivation :

```sh
python3 docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-extract.py
python3 docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-derive-tasks.py
```

La fixture brute SHA `3bd00408a5fdb3df9f17feda430f5a1aad8334b17c244d3b119a09325891c433` contient 402 669 octets ; l'état de tâches dérivé SHA `6c9a2385c9c436b8f8d9d5059ce3b254ae44184ede699992eaab7ceb5ab57a42` contient 20 718 octets. Le flux source A décompressé a pour SHA `8338425db8c92038b98b89ae22c4e6f366a6ebd71b000aa615700acbd530bdd8`. Les liens et empreintes de toutes les parties gzip sont conservés dans la fixture ; aucun brut absent d'un clone n'est requis par les scripts.

Le raccord et la continuation ne sont pas encore admis : partir de la dernière commande réellement installée, acquérir les deux talons par observation, puis rejoindre seulement après le jalon requis la partie debout de la référence de huit secondes. Toute nouvelle géométrie, branche ou durée demande ses propres contrôles. La machine de retrait complet exige en outre une vraie disparition des contacts du pied avant réacquisition ; un simple tour de phases ne doit jamais recréer une ancre sur un contact resté chargé. Le pivot roulant sans disparition reste une autre transition à justifier.

Cette livraison fournit des entrées concrètes aux tests purs de raccord et de transfert. Elle n'ouvre aucun budget physique, ne crée aucun registre d'exécution et ne qualifie ni appui stable, ni continuation debout, ni relevage complet. **Travail de cette extraction : zéro FK, zéro monde et zéro intégration native.**
