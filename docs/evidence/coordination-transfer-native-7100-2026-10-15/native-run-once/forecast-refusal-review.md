# Revue causale du refus de la prévision unique — 15 octobre 2026

Le refus à l’horloge native **78** est causé par la **perte de correspondance du point matériel retenu de l’avant-pied gauche**, segment 19, patch 0. Le second motif du message générique, une dérive de plus de 10 mm, ne s’est pas produit. Cette conclusion provient uniquement des archives et du code figé ; aucun test, monde, sous-pas, FK ou assemblage supplémentaire n’a été exécuté.

## Frontière exacte de l’essai

Les130 sous-pas effectivement exécutés se répartissent en 36 de préfixe technique, 16 de parité technique (deux copies de 4 pas et leurs deux contreparties de 4 pas), 36 de préfixe de l’histoire principale et 42 de prévision native copiée. Les deux paires techniques passent. L’histoire principale reste à A36 : **zéro commande de transfert exécutée dans cette histoire après A36**. Les contreparties techniques ne constituent pas sa continuation.

La première prévision demandait 480 pas depuis 36. Après l’intégration de la commande 77, elle atteint 78 (0,325s depuis l’origine,0,175s après A36). L’observation 78 est refusée avant installation/intégration de la commande 78. Aucun horizon complet n’est certifié et aucune fenêtre réelle n’est admise. L’essai est clos sans relance.

## Prédicat qui a effectivement échoué

| Point retenu | Distance au contact chargé le plus proche à 78 | Dérive tangente depuis l’ancre à 78 | Résultat |
|---|---:|---:|---|
| Gauche, segment 19, patch 0 |90,000026mm|0,186918mm|Correspondance absente (>12mm)|
| Droit, segment 24, patch 1 |0,000014mm|0,202653mm|Correspondance présente|

Le point gauche retenu est localement `[-0.0450000092, -0.0450000092, 0.0600000136]`m. À 77, son correspondant est à 7,45 nanomètres et porte 65,341455 N. À 78, il n’est plus dans les contacts sélectionnés et chargés de l’observation. Le plus proche devient l’autre coin avant, à 90 mm, portant 0,989888 N. Les deux points arrière de ce même avant-pied portent 47,452341 N et 37,110702 N. **L’avant-pied gauche reste chargé à 85,552933 N** ; ce refus ne signifie pas que le pied entier a perdu son appui. Le point retenu droit porte encore 19,137428 N.

Le point gauche transformé par sa pose déjà archivée reste à −3,929819 mm en hauteur, soit 1,347684 mm au-dessus de son ancre initiale située à −5,277503 mm. Il serait donc injustifié de décrire une semelle géométriquement soulevée. Les contacts sauvegardés sont filtrés : ils ne suffisent pas à départager déchargement du point, changement de sélection du manifold ou autre exclusion par l’observateur. Le fait établi est l’absence du point matériel **dans le support chargé exigé par ce candidat**, avec maintien d’autres contacts du segment.

## Dernières observations de la prévision

Les colonnes ci-dessous utilisent l’horloge de l’observation, et non celle de la ligne qui la contient. Une ligne `transfer-preview-step` à N contient les diagnostics post-intégration N, mais sa `transferProposal` provient de l’observation N−1.

| Observation | Charge totale des pieds(N) | Fraction du poids | Capture dans support | Patch gauche/droit présent | Dérive gauche/droite(mm) |
|---:|---:|---:|---|---|---|
|72|649.048|0.916370|non|oui/oui|0.135551/0.080447|
|73|722.078|1.019478|non|oui/oui|0.125079/0.069890|
|74|577.498|0.815351|non|oui/oui|0.117040/0.117148|
|75|768.780|1.085415|non|oui/oui|0.103552/0.117447|
|76|765.429|1.080684|non|oui/oui|0.111463/0.113102|
|77|664.079|0.937591|non|oui/oui|0.121821/0.161699|
|78|597.083|0.843002|non|non/oui|0.186918/0.202653|

À 78, les segments18/19/23/24 portent respectivement214,533/85,553/182,428/114,569N, avec55/78/55/78 sous-pas de charge persistante. Ce sont des appuis avant de l’arrière-pied et des avant-pieds : les régions talonnières ciblées ont **zéro charge et zéro persistance** pendant les 42 échantillons de prévision. Les points talonniers visés sont encore à 20,944878 mm et 21,434727 mm du plan de support.

Le CoM reste dans le support à tous les échantillons de prévision, mais la capture en sort dès 45. À 78, CoM=`[-0.0020006422,0.9756494164,0.0443562903]`m ; vitesse=`[-0.0072414279,-0.0470135771,0.1067842543]`m/s ; capture XZ=`[-0.0042843294,0.0780322212]`m. La proposition précédente demandait une vitesse CoM d’environ −0,149923 m/s enZ ; la vitesse mesurée est positive. Il s’agit d’un écart observé entre intention cinématique et mouvement natif ForceBased, pas d’une preuve que toute commande admissible échouerait.

## Horloges et portée de l’intégrité

`support.native_substep=78`, mais `owner.transfer.decision.lastObserved=77` : `Adapter::observe` travaille sur une copie et ne l’engage qu’après validation ; le refus est atomique. Les champs du tracker sauvegardé à 77 restent donc vrais à 77, sans contredire la perte observée à 78. Le `snapshot` lit directement les poses et vitesses de `world.bodies` : sa géométrie et `simulation_time_s=0.325` sont bien à 78. En revanche, `contact_time_s=0.316666667` et son cache `contacts` correspondent au dernier tick externe terminé, 76. Ses diagnostics clonés peuvent aussi être décalés, tandis que le statut de récupération est rafraîchi. Il faut distinguer les champs au lieu d’attribuer une seule horloge au snapshot entier.

Le champ de résultat `integrity_ok=false` est produit par `result.is_ok() && exact && reconciled`. Le refus contrôlé rend le premier terme faux ; les compteurs restent exacts et réconciliés, les deux vérifications de parité ont passé. Ce champ seul ne démontre ni corruption de fichiers, ni divergence physique copie/réel, ni calcul non compté.

## Décision et limites

Le diagnostic attendu est disponible : le garde conservateur de maintien du point matériel a effectivement échoué sur une observation identifiable, sans excès de dérive. Le refus respecte donc la condition d’arrêt du protocole figé. Aucun succès local, événement de continuation, relevage ou seconde stable n’est établi. Cette seule prévision ne prouve ni impossibilité anatomique ni nécessité générale des mains/genoux, et n’autorise aucune augmentation de force/friction, modification de seuil ou relance.

Le fichier JSON compagnon contient les observations77/78, les derniers échantillons, les calculs élémentaires et les SHA256 des sources. Les valeurs dérivées emploient uniquement position et quaternion déjà archivés, sans arbre cinématique ni création de corps. Références de code : `transfer_adapter.rs:334–371` (rejet atomique et garde), `support_state.rs:210–250` (correspondance/dérive), `native_observation.rs:74–116,154–193` (contacts sélectionnés et filtrage), `transfer_worker.rs:440–510,630` (prévision, frontière réelle et champ d’intégrité), `lib.rs:1203–1278` (horloges et contenu du snapshot).
