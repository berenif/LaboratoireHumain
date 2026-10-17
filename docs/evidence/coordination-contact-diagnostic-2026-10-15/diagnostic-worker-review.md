# Contre-revue du worker de diagnostic borné — 15 octobre 2026

**Aucun blocage détecté dans le patch Rust examiné.** Cette revue statique ne remplace pas les contrôles du superviseur, du gel, du binaire ou de l’autorisation avant lancement. Elle n’a exécuté aucun test, monde, sous-pas natif ou FK et n’a modifié aucun Rust.

La base examinée est `a5c8ff36af4dfbe784277aebc91021040b947a52`. Le worker relu a pour SHA256 `811df09bae544476e03026a9d832f7ea306ff774fe85e42929f974efd2c3f89c`. Les sources et leur comparaison à la base figurent dans `diagnostic-worker-review.json`.

## Budget et arrêt avant transfert réel

L’opération doit être explicitement `contact-diagnostic`. Un champ absent ou une opération inconnue est refusé. Ses permis réservent exactement **52 + 516 = 568 sous-pas** : 36 pas de préfixe technique, deux paires copie/réel de 4 + 4 pas, 36 pas de préfixe principal et au plus un horizon prédit de 480 pas. Les tailles des permis du mode complet sont incompatibles avec celles de ce diagnostic.

Les deux permis ont des identifiants distincts, le même PID et la même échéance, sans réservation préalable dans le worker. Leur temps natif restant est plafonné à 113 s. Les **120 s tout compris** restent à imposer par le superviseur pour inclure démarrage, collecte et arrêt du processus.

La fonction `history` exige une nouvelle parité technique. Si la prévision refuse, le diagnostic se ferme sur erreur. Sinon `finish_after_forecast` vérifie un seul horizon demandé et exactement 36 pas de préfixe principal, puis retourne **avant** `certify_transfer_horizon`, `admit_transfer_window` et la boucle d’exécution réelle. Aucun reçu de fenêtre réelle n’est donc créé par le chemin diagnostic.

`diagnostic_completed` est distinct d’un relevage ou succès local. `no_real_transfer` signifie zéro transfert dans l’histoire principale après A36 : les huit contreparties techniques sont bien des pas natifs exécutés et sont déjà comptés parmi les 52 pas techniques. Le worker accepte une seule opération puis exige EOF; le verrou durable contre une relance dans un nouveau processus appartient au superviseur.

## Préfixe historique : migration d’un seul champ ajouté

La projection nouvelle retire uniquement le chemin exact `support.native_contact_boundary`. Les exclusions préexistantes du libellé `kind` et de `owner.transfer` absent ou nul restent identiques. Aucun champ de `MaterialPatch`, contact, charge, impulsion CCD, moteur, commande, snapshot ou autre état historique n’est supprimé. Les noms semblables ailleurs dans le JSON restent comparés, comme tout champ inconnu. Le comparateur conserve notamment la distinction binaire entre +0 et −0.

Avant projection, la télémétrie vivante doit être présente, typée, de schéma 1, d’horloge conforme aux deux horloges natives de la ligne et composée des 25 segments dans leur ordre canonique. Elle reste sauvegardée intégralement dans les nouveaux états et dans chaque `transfer-preview-step`; l’absence de correspondant ancien n’efface donc pas les nouvelles preuves.

Les tests purs préparés couvrent les tailles exactes, l’absence de mode implicite, les permis trop longs, l’arrêt après un horizon, la présence de télémétrie et son horloge, ainsi que des mutations de charge signée, commande, champ inconnu et état matériel. Ils ont été **lus**, sans être exécutés par cette revue.

## Copie native et invariants

`copy_static_snapshot` emploie `coordination: source.coordination.clone()`. Il n’effectue pas de désérialisation JSON du propriétaire; le cache de continuité exclu de la désérialisation est donc conservé par cette copie. Les deux nouvelles paires techniques vérifient la parité du nouveau binaire avant toute prévision principale. L’ancienne parité n’est pas réutilisée comme qualification.

Le patch examiné ne change aucun seuil physique ou contrôle de continuité, ni couple, masse, friction, collision ou anatomie. Les modules de contact, continuité, adaptation, copie et comptage relevés dans le JSON sont inchangés par rapport à la base. Les références matérielles, le plafond de dérive, le contrat d’actionnement et l’admission native restent applicables.

La taille des traces et le coût de sérialisation peuvent augmenter. Aucune mesure native ne provient de cette revue; le plafond de temps doit fermer l’exécution s’il est atteint, sans remboursement, renouvellement ou essai de variante. Aucun succès local, relevage, seconde stable ou explication définitive du point perdu n’est revendiqué ici.

## Complément avant gel : endpoint obligatoire et horloge de l’échantillon

L’audit du responsable a relevé que l’ancien enregistrement conditionnel de l’endpoint pouvait être omis sans empêcher `diagnostic_completed`. Le chemin diagnostic exige désormais une `SupportObservation` valide, la télémétrie de frontière présente au schéma 1, les 25 segments canoniques et une horloge identique à celle de la simulation copiée.

La contre-revue a ensuite demandé de remplacer la simple présence de l’échantillon par une vérification de son horloge. Le garde reçoit maintenant `Option<u64>` et exige `Some(expected_substep)` pour `LocalSupportSample.native_substep`. Les cas absent et périmé ont été ajoutés au test pur relu.

Si l’observation finale, sa télémétrie ou son échantillon sont incomplets, le worker refuse avant tout succès diagnostique et avant toute fenêtre réelle. Le refus de validation de l’endpoint conserve également le refus ou l’erreur d’exécution initiale de la prévision dans la trace et dans l’erreur retournée. Les données finales disponibles sont conservées, sans nouvelle intégration ni relance. Les deux constats sont clos à la lecture du worker `811df09bae544476e03026a9d832f7ea306ff774fe85e42929f974efd2c3f89c`. Aucun autre fichier source relevé dans la revue n’a changé.

Un refus de prévision peut légitimement laisser l’échantillon diagnostique à N−1 alors que `support` et `native_contact_boundary` décrivent l’état courant N. Le garde strict ajoutera alors un **refus secondaire de preuve d’endpoint**. L’analyse devra conserver cette distinction avec le refus initial de prévision, dont le motif est archivé; elle ne devra pas attribuer N−1 aux poses ou contacts courants, ni déclencher de nouvelle simulation ou assouplir le garde.
