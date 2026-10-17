# Revue du worker dédié à la paire locale

Préparation uniquement, le 13 octobre 2026. Aucun worker Rust n’a été lancé, aucune `Simulation` ni aucun modèle n’a été construit par les tests sélectionnés ; aucun sous-pas natif et aucun calcul FK n’ont été exécutés pendant cette préparation. Le lancement autorisé et son registre restent distincts.

## Corrections instrumentales

Le nouveau target Cargo `coordination-local-pair-worker` conserve l’entrée `transition_worker::main` et la feature expérimentale existante. Les trois exécutables historiques sont inventoriés dans `historical-binaries-before.json` et leur identité est contrôlée après compilation. Aucun contrôleur, solveur, installateur, gain, plafond moteur, profil ou critère de diagnostic n’a été modifié.

Le problème de sérialisation du worker numérique n’était pas présent dans ce chemin : `NativeParityTrace` sérialise directement le monde par `serde_json::to_vec`, puis conserve ces octets. Il ne convertit pas les masques Rapier `u128` en `serde_json::Value`. Cette propriété reste inchangée ; des valeurs synthétiques `u128::MAX`, `u128::MAX−1`, `u64::MAX+1` et `−0.0_f32` sont vérifiées sans monde physique.

Les hooks `cfg(all(test, feature="coordination-worker"))` sont immédiatement avant et après l’unique appel original `world.step_with_events`. Avant l’appel, ils exigent le permis actif, le crédit déjà réservé, l’horloge exacte et les échéances UTC/monotone. L’échéance est revérifiée après l’écriture préparatoire. Après le retour du monde, la completion est enregistrée avant les observations et sérialisations riches. Ces hooks n’écrivent aucune commande ni aucun état physique.

Chaque permis crée `<outputPath>.accounting.jsonl`, avec `flush` et `sync_data` aux admissions/completions. Une admission sans completion ne prouve pas qu’un pas a été achevé : le compte exact devient `null`, les bornes restent explicites, le worker ferme et aucun candidat supplémentaire ne peut intégrer. Les agrégats incertains sont également `null`, avec des sous-totaux `knownCompleted…` distincts. Si le processus disparaît sans reçu, le parent conserve toute la réservation ; les checkpoints ne justifient aucun remboursement.

Après un refus normal de candidat, les places non utilisées du tick sont explicitement abandonnées et journalisées, uniquement sans appel en cours et avec un compte exact. Le débit complet de quatre demeure consommé. Le candidat suivant doit prendre un nouveau débit. Une panique interceptée ferme le travail natif ; elle ne déclenche aucune proposition de remplacement.

## Parité et propriétaire des commandes

La comparaison existante des quatre états est renforcée : comparaison initiale avant le premier pas réel, comparaison de chaque préfixe réel avant le pas suivant, puis comparaison après la finalisation du tick. L’admission du lot de seize et la première prédiction sélectionnée exigent toutes deux `native_kernel_admission_complete`. La mémoire externe `DiagnosticMemory` est également clonée et comparée : diagnostic complet, dernier échantillon et vitesse CoM servant au score. Chaque côté met cette mémoire à jour par `observe` avant de la joindre à la trace ; l’état initial et l’état de fin de tick la contiennent également. Le score garde ses cinq composantes, leur ordre et leurs seuils. Un test synthétique refuse une différence de cette seule mémoire à physique identique. La mémoire Recovery est projetée exhaustivement, avec ancres/cibles/vitesses en bits ; `mode_signature`, contacts, horloge de tick et impulsion moteur signée sont inclus. Les octets du monde conservent contacts/manifolds/impulsions sérialisés ; cela n’est pas une revendication d’identité de tous les espaces de travail transitoires Rapier.

Lecture du propriétaire unique : `configure_motors` rend la main après `owner.configure`, et le hook de trajectoire hors ligne retourne immédiatement lorsque `coordination` est présent. Les prévisions et l’histoire utilisent le même noyau et le même propriétaire explicitement cloné. Aucun second installateur n’a été ajouté.

## Validation et limites

Les filtres exacts consignés dans `build-validation.json` exécutent neuf tests comptables/JSON dans `transition_worker::pure_tests` et cinq tests de parité synthétique `native_plan_preview::tests::pure_`. Ils utilisent des valeurs scalaires, des tableaux JSON et des fichiers temporaires ; les anciens tests construisant une scène ne sont pas sélectionnés. Le contrôle de production ne compile pas les hooks expérimentaux. Le target release est uniquement compilé avec `--no-run`.

La limite locale de 78 736 pas, l’ordre A puis B, la fermeture après B et les 7 100 secondes sont imposés par le manifeste/registre/lanceur de la paire. Le worker conserve en défense la limite familiale de 300 000 et ne reçoit jamais de crédit sans permis parent. La supervision des PID, descendants, groupes et signaux est couverte par la revue séparée `process-supervision-review.md`. Aucune qualification physique supplémentaire ne découle de ces tests ni de la compilation ; les seize pas de vérification restent nécessaires dans le futur processus autorisé.
