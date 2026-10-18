# Dix pas alternés de 18 cm : état provisoire

Essais du 18 octobre 2026, dans le prototype natif optionnel. **Aucune séquence de dix pas suivie de cinq secondes calmes n'est qualifiée.** L'investigation et le diagnostic causal de réponse native restent ouverts.

## Essais arrêtés, preuves conservées

- **Pilote 1, durées raccourcies** : un pas terminé, arrêt pendant la levée du deuxième à **11,720 s / 9 376 intégrations**. Capture −8,150 µm ; glissement chargé gauche/droite 4,588 / 4,708 mm.
- **Pilote 2, suivi d'appui 6D renforcé** : gains position/rotation 200/30, poids 60. Arrêt sur `right_hip_y` pendant la première levée à **4,88875 s / 3 911 intégrations** ; marge articulaire −0,000050133 rad. Aucun pas terminé.
- **Candidat 3, contraintes COM/capture mesurées** : trois pas terminés de **181,968 / 182,012 / 181,509 mm**. Arrêt au transfert du quatrième à **24,93125 s / 19 945 intégrations** : glissement **10,000211 / 9,376005 mm**, dépassant la borne globale inchangée de 10 mm par pied.
- **Candidat 5, relais direct après réception chargée** : un pas acquis, puis refus de sortir de la table FK vérifiée des bras au transfert suivant, à **8,650 s / 6 920 intégrations**. Aucun écrêtage ou élargissement de la table.
- **Candidat 6, relais direct et garde de plage des bras** : deux réceptions de **181,939 / 181,995 mm**, à 7,13875 et 13,28125 s. Arrêt au transfert du troisième à **16,3875 s / 13 110 intégrations** : dépassement de `right_hip_x` de **0,0000887095 rad**. Glissement 8,070 / 7,796 mm. L'audit strict rejette l'essai.

Sous-total vérifié : **53 262 intégrations** pour ces cinq essais seulement. Il exclut les diagnostics sur checkpoints et les autres expériences, notamment le diagnostic causal actif.

## Distance et vitesse physiques

« 180 mm » désigne l'avance du pied mobile depuis sa propre position réelle au début du pas, selon le cap courant ; la cible est 182 mm pour obtenir au moins 180 mm mesurés. Cela ne désigne ni la séparation devant l'autre pied, ni l'avance du COM. À l'arrêt du candidat 6, le milieu des pieds a avancé de **181,399 mm**, le COM de **118,113 mm**. Dix déplacements alternés ainsi définis ne signifient pas automatiquement 1,8 m parcouru.

Les références sont raccourcies en **secondes physiques**, sans accélérer la vidéo : levée 0,8 s, translation 1,2 s, descente 1 s, contre 2 s chacune au témoin. Le premier transfert reste à 4 s. Les essais segmentés utilisent ensuite 2 s de transfert, 1,8 s de stabilisation et au moins 0,25 s stables. Le relais direct utilise 3 s de transfert et enchaîne après 100 ms de réception réellement chargée ; stabilisation et repos sont réservés au dernier pas. Cette coordination ne qualifie pas à elle seule une marche complète plus rapide.

## Intégrité et vérification

MuJoCo 2.1.3 reste à **800 Hz**, le QP de couple à **200 Hz**. Modèle sélectionné, preset, réglages par défaut, moteurs à 95 %, frottements et seuils d'appui, collision, articulation, capture et glissement sont conservés. Aucun effort de racine, pose installée ou reset physique entre les pas. Les reçus indiquent un code inchangé pendant chaque essai, avec empreintes distinctes entre variantes.

Levée et réception exigent toujours 80 échantillons qualifiés ; la fin exige **4 000 échantillons calmes**. Le dépassement de cheville hérité jusqu'à 63,75 ms reste distinct des nouveaux dépassements. L'audit reconstruit indépendamment le hull COM/capture ; une marge négative ou indéfinie échoue. Les contraintes QP sur les faces courantes figées ne constituent pas un certificat de viabilité dynamique. La piste d'amortissement tangentiel a été abandonnée sans gain de séquence validé.

- **30 tests d'audit réussis**, dont relais direct et préservation du mode historique : 200 échantillons stables acceptés, 199 refusés.
- **10 tests contrôleur réussis**, dont parité exacte des couples par défaut avec `0ffb1bb`.
- Preuve antérieure : **63 cas, 61 réussis, 2 ignorés** faute d'anciens assets natifs. Les journaux agrégés distincts de 72 et 73 cas sont conservés séparément. Aucun succès de CI complète n'est revendiqué.

Le manifeste provisoire conserve contrôles exacts, compteurs, réceptions et SHA-256. Il signale les audits plus anciens. Le diagnostic causal actif et son futur résultat restent séparés de ces essais arrêtés.


Les preuves compactes de ce point d'étape sont dans
[le manifeste](../evidence/getup-source-cadenced-2026-10-18/manifest.json).
Les journaux bruts, bibliothèques, XML et caches restent locaux, hors commit.
Le point d'étape ne promeut aucun réglage et ne clôt pas la demande de dix pas.
