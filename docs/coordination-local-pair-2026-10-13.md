# Dernière paire locale A/B de la famille coordination

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

Le [contrat figé](evidence/coordination-local-pair-2026-10-13/launch-contract.json),
l’[audit cumulatif](evidence/coordination-local-pair-2026-10-13/cumulative-budget-audit.md)
et le [préflight de continuation](evidence/coordination-local-pair-2026-10-13/continuation-preflight.md)
précisent les consommations, limites et critères de décision.

## TODO et preuves

- [x] Revue cumulative des quatre anciens registres, sans réouverture.
- [x] Critères physiques inchangés et continuation géométrique archivée relue.
- [x] Registre strict de la dernière paire et comptage incomplet explicite.
- [x] Supervision avec arrêt de groupe, signaux et limite murale externe.
- [x] Worker dédié reconstruit, tests purs et contrôles de production terminés.
- [x] Neuf prérequis appuyés par les preuves actuelles ; manifeste et binaire gelés.
- [ ] Gel publié et SHA distant vérifié, puis unique lancement du registre distinct.
- [ ] Registre clos ; décision causale A/B, premier refus et consommations archivés.
- [ ] Mesures et vidéo issues exclusivement des états enregistrés.
- [ ] Rapport Library, validation et dernier push vérifié.

Les modifications en préparation portent sur l’admission, la mémoire comparée,
le comptage durable et l’arrêt des processus. Elles ne changent pas les actions
physiques. Aucun worker natif A/B n’a été lancé au stade de cette note initiale.

Préflight statique réussi sans worker. Manifeste `2fbcb3db0cd1bccf14082e772b1efba63e567fe7fce35ed8cccbbbf2ac2cdde9`.
Binaire dédié `c1c8a12825428758643a7f6e2fafb0a2710f7dca787c1daee07c27ec59849e9c`.
Les 14 tests Rust purs, le contrôle de production, la compilation release sans lancement,
89 tests de registre, 43 du lanceur, 47 du superviseur réutilisé, sept du builder
et six essais OS inertes passent. La suite globale Rust/TypeScript n’est pas relancée.
