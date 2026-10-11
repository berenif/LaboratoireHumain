# Audit causal de l’actuation native — 2026-10-11

Base `c597959f46f37715035349c8cd0a699b6822111a`, branche locale `investigate/physical-support`. Aucun push/merge. Aucun nouveau parcours long. Récupération désactivée et non qualifiée.

## Expérience discriminante

Les 66 premiers sous-pas sont reproduits dans le pipeline réel à partir des commandes archivées, sans réécriture des corps, des contacts ou des vitesses. Pour chaque branche, les poses et vitesses de tous les segments correspondent exactement au préfixe archivé. Seule la commande du dernier pas de 4,167 ms est perturbée. Le témoin répété donne exactement la même trace. Les résultats sont dans `evidence/native-actuation-audit-2026-10-11/summary.json`.

La vitesse initiale de l’avant-pied droit est 9,61053 rad/s. Le QP prévoit une vitesse finale de 9,16264 rad/s.

| Intervention au dernier pas | Vitesse native finale (rad/s) |
|---|---:|
| Commande originale | 10,48809 |
| Moteur de l’avant-pied seul coupé | 9,16436 |
| Consigne diminuée de 0,02 rad/s | 10,46709 |
| Consigne augmentée de 0,02 rad/s | 10,50566 |
| Consigne fortement freinante, cap inchangé 7,606 Nm | −15,14357 |
| 64 passes, diagnostic uniquement | 10,47965 |
| Couple demandé appliqué directement à ce joint | 9,10282 |
| Tous les couples demandés appliqués directement | 8,37473 |
| Consigne calculée par identification locale, delta −1,3737 rad/s | 9,11904 |

Les couples directs sont appliqués comme paires internes opposées entre parent et enfant, sans effort externe au bassin. Les moteurs concernés sont coupés ; les plafonds anatomiques et le budget total, incluant les moteurs restants, sont contrôlés explicitement. Ces branches restent des expériences, pas un contrôleur livré.

Le moteur dispose d’une autorité réelle : la commande fortement freinante atteint son cap installé et inverse la vitesse en un pas. Ce résultat écarte l’interprétation du refus terminal comme une impossibilité mécanique générale de freiner. Le doublement diagnostique des passes a un effet faible ; il ne justifie pas de changer le budget du moteur physique. La suppression du seul moteur retrouve presque la prédiction QP, mais le remplacement des commandes par des couples directs ne rend pas tout le modèle exact.

## Correction de l’interprétation des impulsions

Le champ historique `signedNm` vaut l’impulsion **de ligne de contrainte** divisée par dt. Le solveur applique `+J1*lambda` au parent et `−J2*lambda` à l’enfant. Il ne s’agit donc pas directement du couple généralisé sur l’enfant.

Au dernier pas, l’impulsion vectorielle de l’enfant est environ `[-0,00172456, -0,00000613, -0,00003175] N·m·s`. L’axe articulaire pointe approximativement vers −X. Sa projection correspond à un couple **positif**, accélérant l’avant-pied, alors que le nombre brut de ligne vaut −0,414 Nm. L’ancien texte qualifiant ce nombre brut de freinage était incorrect. Le repère de vitesse et le signe de la consigne sont cohérents ; l’erreur de diagnostic portait sur l’interprétation de la livraison, et l’erreur de commande sur la conversion accélération/couple demandé en cible du moteur implicite couplé.

La formule découplée `damping*(target−vitesse avant le pas)` ne représente pas l’impulsion réellement résolue pendant les sous-pas, les contacts et les contraintes. Le cas montre que cette approximation de conversion ne suffit pas à produire l’actuation attendue.

## Prédicteur natif et correction testée

Un état Rapier complet est sérialisé juste avant l’intégration native, après préparation des commandes. Il inclut les corps, articulations, contacts, îlots et données de démarrage. Le pipeline de calcul transitoire est recréé. Le clone fait un pas avec les mêmes hooks de collision.

Pour le témoin, la correction identifiée et les branches de couples directs, les **25 positions, rotations, vitesses linéaires et angulaires prédites par le clone correspondent exactement au pas réel**. Le clonage ne touche jamais à l’état réel. La représentation de recherche sérialisée fait 1 074 939 octets sur le témoin ; aucune performance temps réel n’est revendiquée.

Les perturbations ±0,02 rad/s identifient une pente locale de 0,96434 entre consigne et vitesse finale. Les deux points indépendants ±0,01 ont des erreurs de prédiction de −0,000548 et −0,000916 rad/s. Une correction calculée à partir de cette pente rétablit un freinage réel : 9,61053 → 9,11904 rad/s, caps et 32 passes inchangés. Le clone prédit ce résultat exactement. L’erreur par rapport à la cible QP reste −0,04359 rad/s : la linéarisation ne doit pas être extrapolée sans validation native.

Une bissection diagnostique de 18 évaluations visait une erreur de vitesse ≤1e−4 rad/s. Elle est **refusée** : meilleur résidu 1,3256e−4, avec variations locales non monotones jusqu’à environ 0,0026 rad/s. Le critère n’est pas assoupli. Les deux essais rejetés et l’historique détaillé sont archivés. Ce critère numérique ajouté au banc ne constitue pas un nouveau seuil de qualification physique.

## Décision d’architecture et relais Sol

Retenir le simulateur natif complet comme prédicteur discret des commandes moteur dans cette phase de récupération. Le modèle analytique peut proposer une commande ou initialiser une recherche ; son certificat QP ne certifie pas l’actuation native ni la viabilité du mouvement.

L’implémentation complète à confier à Sol est détaillée dans `evidence/native-actuation-audit-2026-10-11/HANDOFF-SOL.md` : capture de l’état préparé, identification locale des sorties couplées, correction bornée, validation native de chaque candidat, comparaison prédiction/réalisation, et changement de modèle local quand les contacts changent. Le freinage, les marges de butée et les réactions réelles doivent faire partie des sorties. Une égalité de vitesse obtenue par une bissection supposant une réponse monotone n’est pas une architecture générale suffisante.

Le relevage reste à réaliser et à qualifier. L’appui main/genou n’est pas démontré nécessaire. Les expériences et hooks restent séparés du code de production ; seuls des diagnostics de livraison explicites peuvent être conservés. Aucun résultat de suite complète n’est annoncé.

## Vérification après retrait du banc expérimental

Les tests ciblés `lh-sim recovery::` passent : 24 réussis, 2 ignorés, 79 filtrés (36,97 s). Clippy `lh-sim --all-targets -D warnings`, `cargo fmt --check` et `git diff --check` passent. `analyze.py` vérifie les préfixes, la répétition exacte et les clones. Ces vérifications ne remplacent ni la suite complète ni une qualification physique ; les échecs hybrides/TypeScript signalés sur le témoin ne sont pas déclarés résolus.
