# Pénétration : mesure conservée, refus par profondeur supprimé

Le propriétaire a demandé de supprimer la limite de pénétration. Le contrat actif v2 retire les deux refus fondés uniquement sur une profondeur finie : 5 mm entre corps non exclus et 80 mm contre l'environnement. Aucun seuil de remplacement n'est introduit. Le contrat v1 et les anciens résultats restent historiques ; leur suppression de refus ne qualifie rétroactivement aucune trajectoire.

Les requêtes géométriques exactes continuent de mesurer les maxima cumulés. L'interface affiche ces deux maxima en millimètres sous « Measured support ». Les collisions, leur réponse dynamique, les exclusions existantes, masses, géométrie de production, friction, moteurs et couples ne sont pas modifiés. Une mesure non finie ou impossible reste une erreur. Les contrôles d'état fini, de propriété des corps/colliders, d'ancrages et de limites articulaires restent actifs. Les durées de récupération et de stabilité restent respectivement 25 s et 1 s. La récupération reste désactivée par défaut jusqu'à qualification.

TODO terminé : retirer les refus ; conserver les mesures et les autres contrôles ; afficher les diagnostics ; aligner le contrat compilé et l'identité du contrat utilisée par l'outillage navigateur ; vérifier sans intégration ; publier les preuves. Le test de conversion des durées utilise désormais l'horloge native actuelle de 800 Hz, au lieu d'attendre les anciens nombres de sous-pas à 240 Hz. Aucun changement d'horloge n'est effectué ici. Les outils de qualification historiques à profil v24/240 Hz restent distincts de la qualification du corps GetUp actuel.

## Vérifications

- 14 tests Rust ciblés passent : 5 mesures/intégrité, 1 contrat, 8 GetUp. Tous construisent ou inspectent des états sans intégration physique.
- Les 57 tests JavaScript d'outillage passent ; ce sont des tests statiques ou avec transport/rapports simulés.
- Le build navigateur/worker WASM et les vérifications syntaxiques JavaScript passent.
- Le test navigateur `node scripts/verify-rust-penetration-ui.mjs` passe avec le mode manuel installé avant la première animation. Un événement d'interface synthétique affiche « body 47.00 mm · ground 120.00 mm » ; le vrai worker conserve ses profondeurs nulles, ses positions, tick 0 et temps 0. Ce test vérifie le formatage, pas une trajectoire physique.
- Les fixtures de chevauchement profond utilisent des colliders synthétiques uniquement dans les tests. L'inspection conserve intégralement le monde sérialisé et les contacts propres du multibody restent activés.

Les journaux et leurs empreintes sont conservés dans [le dossier de preuve](penetration-diagnostic-policy-2026-10-16/validation.json). La suite globale Rust n'a pas été exécutée : elle contient des essais intégrant la physique.

## Limite de preuve et prochain essai

Cette modification consomme **0 sous-pas natif**. Le registre clos `getup-source-baseline-2026-10-16.json` reste inchangé : 299 998 / 300 000 sous-pas chargés, soit **2 restants**. Aucun nouveau calcul physique n'est admis ni lancé.

Il reste à vérifier si WEAK atteint son terminal dans Rapier sans l'ancien refus, si une référence exploitable commande effectivement SLOW et comment évoluent les profondeurs et la stabilité. La réussite de la source MuJoCo ne répond pas à ces questions.

Le prochain essai minimal proposé, soumis à une nouvelle admission via le parent, est **une seule histoire fixée de 4 s**, à 800 Hz : au maximum 3 200 sous-pas réels + 5 000 prévisionnels (250 décisions WEAK × 20), soit **8 200 sous-pas natifs et 180 s murales**. Avant lancement : figer profil, état initial/graine et impact ; journaliser compteurs, empreintes et échéance ; conserver les gardes d'intégrité ; arrêter si la référence échoue, si les diagnostics/counters manquent ou si un plafond est atteint ; aucune variante ni relance. Ce court essai doit établir la chaîne WEAK → référence → premières commandes SLOW. Il ne peut constituer une preuve de relevage complet ni remplacer l'histoire complète puis la qualification des 24 cas.
