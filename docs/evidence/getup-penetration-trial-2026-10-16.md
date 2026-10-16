# Essai Rapier interrompu avant impact — 16 octobre 2026

**L'objectif causal n'a pas été évalué.** L'unique lancement autorisé s'est arrêté après 0,025 s physiques sur une erreur de compteur du lanceur, avant l'impact prévu à 2 s. Aucun échec numérique de Rapier n'a été observé. Aucun second lancement n'a eu lieu.

## Autorisation et budget

Le propriétaire a répondu « Go » à 19:09:59 UTC à la proposition d'un essai supplémentaire limité à trois minutes de calcul. Le [registre incrémental](getup-penetration-trial-2026-10-16.json) a été préparé avant lancement : une histoire réelle de 4 s maximum, 3 200 sous-pas réels + 5 000 prévisionnels maximum, plafond total 8 200, plafond mural 180 s. Les 117 entrées ont été figées sur la physique de `c728c428c7d774934f83c711d05b5bd0590e9b20`. Les tests d'admission et de compteurs ont passé avant lancement ; leurs fixtures n'ont pas détecté la confusion de compteurs décrite ci-dessous.

Le lancement a commencé à 19:14:42 UTC. Consommation réelle vérifiée dans les archives : **20 sous-pas réels + 0 prévisionnel = 20**, et **1,625063235 s murale** pour le lancement, sa fermeture comprise. La réservation supplémentaire de 8 200 sous-pas et 180 s reste retenue intégralement, sans remboursement ni réutilisation automatique. Le registre historique clos reste identique par empreinte, avec ses 299 998 sous-pas réservés ; ses deux sous-pas restants n'ont pas été utilisés.

## Cause et correction

Le lanceur initial déduisait le nombre physique de `snapshot.stamp.sequence - 1`. Or `Runtime::advance_traced` remplace ce champ par le numéro du message d'observation. Après le premier tick, le paquet était donc `sequence=2`, `tick=1`, `simulation_time_s=0.025`. Le lanceur a attendu à tort un seul sous-pas (0,00125 s), puis a fermé le navigateur sur cette assertion. Le monde était alors intact et en mode manuel ; aucune intégration supplémentaire n'était demandée.

Le nombre réel est établi par `Simulation::snapshot`, qui émet exactement `substeps_completed / 800.0` dans `simulation_time_s`. Les deux états archivés sont à 0 et 0,025 s, soit 20 intégrations, avec prévisions et décisions SLOW à zéro dans le rapport du contrôleur. Le lanceur corrigé utilise cette horloge native, vérifie la cadence et l'intégralité du compte, et ignore la séquence de transport. Un test reproduit le paquet fautif et conserve le bon résultat même si sa séquence devient 999. Cette correction est **postérieure à l'essai et n'a pas été relancée physiquement**. Les entrées originales du lanceur sont conservées dans [launch-inputs](getup-penetration-trial-2026-10-16/launch-inputs/validate-getup-bounded.mjs).

## Résultat physique et portée

- Impact : non atteint ; compteurs d'impact, chute et récupération à zéro.
- Prévision WEAK : jamais déclenchée ; admission d'une référence non évaluée.
- SLOW : zéro décision et aucune commande SLOW appliquée. Seule la commande posturale initiale a été intégrée.
- État final : `Upright` comme à l'initialisation, tête à 1,4627361 m, déplacement maximal des origines de corps de 3,16 mm. Cela ne représente aucun relevage ni preuve de stabilité après impact.
- Pénétrations mesurées : zéro dans ces deux états. La règle diagnostic-only n'a donc pas été mise à l'épreuve par cet essai.

La physique, les collisions, la réponse dynamique et les gardes d'intégrité sont restées celles de c728c42. Le bassin n'a reçu aucune force ou couple d'assistance. Aucun reset, changement de pose, ajustement physique, renouvellement ou seconde histoire n'a été effectué. La récupération publique reste désactivée et aucune qualification n'est revendiquée.

## Preuves et validation

[Résultat brut](getup-penetration-trial-2026-10-16/result.json), [états](getup-penetration-trial-2026-10-16/trace.ndjson), [rapport contrôleur](getup-penetration-trial-2026-10-16/reference-report.json) et [audit](getup-penetration-trial-2026-10-16/postrun-audit.json) sont conservés avec leurs empreintes. Après correction, **60 tests JavaScript et 3 tests Python d'admission passent**, sans physique. Le registre clos refuse toute nouvelle admission. Les sources Rust/WASM de production n'ont pas été modifiées ; aucune suite physique supplémentaire n'a été lancée.

La vidéo `LaboratoireHumain-Rapier-essai-interrompu-2026-10-16.mp4` montre exclusivement les deux états enregistrés, maintenus chacun une seconde pour lecture. Les légendes distinguent explicitement ces 2 s de présentation des 0,025 s physiques. Validation : H.264, 1280 × 720, 40 images à 20 fps, décodage complet et dernière image inspectée. [Validation et identité Library confirmée](getup-penetration-trial-2026-10-16/video-validation.json), version 0. Rendu, audit et sauvegarde n'ajoutent aucune intégration.

Le défaut de lanceur est corrigé, mais les questions WEAK → référence → SLOW et le mouvement après impact restent ouvertes. **Toute nouvelle histoire demanderait une nouvelle autorisation ; aucune n'est lancée avec le solde inutilisé.**
