# Exploration A36 — bloquée avant lancement

Décision enregistrée le 2026-10-15T16:08:13.196985+00:00. Préparation commencée à 16:06:17 UTC ; plafond actif : 30 minutes.

Autorisation parent : une seule histoire exploratoire depuis A36, au plus 7 000 sous-pas tous usages, 120 secondes de calcul et 26 secondes simulées, sans renouvellement. Adaptation du pilote existant seulement ; aucun contrôle de production modifié. Arrêt explicite demandé si le contrôleur ne peut fonctionner sans prévision.

**Aucune simulation lancée ; zéro sous-pas et zéro seconde de calcul physique supplémentaires. Aucun budget physique réservé ou débité pour cette demande. Aucune vidéo nouvelle ne peut être produite.**

## Blocage concret

- `transfer_adapter.rs::compile_batch` peut préparer quatre commandes candidates sans intégration. Mais `whole_body_controller.rs::configure_transfer` appelle `require_native_command` avant `install_targets` ; `transfer_adapter.rs::require_native_action`, dans `Authorization::None`, refuse : « No budgeted native transfer window; static feasibility is not admission ». Une proposition disponible n'est donc pas une commande applicable par le pilote sans reçu de prévision.
- La continuation vers debout n'est pas un calendrier indépendant prêt à jouer : `Adapter::nominal` construit `PreparedRaccord` seulement après `policy.continuation_event()`, avec un `LocalReadyReceipt` issu de la stabilité mesurée. Sinon il poursuit le pont local puis son maintien. Ce succès local n'a pas été obtenu dans le diagnostic clos.
- Enregistrer la perte de points d'appui sans la bloquer demanderait aussi de changer `Adapter::observe`, qui refuse actuellement le support retenu non prouvé. Le pilote ne possède aucun commutateur exploratoire pour ces décisions.

Contourner ces trois conditions modifierait le comportement du contrôleur et demanderait une nouvelle voie d'exécution ; cela dépasse l'adaptation du pilote autorisée. Aucun reçu fictif, aucune admission, aucun seuil modifié.

## État conservé

Inspection de processus : aucun worker natif ni lanceur d'exploration actif ; ce répertoire a été créé exclusivement, sans réutiliser une exécution. Dernier code publié avant cette décision : `7074a5a65ee90c9d010db1283a724b3c85ed6690`, branche `investigate/physical-support`.

Correction d'attribution interrompue et preuves du diagnostic précédent laissées intactes dans le worktree, hors de ce commit. Aucun calcul, test, audit supplémentaire, sous-agent ou recherche de paramètres lancé.

Dernier résultat physique conservé : diagnostic clos à 78, 130 sous-pas au total ; point gauche toujours sélectionné, impulsion publiée nulle. Cela ne démontre pas la cause dynamique profonde ni une impossibilité de se relever.

Débits globaux physiques inchangés : 139 508 / 300 000 sous-pas et 226,328294294003192 / 7 200 secondes provisionnées. Préparation réalisée dans la fenêtre déjà engagée, sans nouvelle réservation. Récupération publique désactivée ; `main` inchangée.

**Décision : trajectoire complète non exécutée et non évaluée. Étape arrêtée sur le blocage prévu par l'autorisation parent.**
