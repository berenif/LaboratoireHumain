# Essai exploratoire A36 : chute, aucun relevage

Une histoire réelle a exécuté **26 s / 6 240 sous-pas**, préfixe A36 compris, en **30,709 s murales**, sans prévision ni relance. [Vidéo complète](rendered/main-history.mp4) · [mesures](rendered/measurements.csv) · [graphique](rendered/measurements.png).

Le chemin expérimental applique la séquence articulaire existante (référence debout à 7 s, fin à 8 s), puis son maintien jusqu’à 26 s. Il suit directement cette référence avec vitesses cohérentes et l’installateur ForceBased existant ; il n’utilise pas le retour CoM du solveur de tâches. Les changements d’appui restent libres et les pertes des points initiaux sont des diagnostics. Ce chemin exige le binaire de test et une activation explicite à A36 ; le comportement de production reste inchangé.

- A36 : bassin à **0,9477 m** au-dessus du sol.
- **0,179 s** : point de capture hors des appuis ; **0,329 s** : CoM hors des appuis. Ce sont les premiers indicateurs d’échec, pas une preuve de sa cause dynamique profonde.
- **1,525 s** : perte diagnostique des deux points matériels retenus, sans blocage.
- **1,588 s** : premier contact non-pied chargé, la tête. La chute précède la phase d’extension commandée.
- Bassin minimum **0,1049 m**, final **0,1125 m**. Charges diagnostiques des talons nulles pendant tout l’essai ; aucune observation debout admise.
- Commandes de la séquence terminées ; corps resté au sol. Aucun arrêt d’intégrité physique du noyau. Somme maximale des caps moteurs observée : **760 Nm**, ratio anatomique maximal : **1,0**. Masses, collisions, friction, anatomie et noyau physique inchangés ; aucune aide au bassin, aucun reset.

**Décision : cette séquence de références échoue au relevage dans cet essai.** Aucun succès depuis impact ni qualification des 24 cas ; aucune impossibilité anatomique démontrée. Récupération publique désactivée. Aucun nouvel essai autorisé automatiquement.

Vérification ciblée : compilation du worker, **1 test pur réussi** couvrant toutes les commandes de 36 à 6239, limites et vitesses ; syntaxe des deux scripts vérifiée. Build isolé depuis `e5b59e5` avec les deux fichiers expérimentaux ; correction d’attribution inachevée exclue et conservée dans le worktree d’origine. Code exécuté : `26b4f2fa2de8b6c5f5d411b0c145eb491f0f20e2`.

Réserve complète conservée : **7 000 sous-pas / 120 s**, sans remboursement. Débits globaux : **146 508 / 300 000 sous-pas**, **346,328 / 7 200 s**. Préparation engagée : **69 889 / 144 000 s**, dont 1 800 s pour cette étape. L’essai antérieurement bloqué n’avait rien consommé ; aucun ancien registre réinitialisé. [Compteurs clos](closed.json) · [résultats détaillés](result-metrics.json) · [validation ciblée](validation.json).

La vidéo et les figures utilisent seulement les poses archivées ; aucun calcul physique ni interpolation de poses ajouté au rendu. Les charges sont celles publiées par le solveur, pas des forces continues garanties sur tout l’intervalle CCD.
