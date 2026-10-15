# Continuation unique A36 : plafond corrigé à 7 100 secondes

Le parent autorise explicitement ce nouveau gel et son lancement unique sans
nouvel aller-retour. La [provenance de l'accord](parent-authorization-provenance.json)
est conservée. L'ancien manifeste à 7 200 secondes et ses sources restent
intacts ; aucune famille close n'est rouverte.

Le [protocole complet](protocol.json) reprend exactement le candidat physique
préparé dans le [gel précédent](../coordination-transfer-native-proposal-2026-10-13/future-native-protocol.md).
Seuls le plafond du superviseur, l'identifiant du protocole et les pièces de
gel/autorisation changent. Le binaire physique conserve le SHA-256
`63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5`.

L'enveloppe comprend **131 096 sous-pas maximum**, toutes copies et préfixes
inclus : 52 techniques, 6 244 réels et 260 prévisions de 480 au maximum.
Le plafond mural est **7 100 secondes de bout en bout** : démarrage au plus
30 secondes, permis natifs expirant à 7 093 secondes, sept secondes réservées
à l'arrêt, sans renouvellement ni remboursement.

Les anciens permis totalisent 7 844 pas débités ; le cumul maximal devient
138 940/300 000. Le temps historique prudent de 95,951356994 secondes plus
7 100 donne 7 195,951356994/7 200 secondes. La marge familiale est
4,048643006 secondes. Le [rapprochement indépendant](budget-admission-review.md)
et le contrôle d'absence de doublon doivent être relus avant le lancement.

Une copie de parité refusée ou une prévision inutilisable ferme tout l'essai.
Aucune correction de code, répétition, variante ou prolongation n'est permise
dans cette autorisation. Le jalon talons/recentrage doit être complet au pas
516, puis la continuation doit atteindre le succès original avant le pas
6 000 et conserver une seconde réelle supplémentaire jusqu'à la fin du tick.
Les seuils, moteurs, anatomie, masses, frottements et collisions sont inchangés.

Le préfixe part de la scène analytique historique, puis retrouve l'état observé
A36 par intégration et comparaison. Même un succès complet ne qualifierait
pas le relevage après impact ni les 24 poses/cycles/impacts. La récupération
publique reste désactivée. Les vidéos et mesures seront tirées uniquement
des états effectivement enregistrés, en distinguant technique, réel et
prévision. L'absence de mouvement après refus sera rapportée sans animation
de la trajectoire souhaitée.

Le claim du nouveau dossier est exclusif. Un claim ou une sortie du même
candidat dans l'ancien dossier est aussi une cause de non-lancement ; root
effectue cette vérification transversale immédiatement avant l'unique appel.
