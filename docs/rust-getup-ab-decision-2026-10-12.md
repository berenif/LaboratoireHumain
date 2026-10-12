# Comparaison get_up_control A/B — décision du 12 octobre 2026

Le propriétaire autorise le jalon limité le 12 octobre à 05:48 UTC, après la
revue architecturale de `42fccd3`. Base locale et distante vérifiée :
`42fccd359e88bb0677c40b39310e24313e8cd8d1` sur `investigate/physical-support`.
Checkout propre, aucun changement concurrent constaté. `main` reste à `c597959`.

## Révision explicite de la dépendance P1

L'audit exhaustif des opérations Rapier, du bilan angulaire et de la borne
indépendante d'arrondi est **différé**. P1 reste non certifiée et son registre
reste fermé. Cette décision remplace seulement l'exigence documentaire
« aucune recherche avant certification P1 » pour la comparaison A/B autorisée.
Elle n'autorise aucune campagne d'optimisation ni aucune tolérance nouvelle.

Le défaut de lecture CCD est expliqué ; l'observation corrigée est neutre.
Un résidu angulaire incomplet reste visible. Le retour d'état proposé est une
approximation cinématique qui produit des commandes articulaires bornées ;
il ne revendique aucun certificat de dynamique inverse ou de conservation.

Les exigences physiques restent intégralement celles du contrat original :
anatomie, masses, contacts, gains, caps anatomiques/enveloppe 760 Nm, intégrité,
aucune assistance au bassin, écriture de pose ou reset pendant l'histoire.
Le compteur original, les appuis bilatéraux persistants, le settle de 0,30 s,
le handoff de 0,60 s et le maintien de 1 s restent indépendants du score.
Succès avant 25 s, puis 1 s d'observation supplémentaire dans le même monde.
La récupération reste désactivée par défaut et non qualifiée.

## Expérience figée à préparer

Programme complet local issu de [get_up_control](https://github.com/tianxintao/get_up_control),
déjà adapté et archivé : `reference-ready.json`, SHA-256
`fef70abf3d016cecf6658b0bc5e7a6361d989eb265840acdc1a4fbc497a70c7d`.
Ses clés nominales, sa durée de 20,883333 s et son calendrier ne changent pas.
Témoin A et candidat B utilisent le même exécuteur/terminal ; seul le retour
d'état coordonné borné est désactivé ou activé. Un retour cinématique ne promet
pas la charge effectivement livrée : celle-ci est mesurée dans Rapier.

Préparation plafonnée à deux journées de travail, comptées conservativement
comme 16 heures actives, avec une échéance de sauvegarde à 48 heures calendaires.
Sans exécuteur et terminal prêts, arrêt sans essai. Chaque intégration, y compris
test ou copie, doit passer par le registre. Les vérifications préalables sont
donc statiques, compilation ou tests explicitement sans intégration.

Plafond expérimental : **3 histoires × 26 s, 18 720 pas natifs, un processus
mono-thread, 600 s murales depuis le premier lancement**. Témoin puis candidat ;
répétition figée seulement après succès complet. Aucun réglage entre histoires,
extension automatique ou renouvellement d'un ancien budget. Sans succès complet,
clôture même si un appui ou un délai de collision s'améliore.

Le manifeste figera binaire, programme, paramètres, installateur, terminal et
coordonateur avant le premier lancement. Les anciens registres sont conservés
par SHA. Les preuves porteront sur les contacts anatomiques persistants et leur
horodatage, le suivi articulaire, l'accélération CoM, les couples/caps, les
horloges et le verdict terminal. Vidéo depuis états archivés, légendée selon le
résultat, sans nouvelle simulation de rendu.
