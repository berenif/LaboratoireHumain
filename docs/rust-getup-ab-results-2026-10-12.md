# Comparaison get_up_control A/B — jalon fermé sans relevage

**Deux histoires natives, zéro récupération. Aucun troisième essai.** Le retour
coordonné change réellement la trajectoire et permet à B d'atteindre la fin du
calendrier nominal ; le mannequin reste couché. Le vérificateur physique original
refuse la demande debout. Le candidat n'est pas promu et aucun réglage n'est repris.

Base `42fccd3`, décision documentaire locale `b6813cd`, exécuteur figé au commit
`891b4a4acf74afd3d9c5c7d18b826296b4cc3e1d`. Programme get_up_control inchangé,
SHA `fef70abf3d016cecf6658b0bc5e7a6361d989eb265840acdc1a4fbc497a70c7d`.
Binaire SHA `5a4ce138250f93220f3371e2c8a7582e888480723d26c5ef579c26287991a032`.
305 fichiers d'entrée figés sont vérifiés après les deux histoires. Aucune
retouche entre A et B, aucun reset, assistance ou simulation prospective.

## Résultats effectivement observés

| Observation | A sans correction | B avec correction |
|---|---:|---:|
| Pas natifs intégrés | 1 076 | 5 013 |
| Durée physique | 4,483333 s | 20,887500 s |
| Arrêt | Stagnation du progrès mesuré ≥3 s | Refus du vérificateur debout original |
| Bassin au dernier état | 0,145064 m | 0,108130 m |
| Verticalité finale du torse | −0,246325 | 0,026082 |
| Progrès sur le chemin articulaire | 0,186830 | 0,990343 |
| Erreur nominale articulaire maximale au dernier pas | 0,899324 rad | 0,246404 rad |
| Accélération CoM maximale, histoire entière | 117,862595 m/s² | 117,180801 m/s² |
| Somme absolue de couples délivrés maximale | 697,622803 Nm | 682,095581 Nm |
| Enveloppe de caps installés maximale | 760 Nm | 760 Nm |
| Ratio maximal au cap anatomique | 1 | 1 |
| Vitesse de consigne maximale | 0,959046 rad/s | 1 rad/s |
| Slew de position mesuré maximal | 0,959129 rad/s | 0,999999 rad/s |
| Point persistant postérieur du hindfoot | Aucun | Aucun |
| Handoff / récupération / maintien acquis | Aucun | Aucun |

Les maxima portent sur des durées différentes ; ils ne classent pas deux succès.
Le champ de refus `ownership` est l'enveloppe historique des erreurs de commande ;
il ne signifie pas perte de handles. Assemblage et génération sont conservés.

Les 12 premiers pas ont les mêmes hashes complets corps/moteurs/contacts.
Première divergence à **54,1667 ms**, lorsque les appuis persistants deviennent
disponibles au retour d'état. À 75 ms, l'accélération CoM avant passe de
−0,170733 à −0,260518 m/s². À 150 ms les captures sont intérieures transitoirement,
mais à 550 ms elles sont extérieures dans les deux histoires, sans talon arrière.
À 4,479 s, B réduit l'erreur nominale maximale de 0,899708 à 0,675165 rad ;
les bassins restent tous deux autour de 0,146 m. Cela ne représente pas une montée.

Après la demande Stand à la fin nominale, B a un bassin à 0,108130 m et une
verticalité du torse de 0,026082. Le refus original de la pose debout s'applique
avant tout settle/handoff. Ni compteur ni état Upright n'ont été forcés.
À cet instant, tête, mains, torse, lombaires et avant-pieds fournissent les
contacts persistants ; suivre le chemin articulaire ne remet pas la racine debout.

Le paramètre d'offset est 0,12 rad en `f32`; la différence entre commandes stockées
et nominal atteint 0,1200000048 rad par quantification. Cette observation reste
explicite ; elle n'introduit aucune tolérance de qualification. Les véritables
bornes anatomiques et de vitesse sont vérifiées par le validateur original.

## Conclusion causale et limites

Le retour d'état a une influence mesurée ; il ne suffit pas au relevage dans
cette configuration figée. La correction cinématique relative ne traite pas
la rotation libre de la racine. Elle maintient les surfaces réellement présentes,
y compris les contacts passifs tête/tronc, sans gérer les annotations de contact
nominales ou leur ordre de décharge. Ces limites expliquent pourquoi ce résultat
ne permet pas de juger un planificateur hybride complet ; leur responsabilité
individuelle n'est pas isolée par une nouvelle ablation.

**Décision recommandée : ne pas prolonger cette famille ni optimiser ses gains.**
Avant tout nouvel essai, examiner statiquement le transfert souhaité/réel,
la libération des contacts passifs et l'orientation de la racine libre dans une
séquence entière. Aucune nécessité mathématique de mains/genoux et aucune
impossibilité pieds seuls ne sont démontrées. L'audit exhaustif du solveur reste
différé et non certifié ; il n'est pas la prochaine dépense recommandée.

Le champ historique `loadedFeet` du statut n'est pas rafraîchi par cet adaptateur
expérimental. Les contacts horodatés/anatomiques et les prédicats physiques
originaux, qui utilisent les charges intégrées, constituent les preuves utilisées.
Les phases/cibles et la projection articulaire ne sont jamais des preuves debout.

## Budget, vérifications et livraison

Préparation plafonnée à deux journées : 1 078,709 s depuis l'ouverture du registre,
ou conservativement 1 514,646 s depuis l'autorisation (démarrage/lectures inclus).
**2 histoires, 6 089 pas, 25,370833 s physiques, 12 480 pas réservés conservés.**
Le dernier processus finit à 06:14:02,961206 UTC ; 48,202256 s murales depuis
le premier lancement, 31,150602 s d'exécution native cumulée. Plafond 600 s respecté.
Le registre est `closed-no-complete-recovery`. Le troisième essai est interdit
faute de succès complet. Les registres P1 et 96 histoires conservent leurs SHA.

Build release, Clippy strict tous targets avec/sans fonctionnalité, trois nouveaux
tests Rust sans intégration, handoff original 143/144 sans intégration et six
tests Python du registre passent. Aucun handoff positif physique, audit exhaustif,
suite complète, qualification générale ou correction des échecs hybrides et des
quatre tests TypeScript historiques revendiqués. La récupération reste inactive
par défaut. Le code expérimental rejeté reste sous `cfg(test)` et n'est pas promu.

- [Vidéo A/B, 5,3 s, poses natives archivées, lecture environ 4×](media/getup-ab-2026-10-12/getup-ab-recorded.mp4)
- [Étapes comparées, tous deux en échec](media/getup-ab-2026-10-12/getup-ab-milestones.png)
- [Suivi, bassin, accélération et couples](media/getup-ab-2026-10-12/getup-ab-comparison.png)
- [Reçu numérique et empreintes](getup-ab-results-2026-10-12.json)

Ces visuels sont des rendus scientifiques des états enregistrés, pas des captures
du navigateur. Zéro intégration supplémentaire pour les produire. Les états
arrêtés restent fixes, avec légende explicite, sans extrapolation de trajectoire.
Traces, registre, manifeste et binaire : `evidence/getup-ab-2026-10-12/`.

La vidéo est également enregistrée dans Library, sous `/getup-ab-recorded.mp4`,
version initiale reçue. Réception confirmée, 133 392 octets ; métadonnées locales appliquées.
SHA-256 `0da358b1e24d121e20fc038a77512d301d76f319256b7be067c77e94ad7fb011`.

La revue automatique rejette les pushes en retenant l'ancienne instruction
« sans push » et en jugeant la nouvelle délégation non fiable. Les sauvegardes
locales restent possibles ; aucune publication distante ni restauration de
connexion n'est revendiquée. Une confirmation directe est demandée pour le push.
