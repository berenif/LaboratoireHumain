# GetUp : vérification bornée du candidat corrigé, 16 octobre 2026

La chaîne préentraînée est branchée dans Rust/WASM, mais aucune référence admissible et aucun relevage ne sont obtenus. Deux histoires physiques ont été exécutées depuis l'admission du retest, après un lancement avorté avant création du navigateur. Les trois réservations sont conservées intégralement. La récupération reste désactivée par défaut ; aucune qualification de 24 poses/cycles/impacts n'est revendiquée.

## Décisions et résultats

Le candidat `be04ad4e82163628ea89746dbd60305e81740dff` corrige les observations à partir des vitesses généralisées courantes et des repères normalisés, ainsi que le transport du rapport. Le lanceur sauvegarde désormais le rapport complet avant les assertions et captures. Une lecture dans Chromium à zéro sous-pas a vérifié ce transport sans modifier l'état physique.

| Admission | Sous-pas réels | Prévisions | Temps mural réel | Résultat |
| --- | ---: | ---: | ---: | --- |
| `40c3dad` | 0 | 0 | 0,099 s | Exécutable Playwright absent ; aucun navigateur créé. |
| `0fa8643`, physique `be04ad4` | 2 881 | 16 à 200 Hz | 42,644 s | Refus de liaison fixe `right_hand` à 0,080 s prévisionnelle. |
| `ea15d2845e30e857579862fc751bdebd28dff038` | 2 881 | 232 à 800 Hz | 43,271 s | Refus d'auto-contact `left_thigh/left_foot` à 0,290 s prévisionnelle. |

Les deux histoires réelles atteignent la chute mesurée à 3,60125 s. Leurs poses initiales prévisionnelles, première observation et première commande WEAK sont exactement identiques. Les rapports complets sont conservés : quatre commandes WEAK dans le premier essai et douze dans le second, dont la dernière interrompue. Dans les deux cas : zéro référence utilisable, zéro décision SLOW physique, zéro relevage. Le second essai atteint seulement 0,319175 m de hauteur de tête dans la prévision ; le terminal source exige plus de 1,4 m, puis une décision supplémentaire.

## Correction causale limitée

Le premier rapport montre que la main droite, rigidement liée à l'avant-bras, est arrêtée à mi-chemin du mouvement de son parent : fraction de translation 0,50000018 et de rotation 0,49999990. L'écart d'ancrage est de 20,986 mm et l'écart angulaire de 0,063538 rad ; le contrôle natif refuse 0,06354662 rad pour une limite inchangée de 0,06 rad. Le code CCD vendorié applique précisément un arrêt de pose individuel après la résolution. La fraction CCD n'a pas été instrumentée séparément : cette attribution repose sur les poses archivées et l'inspection du code, sans replay.

La branche remplaçait le pas réel de 1,25 ms par les 5 ms du simulateur source. Cette divergence d'intégration a été supprimée : le clone garde maintenant ses paramètres natifs, et l'acteur WEAK reste à 40 Hz, soit 20 sous-pas natifs par commande. L'horizon maximal reste de 250 décisions, désormais comptées comme 5 000 sous-pas. Le temps des archives utilise ce même pas. Aucun paramètre du monde réel, moteur, collision, masse, friction, couple ou seuil n'a changé. Aucun état prévisionnel n'est copié dans le monde réel.

Cette correction supprime le premier refus observé et prolonge la prévision jusqu'à un autre refus. Le nouvel arrêt est une pénétration réelle de 6,2771253 mm entre cuisse et pied gauches, supérieure aux 5 mm inchangés. Une reconstruction indépendante des distances entre axes de capsules, avec les poses archivées et les rayons source, trouve 6,3421142 mm. L'écart de 65 µm entre méthodes ne remet pas en cause le dépassement. À l'arrêt final, les trois liaisons fixes ont une rotation relative nulle ; leurs écarts géométriques d'ancrage restent inférieurs à 24 µm.

Le transfert de la politique préentraînée reste donc non qualifié. Ces résultats ne démontrent ni impossibilité anatomique ni cause exclusivement liée à la différence MuJoCo/Rapier. Ils ne justifient aucun relâchement du seuil ou réglage empirique des collisions.

## Vérifications

- Huit tests Rust GetUp ciblés passent sans intégration physique : observation, copie exacte du monde, cadence prévisionnelle, modèle/actionnement, PD source, splines et acteurs.
- Cinquante-six tests JavaScript d'outillage passent, dont lecture/réjection du rapport sans avance ni reset.
- Build navigateur et initialisation WASM à zéro sous-pas réussis. L'avance de zéro tick modifie seulement le numéro de publication ; les poses, diagnostics et temps physiques restent identiques.
- Sur 32 observations fixes, l'écart maximal de commande WASM/PyTorch est de 3,2954e-6.
- Sur les douze commandes WEAK réellement archivées du dernier essai, l'écart maximal avec PyTorch est de 3,9617e-6 ; l'écart des caractéristiques géométriques reconstruite/archivée est inférieur à 3,186e-7.
- Les audits et rendus n'ont exécuté aucune intégration physique. Les checkpoints épinglés sont chargés uniquement avec `weights_only=True`. Aucun MuJoCo, entraînement ou service de calcul externe n'a été exécuté.

Ces vérifications sont ciblées ; elles ne déclarent pas verte la suite historique globale Rust/TypeScript.

## Budget fermé et preuves

Le précédent registre fermé facturait 214 998 sous-pas et 2 926,328294 s. Les trois nouvelles réservations sont de 23 650, 23 650 et 27 400 sous-pas, chacune avec 900 s murales. Le lancement sans navigateur n'est pas remboursé. Le cumul fermé est **289 698 / 300 000 sous-pas** et **5 626,328294 / 7 200 s**. Le reliquat est de **10 302 sous-pas** et **1 573,671706 s**, sans renouvellement automatique. La consommation physique réelle de ce travail est de **6 010 sous-pas**, dont **248 prévisionnels**, en **86,014446 s** de lancements cumulés.

Registres chaînés par SHA-256 :

- `docs/evidence/getup-policy-retest-2026-10-16.json` : lancement avorté, zéro physique.
- `docs/evidence/getup-policy-retest-browser-2026-10-16.json` : retest `be04ad4`, rapport complet et diagnostic de liaison.
- `docs/evidence/getup-policy-native-clock-2026-10-16.json` : dernier essai, audits, plafonds et bilan final.

Les répertoires correspondants sous `outputs/` conservent les entrées figées, binaires WASM, snapshots, rapports natifs, observations/commandes, journaux et captures. Le clip final contient uniquement les 74 captures du navigateur, à 20 images/s : H.264, 1280×900, yuv420p, 3,7 s, décodage intégral validé. La figure comparant les prévisions utilise exclusivement les poses archivées ; leurs instants terminaux différents sont indiqués.

## TODO fermé

- [x] Vérifier physiquement le candidat corrigé et conserver le rapport auparavant perdu.
- [x] Isoler la divergence de pas prévisionnel, la corriger et vérifier une dernière histoire bornée.
- [x] Auditer les commandes archivées, confirmer le refus géométrique et produire les preuves sans replay.
- [x] Fermer tous les registres sans remise à zéro ni remboursement.
- [ ] Obtenir une référence admissible, appliquer SLOW physiquement et démontrer un relevage stable : non atteint, aucune nouvelle histoire autorenouvelée.
- [ ] Qualification distincte des 24 poses/cycles/impacts : non commencée.
