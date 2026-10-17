# Dos, ventre et côté : deux seuils transitoires, une fixture non admise

**Dos et ventre atteignent le critère natif d'une seconde debout stable, puis leur compteur de stabilité courante revient à zéro avant le dernier paquet archivé.** Les corps restent redressés, mais le maintien du critère est fragile. La construction latérale roule pendant son tassement ; elle finit en équilibre oblique vers le ventre et n'est pas admise comme départ sur le côté. Le contrôleur n'est donc pas évalué sur ce troisième cas.

Le lot consomme **124,722222910 s de calcul physique réel sur 180 s**, et **24 640 sous-pas**, tassement et prévisions compris. Le solde de **55,277777090 s** n'est pas facturé comme dépense réelle. Aucun réglage, variante de graine, assistance ou relâchement de seuil n'est tenté après les résultats.

## Construction et admission des fixtures

Les trois constructions utilisent le modèle GetUp courant et exactement les mêmes 21 coordonnées articulaires initiales, masses, collisions, friction, poids appris et paramètres de commande. À zéro pas uniquement, la racine libre est tournée rigidement : dos −π/2 autour de X, ventre +π/2 autour de X, côté +π/2 autour de Z. Tous les colliders actifs sont placés avec 2 mm de garde initiale commune. La fonction refuse une seconde initialisation ou une modification après intégration.

Le corps se tasse ensuite réellement sous gravité, contacts et ressorts passifs, avec commandes motrices nulles. Cette préparation est comptée dans le budget et limitée à 2 s par définition figée du lot. L'admission exige 120 pas consécutifs avec orientation déclarée, tête ≤ 0,5 m, charge sol ≥ 80 % du poids, vitesses des CoM corporels ≤ 0,22 m/s et angulaires ≤ 0,65 rad/s. Ces vitesses proviennent des coordonnées généralisées actuelles, pas des caches de vitesse des rigid bodies.

Dos et ventre sont admis à **0,5 s physique**, respectivement après 140 et 191 pas consécutifs répondant aux critères. Le déclenchement ne modifie aucune pose ou vitesse : le lanceur compare les segments avant/après cette transition sans pas physique. Le contrôleur reçoit alors l'état réel et produit sa prévision WEAK puis ses commandes SLOW habituelles. Le délai de 25 s et le critère de station stable sont inchangés. L'époque de délai est le déclenchement de la fixture ; aucun impact n'est fabriqué, et les compteurs d'impact restent à zéro.

## Résultats

| Cas | Fixture admise | Durée physique totale | SLOW observé | 1 s native atteinte après déclenchement | Compteur stable final |
| --- | --- | ---: | ---: | ---: | ---: |
| Dos | Oui | 12,3 s | 11,8 s / 472 décisions | 11,7725 s | 0 s |
| Ventre | Oui | 14,6 s | 14,1 s / 564 décisions | 14,05125 s | 0 s |
| Côté | Non : orientation finale incorrecte | 2 s de tassement | Non déclenché | Non évalué | Non applicable |

WEAK termine en 48 décisions / 960 pas pour le dos, et 28 décisions / 560 pas pour le ventre. Les deux références partent exactement de l'état réel. Aucun refus d'intégrité ou de référence n'est observé. Les têtes finales sont à **1,4509 m et 1,4551 m**, mais le mode natif final est `Recovering`, en accord avec la stabilité courante nulle. Ces franchissements minimaux ne constituent pas un maintien prolongé.

Les observations sont archivées au plus toutes les 100 ms. Le dernier compteur avant franchissement vaut 742 pas à 12,2 s pour le dos, et 759 pas à 14,5 s pour le ventre. Le paquet suivant porte le drapeau natif persistant `prototype_standing_passed=true`, mais son compteur courant a déjà été réinitialisé. Un compteur ne peut gagner que 1 par intégration ou repartir de zéro : dans un intervalle de 80 pas, il est impossible de refaire une chaîne de 800 pas après une remise à zéro. On déduit donc les franchissements à **12,2725 s et 14,55125 s physiques**, sans interpoler les poses. Le critère natif existant a bien été atteint ; sa perte rapide doit rester visible dans l'interprétation. La cause précise de cette perte n'est pas isolée par ce lot, et aucun réglage n'est appliqué pour la masquer.

La fixture de côté est mécaniquement tassée à 2 s : vitesse maximale 0,00652 m/s, vitesse angulaire maximale 0,01753 rad/s, charge sol 396,293 N. Son axe de face a cependant une composante verticale de −0,7755 et son axe latéral de +0,6081 ; la définition latérale exige respectivement |face| ≤ 0,35 et axe latéral ≥ 0,8. Elle a roulé vers une position oblique ventrale. Aucun WEAK ou SLOW n'est lancé sur cette fixture non représentative. Cela ne constitue ni un échec du contrôleur depuis le côté, ni une impossibilité anatomique.

## Budget et conservation

| Cas | Pas réels | Pas prévisionnels | Calcul mural réel |
| --- | ---: | ---: | ---: |
| Dos | 9 840 | 960 | 54,835364411 s |
| Ventre | 11 680 | 560 | 60,698345233 s |
| Côté | 1 600 | 0 | 9,091513387 s |
| **Total** | **23 120** | **1 520** | **124,625223031 s** |

Le temps du lot complet, petites opérations du superviseur comprises, est **124,722222910 s**. Le temps disponible est réparti selon le reliquat réel et les cas restants ; aucun plafond inutilisé n'est débité comme consommation. Un timeout externe limite chaque processus et le lot global. Les anciens registres et les modifications étrangères à cette tâche restent identiques par empreinte.

Les [archives](getup-three-poses-2026-10-16/archives.json) conservent les histoires, commandes et checkpoints complets à l'initialisation, au déclenchement, tous les 2 s physiques et avant fermeture. Les trois états finaux ont été rechargés **sans intégration** : positions, contrôleur, références et mémoire de fixture sont identiques. L'[audit compact](getup-three-poses-2026-10-16/postrun-audit.json) distingue admission de fixture, franchissement du critère natif et stabilité finale. **16 tests Rust ciblés, 61 tests JavaScript**, le build et les préflights WASM à zéro pas passent. La suite globale historique n'est pas revendiquée.

Le clip `LaboratoireHumain-GetUp-dos-ventre-cote-2026-10-16.mp4` compare trois histoires indépendantes, un corps par cas. Il montre les franchissements entre échantillons et la perte du compteur courant, puis identifie le côté comme non admis. Les images restent explicitement figées après la fin de chaque histoire. Validation : H.264, 1600 × 900, 312 images à 20 fps, décodage complet et dernière image inspectée. [Sauvegarde Library confirmée](getup-three-poses-2026-10-16/video-validation.json), version 0. Le rendu vient exclusivement des archives et n'ajoute aucun pas physique ; son temps est journalisé séparément.

## Travail restant

Il reste à construire et valider mécaniquement une fixture qui demeure réellement latérale après tassement, puis à tester le même contrôleur depuis cette fixture. Aucune variante improvisée n'est lancée avec le reliquat. La fragilité du maintien dos/ventre demande aussi un diagnostic ciblé avant tout ajustement. Le lot n'est pas une qualification des 24 poses/cycles/impacts, et la récupération publique reste désactivée.
