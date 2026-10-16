# Relevage prototype obtenu dans Rapier — 16 octobre 2026

**Le corps se relève après l'impact et satisfait le critère physique natif d'une seconde debout stable.** Cette preuve est atteinte à **7,79125 s physiques, soit 5,53 s après l'impact**. À l'arrêt à 9,8 s, la stabilité continue vaut **3,00875 s**, après **6,19875 s de SLOW**. Aucun refus physique n'est observé. Il s'agit d'un seul cas sur le modèle GetUp courant, pas de la qualification des 24 poses/cycles/impacts.

## Budget amendé et dépenses réelles

Le parent a confirmé que les 8 200 sous-pas étaient une estimation de planification ; le plafond utilisateur est **180 s de calcul réel cumulé**. Le [registre ajouté](getup-complete-replay-2026-10-16.json) conserve ce plafond et l'amendement autorisant **10 000 sous-pas supplémentaires** après les 3 760 déjà consommés. Aucun ancien registre, dont celui des 299 998 sous-pas réservés sur l'ancien budget de 300 000, n'est modifié.

| Lancement conservé | Pas réels | Prévisions et validation | Temps mural du lancement |
| --- | ---: | ---: | ---: |
| Premier défaut de compteur | 20 | 0 | 1,625063235 s |
| Histoire bornée à 4 s | 3 200 | 540 | 36,929941929 s |
| Défaut de vérification du checkpoint à 2 s | 1 600 | 0 | 8,089610727 s |
| Histoire finale à 9,8 s | 7 840 | 540 WEAK + 20 restauration | 43,616624358 s |
| **Cumul** | **12 660** | **1 100** | **90,261240249 s** |

Le total natif est **13 760**. Les 10 000 pas supplémentaires autorisés en planification sont entièrement comptés : 1 600 du lancement interrompu + 8 400 du rejeu corrigé. Ce plafond arrête l'histoire finale à 9,8 s plutôt qu'aux 10 s visées ; le relevage et plusieurs secondes de maintien ont alors été observés. Le plafond utilisateur mural de 180 s n'est pas augmenté. Le superviseur externe impose le reliquat exact, avec une marge d'arrêt de 100 ms.

Le défaut intermédiaire concernait une conversion JSON d'une table de contacts Rapier à clés composées, dans la vérification du checkpoint. Il n'était pas une faute physique. La correction conserve le monde en binaire, vérifie sa restauration typée et écrit le fichier **avant** toute vérification. Le temps et les pas de ce défaut restent débités. Il n'y a ni réglage, ni nouvelle graine, ni changement de paramètres physiques entre les lancements.

## Histoire et critères physiques

Le même runtime WASM est exécuté directement dans Node, sans travail de rendu pendant la simulation. Les **41 observations du préfixe de 4 s** correspondent exactement aux poses et vitesses archivées du précédent essai. L'état initial, le profil, le sol et la commande d'impact à 2 s sont identiques. Aucun reset ni remplacement de poses n'intervient pendant l'histoire.

L'impact est mesuré à 2,26125 s. WEAK part de l'état réel à 3,60125 s et produit la même référence admise en **27 décisions / 540 pas**. SLOW réalise ensuite **248 décisions**, achève le suivi de sa référence et poursuit le maintien. À la fin, la tête est à **1,4453 m**, les deux pieds portent **202,735 N et 197,824 N**, soit **99,97 % du poids**, sans autre appui chargé.

Le critère natif reste inchangé : tête au-dessus de 1,4 m, torse vertical ≥ 0,97, charges persistantes des deux pieds, au moins 80 % du poids sur les pieds, aucun autre appui ≥ 3 N, CoM et point de capture dans les contacts porteurs avec marge de 3 mm, vitesses des CoM corporels ≤ 0,22 m/s et angulaires ≤ 0,65 rad/s, dérive des pieds ≤ 1 cm, pendant 800 pas natifs continus. Le maintien final compte **2 407 pas**, soit 3,00875 s.

Un [audit indépendant des archives](getup-complete-replay-2026-10-16/postrun-audit.json.gz) reconstruit les vitesses depuis les coordonnées généralisées et les poses exactes avant décision, sans utiliser les caches de vitesse des rigid bodies. Sur **120 états SLOW de la période stable**, les maxima sont 0,1469 m/s, 0,3351 rad/s et 5,023 mm de dérive des pieds. Les 31 paquets d'appuis échantillonnés sont cohérents avec le critère ; l'observateur natif reste celui qui vérifie chaque sous-pas.

Le couple demandé maximal mesuré est **607,90 Nm**, sous le budget de 920 Nm. Les erreurs maximales d'ancrage et de limite articulaire sont 1,37 × 10⁻⁷ m et 0,000177 rad. Aucun bassin assisté, changement de masse/friction/collision/anatomie ou couple hors contrat n'est introduit. Les pénétrations restent mesurées, diagnostic-only comme demandé : maximum propre réel 0,598 mm, sol 11,386 mm. Les collisions et les autres gardes d'intégrité restent actives.

## État désormais restaurable

Les checkpoints à **0, 2, 4, 6, 8 et 9,8 s** conservent le monde Rapier binaire et la mémoire auxiliaire Runtime/Simulation : corps, colliders, articulations, contacts, paramètres, chronologie, appareil d'impact, contrôleur, références et compteurs. Le modèle immuable est vérifié contre la définition canonique. Cette voie est limitée au protocole GetUp sans contrôleurs ou saisie incompatibles ; elle refuse de les omettre silencieusement. Rapier reconstruit ses espaces de travail pipeline/CCD documentés comme transitoires.

À 4 s, une copie restaurée exécute **un tick de 20 pas**, entièrement compté comme prévision de validation. Son snapshot suivant et son contrôleur sont **exactement identiques** à ceux du monde resté vivant. Le checkpoint final a ensuite été rechargé dans un autre processus, **sans pas physique** : poses, références, contrôleur et stabilité à 3,00875 s sont conservés. Les [archives et empreintes](getup-complete-replay-2026-10-16/archives.json) incluent les six fichiers compressés et l'identité du runtime correspondant.

## Livrables et portée

Le build, **15 tests Rust ciblés**, **61 tests JavaScript** et les préflights de sauvegarde/restauration passent. Les vérifications statiques n'intègrent aucune physique ; les seuls pas supplémentaires de restauration sont les 20 explicitement comptés. La suite globale historique et la qualification des 24 cas n'ont pas été exécutées.

La vidéo `LaboratoireHumain-Rapier-relevage-prototype-2026-10-16.mp4` montre la chute, le relevage et le maintien depuis les archives, sans interpolation ni calcul physique de rendu. Validation : H.264, 1280 × 720, 216 images à 20 fps, décodage complet et images intermédiaire/finale inspectées. Elle dure 10,8 s, dont 1 s explicite de maintien de la dernière pose à 9,8 s physiques. [Validation et identité Library confirmée](getup-complete-replay-2026-10-16/video-validation.json), version 0.

Ce résultat établit un relevage prototype réel sur ce cas du modèle GetUp actuel (25 handles, 21 articulations motorisées). Il ne qualifie pas les autres poses ou impacts et ne valide pas à lui seul l'ancienne anatomie à 46 coordonnées. La récupération publique reste désactivée jusqu'à qualification distincte. Aucun essai supplémentaire n'est lancé.
