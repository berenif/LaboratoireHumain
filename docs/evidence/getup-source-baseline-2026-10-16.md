# Témoin positif GetUp source — 16 octobre 2026

**Le témoin source réussit un relevage SLOW complet avec station debout physiquement vérifiée. Il ne satisfait pas le contrat de collision de l'app.**

Configuration fixée avant exécution : seed 0, speed 0,5, modèle/poids au commit `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`, MuJoCo 2.1.3 officiel, WEAK 40 décisions/s et physique 200 Hz, SLOW 40 décisions/s et physique/PD 800 Hz. Le power WEAK est 0,66342, valeur `curr_power` réellement utilisée par `organize_args`; celui de SLOW est 1.

Le protocole dm_control 1.0.1 est reproduit via l'ABI inspectée : tirage collision-free avec le RandomState original, 400 pas de préparation du constructeur sur le XML dm_control original, rechargement du modèle source puis 400 pas à commande nulle par tentative. La référence utilise la fonction source de splines, puis le premier état interpolé avec qvel multipliées par 0,5 est installé dans le modèle SLOW distinct, comme dans le source. Cette initialisation expérimentale n'est pas une histoire continue d'impact de l'app.

WEAK réussit sa première tentative : **33 décisions / 165 pas / 0,825 s** après la chute préparatoire ; hauteur de tête finale **1,541372 m**. Ses 33 échantillons donnent 66 cibles interpolées. SLOW termine **216 décisions / 4 320 pas / 5,4 s**, dont les 150 décisions finales de maintien source.

La première seconde continue de station stable est vérifiée à **3,4775 s** après le départ SLOW. La fenêtre finale dure **2,9225 s**. Pendant la dernière seconde : tête ≥1,4525 m, verticalité du torse ≥0,999451 ; pieds ≥400,55 N pour un poids de 400,68 N ; aucun autre appui au sol ; vitesse maximale des corps ≤0,00662 m/s et ≤0,01724 rad/s ; dérive des pieds ≤8,58 mm ; CoM et point de capture à plus de 57 mm dans l'enveloppe des vrais contacts résolus des pieds. L'audit des 4 320 états retrouve exactement les compteurs de stabilité, sans nouvelle intégration.

| Phase | Pic d'auto-pénétration | Pic de pénétration du sol |
| --- | ---: | ---: |
| Chute préparatoire source | 13,18 mm | 29,30 mm |
| WEAK | **47,17 mm**, cuisse/pied gauches | 20,20 mm |
| Épisode SLOW complet | **8,07 mm**, tibia/avant-bras droits | 16,93 mm |

SLOW dépasse 5 mm pendant 20 sous-pas, soit 25 ms. Ce diagnostic source explicitement autorisé poursuit la mesure au-delà du seuil ; le seuil de l'app n'a pas changé. Le succès source ne qualifie donc aucun relevage du projet et n'autorise pas l'activation de sa récupération.

Un incident de sérialisation NumPy a interrompu le premier pas SLOW après le teacher : 966 pas étaient consommés. La correction convertit les prédicats en booléens Python, avec un test de sérialisation sans physique. La préparation et WEAK n'ont pas été rejoués. L'épisode SLOW complet a repris depuis son état initial archivé ; observation et première sortie du réseau ont été vérifiées identiques. Le pas interrompu reste compté, sans état détaillé archivé. La reprise et les deux durées de calcul sont recensées dans le même plafond initial, sans remboursement ni remise à zéro.

Les **5 286 pas réels** comprennent constructeur 400 + préparation 400 + WEAK 165 + pas SLOW interrompu 1 + SLOW complet 4 320. Le calcul physique cumulé prend **8,9849 s**. La réservation de 9 760 pas/180 s reste intégralement chargée : cumul fermé **299 998 / 300 000 pas** et **5 926,3283 / 7 200 s**, reliquat **2 pas** et **1 273,6717 s**. Aucun autre calcul physique n'est lancé.

Écarts logiciels déclarés : protocole dm_control 1.0.1 reproduit dans le petit diagnostic ABI au lieu du framework complet ; PyTorch 2.9.1 CPU pour `weights_only=True`, contre 1.5.1 dans les anciennes dépendances ; NumPy 2.3.5 et SciPy 1.17.0, non épinglés par le dépôt d'origine. Les 32 cas de randomisation, 32 d'acteurs et 32 de PD passent sans intégration ; les splines exécutent uniquement la fonction source épinglée. Aucun entraînement, service payant ou changement du moteur de l'app.

**Implication :** les poids et la chaîne source produisent un relevage stable dans leur distribution d'origine. Les échecs depuis l'état natif et le rejet par l'app ne démontrent pas des poids inopérants. Ils laissent des écarts de départ/dynamique et une incompatibilité de collision concrète : le teacher source réussi et son SLOW réussi dépassent tous deux 5 mm. La prochaine étape doit traiter la génération et le suivi de références sous ce contrat ; modifier la friction ou lever le seuil n'est pas justifié. Un nouveau budget physique devra être approuvé avant toute autre campagne.
