# Comparaison diagnostique MuJoCo / Rapier — 16 octobre 2026

**Le WEAK source viole lui aussi le seuil de 5 mm depuis cet état.** La comparaison n'établit aucune correction supplémentaire d'ordre, de signe ou d'observation à appliquer dans l'app. Aucun relevage n'est qualifié et aucune commande SLOW physique n'a été ajoutée.

Le même état généralisé est reconstruit depuis l'archive Rapier `459426d` : racine, 21 angles et 27 vitesses. MuJoCo 2.1.3 provient du paquet officiel PyPI vérifié par SHA-256 ; les XML et poids restent épinglés à `5ad52fb`. Les poses sont appariées à 70 µm et 6e-7 rad près après normalisation des quaternions natifs. Les états internes de résolution et de warm-start ne sont pas transférables entre moteurs.

Trois histoires fixes de 0,30 s ont été exécutées une seule fois, toutes à 40 décisions/s :

| Diagnostic MuJoCo | Pas | Intégrations | Premier dépassement 5 mm | Pic d'auto-contact |
| --- | ---: | ---: | ---: | ---: |
| XML source WEAK, retour d'état | 5 ms | 60 | 0,085 s, tibia/tibia | 33,678 mm |
| XML 800 Hz, retour d'état | 1,25 ms | 240 | 0,085 s, tibia/tibia | 34,881 mm |
| XML 800 Hz, commandes Rapier exactement rejouées | 1,25 ms | 240 | 0,090 s, tibia/tibia | 40,823 mm |

Ces diagnostics source poursuivent les mesures jusqu'à 0,30 s pour observer les contacts ; chaque dépassement demeure un **échec de qualification**. Le seuil de l'app est inchangé. Le Rapier archivé avait été arrêté à 0,290 s sur 6,277 mm entre cuisse et pied gauches. À cette même configuration articulaire, MuJoCo mesure 6,352 mm : la paire est bien activée dans le moteur source.

Sous les commandes Rapier exactement identiques, l'écart apparaît avant la deuxième décision : à 25 ms, 0,053409 rad sur la cheville gauche et 3,034 mm au maximum sur un corps. À 290 ms, l'écart maximal atteint 0,356565 rad et 122,063 mm. L'effort généralisé MuJoCo correspond exactement à `gear × commande` sur les 540 pas. En boucle fermée, les commandes divergent ensuite : écart maximal de 0,372 dès la décision à 25 ms pour le cas 800 Hz.

Les contrôles sans intégration confirment les masses, tenseurs d'inertie, centres de masse, armatures, amortissements, raideurs et limites déclarées. Les angles et vitesses généralisées sont reproduits exactement. Les observations géométriques diffèrent au plus de 69 µm sur les douze états archivés, la vitesse du CoM de moins de 1e-4 m/s. Sur l'observation initiale reconstruite, la première commande diffère d'environ 5,04e-5 ; sur l'observation archivée identique, l'écart d'inférence est de 3,96e-6.

Cela démontre un écart mécanique/de résolution sous commandes identiques et l'incompatibilité de cette référence WEAK avec notre seuil, même dans son moteur d'origine pour ce départ. Cela ne prouve ni l'absence de tout défaut mécanique de port, ni l'appartenance de cet état à la distribution d'entraînement, ni une impossibilité anatomique. La normalisation géométrique et les warm-starts différents limitent l'appariement.

**Étape suivante justifiée :** conserver le rejet de cette référence. Aucune exclusion de collision, relâchement de seuil ou migration ne découle de ces résultats. Avant un nouvel essai physique, un audit sans intégration des matrices de masse et efforts passifs/de contact permettrait d'isoler un éventuel défaut mécanique résiduel ; une génération de référence conforme aux contraintes reste nécessaire pour admettre SLOW.

Le code diagnostique compile avec les en-têtes exacts et `-Wall -Wextra -Werror`. La garde de compteur refuse un pas avec budget nul sans changer l'état. Les trois histoires totalisent **540 pas**, **1,926 s murale**, zéro avertissement et zéro nouveau pas Rapier. Les 30 appels explicites de préparation/forward sont recensés et n'intègrent pas le temps. Analyse et rendu utilisent uniquement les archives.

Le registre chaîné est fermé à **290 238 / 300 000 pas** et **5 746,328 / 7 200 secondes réservées** ; reliquat **9 762 pas et 1 453,672 s**, sans renouvellement automatique. Les anciennes preuves et modifications locales étrangères sont préservées. Les résultats numériques sont dans `outputs/getup-mujoco-comparison-2026-10-16/`, avec états par pas, commandes, comparaisons et dépendances vérifiées. Le clip est une lecture ralentie des poses archivées, sans interpolation ni nouvelle simulation.
