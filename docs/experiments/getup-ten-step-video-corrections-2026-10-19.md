# Dix pas : corrections après revue de la vidéo du 19 octobre

Statut : quatre revues indépendantes incorporées ; cinquième revue d’intégration ci-dessous, achevée avant modification du contrôleur.

## Référence effectivement regardée

Vidéo de dix pas, 83,35875 s, 834 images, 800 × 450, empreinte e193082d… ; planche de douze instants, séquence 49–51 s et dernière image décodées et inspectées. Les dix avances de chaque pied valent 181,02–182,01 mm ; le COM avance de 0,859 m. Le transfert latéral et les pauses de phase dominent les pixels. Le bassin reste penché (24,11° finaux), malgré un tronc presque vertical (1,81°). Les bras opposés bougent peu à cette cadence. La vidéo finale ne démontre pas un bassin/tronc aligné.

## Plan final compact

1. Restaurer les entrées natives perdues sans prétendre retrouver les anciens journaux. Lancer les nouveaux essais avec provenance explicite, dans un seul monde continu et un plafond de calcul suffisant.
2. Partir du contrôleur d’appui robuste fonctionnel. Réutiliser le mode existant continuous-com-causal, déjà éprouvé sur six avances, pour retirer seulement l’attente nominale de sélection du risque et conserver une progression COM C2. Garder le transfert minimal de 4 s au premier pas et 2 s ensuite, les 100 ms/21 commandes d’appui observé, la correction bornée achevée et les 80 échantillons de déchargement. La borne nominale entre réceptions reste 6,52 s. Ne reprendre aucun gros optimiseur whole-body hors ligne.
3. Éprouver la cadence seule d’abord. Puis une seule préférence terminale expérimentale, après réception finale et double appui qualifié : figer une fois l’orientation mesurée du bassin, viser la verticale de même lacet avec correction SO(3) limitée à 6°, quintique de 4 s et poids doux 0,5 activé par la même quintique. Garder exactement tronc, COM, articulations, pieds, bras et barrières. Commencer les 4 000 échantillons de calme seulement après la fin de la correction, puis tenir la cible fixe. Un pilote natif court précède toute séquence entière avec cette préférence ; aucun balayage de poids, nouvelle géométrie ou relaxation. L’inclinaison du bassin pendant la marche reste un défaut explicitement non corrigé par ce pilote terminal.
4. Vérifier et enregistrer toutes les intégrations : dix avances alternées ≥180 mm, glissement cumulé séparé par pied ≤20 mm (arrêt strictement au-delà, jamais de remise à zéro), charge de chaque corps hors pieds <3 N, contraintes articulaires/moteur/capture et appui robuste inchangés. Conserver la marge COM/capture définie ≥0 à chaque intégration, ≥3 mm à l’admission robuste et pendant le calme ; publier séparément le test plus strict ≥3 mm sur toute la séquence. L’ancienne marge 2,9514 mm manque ce dernier critère de 48,6 µm. Garder la grâce articulaire initiale de 63,75 ms, sans extension. Le calme final doit durer 4 000 échantillons natifs consécutifs, soit 5 s. Mesurer séparément l’alignement bassin/tronc ; ne pas le déduire du calme.
5. Rendre la séquence réellement intégrée à 1×, face/profil et fin entière. Comparer temps entre réceptions, attente par phase, balancement latéral, inclinaison bassin/tronc, flexion des genoux, mouvement opposé des bras, distance COM et glissement. Conserver la meilleure référence fonctionnelle si une correction échoue et publier les échecs significatifs avec leurs causes.

Critères : cible d’intervalle médian <7,63 s entre réceptions ; <5 s est hors portée des horloges conservées et hors de cette correction. Publier les neuf intervalles, le transfert le plus long et la fraction de temps de marche à vitesse COM avant <5 mm/s. La préférence terminale vise une réduction ≥5° du bassin et de l’angle relatif tronc/bassin durant les cinq secondes finales, avec tronc ≤5° ; une amélioration plus faible reste chiffrée comme partielle. Publier aussi RMS, percentile 95 et maximum des inclinaisons par phase. Le passage de toutes les contraintes physiques et des dix pas reste obligatoire pour qualifier une nouvelle marche. Aucun changement de main, de déploiement, de forces de racine ou des paramètres physiques.

## Revues

1. Revue visuelle : trois planches réellement inspectées ; remplace l’amélioration infinitésimale du bassin par un objectif ≥5° et ajoute les temps d’attente et métriques de phase.
2. Revue de cadence causale : confirme le mode existant, interdit de raccourcir les horloges/admissions, précise la borne 6,52 s et ajoute un plafond de calcul configurable et enregistré.
3. Revue des contraintes et preuves : distingue capture ≥0 et réserve 3 mm, garde la règle strictement >20 mm, impose compteur/journal/checkpoint concordants et audit complet distinct du rendu.
4. Revue de faisabilité posturale : rejette l’ancien cône dur à 12° ; réduit la correction au maintien final, poids unique 0,5, 6°/4 s, activation unique, calme décompté après la rampe.
5. Revue finale d’intégration : vérification du plan révisé contre les quatre rapports et le code existant. La cadence peut réutiliser les admissions et témoins actuels ; la préférence terminale doit avoir son propre profil et sa propre preuve de références. Aucun témoin de déchargement antérieur ne qualifie une référence modifiée. Les métriques historiques sans journal perdu restent des références archivées, pas une nouvelle preuve échantillon par échantillon. Les critères physiques, cadence, qualité de capture et posture gardent des verdicts séparés. Sans pilote terminal concluant, livrer la cadence vérifiée et signaler le défaut postural résiduel, sans faux succès global.

Les cinq passes sont réelles et distinctes. La passe 5 a confronté le document révisé, les conclusions des quatre angles précédents et les interfaces de référence/quiet/audit avant tout changement de code.

## Mise en œuvre

- Cadence : mode existant `continuous-com-causal`, sans changement des horloges ou admissions ; nouveau plafond de calcul explicite, fini et enregistré (`--wall-cap-s 3600` pour les essais).
- Posture : profil opt-in `terminal-pelvis-soft-v1` dans `source_terminal_pelvis.py`, réservé au maintien terminal. Le pilote à un pas passe son audit indépendant, ses 21 témoins d’appui et ses trois témoins de commandes posturales ; aucune intégration supplémentaire pendant ces relectures.
- Mesure : `analyze_source_gait_quality.py` sépare les intervalles de réception, l’attente, les inclinaisons, les marges COM/capture et le déplacement propre des pieds du déplacement du COM.
- Rendu : les marges capture indéfinies ne peuvent plus passer silencieusement. Un verdict physique complet exige un audit indépendant lié au même journal ; le test de réserve à 3 mm reste distinct. Le rendu conserve la durée rationnelle exacte, les poses enregistrées distinctes et le dernier échantillon réel.
- Audit : trois anciens réducteurs chargeaient tout le journal de commandes ; l’essai de dix pas a révélé une allocation excessive. La lecture et les empreintes sont désormais progressives, avec les mêmes calculs et contrôles. Le pilote et les nouvelles séquences ne sont pas réintégrés pour cette correction du lecteur.

## Résultats physiques enregistrés

La cadence seule termine **60 409 intégrations / 75,51125 s**, avec dix avances ≥180 mm puis 5 s calmes. Glissement gauche/droite : **15,78344 / 12,77292 mm**. Les neuf intervalles de réception sont **6,870 ; 6,87125 ; 6,875 ; 6,875 ; 6,875 ; 6,875 ; 6,875 ; 6,86875 ; 6,875 s**. Leur médiane vaut **6,875 s**. Le bassin final reste incliné à **24,33711°**.

La version combinée termine **63 612 intégrations / 79,515 s**, incluant la correction puis **4 000 nouveaux échantillons calmes**. Le maintien supplémentaire vaut **4,00375 s** par rapport à la cadence seule. Glissement : **15,79706 / 12,79084 mm**. Bassin final **19,13974°**, tronc **1,79471°**, angle relatif **20,18120°**. La comparaison point par point des cinq secondes calmes avec la cadence seule donne un gain de bassin minimal/moyen/final de **4,11784 / 5,09152 / 5,19737°**. L’objectif doux de gagner ≥5° à chacun des 4 000 échantillons n’est pas entièrement atteint. Face au seul dernier pixel historique à 24,11°, le gain final vaut environ **4,97026°** ; les anciens échantillons calmes ne sont pas disponibles pour une comparaison complète.

Les **56 409 premiers échantillons**, jusqu’à l’activation terminale, ont exactement les mêmes champs physiques et de commande dans les deux nouvelles séquences. Les 51 checkpoints de phase ont les mêmes liaisons modèle/bibliothèques et le même état natif avant la pile de travail MuJoCo. Des traces diffèrent dans cette pile inactive (`pstack=0`) : l’identité des fichiers complets n’est donc pas affirmée. Aucun de leurs octets n’a été réécrit pour la comparaison.

## Limites maintenues visibles

- La réserve capture stricte de **3 mm sur toute la séquence échoue** : minimum **2,9837103 mm**, au pas 9 en réception, pendant 15 échantillons. La marge reste définie et positive à chaque intégration ; les admissions et le calme conservent leurs seuils. Aucun arrondi ne transforme ce déficit en réussite.
- Environ **36,44 %** du temps de marche après le placement initial des bras, stabilisations intermédiaires comprises et maintien final exclu, reste à une vitesse COM avant absolue <5 mm/s. La marche conserve des arrêts et un transfert latéral important.
- Le bassin pendant la marche n’est pas corrigé par la préférence terminale. Sa pointe mesurée atteint environ **40,51°**. La posture finale reste au-dessus d’un alignement à 5°.
- La petite excursion articulaire de départ et sa grâce héritée de 63,75 ms sont conservées explicitement. Le modèle reste la variante sélectionnée hanches/genoux avec `impratio=8`, sans qualification du modèle original GetUp inchangé.
- Les verdicts physiques complets des deux séquences et le rendu final sont encore en cours de vérification à ce stade du document. Le résumé de preuves final précisera ces verdicts, les tests, les empreintes et les fichiers livrés.
