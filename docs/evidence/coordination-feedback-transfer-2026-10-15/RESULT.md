# Retour d’état du transfert : candidat rejeté

[Vidéo réelle](rendered/main-history.mp4) · [comparaison graphique](comparison.png) · [mesures CSV](comparison.csv) · [résultats structurés](comparison.json).

L’analyse des archives montre que le CoM et le tronc accélèrent vers l’avant pendant la progression de la flexion des hanches ; les genoux changent peu et les chevilles mesurées sont proches de leur limite supérieure de 20°. Le défaut de freinage/recentrage est une hypothèse soutenue par cette chronologie, pas une cause dynamique unique démontrée.

Un seul candidat a été essayé : posture de référence A36 maintenue pendant deux secondes, retour du CoM et de sa vitesse vers le centre du polygone chargé, freinage du tronc et objectif de pieds horizontaux, puis continuation de la référence complète avec ce même retour d’état. Le solveur existant commande 46 coordonnées à 60 Hz. Les contacts actuels sont reconstruits à chaque décision comme guides souples ; aucun verrouillage d’un point matériel ni reçu de prévision. Les objectifs cinématiques ne constituent pas une identification de la réponse ForceBased.

| Mesure | Référence archivée | Candidat avec retour |
|---|---:|---:|
| Première sortie du point de capture du polygone avec marge | 0,179 s | 0,167 s |
| Premier contact chargé tête/tronc/bassin au sol | 1,588 s | 1,354 s |
| Hauteur du bassin à 3,433 s | 0,1352 m | 0,2303 m |
| Hauteur du CoM à 3,433 s | 0,1143 m | 0,2325 m |
| Norme vitesse bassin à 3,433 s | 0,0117 m/s | 0,0082 m/s |
| Norme vitesse CoM à 3,433 s | 0,0090 m/s | 0,0128 m/s |
| Charge maximale diagnostique des talons | 0 N | 0 N |
| Station debout avec critères de vitesse | 0 s | 0 s |

**La chute arrive 0,233 s plus tôt.** La hauteur résiduelle supérieure correspond à une autre posture au sol, pas à un relevage. À la dernière observation valide, les appuis chargés sont surtout bras/main gauches et pied gauche. Les charges publiées ne sont pas des forces continues garanties sur tout l’intervalle CCD.

**Arrêt physique à 3,45 s / 828 sous-pas exacts :** interpénétration `leftForefoot/rightFoot` de **5,0398 mm**, limite inchangée **5 mm**. Le noyau a arrêté l’histoire avant l’extension de la référence ; aucun seuil n’a été relâché. La séquence complète existe dans le candidat, mais n’a pas pu être exécutée jusqu’au bout. Le bassin dans le dernier état archivé est à 0,2304 m.

Décision : **rejeter ce candidat**. Ce résultat ne valide pas cette traduction des objectifs de transfert en références articulaires ; il ne prouve ni une impossibilité anatomique ni une cause unique. Aucun relevage depuis impact ni qualification des 24 cas. Production et récupération publique inchangées ; attribution inachevée conservée séparément et exclue du build.

Code exécuté : `47e856e20d97248646ab85d0a11d761d9688d365`. Deux tests purs ciblés passent, zéro physique dans les tests. Une seule histoire, zéro prévision, **5,055 s murales**. Réserve complète **7 000 sous-pas / 120 s** conservée, sans remboursement ni relance. Débits globaux : **153 508 / 300 000 sous-pas**, **466,328 / 7 200 s** ; préparation engagée **71 689 / 144 000 s**. Traces, compteurs et vidéo conservés ; aucun ancien registre réinitialisé.
