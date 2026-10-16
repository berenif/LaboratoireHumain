# WEAK admis, SLOW réellement commandé — essai borné à 4 s

**La chaîne prévision → référence → commandes SLOW fonctionne sur cet état précis. Aucun relevage n'est démontré.** La reprise corrigée atteint 4 s physiques sans refus d'intégrité. Le corps est encore couché à l'arrêt, après seulement 0,39875 s de SLOW réel.

## Amendement au même budget

Le parent a explicitement confirmé que l'autorisation « Go » de 19:09:59 UTC porte sur **180 s de calcul réel cumulé**, et couvre la reprise après le défaut de lanceur. Cet [amendement appendu séparément](getup-penetration-retry-2026-10-16.json) remplace la règle documentaire précédente qui interdisait de réutiliser le solde de cette réservation. Il n'ajoute aucun budget. Le premier registre et le registre historique sont conservés identiques par empreinte.

La reprise disposait donc de **8 180 sous-pas et 178,374936765 s**, après déduction des 20 sous-pas et 1,625063235 s du lancement interrompu. Les gardes vérifient le compte natif depuis l'horloge physique, sa monotonie, la cadence et le plafond cumulé. Avant chaque tick réel, le lanceur vérifie aussi que toute prévision encore possible de 5 000 sous-pas rentrerait dans le reliquat. Les tests couvrent le refus d'une remise à zéro des dépenses ou d'une augmentation de ce plafond.

| Lancement | Sous-pas réels | Sous-pas prévisionnels | Calcul mural du lancement |
| --- | ---: | ---: | ---: |
| Défaut de lanceur conservé | 20 | 0 | 1,625063235 s |
| Reprise corrigée | 3 200 | 540 | 36,929941929 s |
| **Cumul du même budget** | **3 220** | **540** | **38,555005164 s** |

Le total est **3 760 / 8 200 sous-pas**. Les 299 998 sous-pas réservés dans l'ancien budget de 300 000 restent inchangés ; ses deux sous-pas restants ne sont pas utilisés. Aucun nouveau réglage physique, variante, entraînement, changement de masse, friction, collision, anatomie ou couple n'est introduit. La reprise depuis le même constructeur initial est explicite ; aucun reset ni remplacement d'état n'intervient dans l'histoire. L'essai s'arrête à son plafond physique de 4 s, sans prolongation.

## Résultat causal sur la prévision

L'impact demandé à 2 s est effectivement mesuré à 2,26125 s. WEAK démarre depuis l'état réel à **3,60125 s**, en conservant exactement ses poses et son mouvement dans une copie du monde. La prévision atteint son terminal source en **27 décisions, 540 sous-pas / 0,675 s prédite**. La tête prédite atteint 1,5015 m ; ce terminal de hauteur ne constitue pas une station debout stable qualifiée. La référence contient 55 trames, cible finale comprise, à vitesse source 0,5.

La comparaison avec l'essai historique à 800 Hz confirme le même état initial des 25 corps, les mêmes 12 premières observations/actions/poses avant décision et les mêmes 11 premières trames physiques complètes. L'ancien essai s'arrêtait à **232 sous-pas**, pour une pénétration cuisse/pied de **6,277 mm** dépassant l'ancien seuil de 5 mm. La nouvelle prévision poursuit jusqu'au terminal, avec un maximum propre de **6,409 mm**, toujours mesuré. Cela isole l'effet de ce refus sur cette prévision ; cela ne démontre ni la qualité générale des prévisions ni la réussite de SLOW.

## Commandes et mouvement réels

SLOW produit **16 décisions**, aux sous-pas réels 2 881 à 3 181, chacune suivie d'intégrations du monde réel. Les 16 diagnostics de fin de tick correspondent indépendamment au PD source, aux résidus et à la référence archivés : écart maximal de commande inférieur à **3 × 10⁻⁸**, écart de couple demandé inférieur à **6 × 10⁻⁵ Nm**. Les limites de commande et le budget de 920 Nm sont respectés ; aucune force ou couple d'assistance n'est appliqué au bassin. Ce contrôle vérifie la livraison de la commande au chemin de dynamique, pas sa réussite physique.

La tête réelle passe de **0,0895 m au démarrage de SLOW à 0,1465 m** à 4 s. Le mouvement articulaire est réel, mais le corps reste couché. La stabilité debout mesurée est **0 s**, les récupérations restent à zéro et aucune qualification n'est revendiquée. La référence complète n'a pas fini d'être suivie à cette échéance : sa dernière cible ne serait atteinte qu'aux environs de 4,95125 s. L'essai ne tranche donc pas l'issue d'une histoire complète.

Les maxima mesurés dans l'histoire réelle sont 0,548 mm entre corps et 7,750 mm contre le sol. Les collisions et leur réponse dynamique restent actives ; la pénétration reste diagnostic-only. Les autres gardes d'intégrité restent inchangées. La récupération publique reste désactivée par défaut.

## Preuves et vidéo

Les [161 états réels compressés](getup-penetration-retry-2026-10-16/trace.ndjson.gz), le [rapport de prévision et commandes](getup-penetration-retry-2026-10-16/reference-report.json.gz), le [résultat brut](getup-penetration-retry-2026-10-16/result.json) et l'[audit](getup-penetration-retry-2026-10-16/postrun-audit.json) sont conservés. Le registre contient les empreintes compressées et décompressées, les 117 entrées figées et les contrôles de conservation des anciens registres. **61 tests JavaScript et 5 tests Python d'admission passent**, sans intégration supplémentaire. Aucun changement Rust/WASM de production n'a nécessité de nouveau build ; aucune suite physique supplémentaire n'a été exécutée.

La vidéo `LaboratoireHumain-Rapier-WEAK-SLOW-4s-2026-10-16.mp4` montre les poses réelles archivées et un graphique séparé de la prévision. Elle dure 5 s, dont 1 s de maintien explicite de la pose finale à 4 s physiques. Validation : H.264, 1280 × 720, 100 images à 20 fps, décodage complet et dernière image inspectée. [Validation et sauvegarde Library confirmée](getup-penetration-retry-2026-10-16/video-validation.json), version 0. Audit et rendu n'intègrent aucune physique ; leurs temps sont journalisés séparément.

La vérification bornée demandée est terminée. Le prochain résultat nécessaire pour conclure serait une histoire complète jusqu'à la station debout stable, puis une qualification distincte des 24 cas. Aucune histoire supplémentaire n'est lancée ici.
