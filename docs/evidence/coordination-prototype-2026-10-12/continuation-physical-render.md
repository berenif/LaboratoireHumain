# Rendu du protocole physique clos

[Image des jalons A et de la dernière prévision B](continuation-physical-results.png) · [Vidéo de A, 0–4 s](continuation-A-actual-history.mp4) · [Reçu SHA-256](continuation-physical-render-receipt.json)

L'image juxtapose les états réellement exécutés de **A** à 0, 1, 2, 3 et 4 secondes, puis la dernière copie prédite de **B**, `preview-0-3`, au pas 12 (0,05 s). La vidéo présente uniquement A : 81 poses archivées espacées de 12 pas natifs, à 20 images/s, soit 4,05 s avec la dernière image conservée. Les silhouettes utilisent le maillage canonique placé aux 25 poses de corps déjà enregistrées ; elles ne reconstruisent aucune chaîne articulaire. Les points jaunes reprennent le CoM mesuré, les points verts les contacts courants archivés de charge verticale au moins 3 N. L'état initial ne possède pas ces mesures et n'en invente aucune.

**A échoue au jalon local.** Les 960 pas réels (4 s à 240 Hz) n'ont produit aucune séquence stable : maximum 0/132 pas et `local_success=false`, aucune récupération observée. L'image montre la chute entre 1 et 2 s ; à 4 s le bassin est à 0,133282 m. Les talons ont une charge nulle et la charge courante hors pieds atteint 489,080 N dans le dernier diagnostic. Ce résultat ne valide ni l'acquisition physique des talons ni une continuation complète vers debout.

**B est refusé avant action réelle.** Ses quatre prévisions contiennent chacune 12 pas ; `actualNativeSteps=0`. La raison enregistrée est « All four native predictions refused; no substitute proposal ». Le panneau B représente la dernière de ces prévisions, et ne constitue pas une cinquième exécution ni une continuation de A.

Le [script](continuation_physical_render.py) et l'[extrait compact des poses](continuation-physical-render-states.json.gz) permettent de reproduire les visuels avec Python, NumPy, Matplotlib et FFmpeg déjà disponibles :

```sh
python3 docs/evidence/coordination-prototype-2026-10-12/continuation_physical_render.py
```

L'extrait reprend les poses, mesures et diagnostics sélectionnés de `native-run/001-A.json`, `001-A.states.jsonl`, `002-B.json` et `002-B.states.jsonl`. Le reçu conserve leurs empreintes, celle du maillage canonique et celles du script, de l'extrait et des deux visuels. Le script réutilise l'extrait s'il existe et refuse un maillage dont l'empreinte a changé. Les fichiers de cette livraison pèsent ensemble moins de 1 Mo ; les traces natives originales restent intactes.

Cette opération de rendu a effectué **0 pas physique, 0 FK et 0 nouvelle requête géométrique**. Elle ne modifie ni les programmes gelés ni le résultat du protocole clos.
