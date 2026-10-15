# Rapprochement indépendant du budget global — 15 octobre 2026

**Le candidat tient dans le solde de sous-pas, mais son protocole figé à 7 200 secondes ne tient pas dans le solde mural global. Aucun lancement ni nouvelle réservation native n’est admissible sous ce gel.** Audit en lecture seule ; seuls ces deux nouveaux fichiers de décision sont écrits. Zéro simulation, monde, FK, build ou test.

L’origine de cette famille est le 12 octobre 2026 à 20:14 UTC, selon `coordination-prototype-2026-10-12/preparation-start.json` : 300 000 sous-pas et 7 200 secondes de calcul physique. Les quatre emplacements d’histoire historiques sont utilisés et leurs lots restent clos. La nouvelle instruction autorise distinctement une continuation, sous réserve du solde global ; elle ne remet aucun ancien emplacement ni permis en circulation.

## Sous-pas : réservations conservées intégralement

| Registre clos | Débit natif | Réalisé connu au registre |
| --- | ---: | ---: |
| Prototype initial, `budget/ledger.json` | 3 856 | 1 024 |
| Parité initiale, `native-run/physical-ledger.json` | 44 | `null` |
| Renouvellement de parité | 44 | `null` |
| Correctif de sérialisation | 44 | 44 |
| Dernière paire locale, `budget/ledger.json` | 3 856 | 1 040 |
| **Total** | **7 844** | **2 108 connus, total exact non certifié** |

Pour les deux registres A/B, la somme porte sur `payload.reservations[*].nativeSteps`, confirmée par `payload.events[-1].counters.reservedNativeSteps`. Pour les trois lots techniques, elle porte sur `payload.reservedNativeSteps`. Les champs connus sont respectivement `actualNativeSteps` des réservations et `payload.actualNativeStepsKnown`. Le JSON annexe donne chaque chemin et son empreinte relue.

Le rapprochement des reçus externes borne les intégrations historiques à 2 108–2 109 : premier lot de parité arrêté avant le lancement, renouvellement borné à 0–1 appel par son audit. Cette borne n’est pas une mesure native exacte et ne remplace aucun `null`. Les deux histoires B ont intégré zéro pas réel d’après leurs reçus externes, mais conservent leurs 960 pas et leurs emplacements chacun.

Solde : **300 000 − 7 844 = 292 156 sous-pas**. Besoin candidat : **131 096**. Cumul projeté : **138 940**, solde projeté **161 060**. Les anciens 82 724 étaient une projection avant lancement de la paire, non une réservation de tous ses appels : seuls les permis effectivement inscrits, 3 856, sont débités. Cela ne rembourse aucun permis. Les réservations internes aux ticks sont déjà comprises dans les permis parents.

## Temps mural : marge insuffisante pour le protocole figé

| Fenêtre historique documentée | Secondes |
| --- | ---: |
| Prototype : `physics.startedUtc` → événement final | 7,930429 |
| Parité : `elapsedWallSeconds` | 0,028446437 |
| Renouvellement : `elapsedWallSeconds` | 0,193373985 |
| Sérialisation : `elapsedWallSeconds` | 1,477786572 |
| Paire locale : `physics.startedUtc` → événement final | 12,321321 |
| **Somme documentée** | **21,951356994** |

Cette somme n’inclut pas toutes les phases historiques de démarrage et de récupération du PID. Le solde maximal arithmétique, avant ces phases non mesurées, est donc déjà inférieur à **7 178,048643006 secondes**. Il ne peut être présenté comme un solde exact utilisable.

La paire locale donne un début de supervision monotone de 15 946,150786894 et une première réservation à 15 946,230312276 : écart de **0,079525382 seconde**, donc environ **12,400846382 secondes** du début de supervision à la fermeture du registre. `run-result.json` est écrit après `worker.stop`, mais n’archive pas l’horodatage monotone de récupération finale ; la durée exacte complète reste inconnue. Le reçu `process-supervision.json` conserve les phases bornées 30 + 7 100 + 7. Son plafond de 7 137 secondes n’est pas une mesure de temps consommé.

Pour une planification conservatrice cohérente avec l’audit cumulatif précédent, on ajoute les allocations 30 secondes de démarrage et 7 secondes d’arrêt pour chacun des deux lancements A/B, soit **74 secondes**. Débit de planification : **95,951356994 secondes** ; solde conservateur : **7 104,048643006 secondes**. Ce sont des allocations prudentes pour des phases non entièrement mesurées, pas une durée passée certifiée. Les maxima historiques des lots techniques et de la paire ne sont pas des réservations natives additionnelles ; on conserve leurs temps inscrits et les allocations explicites, sans confondre délai autorisé et durée réalisée.

Le runner candidat `run_transfer.py` fixe `WALL_SECONDS = 7200` (ligne 47), impose exactement `LIMITS` à l’autorisation (lignes 116–118), puis dérive ses échéances de cette constante (lignes 203 et 213). Il n’expose aucune réduction de durée à l’exécution. Le manifeste autorisé `4f3708fd50cd4dabf473b7a6bff528ee11ad798cf25dfbeaf0f6141c156818f7` et le binaire `63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5` ne doivent donc pas être contournés par une modification ou un wrapper improvisé. Même le solde maximal optimiste exclut 7 200 secondes nouvelles.

## Périmètre et conclusion

Les registres `coordination-launcher-portability-2026-10-13/e2e` sont des processus C factices, explicitement sans physique. `foot-coordination-2026-10-12` relève de l’autorisation distincte antérieure de 11:39 UTC : ses 12 480 pas réservés et 6 235 connus restent dans son propre registre. `intermediate-transfer-prep-2026-10-12` précède aussi l’origine et déclare zéro pas nouveau. Les sous-registres de continuation géométrique, tests purs et lectures/rendus d’archives n’ajoutent pas d’intégrations.

**Décision : arrêt avant exécution.** Les nombres sont suffisants pour établir l’incompatibilité du délai figé avec le plafond global, sans estimer un succès physique, consommer un permis ou rouvrir une famille. Une correction du protocole temporel exigerait un nouveau gel cohérent et son admission explicite par le parent ; cet audit ne la réalise pas. Aucune réservation de préparation supplémentaire n’est créée.
