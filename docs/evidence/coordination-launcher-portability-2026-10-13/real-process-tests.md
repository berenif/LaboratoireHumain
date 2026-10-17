# Essais du lanceur avec processus C factices

**16/16 essais réussis ; zéro intégration physique ; aucun worker Rust exécuté.**
Le [résumé, avec les SHA des sources](e2e/summary.json), porte explicitement
`physicalIntegrations: 0` et `protocolCountsAreSynthetic: true`.
Les nombres « 12 » et « 44 » des reçus du protocole sont entièrement fabriqués.
Les claims, manifests et fichiers appelés `physical-ledger.json` appartiennent
exclusivement à l'autorisation `SYNTHETIC-launcher-portability-physical-integrations-zero`.
Aucun ancien registre physique n'a été réécrit.

Le [worker C](../../../scripts/fixtures/coordination-fake-worker.c) ne contient
ni simulation, ni appel Rust/Rapier. Le [harness](../../../scripts/test-coordination-launcher-portability.py)
le compile avec `cc -std=c11 -O2 -Wall -Wextra -Werror -pthread`, puis utilise
les véritables `NativeWorker`, pipes, inspecteur `/proc`, alarmes et handlers du
lanceur. Aucun processus n'est simulé par un mock. Seuls l'identifiant synthétique
d'autorisation et le délai de **2 secondes** sont substitués dans les processus
de test ; le fichier du lanceur reste inchangé.

| Famille | Résultat observé |
|---|---|
| Succès et double claim | Reçu factice 44 accepté, EOF et sortie normale ; second claim refusé sans second lancement. |
| Refus partiel | Reçu factice 12 conservé ; fermeture en échec, remboursement nul. |
| Crash avant `ready` et après permis | Fermeture en échec, quantité inconnue sans reçu valide, processus arrêté. |
| Délai avant `ready`, après permis et après résultat | Arrêt du groupe à la même échéance de test ; pas de prolongation pour attendre EOF. |
| SIGTERM et SIGINT après permis | Arrêt et fermeture en échec ; handlers restaurés et alarme désarmée. |
| SIGTERM pendant construction | Le vrai `NativeWorker` est créé puis retenu avant affectation ; le scan récupère et arrête le processus même sans référence retournée. |
| Enfant et second thread | Refus avant envoi du permis. |
| Enfant préexistant indépendant | Préservé par le nettoyage du lanceur, puis arrêté et récolté par le harness. |
| Préflight sans exécution | Aucun processus, claim ou répertoire de sortie créé. |
| Capacités absentes et alarme préexistante | Refus avant claim, sortie ou lancement. La capacité absente utilise un répertoire `PROC` vide réel de test, sans feindre un EACCES de l'hôte. |

Sur **cet hôte**, `/proc/<pid>/task/<pid>/children` est réellement absent,
alors que `/proc/<pid>/stat` existe : le succès et le refus de l'enfant exercent
donc réellement le remplacement par le scan des PPid. Le délai externe observé
des cas à échéance est d'environ **2,06–2,07 s**, incluant le démarrage Python
du harness et la fermeture du reçu, avec un permis interne de 2 s.

Le [recensement final](e2e/cleanup-audit.json) confirme que les **15 PID factices**
n'existent plus dans `/proc`. Un enfant terminé était initialement zombie après
l'arrêt du groupe ; le harness, déclaré subreaper, l'a récolté. Aucun processus
actif n'a survécu au retour du lanceur. Les reçus individuels conservent les PID,
délais, motifs exacts, états immédiatement après retour et copies des artefacts
du protocole. Leurs chemins `/tmp` sont historiques : les entrées et le binaire
factices temporaires ont ensuite été supprimés.

SHA-256 du lanceur testé, inchangé pendant toute la suite :
`e5af00efb904bc203704bc88514a8e8a803dad36b3bca669a017c90ca24f97da`.
Les commandes et SHA du binaire/source C sont dans [fixture-build.json](e2e/fixture-build.json).

Pour reproduire uniquement ces essais factices, choisir un nouveau répertoire :

```sh
python3 scripts/test-coordination-launcher-portability.py --output-dir /tmp/launcher-portability-new-evidence
```

Cette preuve couvre le protocole et la supervision des processus. Elle ne
qualifie aucune commande motrice, parité physique ou continuation et ne rouvre
aucun budget natif.
