# Supervision du nouveau lot local A/B — préparation uniquement

Le runner utilise les primitives déjà vérifiées de `scripts/run-coordination-numeric-parity.py`, importées sans modifier ce fichier historique. Ce fichier doit donc figurer dans le gel de code du nouveau lot.

- Le worker démarre dans sa propre session et son propre groupe de processus. Les boucles d’envoi et d’attente vérifient l’identité du processus, le hash du binaire, un unique thread, l’absence d’enfant et le groupe privé. Un écart ferme le protocole.
- Le minuteur système indépendant (`ITIMER_REAL`) est armé avant la construction du worker. `SIGALRM`, `SIGINT` et `SIGTERM` tuent le groupe ; le recensement par `/proc/*/stat` couvre aussi une interruption de constructeur avant affectation de la référence Python du worker.
- Pour `local-pair`, l’échéance externe vaut **30 + 7 100 + 7 = 7 137 secondes**. La borne physique du registre reste 7 100 secondes. Le reçu `process-supervision.json` est écrit avant tout démarrage de worker. La sortie normale attend au plus deux secondes puis tue le groupe ; la phase de récupération du PID attend au plus une seconde après SIGKILL. Une erreur tue immédiatement le groupe et ne transmet aucune nouvelle permission physique.
- Avant toute comptabilisation, le fichier JSON de preuve doit annoncer exactement le même `actualNativeSteps` entier et le même objet `outcome` que le message NDJSON. Entier/flottant ou booléen/entier ne sont pas interchangeables. Une contradiction conserve la réservation et ne déclare aucun pas contradictoire comme exact.
- Les protocoles local A/B et historique gardent leurs ordres d’histoires et leurs critères physiques. Les faux workers ont été adaptés pour écrire des preuves cohérentes, sans simuler de monde.

Validation : **43 tests purs** du runner, **47 tests purs** du superviseur historique et **6 essais OS avec programme C inerte** passent. Les six cas réels vérifient la sortie normale, l’alarme, SIGTERM, l’alarme en constructeur, le refus d’un thread supplémentaire et le refus d’un enfant. Ils ont pris environ 0,677 seconde au total ; aucun descendant ne reste et **zéro sous-pas natif, zéro FK et zéro simulation** ont été exécutés. Le programme de test ne contient que lecture de pipe, attentes, un thread ou un enfant inerte selon le cas.

Les journaux et les hashes des sources validées figurent dans `process-supervision-validation.json`. Ces tests établissent les mécanismes de protocole et de processus ; ils ne constituent pas une preuve de faisabilité physique du recentrage ou du relevage.
