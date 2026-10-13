# Audit indépendant de l'échec d'environnement avant lancement

Lecture indépendante du 13 octobre 2026 après clôture du lot unique
`coordination-numeric-parity-2026-10-13T06:44Z`. Aucune réexécution du
superviseur, aucun worker natif lancé, aucune intégration et aucune modification
du code gelé ou des registres par cet audit. La seule écriture est cette note.

**Conclusion : le chemin d'échec observé précède la construction et le lancement
du worker natif. Il a donc exécuté zéro sous-pas natif. La parité physique n'a
pas été évaluée et reste inconnue.** Le registre brut conserve toutefois
`actualNativeStepsKnown: null` : l'audit n'en change pas la valeur et ne la
présente pas comme un reçu émis par le worker.

## Preuves conservées

Le registre `native-run/physical-ledger.json` porte `closed-failure`, une
réservation de 44 pas, zéro remboursement, `worker: null` et seulement les
événements de réservation et de clôture. Le début est
`2026-10-13T07:15:05.374339+00:00` ; la clôture est journalisée à
`07:15:05.402794+00:00`, pour une durée mesurée de `0.02844643699995686` seconde.
Le motif exact est :

```text
FileNotFoundError: [Errno 2] No such file or directory: '/proc/3658/task/3658/children'
```

`run-result.json` et `supervisor.stderr.log` concordent avec ce motif. Le
protocole `protocol.ndjson` contient zéro octet. `native-result.json`,
`native-trace.jsonl` et `worker.stderr.log` n'existent pas. Cette dernière
absence concorde avec un échec avant même le constructeur du worker : son
premier acte aurait créé `worker.stderr.log` avant `Popen`.

L'enveloppe SHA-256 du registre a été recalculée : valide. Les **294 fichiers
gelés** du manifeste et les **7 fichiers protégés** de la préparation
(six anciens registres et l'ancien binaire) ont été relus et leur SHA-256
comparé : tous inchangés. La réservation et la réclamation d'autorisation
restent consommées selon le protocole ; aucun remboursement ni nouvelle
admission n'est inféré de l'absence d'intégration.

| Pièce relue | SHA-256 |
| --- | --- |
| `scripts/run-coordination-numeric-parity.py` | `9d57bcefac46a1f769378e237edbcdf63a01a10204ba9e7cf6bfdf3daa513db5` |
| `native-run/physical-ledger.json` | `73b2e3bff52aadf0414be2f7a4a96fa4cd5da7b02de0945434e4f136bbf476a6` |
| `native-run/run-result.json` | `18773eabaa5b2a7d5cc78a8b1a59e1e84370c600f2d78ad8f5735d7f5a461183` |
| `manifest.json` | `40e492a29d723118cb26990b9dd2866d498d9a1a5d5ae815d7aae1d9d92a407a` |

## Chemin source qui établit l'absence de lancement natif

Dans le superviseur gelé :

1. `run`, ligne 395, construit le registre, réserve le lot et initialise
   ensuite `worker = None`.
2. L'entrée dans `hard_deadline` appelle `child_pids()` ligne 364, **avant**
   le `yield` de ce gestionnaire de contexte.
3. `child_pids`, lignes 261–262, lit précisément
   `/proc/{os.getpid()}/task/{os.getpid()}/children`. Le PID de cette lecture
   est celui du superviseur, pas un PID de worker.
4. L'exception à cette lecture empêche d'entrer dans le corps du contexte ;
   l'affectation `worker = worker_factory(...)`, ligne 416, n'est pas atteinte.
   Le constructeur `NativeWorker`, lignes 277–281, ne crée donc ni son journal
   stderr ni son sous-processus natif.
5. L'exception est capturée par `run`, qui conserve `worker=None`, contrôle
   les empreintes finales et clôt le registre sans remboursement.

L'absence d'un reçu natif serait insuffisante seule pour exclure un lancement.
Ici, son absence est corroborée par l'emplacement précis de l'échec dans le
code gelé, les événements du registre et l'absence du fichier que le
constructeur aurait créé avant tout sous-processus.

## Vérification de l'environnement en lecture seule

Une sonde Python distincte a lu ses propres métadonnées, sans lancer de worker,
installer d'alarme ou envoyer de signal. Son PID vivant était `3747` :

- `/proc/self/status` et `/proc/3747/status` sont lisibles et rapportent
  `Pid: 3747`, `Tgid: 3747`, `Threads: 1`.
- `/proc/3747/task` contient `3747`, et `/proc/3747/task/3747` est lisible.
- `/proc/3747/task/3747/children` et
  `/proc/self/task/3747/children` échouent tous deux avec `ENOENT`. La liste
  du répertoire du thread ne contient effectivement pas `children`.
- Sur une seconde sonde vivante, les lectures de `status`, `stat`, le lien
  `exe` et l'ouverture binaire de `/proc/<pid>/exe` réussissent ; les champs
  de nombre de threads et d'identité temporelle utilisés par le superviseur
  sont disponibles.
- La consultation de l'alarme courante, sans la modifier, rapporte `(0, 0)`.
- `/proc/config.gz` est lisible et contient
  `# CONFIG_CHECKPOINT_RESTORE is not set`. Cette observation est cohérente
  avec l'absence de l'interface attendue ; la conclusion opérationnelle
  repose sur l'absence directement vérifiée de `children`, sans attribuer
  une cause supplémentaire non démontrée.

Ce contrôle sur un processus vivant exclut l'explication limitée selon
laquelle seul le PID `3658` aurait disparu avant une lecture après coup.
La capacité manquante concerne bien l'environnement actuel.

## Précondition omise et correction future envisageable

`preflight`, lignes 143–198, vérifie les entrées, empreintes, rôles, flags de
prérequis, chemins et fichiers protégés. Il ne vérifie pas la disponibilité
des interfaces `/proc` dont dépendent `child_pids` et `inspect_process`.
La réservation durable précède donc le premier accès à une interface pourtant
indispensable au superviseur. Les tests avec faux workers injectent une alarme
inerte, et les tests du watchdog remplacent `child_pids` par des ensembles
synthétiques : ils valident la logique de fermeture mais pas cette capacité
réelle de l'environnement. La revue préalable n'avait pas identifié cette
précondition manquante ; son avis favorable ne constituait pas une preuve de
compatibilité du superviseur avec `/proc` dans cet environnement.

Une correction future devrait ajouter **avant toute réclamation d'autorisation
et réservation** un contrôle en lecture seule des capacités indispensables :
lecture effective et parsing de `children` sur le processus courant, ainsi que
des métadonnées et de l'exécutable utilisés pour l'identité. L'absence d'une
capacité doit refuser ce futur préflight avec un motif explicite. Une lecture
qui échoue ne doit jamais être assimilée à un ensemble vide d'enfants.

Ce contrôle détecterait le problème plus tôt ; il ne rendrait pas cette
interface disponible ici. Une exécution ultérieure requerrait soit un
environnement où les capacités requises sont présentes, soit une nouvelle
conception du mécanisme de supervision, avec preuve équivalente de nettoyage
des processus y compris pendant la construction, revue et tests dédiés avant
nouveau gel. Il ne suffit pas de supprimer la vérification ni le watchdog.

**Aucune de ces modifications n'est implémentée dans ce lot clos. Aucun
contournement, nouvel essai, remboursement ou renouvellement n'est effectué.**
Toute future exécution relève d'une décision et d'une autorisation distinctes.
