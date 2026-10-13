# Audit du lot distinct de parité native — 13 octobre 2026

L'unique invocation autorisée a échoué **avant le lancement du worker natif**.
Le registre est clos, les 44 sous-pas restent réservés sans remboursement et
aucun réessai n'est autorisé par ce lot. Il n'y a aucun résultat nouveau de
parité dynamique ni de relevage.

Cet audit lit uniquement les sources figées, les reçus et leurs empreintes.
Aucun worker, test natif, compilation, FK ou intégration n'a été exécuté pour
l'audit. Aucun fichier figé ni registre n'a été modifié ; cette note est un
nouveau livrable hors manifeste.

## Zéro lancement et zéro intégration pour cette invocation

Le motif conservé dans `native-run/physical-ledger.json` et `run-result.json`
est :

```text
FileNotFoundError: [Errno 2] No such file or directory:
'/proc/3658/task/3658/children'
```

Dans le runner figé `scripts/run-coordination-numeric-parity.py`,
`hard_deadline()` appelle `child_pids()` à la ligne 364, **avant** son `yield`
ligne 380. L'entrée dans ce contexte, ligne 413, précède l'appel à
`worker_factory` ligne 416. Le lancement `subprocess.Popen` se trouve dans le
constructeur `NativeWorker`, ligne 281. L'exception enregistrée interrompt
donc l'exécution avant ce constructeur et avant toute possibilité d'envoyer un
permis natif.

Les pièces observées concordent avec ce chemin :

- `worker: null` ; aucun événement d'association de PID ou d'envoi de permis.
- Deux événements seulement : `reserve-entire-batch-before-spawn`, puis
  `close-without-refund`.
- `protocol.ndjson` existe et mesure exactement **0 octet**.
- `worker.stderr.log` est absent ; ce fichier aurait été ouvert dès la ligne
  277 du constructeur, avant `Popen`.
- `native-result.json` et `native-trace.jsonl` sont absents.

Ces éléments établissent zéro appel au constructeur/lancement natif et donc
**zéro sous-pas natif exécuté par cette invocation**. Le compteur officiel
`actualNativeStepsKnown` reste toutefois `null`, car le runner n'a reçu aucun
reçu natif. Cet audit distingue cette valeur conservée de sa conclusion fondée
sur le chemin d'arrêt ; il ne remplace pas `null` par zéro dans le registre.

## Comptabilité et intégrité

| Élément | Constat |
| --- | --- |
| Autorisation | `coordination-numeric-parity-2026-10-13T06:44Z` |
| Statut terminal | `closed-failure` |
| Réservation définitive | 44 sous-pas ; remboursement 0 |
| Début | `2026-10-13T07:15:05.374339+00:00` |
| Clôture | `2026-10-13T07:15:05.402794+00:00` |
| Temps monotone enregistré | `0.02844643699995686 s`, inférieur aux 120 s |
| Compteur natif reçu | `null` ; aucune réponse de worker |
| Vérification terminale enregistrée | `terminalFrozenFilesVerified: true` |
| Autorisation d'histoire complète / qualification | Toutes deux `false` |

Les empreintes ont été recalculées avec Python standard en lecture seule :

- Manifeste :
  `40e492a29d723118cb26990b9dd2866d498d9a1a5d5ae815d7aae1d9d92a407a`.
  **294 fichiers sur 294** correspondent, sans absence ni différence.
- Nouveau binaire :
  `9e125785fad85b0befc9373c3116d66aa32e8050a998ce08974b0da29d2760cc`.
- Les **six anciens registres et l'ancien binaire** correspondent tous aux
  sept empreintes inscrites avant exécution dans `preparation.json`.
  Le registre physique du 12 octobre conserve notamment
  `2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07` ;
  l'ancien binaire conserve
  `1bb4b91ac8ebfbe382d9525e524b85040ca9890bc90a0ef0467f1ef2b64c8b57`.
- L'empreinte de l'enveloppe du nouveau registre correspond exactement à la
  sérialisation canonique de son `payload`.
- `run-result.json.budget` est identique à ce `payload` ; le claim reprend
  exactement autorisation, réservation, plafond, début, échéance et manifeste.

| Reçu post-exécution | SHA-256 du fichier |
| --- | --- |
| `authorization-claim.json` | `475daae5e690f18cc6624cc280ff97a118374319a0d20687c01197e19b2d2078` |
| `native-run/physical-ledger.json` | `73b2e3bff52aadf0414be2f7a4a96fa4cd5da7b02de0945434e4f136bbf476a6` |
| `native-run/run-result.json` | `18773eabaa5b2a7d5cc78a8b1a59e1e84370c600f2d78ad8f5735d7f5a461183` |
| `native-run/protocol.ndjson` vide | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |

## Cause et limite de la préparation

Le recensement des enfants du processus parent dépend d'un chemin `/proc`
construit à partir de `os.getpid()`. Ce chemin était indisponible dans
l'environnement de cette invocation. La cause démontrée est cette dépendance
d'environnement non satisfaite ; les pièces ne suffisent pas à identifier
précisément une politique de montage, un espace de PID ou une virtualisation.

La préparation statique avait vérifié les fichiers et prérequis déclarés,
mais elle n'avait pas exercé cette lecture réelle de `/proc` avant réservation.
Les tests purs du watchdog utilisaient des PID et signaux factices. Ils
vérifiaient sa logique sans établir la disponibilité de cette interface dans
l'environnement d'exécution. Le prérequis manquant relève donc du lanceur et
de son admission environnementale ; il ne constitue pas un refus de la
correction numérique, du support physique ou de la parité native.

Le mécanisme a conservé sa fermeture en cas d'erreur : aucun lancement après
le refus, aucune remise à disposition des 44 pas, claim durable conservé et
anciens registres intacts. Un éventuel correctif de ce prérequis ne serait pas
un motif pour reprendre ce lot clos ; toute nouvelle tentative exige une
autorisation distincte. Aucun contournement ni changement du code figé n'a
été effectué dans cet audit.
