# Superviseur A36 — gel distinct de 7 100 secondes

`run_transfer.py` reprend le superviseur gelé à 7 200 secondes dans un nouveau dossier, avec une seule modification comportementale : un plafond global réduit à 7 100 secondes. Le nouveau `protocolId` est `observed-a36-single-transfer-and-continuation-7100-2026-10-15`. L’interface native `transfer-run-v1`, le budget de 131 096 sous-pas, l’unique histoire, le binaire physique et les preuves historiques sont préservés. Son import et son mode par défaut ne lancent aucun worker. Le manifeste et l’autorisation parent actualisés sont préparés séparément par le responsable de l’exécution ; ce code ne crée aucun accord ni claim réel.

Inspection utilisable dès que le manifeste et le binaire final existent :

```sh
python3 docs/evidence/coordination-transfer-native-7100-2026-10-15/run_transfer.py --manifest /chemin/absolu/manifest.json
```

Cette commande vérifie les fichiers et les prérequis sans fichier d’approbation, sans réservation physique, sans création de répertoire de sortie et sans processus enfant. Elle retourne `inspectionGrantsNativePermission: false`, `authorizationVerified: false` et `nativeSteps: 0`. Le binaire est lu pour son empreinte ; il n’est pas exécuté.

L’option `--execute` exige en plus `--authorization /chemin/absolu/authorization.json`. Cette autorisation doit être distincte, approuvée explicitement, datée, à usage unique, et liée au SHA-256 exact du manifeste. Il n’existe aucun accord approuvé fourni par le superviseur. Une inspection positive ne remplace pas cet accord.

Le manifeste a exactement les clés `schema`, `protocolId`, `authorizationId`, `binaryPath`, `outputPath`, `inputs`, `files`, `prerequisites`. Les constantes `MANIFEST_SCHEMA` et `PROTOCOL` donnent les valeurs attendues. L’identifiant doit commencer par `transfer-a36-distinct-` et contenir au moins 30 caractères. `outputPath` doit désigner un nouveau sous-répertoire direct du dossier de ce superviseur. Aucun autre répertoire ni nouvelle tentative après un claim ne sont admis.

`inputs` contient exactement `programPath`, `tasksPath`, `entryPath`, `prefixFramesPath`, tous absolus et canoniques. Les fichiers doivent être présents dans `files` avec leurs rôles respectifs `program`, `tasks`, `entry`, `prefix`. L’entrée doit être A36 et le préfixe doit contenir exactement 36 lignes archivées. La vérification détaillée de chaque ligne est également effectuée par le worker lors du futur rejeu compté.

Chaque élément de `files` contient exactement `path`, `sha256`, `role`. Aucun doublon de chemin ni lien symbolique n’est accepté. Tous les rôles suivants sont requis : `binary`, `code`, `program`, `tasks`, `entry`, `prefix`, `model`, `profile`, `criteria`, `validation`, `protocol`, `preparation`. Un même rôle peut contenir plusieurs fichiers, sauf `binary`, qui doit désigner exactement le binaire dédié déclaré. Les rôles sont distincts par chemin. Le rôle `code` doit contenir tout l’inventaire retourné par `required_code_paths()` : tous les fichiers Rust `.rs`, `Cargo.toml`, `Cargo.lock` hors `target`, ce superviseur et `scripts/run-coordination-numeric-parity.py`. D’autres sources/tests peuvent être gelés en supplément.

Le rôle `preparation` doit inclure le registre original `coordination-transfer-candidate-2026-10-13/preparation-reservation.json`, dont l’empreinte est fixée dans le code. Ses 14 preuves historiques — cinq registres clos, quatre binaires et cinq manifestes — sont vérifiées sans modification. Le binaire proposé doit être différent des binaires historiques protégés.

Les huit prérequis suivants doivent être vrais et justifiés par les preuves du manifeste :

- `static_tests_passed`
- `dedicated_release_built_without_execution`
- `unchanged_physical_contract`
- `archived_a36_prefix_available`
- `complete_decision_state_parity_reviewed`
- `single_process_single_thread`
- `full_continuation_and_stop_rules_frozen`
- `distinct_native_authorization_required`

Le superviseur ne prouve pas ces propriétés physiques à partir de ces booléens. Il fige la déclaration et ses pièces ; les essais et la revue doivent la justifier. La parité native et la réussite physique restent à observer sous un protocole distinct autorisé.

Lors d’une future exécution autorisée, un claim exclusif et deux réservations durables débitent immédiatement 52 + 131044 = 131096 sous-pas. La première réserve couvre le préfixe technique de 36 pas et quatre branches de parité de quatre pas. La seconde couvre une histoire réelle de 6244 pas au maximum, préfixe compris, et au plus 260 prévisions de 480 pas. Aucun remboursement ni renouvellement n’existe.

Le même chronomètre UTC/monotone borne démarrage, calcul et arrêt à 7100 secondes. Le démarrage est limité à 30 secondes. Les permis expirent sept secondes avant la limite globale ; cette réserve permet l’arrêt du worker. Le superviseur utilise les fonctions déjà revues d’inspection Linux, de tubes et d’arrêt de groupes de processus du superviseur numérique historique, sans appeler son ancienne fonction `run`, son budget ou son autorisation. Les signaux d’interruption arrêtent aussi un enfant créé pendant un constructeur interrompu, sans viser les enfants préexistants.

Après `ready` avec le PID et `protocol: transfer-run-v1`, les deux réservations sont liées à ce même PID avant l’unique requête. Le nombre de threads, l’absence d’enfants, l’identité de démarrage et l’empreinte de l’exécutable sont vérifiés avant et pendant l’opération. Tous les fichiers gelés sont revérifiés avant le démarrage, après `ready` avant l’envoi, après le résultat et lors de la fermeture. Le worker attend EOF après son résultat ; il ne traite aucune seconde opération.

Le registre conserve zéro pas effectué tant qu’aucune requête n’a pu être envoyée. Avant le premier octet d’envoi, il enregistre durablement l’incertitude sur les deux réserves complètes. Un reçu cohérent précise ensuite les comptes et les bornes `completed..admitted` ; un appel interrompu n’est pas présenté comme un compte exact. Après arrêt du processus, les journaux natifs `before-integration` / `after-integration` peuvent resserrer ces bornes. Un fichier absent, incomplet ou incohérent ne supprime jamais l’incertitude ; une dernière ligne partielle laisse conservativement un appel potentiel supplémentaire. Les événements du superviseur sont chaînés par SHA-256 et chaque modification est synchronisée sur disque avant l’action correspondante.

La vérification de cette variante est `deadline-validation.log` : 49 tests passent, dont 46 tests purs et trois tests de processus inertes. Les tests purs utilisent un worker mémoire et des accords temporaires marqués synthétiques ; `Popen` y est explicitement interdit. Les trois autres tests n’exécutent que de minuscules scripts Python temporaires pour vérifier tubes/EOF, arrêt forcé et interruption pendant un constructeur. Aucun binaire du projet, monde, FK ou intégration native n’est exécuté. Les accords synthétiques sont supprimés avec leurs répertoires temporaires et ne sont jamais utilisables pour le projet.

Les contrôles propres à la réduction vérifient les constantes exactes, le rejet d’un ancien accord à 7 200 secondes, la soustraction du temps de démarrage du temps disponible et les dernières échéances admissibles. Depuis l’origine monotone du registre : démarrage strictement avant 30 s, arrêt des permis natifs à 7 093 s, fin globale strictement avant 7 100 s. Les sept dernières secondes sont réservées à l’arrêt ; elles ne sont pas ajoutées aux 7 100 secondes. Une réception à l’échéance native ou une fermeture à l’échéance globale échoue. Les réserves sont toujours durables avant le démarrage, liées au PID avant envoi, sans remboursement ni renouvellement.

La validation physique reste ouverte : des empreintes, des commandes admissibles et des tests de supervision ne prouvent ni la reprise de charge, ni l’acquisition des talons, ni un relevage. Seul un futur protocole distinct, borné et autorisé pourra produire cette preuve.
