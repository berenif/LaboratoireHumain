# Parité native renouvelée : refus de sérialisation

Autorisation distincte du propriétaire le 13 octobre 2026 à 10:06 UTC,
message `Sentinel_1d94b2e17ea481919720d6494ad746fc`. Base distante vérifiée
`122d8eb6aef675fa2a7aed551a057ca298665789`. Gel publié et vérifié au commit
`334441445a9f5d97f18aa5d2956658685daf9e9a` ; binaire natif inchangé.

Le lanceur corrigé a réellement créé le worker, vérifié son identité et son
thread unique, puis envoyé l’unique permis. Le lot a commencé à 10:09:55,994 UTC
et s’est fermé en échec après **0,193 s**, sous son plafond de 120 s.
**44 sous-pas réservés, aucun remboursement, aucune reprise de ce registre.**

Le worker a paniqué dans `common_state`, ligne 408 du code figé, avec l’erreur
JSON `number out of range`. Une ligne complète d’état initial est archivée ;
le fichier de résultat final est vide. Le compteur officiel reste donc
`actualNativeStepsKnown: null` : ce résultat ne doit pas être décrit comme
zéro intégration. Le décompte déduit du chemin de code doit rester distinct
d’un reçu natif absent. Aucun résultat de parité ni de relevage n’est acquis.

La conversion de `PhysicsWorld` en `serde_json::Value` refuse les entiers
`u128` dépassant `u64::MAX`, présents dans les masques internes Rapier.
Le défaut porte sur le journal d’observation, sans démontrer de défaut
anatomique ni d’insuffisance du contrôleur. Une correction pure de la
sérialisation exacte et du reçu d’erreur est préparée séparément. Elle ne change
ni gains, caps, masses, friction, anatomie, seuils ou trajectoire.

Les [registres](evidence/coordination-native-parity-renewal-2026-10-13/native-run/physical-ledger.json),
le [stderr du worker](evidence/coordination-native-parity-renewal-2026-10-13/native-run/worker.stderr.log)
et les [archives sans perte](evidence/coordination-native-parity-renewal-2026-10-13/archive-index.json)
sont conservés. Les 308 fichiers gelés et les sept pièces protégées ont été
vérifiés après fermeture, avant la correction ultérieure. Le lot précédent
à zéro intégration reste inchangé et clos.

Le propriétaire a autorisé les corrections et vérifications techniques
ordinaires sans questions répétées. Le prochain lot remplace ce refus logiciel
avec **44 sous-pas maximum toutes branches comprises et 120 s**, dans un
registre distinct annoncé avant lancement. Ce remplacement est limité à une
vérification identique après tests purs ; aucun essai de relevage long ni
variante physique n’est lancé dans ce cadre.
