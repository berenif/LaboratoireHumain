# Reprise du 15 octobre : budget et conservation

La reprise est opérationnelle, sans réinitialiser les compteurs ni les échéances. La réservation de préparation est limitée à **09:27:54–12:27:54 UTC le 15 octobre 2026**, soit **10 800 s** engagées en plus des **46 489 s** déjà débitées. Le total engagé atteint **57 289 / 144 000 s**, avec **86 711 s** disponibles avant une autre réservation explicite. L'échéance globale reste le **17 octobre 2026 à 20:14:00 UTC**.

Ces valeurs sont des **débits conservateurs réservés**, pas une mesure exacte du temps de travail actif. La coupure de crédits n'ajoute aucune seconde de travail exécuté ; les débits déjà engagés ne sont pas remboursés. La nouvelle fenêtre découle de l'instruction explicite du propriétaire de continuer après rétablissement des crédits. Elle ne prolonge aucune fenêtre expirée et n'autorise aucun renouvellement physique.

La [réservation de préparation](preparation-reservation.json), SHA256 `c49f243654d69e00804045eaac409aa54cb8839f8ef3b0db28a8c2b86d8ea8e5`, autorise uniquement la revue, l'implémentation, la compilation et les tests purs sur archives. **Zéro nouveau sous-pas natif, monde, FK ou assemblage géométrique n'est autorisé par cette réservation.** Une éventuelle autre question de FK resterait soumise à sa réservation spécifique préalable, au maximum 500 poses et 30 minutes, dans le budget global inchangé.

## Travail récupéré

L'audit du **2026-10-15T09:32:41.057737+00:00** confirme les **48 fichiers** du snapshot `/workspace/lh-resume-2026-10-15/surviving-work.json` et leurs octets dans `surviving-work.tar.gz`. L'empreinte du tar est `9e6888b3177aa8601ef82cee8df733bd2f9484f43eb9d8fb68e00093b9ecace8`. Le snapshot, créé à `2026-10-15T09:28:43.276569+00:00`, indique le HEAD `bead1a2b8b7b838a828203ef4572b853be6cfc9e`. Il conserve le travail non commité après le dernier jalon annoncé et permet sa récupération sans écrasement. Les notes, résultats de tests purs, résultat A36, continuation, adaptateur et proposition de protocole ont survécu.

Au moment de la comparaison, `rust/crates/sim/src/lib.rs` avait déjà changé après le snapshot ; sa version de début de reprise demeure conservée dans le tar. Ce changement courant ne remplace aucune preuve historique. Les neuf fichiers de code récupérés ne constituent pas encore un candidat figé ni un binaire physique validé :

- `rust/crates/sim/src/lib.rs` — SHA256 `f3b2394d8898b715e151c3cf27ded8c5d7830c63d156c73f7dbacdd4c5497f86`.
- `rust/crates/sim/src/recovery/planner/load_transfer.rs` — SHA256 `555c93dae299b029cf8e78f5e048833de2bb8f989e3fadf366bfa3705582fcee`.
- `rust/crates/sim/src/recovery/planner/mod.rs` — SHA256 `ad159b7097f5d4c5ce72b65c2efd0457a53f66d8886c59f8612c126e58f0c839`.
- `rust/crates/sim/src/recovery/planner/native_plan_preview.rs` — SHA256 `95a5999b53b778c694596cad815dc75683d4f2e6c61d60bf5c762396be8769d7`.
- `rust/crates/sim/src/recovery/planner/observed_raccord.rs` — SHA256 `68aa9a55dff33b201cbe8052eef8bdd0255f7982d2baa3fe6bae534b2295fea7`.
- `rust/crates/sim/src/recovery/planner/transfer_adapter.rs` — SHA256 `70fb9feedf19784a4e9e076768695ae9f29f7c88a6306a51e71da16bafe15e90`.
- `rust/crates/sim/src/recovery/planner/transfer_adapter_tests.rs` — SHA256 `a5c2b8378641184da98e774ca1ac2af0bee489d7671873ff0b701c448ef5e7d1`.
- `rust/crates/sim/src/recovery/planner/transfer_worker.rs` — SHA256 `001e0fd4fb8f6877daf98329161d9c26082697f3b03607c3e398416298e86c34`.
- `rust/crates/sim/src/recovery/planner/whole_body_controller.rs` — SHA256 `a4597646248f80817a2079d0a53da52e408263f344abcffa40399780297c27a9`.

## Conservation vérifiée

**14/14 pièces protégées sont identiques** : cinq registres physiques clos, quatre binaires et cinq manifestes. Les cinq manifestes correspondent aussi aux octets de leurs commits gelés. Les **1 157 versions sources** du périmètre historique sont toutes vérifiées : 54 + 271 + 272 + 273 + 287. L'audit élargi vérifie **1 183 entrées complètes** des listes de code des manifestes ; les 26 entrées supplémentaires sont les scripts de documentation et données annexes exclus du décompte historique, sans écart d'empreinte.

Les vérifications détaillées figurent dans [integrity-audit.json](integrity-audit.json) et [historical-source-verification.json](historical-source-verification.json). Le calcul porte sur les octets des objets Git aux commits gelés ; les sources courantes corrigées n'ont pas à égaler ces anciennes versions.

## Fenêtres closes et limites conservées

La fenêtre d'assemblage du 13 octobre reste **close** : **un appel réel**, **douze inspections débitées sans remboursement**, aucun appel supplémentaire permis. Le total d'inspections reste **9 185 / 20 000**. Le résultat archivé A36 contient quatre commandes de 46 axes, mais ne démontre ni transfert physique, ni acquisition des talons, ni relevage. Le code a changé après cet assemblage ; cet ancien test ne certifie pas l'arbre courant.

Les familles physiques restent closes, avec **7 844 sous-pas historiques engagés** et **4/4 places d'histoire consommées**. Aucun de ces débits ou registres n'est modifié. Cet audit a uniquement lu des fichiers, du JSON, un tar et des objets Git, puis calculé des empreintes : **zéro test, compilation, FK, assemblage, monde ou sous-pas natif exécuté par l'auditeur**.
