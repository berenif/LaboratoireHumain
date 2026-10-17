# Audit final de préparation statique, sans clôture des registres

**Conclusion : les chemins ciblés restent sans intégration physique, construction de monde, FK ou reconstruction des propriétés de masse.** Les preuves historiques sont intactes. Cet audit prépare la clôture documentaire ; il ne ferme ni ne modifie la réservation ouverte et n'autorise aucune exécution physique.

Audit effectué le **2026-10-13T11:56:10.917810+00:00**. Travail de cet auditeur : lecture de sources et journaux, comparaison d'empreintes et d'objets Git, rédaction de cette note. Aucun test, compilation, worker, rendu, FK ou simulateur lancé par cet auditeur. Les résultats de tests ci-dessous proviennent exclusivement des journaux produits par le parent.

## Périmètre des commandes déjà vérifiées

- Le filtre `command_proposal::tests` exécute cinq tests : **5 réussis, 0 échec** dans [command-proposal-tests.log](command-proposal-tests.log). `metadata()` désérialise le JSON du modèle avec le `Deserialize` dérivé ; il n'appelle ni `Model::canonical()` ni `Model::validate()`. Les appels à `axes`, `initialize`, `compile_problem` et `validate_command` lisent les métadonnées anatomiques et compilent les consignes q/dq. Ils n'installent aucun moteur. Les quatre commandes f32 sont vérifiées ; la relation q/dq est comparée avec `to_bits`. Les bandes matérielles, coefficients et obligations sont conservés, et l'admission réelle retourne toujours un refus.
- Le filtre `support_schedule::tests` exécute neuf tests : **9 réussis, 0 échec** dans [support-schedule-tests.log](support-schedule-tests.log). Les données sont des tableaux synthétiques ou des observations archivées. Les appels transitifs se limitent à la validation de ces données, aux charges, à l'enveloppe d'appui/capture, au suivi des points matériels et à leur archivage. Aucun `Model`, `Simulation`, monde, intégrateur ou FK n'est construit.
- Le [contrôle de production](production-check.log) termine avec `Finished dev profile`. `cargo check` compile/analyse les fonctions sans appeler leurs corps ; les crates du dépôt concernées n'ont aucun `build.rs` et Rapier déclare `build = false`. Ce contrôle n'exécute ni le worker `harness=false` ni les tests du dépôt. Il ne constitue pas une validation physique.

**Précision de portée :** `MaterialPatch::observe` transforme arithmétiquement un point local avec une pose fournie ; `supported_transfer` calcule une enveloppe et un point de capture sur les observations fournies. Il serait incorrect d'écrire « zéro arithmétique géométrique ». Il s'agit de contrôles numériques sur fixtures, sans reconstruire l'anatomie, générer une nouvelle trajectoire, faire une FK ou inspecter une nouvelle géométrie physique. Ils ne consomment aucun nouvel état de la grille d'inspection géométrique.

Le raccord `Controller::prepare_command_proposal_observed` appelle en revanche `problem_observed` puis `assemble_observed`, qui reconstruisent les tâches/Jacobiennes à partir de poses. Il est compilé, mais aucun des quatorze tests ciblés ne l'appelle. Son exécution est exclue de ce lot sans nouvelle inspection géométrique. Les autres filtres de tests historiques utilisant `Model::canonical()` ne sont pas couverts ni admis par cet audit.

La métrique signalée lors de la première lecture est corrigée : `continuous_qp_constraint_error` / `continuousQpConstraintError` désigne désormais le résidu du QP continu, tandis que `verify_rates` contrôle les commandes émises. Les [quatre résultats exportés](static-command-results.json) indiquent tous `NativePreviewRequired`, zéro pas natif, zéro monde et zéro FK. Ce reçu numérique n'est pas une mesure de faisabilité physique.

Le dernier ajout instrumental a été relu : il sérialise les décisions déjà calculées par le test sur les quatre archives B16, avec écriture exclusive du reçu. Le [journal instrumenté](support-schedule-tests-evidence.log) confirme à nouveau **9 réussites, 0 échec**. Les [huit refus exportés](static-support-release-results.json) conservent exactement les trackers et obligations avant/après refus ; tous restent `PreviewRequired`. Cet export ne réalise aucun calcul natif ou FK supplémentaire.

## Acquisition, horloge et refus

Lecture indépendante concordante du module `support_schedule` :

1. Le retrait exclut les deux segments du pied concerné : arrière-pied et avant-pied. Le test sur les quatre états B16 archivés refuse les deux retraits, sans mutation ni création d'une route admise.
2. Une nouvelle acquisition exige d'abord une observation postérieure au retrait où le support porteur a disparu des deux segments. Un simple aller-retour entre phases ne permet pas de réancrer le même appui. Cette disparition du support chargé ne prouve pas une séparation géométrique du sol.
3. Le point candidat possède son propre compteur de 12 observations. Il ne peut hériter des 100 observations artificiellement affectées au segment dans la fixture, ni d'une proximité du but mondial. La correspondance est contrôlée contre le premier point local de la série ; une dérive incrémentale ne rafraîchit pas cette référence. Le parcours synthétique nominal commence ce compteur au sous-pas 14 et acquiert au sous-pas 25.
4. `observe` impose exactement le sous-pas précédent +1 et un intervalle de 1/240 s avec la tolérance déclarée. Il travaille sur une copie avant de publier le nouvel état ; doublon, saut ou donnée non finie laissent l'objet inchangé. Les appels répétés à `acquire_observed` n'ajoutent aucune observation.
5. Les anciennes identités, ancres et dérives restent archivées. Le pivot ne remet pas la dérive à zéro. Chaque décision reste `PreviewRequired` : les séquences synthétiques ne prouvent aucun transfert natif ni continuation vers debout.

## Protection des preuves historiques

Contrôle à **2026-10-13T11:54:04.903782+00:00** : **14/14 pièces inchangées** par rapport à [preparation-reservation.json](preparation-reservation.json) : cinq registres physiques clos, quatre binaires release distincts et cinq manifestes. Les cinq statuts de registre correspondent aussi à la réservation. Les octets de chaque manifeste correspondent à sa version dans le commit gelé.

La comparaison des sources historiques utilise les objets Git des commits gelés, et non les versions courantes volontairement corrigées. **1 157 versions de fichiers sources** `.rs`, `.py`, `.toml` et `.lock`, sous `rust/` ou `scripts/` hors `target/`, correspondent à leurs SHA dans les cinq manifestes :

| Manifestation historique | Commit gelé | Versions sources vérifiées |
| --- | --- | ---: |
| Prototype initial | `b6e711d3dc58909979295df81437e6a5579c41db` | 54 |
| Parité initiale | `08aa8684911a87e0c3d6565855a0bb2c84340024` | 271 |
| Renouvellement parité | `334441445a9f5d97f18aa5d2956658685daf9e9a` | 272 |
| Correctif sérialisation | `b37eb9a566e1c819a7077c067ded5696f425ca8b` | 273 |
| Paire locale finale | `65925ef032197639f6ebd6c8e897084a5735cacf` | 287 |

Ce total compte séparément les versions communes à plusieurs gels ; il ne désigne pas 1 157 fichiers distincts. Les binaires sont seulement lus pour leur empreinte, jamais exécutés.

## Budget à conserver lors de la clôture

La réservation lue demeure `reserved-static-only`, SHA256 `122f7e019d7384b639535f784aa35d89d15e298b704641a1efff697de05e3695`. Elle a commencé le 13 octobre à **11:32:52 UTC** et expire à **13:32:52 UTC**. Les **7 200 s** restent intégralement débitées, sans remboursement ni renouvellement : **28 489 + 7 200 = 35 689 / 144 000 s**, soit **108 311 s non réservées** dans le plafond de préparation originel. L'échéance calendaire originelle reste le **17 octobre à 20:14:00 UTC**.

Les compteurs réservés d'inspection restent **9 173 / 20 000**, sans nouvelle FK ni inspection de trajectoire. Le correctif ouvre **0 sous-pas natif et 0 histoire physique**. Le total historique chargé demeure **7 844 sous-pas sans remboursement**, avec **4 emplacements d'histoire consommés sur 4**. Les inconnues historiques ne sont ni réestimées ni remboursées par cet audit. La famille reste close même si le solveur de commandes devient faisable.

À la date de cette note, aucun dépassement de la réservation n'est constaté. Le parent peut enregistrer sa clôture documentaire dans la réservation ouverte, en conservant intégralement les débits ; cette note ne constitue pas cet événement de clôture. La qualification physique, la continuation complète et une nouvelle entrée accroupie native restent hors de ce lot.

## Empreintes du snapshot examiné

| Fichier | SHA256 |
| --- | --- |
| `rust/crates/sim/src/recovery/planner/command_proposal.rs` | `d09cf9cfbeb650eddbcf2008fa214136f1bb774f2a5bebf44298c77a50c87d5b` |
| `rust/crates/sim/src/recovery/planner/support_schedule.rs` | `243e1b712a6a0f02d59480b78205b7bb8d3f6a913b30589d526f06842826997c` |
| `rust/crates/sim/src/recovery/planner/whole_body_controller.rs` | `b6db6f975a5192a4db60d4641f3da312888ff78dcd376b0c87e8c3e2bd09c244` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/command-proposal-tests.log` | `6d75834e8d0f55029f29451d891d900568e4e195ca05603007f759c1a4ff5bd8` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/support-schedule-tests.log` | `d3f51453694c047fab9f9e4a5b8f0c05c5c8d2a0fceb24f34c9a0828a9ca50c3` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/production-check.log` | `81bab5d1bf5c9468327f6da1722762ad25728b2c608cfe6a765d93dd26f877f3` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/static-command-results.json` | `269731897c231e33c2b653d0913a2bd66ca9462f511f121c63d0e43e7064754a` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/support-schedule-tests-evidence.log` | `dcb3601c031a1f8111b1ca6a9270291d405fa500c8b2aa8936cbcf22d585e471` |
| `docs/evidence/coordination-support-reformulation-2026-10-13/static-support-release-results.json` | `ea8a54b2d60a3e5fae9317c34f774081b1a69844520a9f82086fa2b2e208a6dd` |

Dernière lecture du snapshot stabilisé et des reçus instrumentés : **2026-10-13T11:56:48.259122+00:00**.
