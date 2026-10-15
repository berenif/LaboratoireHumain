# Clôture de la fenêtre d'assemblage et audit d'intégrité

**Un assemblage observé a été exécuté et terminé ; les douze inspections réservées restent débitées.** Le test final réussit et produit quatre commandes communes pour les 46 axes, aux sous-pas 36–39. Cette fenêtre est close sur instruction du parent, sans nouvel appel. La préparation globale reste ouverte.

Le ticket unique est antérieur à l'exécution. Son reçu d'utilisation est écrit et synchronisé avant l'appel à `transfer_problem_observed`, qui contient l'unique `assemble_observed` de ce test. Début du reçu : **2026-10-13T12:44:48.911000+00:00** ; fin du résultat : **2026-10-13T12:44:48.924000+00:00** ; échéance **2026-10-13T12:55:17.675660+00:00**. L'intervalle de 13 ms entre ces deux reçus ne comprend pas toute la compilation ni tout le test. Le [journal final](geometry-command-tests-final.log) indique **1 test réussi, 0 échec**. Les deux journaux précédents se terminent en erreur de compilation, avant exécution du test ; ils n'ajoutent aucun assemblage.

Le [résultat A36](archived-a36-command-result.json) contient quatre consignes et aucune erreur de compilation numérique. Les deux obligations matérielles conservent leurs identifiants, points locaux, ancres et dates d'acquisition pendant ce lot de commandes. L'autorisation de l'adaptateur reste `unarmed`, les reçus natifs sont vides, le diagnostic local est faux et les talons ne sont pas acquis. Cette faisabilité de commande ne prouve ni appui physique, ni relevage. Les comparaisons facultatives d'assemblage à zéro/rotation n'ont pas été exécutées ; les 11 appels non utilisés restent facturés.

## Portée du code effectivement testé

L'amendement des sources est antérieur au reçu d'utilisation. Ses 18 empreintes correspondent exactement à la validation avant/après du parent. **Le code courant a toutefois changé après l'essai.** Le parent précise que les changements sémantiques concernent l'admission native, le terminal, les reçus/checkpoints et la progression configure/passive, sans changement annoncé aux tâches nominales, à l'assemblage ou au solveur ; des tests purs ont aussi été ajoutés.

- `rust/crates/sim/src/recovery/planner/whole_body_controller.rs` : essai `e54ad15e77e8c795305f6907ac243ce250550c652d6b323aefa58dd821080397`, actuel `15ab0b9a5707c92eb71c30a2e3678cd1b252ace75d284c99398d7e7f597fd90c`.
- `rust/crates/sim/src/recovery/planner/transfer_adapter.rs` : essai `2e40c8cde9182506a3d13b1c1396551f1fe4a959b068a2c01ca5d693ac3a594c`, actuel `2764affc6776dc5d94e00fd2e35019234af7792a733f2078e93d9eff0434b665`.
- `rust/crates/sim/src/recovery/planner/native_plan_preview.rs` : essai `d7f1d6cc077733690fc74e54970f66268c356d43abe3804d6e80331b1d8c1181`, actuel `54cbaf1a9f374a5226e64a4e0f167daf409aa73bff6ce12113cb321f1f66a53b`.
- `rust/crates/sim/src/recovery/planner/transfer_adapter_tests.rs` : essai `a9cf2779ca0714e6f1e564c26ea51d577aa0f0fd2641de9d93dc6a2f049b8cc8`, actuel `45cb7f3f7acd32784041c23f1974d334f0a21f9cc0ca8a4533c22d1885c7956b`.
- `rust/crates/sim/src/recovery/planner/observed_raccord.rs` : essai `1a9af58e0d0ed39a806f23a61ef2c772e6841242e98aec391b86c9ab23bf9e66`, actuel `68aa9a55dff33b201cbe8052eef8bdd0255f7982d2baa3fe6bae534b2295fea7`.

Ces écarts sont consignés, sans supposer une équivalence complète. Les octets du code testé n'ont pas été archivés séparément avant ces changements ; les empreintes seules ne restituent pas leur contenu. Le problème assemblé, les commandes et le reçu avant/après restent archivés. Une recompilation pure sur ce problème peut vérifier séparément le compilateur actuel, sans nouvel assemblage, mais elle ne rejoue pas les tâches et ne prouve pas l'identité du code d'assemblage final. **Cet audit ne certifie pas que l'arbre final a exécuté l'assemblage décrit.** Aucun ancien ticket n'a été réécrit.

## Conservation et budget

À **2026-10-13T12:52:02.653691+00:00**, **14/14 pièces historiques** sont inchangées : cinq registres clos, quatre binaires et cinq manifestes. Les octets des manifestes correspondent à leurs commits gelés ; **1 157 versions sources** vérifiées dans Git correspondent à leurs empreintes historiques (54 + 271 + 272 + 273 + 287). Les sources courantes corrigées ne remplacent pas ces anciennes preuves.

Budget conservé : **9 173 + 12 = 9 185 / 20 000 inspections** ; **46 489 / 144 000 s** de préparation, sans supplément pour cette sous-fenêtre déjà comprise dans les 10 800 s. Échéance du sous-jalon : **15:01:54 UTC** ; échéance originelle : **17 octobre, 20:14:00 UTC**. **Zéro FK, monde ou sous-pas natif** dans cette fenêtre. Les **7 844 sous-pas historiques chargés et les quatre places d'histoire consommées** ne changent pas.

Audit réalisé par lecture de code et de JSON, calcul d'empreintes et comparaison d'objets Git uniquement. Aucun test, compilation, assemblage, FK ou intégration lancé par cet auditeur ; aucune réservation ni résultat antérieur modifié. Le bilan détaillé et les SHA sont dans [final-integrity-audit.json](final-integrity-audit.json), SHA256 `147f0b8754e18b1277b17051257ae9798e5f4069e14ce22ae2f73871f43e7d23`.
