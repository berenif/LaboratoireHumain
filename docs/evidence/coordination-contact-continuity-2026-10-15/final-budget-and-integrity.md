# Contrôle final du budget et des preuves — correction statique

**Les preuves physiques closes et tous les débits sont inchangés.** Audit du 2026-10-15T15:10:16.403000+00:00, sans test, build, exécution native, nouveau monde, FK ou assemblage. Seuls les deux nouveaux fichiers de cette revue sont écrits.

Les empreintes du bilan initial sont toutes retrouvées : **14 pièces historiques protégées**, **le binaire physique du candidat** et **27 pièces du lot clos et de son autorisation**, soit **42 fichiers vérifiés**. Le registre A36 reste `closed-refused`, à **130 intégrations exactes**, pour **131 096 sous-pas débités**, sans remboursement. Le claim unique est inchangé ; aucune autorisation, aucun manifeste d’exécution, claim ou registre physique n’est créé dans cette correction. Le PID 5506 est absent et aucun worker natif du candidat n’est détecté. Les limites de lecture de quelques exécutables de services via `/proc` sont conservées dans le JSON, sans contournement.

Le cumul reste **138 940 / 300 000 sous-pas débités**, distinct de **2 238 intégrations connues** dont le total historique demeure inexact. Le reliquat arithmétique de **161 060** n’est pas une permission d’essai. Les 130 966 pas inutilisés du lot A36 restent débités. Le temps documenté cumulé reste **32,328294294003192 s** et le rapprochement prudent, incluant la provision historique de 74 s, reste **106,328294294003192 s**. Aucun compteur ni famille close n’est remis à zéro.

La fenêtre de préparation **14:12:53–17:12:53 UTC** est encore active. L’engagement reste **68 089 / 144 000 s**, laissant **75 911 s non engagées** ; aucune nouvelle réserve ni remboursement n’est ajouté. Ces réserves ne sont pas présentées comme une mesure du travail effectivement exécuté.

## Sources modifiées et sources figées

Les manifestes candidats restent inchangés. Leurs **344 puis 357 entrées suivies par Git** concordent toujours avec les objets du commit `da4575170c6ae9b0858c0851a6f1ed15fe1ce858` ; leur unique binaire non suivi par Git est contrôlé directement et conserve `63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5`. Les cinq manifestes historiques antérieurs restent liés à leurs commits d’origine.

Les sources Rust du worktree ont désormais changé dans le périmètre autorisé : **11 fichiers anciens diffèrent**, et **3 fichiers Rust nouveaux** ne figurent pas dans les anciens inventaires. Le JSON donne leurs chemins et empreintes. **Les anciens manifestes ne correspondent donc plus au worktree courant et ne qualifient pas son code.** Aucune parité ou permission native de l’ancien binaire n’est transférée à la correction.

## Validation statique conservée

Les journaux et leurs empreintes confirment **82 tests uniques finalement réussis** : premier passage **79 réussites et 3 échecs**, puis les **trois mêmes tests réussis** après correction des fixtures. Les traces du premier échec sont conservées. Ces reçus déclarent zéro sous-pas, monde ou FK ; aucun test n’est rejoué par cet audit. Le journal de compilation du worker debug indique la réussite de la construction demandée avec `--no-run` par le responsable. Le binaire release physique est préservé et aucun worker dédié n’est exécuté ici. Ces validations ne démontrent aucun transfert, acquisition des talons, relevage ou seconde stable.

## Proposition suivante sans ouverture de budget

La proposition documentaire suivante reste **non approuvée et non débitée** : **568 sous-pas maximum**, répartis en **52 de parité avec préfixe**, **36 de préfixe principal** et **480 de prévision**, sous **120 secondes tout compris** et avec **zéro transfert réel après A36**. Un futur accord explicite, un gel adapté et une admission distincte seraient nécessaires. La compatibilité arithmétique avec le reliquat ne donne pas cette autorisation ; ni les preuves ni les registres clos ne sont rouverts.
