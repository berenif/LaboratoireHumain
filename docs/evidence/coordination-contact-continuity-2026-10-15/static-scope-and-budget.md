# Continuité des contacts : périmètre statique et budget préservé

**La correction autorisée est statique. Aucun nouvel essai physique n’est ouvert.** État du 2026-10-15T14:44:58.164385+00:00, ancré au commit `da4575170c6ae9b0858c0851a6f1ed15fe1ce858`. Le protocole A36 reste `closed-refused` ; son claim exclusif et son débit intégral restent conservés. Cet audit écrit seulement ces deux notes, lit les archives et les objets Git, sans test, build, modèle, monde, FK, assemblage géométrique ou sous-pas.

## Préparation déjà engagée

La fenêtre existante **14:12:53–17:12:53 UTC**, le 15 octobre 2026, est active et couvre ce travail. Ses **10 800 secondes** ont déjà été débitées dans les **68 089 / 144 000 secondes engagées**. Il reste **75 911 secondes non engagées**, sans nouvelle réservation ni remboursement dans cette étape. Le temps effectivement travaillé de toute la préparation n’est pas certifié ; une réserve engagée ne prouve pas autant de travail exécuté. L’échéance de cette fenêtre reste **17:12:53 UTC**, sans renouvellement automatique. La limite calendaire globale reste le 17 octobre à 20:14 UTC.

## Sous-pas : débit réservé et activité réalisée distincts

| Compte | Sous-pas |
| --- | ---: |
| Débits des cinq registres historiques clos | 7 844 |
| Débit intégral du successeur A36 clos | 131 096 |
| **Cumul réservé et débité** | **138 940 / 300 000** |
| **Solde arithmétique non débité** | **161 060** |
| Réalisé historique connu, total exact non certifié | 2 108 |
| Réalisé A36 réconcilié exactement | 130 |
| **Réalisé cumulé connu, total exact non certifié** | **2 238** |

Les archives antérieures bornaient les intégrations historiques à 2 108–2 109 ; après les 130 exactes d’A36, la borne dérivée devient 2 238–2 239. Cette inférence ne remplace aucun ancien compteur inconnu. Les **130 966 emplacements A36 non exécutés restent débités**, sans remboursement. Les quatre histoires anciennes et le successeur distinct restent consommés et clos. Le solde arithmétique ne donne aucune permission de relance.

## Temps physique après clôture

Les fenêtres historiques documentées totalisent **21,951356994 s**, auxquelles s’ajoutent les **10,376937300003192 s** du superviseur A36 clos : **32,328294294003192 s documentées**. La provision historique de **74 s**, pour phases de démarrage/arrêt incomplètement mesurées, demeure inchangée. Le rapprochement prudent vaut donc **106,328294294003192 s**, avec un reste arithmétique prudent de **7 093,671705705996808 s** sur 7 200. La provision n’est pas du temps exécuté mesuré ; le reste ne renouvelle aucun accord. L’autorisation A36 plafonnée à 7 100 s reste close. Préparation et pauses de service restent séparées du calcul physique.

## Empreintes pour le contrôle après correction

Le JSON archive les empreintes des **14 pièces historiques protégées**, du **binaire physique inchangé**, et de **27 pièces du lot clos et de son autorisation**. Les fichiers bruts et gzip restent liés par l’index d’archives vérifié au commit de référence. Le worker 5506 est absent ; un seul claim A36 demeure, celui de l’essai clos. Aucun nouveau claim n’est créé.

La vérification des deux manifestes candidats passe **contre les objets Git de `da457517…`** : **344/345** puis **357/358** entrées sont vérifiées au commit ; l’unique entrée restante de chaque manifeste est le même binaire non suivi par Git, vérifié directement avec SHA-256 `63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5`. Les cinq manifestes plus anciens gardent leurs propres commits d’origine, consignés dans le JSON. Leurs anciennes versions Rust ne sont pas assimilées à celles de `da457517…`.

La correction explicite peut modifier les sources Rust. **Après cette correction, les manifestes figés ne devront pas être présentés comme correspondant encore au worktree modifié.** Il faudra comparer leurs sources aux objets Git du commit de gel approprié, vérifier séparément les binaires et preuves immuables, puis montrer les modifications nouvelles comme travail statique non qualifié. Aucun manifeste historique ne doit être réécrit pour valider ces sources nouvelles. Les empreintes de cette note permettent un contrôle après correction sans nouveau calcul physique.
