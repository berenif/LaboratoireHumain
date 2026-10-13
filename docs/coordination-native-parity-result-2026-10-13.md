# Parité native démontrée sur le premier tick contraint

**44 sous-pas natifs exactement exécutés en 1,477787 seconde murale.** Le lot
correctif est `closed-success` : intégrité, préfixe archivé, commandes installées,
parité copie/exécution et présence des diagnostics sont tous vérifiés.
**Aucun relevage, transfert local ni stabilité prolongée n’est qualifié.**

Le gel a été publié et son SHA distant vérifié avant exécution :
`b37eb9a566e1c819a7077c067ded5696f425ca8b` sur `investigate/physical-support`.
`main` reste à `c597959f46f37715035349c8cd0a699b6822111a`.
Le lot a démarré le 13 octobre à **10:22:22,086 UTC** ; plafond 44 pas et 120 s,
un processus, un thread. Aucun worker natif ne reste actif après sa fermeture.

## Ce que les pas prouvent

Le préfixe réel de **12 pas à 240 Hz** correspond aux archives, y compris
2 341 motifs binaires f32 de corps et moteurs par état. Depuis ce checkpoint,
chaque facteur possède une copie prédite de quatre pas et une exécution de
quatre pas sur une instance distincte. Tous les pas sont comptés :
`12 + 4 × (4 + 4) = 44`.

| Facteur | Itérations du solveur resserré | Commandes exactes par chemin | Parité native |
| ---: | ---: | ---: | --- |
| 0 | 8 | 4 | 4 états et fin de tick égaux |
| 0,5 | 8 | 4 | 4 états et fin de tick égaux |
| 1 | 12 | 4 | 4 états et fin de tick égaux |
| 1,5 | 12 | 4 | 4 états et fin de tick égaux |

L’audit indépendant retrouve **32 installations de commandes**, soit
**1 472 vérifications d’axes**, les 16 paires d’états après pas, les 16 paires
d’installations et les quatre états après tick. Positions, vitesses, mémoire,
contacts, impulsions et état sérialisé concordent exactement. Les gains
canoniques, l’actionnement `ForceBased` et les caps sont conservés ; la somme
des caps relue reste inférieure ou égale au budget de 760 Nm. Cela ne transforme
pas un cap moteur en couple effectivement livré.

Chaque comparaison couvre **16,67 ms**, de t=0,0500 à t=0,0667 s. Le test utilise
la fixture enregistrée `landed-crouch-left`, sans nouvel impact. Les diagnostics
locaux restent faux et les talons sont à 0 N sur ces derniers échantillons :
le test de transfert de quatre secondes reste à faire. Une prévision à deux
secondes, le relevage ≤25 s et la seconde stable ne sont pas validés par ce lot.
La récupération reste désactivée par défaut ; aucune qualification des 24 poses,
cycles ou impacts n’est effectuée.

Le journal CCD du chemin réel est archivé séparément. La comparaison des états
exposés ne certifie pas des caches internes que la sérialisation Rapier n’expose
pas. La preuve porte sur les quatre pas effectivement comparés, pas sur une
équivalence universelle de toute prévision future.

## Refus conservés et correction

La tentative du matin avait été arrêtée avant tout worker par l’interface
`/proc/.../children` absente. Le lanceur corrigé a ensuite démarré le worker,
mais le [lot renouvelé](coordination-native-parity-renewal-2026-10-13.md) s’est
interrompu sur une erreur de sérialisation JSON. Son compteur officiel reste
inconnu, avec une borne de code de **0–1 appel Rapier**, jamais remplacé par zéro.
Ses 44 pas réservés ne sont pas remboursés.

Le défaut était reproductible sans physique : `serde_json::Value` ne conserve
pas les entiers u128 dépassant u64. Le masque réel de contact
`170141183460469231731687303715884105728` (2^127) apparaît dès le premier pas.
Le monde est désormais archivé comme texte JSON exact dans une enveloppe
`{encoding, text}`, sans conversion en flottant ni changement global de serde.
Un compteur minimal est écrit durablement avant l’état détaillé ; une erreur
interceptée produit un reçu, ou des bornes explicites si le compte est incertain.

**8 tests purs**, la compilation de production et la compilation release dédiée
ont passé avant lancement. Aucun monde ni intégration n’est créé par ces tests.
Le nouveau binaire est distinct ; les anciens binaires et registres sont conservés.
Les 326 fichiers gelés et sept pièces protégées ont été vérifiés après fermeture.

## Preuves et capture

- [Registre clos et comptage](evidence/coordination-native-serialization-2026-10-13/native-run/physical-ledger.json).
- [Résultat natif complet compressé](evidence/coordination-native-serialization-2026-10-13/native-run/native-result.json.gz).
- [Trace de tous les états compressée](evidence/coordination-native-serialization-2026-10-13/native-run/native-trace.jsonl.gz).
- [Empreintes et validation sans perte](evidence/coordination-native-serialization-2026-10-13/archive-index.json).
- [Audit indépendant de budget](evidence/coordination-native-serialization-2026-10-13/post-run-budget-audit.md).
- [Capture des poses enregistrées](evidence/coordination-native-serialization-2026-10-13/recorded-poses.png).

La capture transforme seulement les maillages selon les 25 poses archivées :
aucune intégration, interpolation, FK ou requête de collision nouvelle. Le
compteur final distingue 44 admissions et 44 pas terminés, zéro réservation
locale pendante. La trace contient aussi 11 répétitions de comptage en fin de
tick : ses 55 lignes de compteurs ne sont pas 55 intégrations.

## Prochain jalon causal, préparation en cours

Préparer **une paire A/B de quatre secondes chacune**, mêmes référence, fixture,
profil, gains, caps et critères. A désactive la coordination ; B utilise les
quatre facteurs déjà figés avec sélection prédictive à 10 Hz maximum. Aucun
réglage après lecture du premier résultat. Le coût plafonné inclut :

- vérification native dans le même processus : **16 pas** ;
- deux histoires réelles : **1 920 pas** ;
- au plus 40 appels, quatre prévisions de 480 pas : **76 800 pas** ;
- total **78 736 pas**, zéro sonde supplémentaire, plafond mural **2 h**.

Les 16 pas sont payés explicitement : le reçu du lot court n’est pas transformé
en admission fictive pour un autre processus. Le protocole doit fermer après B,
que son critère local réussisse ou échoue, sans lancer B-repeat ou B60. Cette
préparation et ses tests purs se poursuivent dans l’enveloppe de préparation
existante ; aucun nouveau lot A/B n’a encore été ouvert ou lancé.

La continuation vers debout reste une décision distincte, avec les commandes,
appuis courants, passage de propriétaire, terminal original et stabilité à
vérifier avant une histoire complète. Le succès local ne peut pas devenir un
succès de relevage. La [proposition chiffrée de continuation](evidence/coordination-native-parity-renewal-2026-10-13/next-causal-milestone.md)
expose les coûts ; une histoire de 26 s dépasse le périmètre des histoires
locales de quatre secondes et n’est pas exécutée ici.
