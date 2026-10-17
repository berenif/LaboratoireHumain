# Admission du candidat A36 : arrêt avant lancement

L'autorisation conditionnelle reçue du parent demande une continuation unique,
au plus 131 096 sous-pas et 7 200 secondes, pour le candidat
`29df604533439e9ed7570698f2287fc92747d6ef`. Elle impose aussi de rester dans le
solde du plafond global initial de 300 000 sous-pas et 7 200 secondes de calcul,
sans rouvrir les familles closes, et de ne rien exécuter si le solde est
insuffisant. **Le lancement est bloqué par le délai du protocole figé. Aucun
worker physique, claim, nouvelle réservation ou sous-pas natif n'a été créé.**

## Intégrité et absence de doublon détecté

Le checkout est resté sur la branche de progrès au commit demandé. Les SHA
distants ont été vérifiés avant toute écriture ; `main` reste
`c597959f46f37715035349c8cd0a699b6822111a`.

L'inspection sans exécution confirme les 345 fichiers figés et les 14 pièces
historiques protégées. Le manifeste conserve le SHA-256
`4f3708fd50cd4dabf473b7a6bff528ee11ad798cf25dfbeaf0f6141c156818f7` et le binaire
`63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5`.
Il n'existe ni `authorization-claim.json`, ni registre physique, ni répertoire
`native-run-once` dans le lot du candidat. Aucun worker de coordination/parité
ni compilateur n'est détecté dans les processus visibles.

Linux refuse la lecture de certains liens `/proc/PID/exe` de services,
sandbox et deux anciens processus Git ; leurs noms et statuts restent visibles.
L'[audit de processus et d'intégrité](evidence/coordination-transfer-budget-decision-2026-10-15/prelaunch-integrity-and-duplicate-check.json)
conserve cette limite et ne prétend pas avoir identifié octet par octet tous
les exécutables du système. Aucun refus n'a été contourné. Le blocage budgétaire
suffit indépendamment à interdire le lancement.

## Réconciliation du plafond global

Les cinq registres clos sont relus sans modification. L'audit indépendant
conserve les sources, champs et empreintes dans les
[preuves de décision](evidence/coordination-transfer-budget-decision-2026-10-15/).

| Poste | Débit natif conservé |
| --- | ---: |
| Prototype initial | 3 856 |
| Trois lots techniques de parité, 44 chacun | 132 |
| Paire locale du 13 octobre | 3 856 |
| **Total déjà débité** | **7 844** |
| **Solde sur 300 000** | **292 156** |
| Nouveau candidat proposé | 131 096 |
| Cumul si admis, non débité ici | 138 940 |

Les pas réservés mais non utilisés ne sont pas remboursés. Les appels jamais
réservés ne sont pas ajoutés artificiellement : l'ancien maximum projeté de
82 724 n'est pas la dépense réelle des permis. Les quantités réalisées connues
totalisent 2 108 ; certains anciens compteurs restent inconnus, donc cette
somme n'est pas certifiée comme le nombre physique exact de toute la famille.
Les essais synthétiques de supervision et les campagnes antérieures distinctes
ne sont pas confondus avec cette enveloppe. Les quatre anciens emplacements
d'histoire restent consommés ; le nouvel accord concerne un successeur unique.

| Poste temporel documenté | Secondes |
| --- | ---: |
| Prototype initial, réservation à clôture | 7,930429 |
| Trois lots techniques | 1,699606994 |
| Paire locale, réservation à clôture | 12,321321 |
| **Fenêtres documentées cumulées** | **21,951356994** |
| Couverture conservatrice des démarrages/arrêts incomplets, deux fois 30 + 7 | 74 |
| **Ancien temps imputé prudemment** | **95,951356994** |
| **Solde planifiable conservateur sur 7 200** | **7 104,048643006** |

Les 74 secondes sont une provision prudente de phases, pas une mesure de temps
effectivement exécuté. Même en ignorant entièrement cette provision, le solde
ne dépasserait pas 7 178,048643006 secondes : un nouveau délai plein de 7 200
secondes ne peut donc pas respecter le plafond cumulatif. Les pauses de service
et le travail de préparation ne sont pas ajoutés au temps de calcul physique.

## Décision applicable au gel actuel

`run_transfer.py` fixe `WALL_SECONDS = 7200`, exige cette valeur exacte dans
`authorization.limits` et construit ses échéances à partir de cette constante.
Il n'expose aucun paramètre pour l'abaisser. Modifier le superviseur changerait
son SHA et celui du manifeste explicitement autorisé ; un wrapper temporel
extérieur ne fait pas partie du dispositif figé et revu. Aucun de ces chemins
n'est employé pour contourner la condition du parent.

**Résultat : admission budgétaire refusée avant exécution sous le manifeste
actuel.** Le candidat physique n'a pas échoué : il n'a pas été lancé. Il n'y a
donc aucun mouvement ou métrique physique nouvelle à capturer, aucune preuve
de talons acquis et aucun relevage à déclarer.

Une version bornée à **7 100 secondes de bout en bout** tiendrait dans la borne
prudente, avec 4,048643006 secondes de marge familiale. Ce serait un nouveau
gel du superviseur/manifeste et une autorisation liée à son nouveau SHA ; le
binaire physique pourrait rester identique. Cette possibilité est rapportée,
sans modification du candidat ni ouverture d'un budget dans cette étape.

Les registres physiques, binaire et manifeste restent inchangés. La récupération
publique reste désactivée. Cette lecture et cette documentation utilisent la
fenêtre de préparation existante, jusqu'au 15 octobre à 12:27:54 UTC, sans
nouvelle réserve : 57 289/144 000 secondes engagées. Aucun test, build, FK,
assemblage ou calcul physique n'a été rejoué.
