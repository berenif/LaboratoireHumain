# Lanceur corrigé : supervision réelle, aucune nouvelle physique

Le lot natif précédent reste clos : **zéro sous-pas natif exécuté**, 44 réservés
sans remboursement. Cette intervention corrige uniquement le lanceur et teste
son protocole avec un exécutable C factice sans moteur physique. Elle n’ouvre
aucune autorisation native et ne qualifie ni la parité dynamique ni le relevage.

## Correctif causal

Le lanceur utilisait `/proc/PID/task/PID/children`, une interface absente de cet
environnement. Il recense désormais les PID et leur parent via le champ PPid
de `/proc/PID/stat`, également utilisé pour vérifier l’identité et le temps de
création du processus. La surveillance du nombre de threads et de l’empreinte
de l’exécutable reste active. Il s’agit d’une compatibilité entre configurations
Linux ; le lanceur ne prétend pas fonctionner sans les métadonnées `/proc`.

Le préflight lit ces interfaces sur le processus courant avant toute réservation
ou création de claim. Un PID disparu pendant le recensement est ignoré ; une
interdiction d’accès, l’absence du parent attendu ou une interface indisponible
font échouer l’admission. Aucun refus d’accès n’est transformé en succès vide.

Les signaux SIGINT et SIGTERM suivent maintenant le même chemin d’arrêt que
le délai : arrêt du groupe et des nouveaux enfants, exception, fermeture du
registre. L’installation des gestionnaires et l’armement du timer sont protégés
par `try/finally`. Le motif initial est conservé si le nettoyage échoue.
Les processus déjà présents avant le démarrage sont préservés.

Le comptage, les contrôles des reçus, la réservation avant lancement et le claim
exclusif empêchant une seconde exécution restent inchangés. Le plafond natif
n’est ni augmenté ni réinitialisé. La source figée du lot précédent reste
accessible au commit `08aa8684911a87e0c3d6565855a0bb2c84340024` ; le correctif
est ultérieur et ne réécrit pas ce manifeste ni son registre.

## Vérifications

Les 47 tests Python passent, dont les nouveaux cas PPid avec nom contenant
espaces et parenthèses, PID disparu, parent invisible, interface absente et
accès interdit avant claim. Ils ne sont pas présentés comme une exécution native.

Les **16 cas de bout en bout passent** dans cet environnement Linux, avec
les vrais processus, tubes, inspections, alarmes et signaux du lanceur. L’ancienne
interface `children` est effectivement absente ; le nouveau recensement fonctionne.

| Chemin exercé | Résultat constaté |
| --- | --- |
| Succès et dialogue complet | Reçu factice accepté ; fermeture propre |
| Second lancement | Claim existant refusé, aucun nouveau worker |
| Refus partiel et crash | Registre factice fermé en échec, sans remboursement |
| Timeout avant ready, après permis, après résultat | Processus arrêtés, aucun survivant actif |
| SIGINT/SIGTERM, dont fenêtre du constructeur | Arrêt et clôture ; gestionnaires et alarme restaurés |
| Métadonnées absentes ou alarme préexistante | Refus avant claim et avant lancement |
| Enfant ou second thread créé par le worker | Violation détectée et worker arrêté |
| Enfant antérieur au test | Préservé par le nettoyage du lanceur |

Le délai est réduit à deux secondes uniquement dans les processus de test ;
le plafond de production reste 120 s. Les cas de timeout se terminent en environ
2,06–2,07 s, démarrage du processus de test et nettoyage compris. Aucun survivant
actif n’est observé. Les enfants factices tués sont récoltés par le banc de test.

Les [reçus des 16 cas](evidence/coordination-launcher-portability-2026-10-13/e2e/summary.json)
et la [vérification finale](evidence/coordination-launcher-portability-2026-10-13/validation.json)
conservent les empreintes des sources et marquent chaque compteur comme synthétique.
Les 12 artefacts protégés sont inchangés, dont le claim et le registre du lot clos.
Parmi les 294 fichiers de son manifeste, seuls les deux scripts Python sont
modifiés par cette correction autorisée ; leurs octets historiques sont retrouvés
et vérifiés dans le commit préparatoire. Les 292 autres fichiers sont inchangés.

Commande reproductible, exclusivement factice, vers un dossier neuf :

```sh
python3 scripts/test-coordination-launcher-portability.py --output-dir /tmp/lh-launcher-inert-check
```

Le test compile seulement `scripts/fixtures/coordination-fake-worker.c` avec `cc`.
Il ne lance aucun exécutable Rust ni aucune simulation.

## Portée et suite

Le test factice prouve uniquement le lancement, le dialogue, l’arrêt et la
comptabilité du superviseur. Les valeurs « 44 » qu’il émet dans le protocole
sont synthétiques : le programme ne simule aucun corps et n’intègre aucun pas.

Une future exécution du véritable worker de simulation nécessite encore une
autorisation native distincte et un manifeste correspondant à la nouvelle source.
Le registre clos ne peut pas être réutilisé. Aucun changement de physique,
contrôleur, seuil, profil ou récupération par défaut n’est apporté ici.
