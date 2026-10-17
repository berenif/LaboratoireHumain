# Commande native sous contacts — 11 octobre 2026

Le défaut d'interface est isolé : **une consigne de vitesse ForceBased n'est pas
une commande de couple moyen**. Le contrôle nul correct conserve les moteurs
existants ; il reproduit les **2 891 pas corporels** du maintien précédent.
Après 10 s, ses **491 pas** et les **829 pas** du transfert terminé correspondent
bit à bit aux prédictions natives, pour les 25 poses/vitesses et toutes les
impulsions moteur scalaires, absolues et vectorielles. Le transfert reste
physiquement insuffisant : **45,269° bassin / 54,729° torse**, puis stagnation.
Le jalon latéral de 75° et le relevage ne sont pas acquis.

HEAD `c597959f46f37715035349c8cd0a699b6822111a`, branche locale
`investigate/physical-support`. Aucun push, merge ou publication ; récupération
désactivée et non qualifiée. Anatomie, contacts, profils, caps et seuils conservés.
Les corrections locales de l'autre environnement ne sont pas revendiquées.

## Pourquoi le contrôle nul converti changeait les charges

Les audits [d'actuation](rust-native-actuation-audit-2026-10-11.md) et
[du prédicteur natif](rust-native-control-2026-10-11.md) sont réutilisés.
Rapier subdivise ici chaque pas externe de 4,167 ms en **25 sous-pas de
0,167 ms** : 20 itérations de base et 5 supplémentaires. Les 32 itérations PGS
internes sont un autre paramètre. ERP/CFM, Jacobiennes, limites et contacts sont
résolus pendant ces sous-pas ; les impulsions délivrées sont cumulées. Le signe
du couple généralisé enfant est l'opposé de celui de l'impulsion de ligne.

Sur le même témoin chargé à 10 s, trois clones en lecture seule donnent :

| Commande au prochain pas | Vitesse ceinture X | Couple moyen enfant X livré | Normale chaîne droite | Normale torse |
| --- | ---: | ---: | ---: | ---: |
| Commande conservée | 0,26715 rad/s | 6,62815 N·m | 53,469 N | 294,589 N |
| Ancien inverseur, pas externe | 0,14002 rad/s | 5,05749 N·m | 60,084 N | 285,005 N |
| Inverseur au pas interne, état figé | 0,14406 rad/s | 5,05868 N·m | 58,775 N | 287,980 N |

L'inverseur vise le couple moyen précédent de 6,58572 N·m. L'évaluation figée de
la commande originale au pas interne vaut 10,59870 N·m, au-dessus du cap alloué
de 6,83742 N·m ; la résolution native plafonnée livre 6,62815 N·m. Remplacer
cette commande par l'inverse d'une impulsion moyenne change donc le moteur et
ses réactions couplées. Corriger seulement `dt` n'en restaure pas la neutralité.
Ce résultat causal est cohérent avec les 6,665 N·m demandés / 5,266 N·m livrés
dans l'essai précédent ; il ne décompose pas chaque réaction de cet ancien cas.
La formule `D + dt_externe × K` décrite précédemment n'est pas un contrat exact
de couple sous contacts.

Le contrat vérifié emploie `PreparedNativeStep`, capturé après préparation des
forces, moteurs et paramètres PGS. Un delta nul laisse les consignes intactes,
sans conversion ni notification d'édition supplémentaire. Les deltas non nuls
portent uniquement sur les vitesses articulaires. Le planificateur évalue les
charges et le moment cinétique de la résolution native complète ; aucun monde
prédit n'est transplanté et aucune force ou pose n'est écrite au bassin.

## Transfert réellement exécuté et limite observée

| Histoire depuis la chute complète | Temps physique | Bilan |
| --- | ---: | --- |
| 01 : commande conservée, delta nul | 12,04583 s / 2 891 pas | Témoin exact ; 43,596° / 54,827°, stagnation originale |
| 02 : réponse native, décision tous les 12 pas | 13,45417 s / 3 229 pas | 45,269° / 54,729°, recontact puis stagnation originale |
| 03 : même objectif, décision tous les 4 pas | inconnu | Arrêt externe à 120,030 s murales ; aucun bilan terminal récupéré |

Cas 01 et 02 partagent leurs 2 400 premiers pas. Douze deltas de vitesse sur
lombaires, torse, ceinture et bras droits sont bornés à ±0,25 rad/s. L'objectif
porte sur la décharge de la chaîne droite, le contact torse réellement chargé
et le moment cinétique utile, avec horizon natif de quatre pas. Le box QP audité
fournit une proposition ; son certificat concerne cette proposition avant
clipping cumulé. Seule une amélioration native non linéaire fraîche, après les
inspections physiques originales, accepte la commande. Aucune optimalité après
clipping n'est revendiquée. 57 décisions sur 70 sont acceptées, au plus 19
évaluations par décision, plus les trois comparaisons initiales en lecture seule.

Le premier horizon prédit une chaîne droite à 0 N et le torse à 333,02860 N.
À **10,01667 s**, ces deux valeurs sont exactement celles du monde réel.
Cependant, le bras reprend une charge dès **10,02083 s**, sous la même commande,
avant la décision suivante à 10,050 s. La normale de la chaîne reste sous 3 N
pendant seulement six pas, soit 25 ms ; cela ne certifie ni clearance géométrique
ni appui latéral. Le problème restant est donc la tenue du transfert et du moment
au-delà de l'horizon bref, pas une divergence entre commande native prédite et
commande réellement exécutée.

Dans la fenêtre commune 10–12,04583 s, les deltas moyens face au témoin valent
bras −4,845 N, torse −7,085 N, main gauche +14,865 N, cuisse gauche −18,384 N
et cuisse droite +15,408 N. Le transfert voulu bras→torse n'est pas soutenu.
Le delta de moment global Z par `dH/dt` vaut +0,00212 N·m, celui d'accélération
COM X −0,000494 m/s². Le roulis du bassin progresse de 1,672°, tandis que celui
du torse baisse de 0,098°. Le système finit de nouveau immobile, dans l'enveloppe
COM/capture réelle, avec le bras porteur à 51,870 N et le torse à 278,294 N.
Tête/nuque à 0 N après 10 s, assistance directe au bassin nulle ; norme vectorielle
maximale livrée 0,987 du plafond alloué. Les 242 instantanés communs conservent
exactement les positions cibles et les caps des 24 joints ; gains/modèles ne sont
pas modifiés par l'API de delta de vitesse.

À l'entrée de la nouvelle tâche à 10 s, son compteur de progrès est initialisé
sur le roulis mesuré. Les critères de 1° de progrès, 3 s de stagnation et horizon
global 25 s restent identiques. Le dernier progrès réel est à 10,45417 s ; l'arrêt
à 13,45417 s n'est pas une prolongation arbitraire du délai. Le verdict historique
de recontact reste faux, aucun monde ou compteur de qualification n'est reset.

Le troisième essai teste uniquement l'alignement cadence/horizon. Il a chevauché
la compilation Rust finale ; son arrêt mural ne mesure donc pas un débit natif
intrinsèque et ne permet **aucune conclusion physique**. Le budget est clos :
trois tentatives, **223,108 s murales**, 25,500 s physiques dans les deux histoires
terminées et durée physique inconnue de la troisième. Aucun quatrième essai.
Cette limite et l'échec du cas 02 ne prouvent pas l'impossibilité d'autres
trajectoires, ni celle d'un relevage pieds seuls. La route intermédiaire avec
main chargée reste partielle ; aucune phase suivante n'est qualifiée.

## Correctif conservé et preuves

Les contrôleurs expérimentaux sont retirés, les six sources de départ restaurées
exactement et la projection pure antérieure conservée. Le changement retenu est
le contrat documenté et sa régression dans
[native_predictor.rs](../rust/crates/sim/src/recovery/native_predictor.rs) : delta
nul sous contacts chauds, caps/gains/cibles/modèles intacts, égalité bit à bit des
25 corps et des impulsions moteur. Le replay chargé existant vérifie désormais
aussi les impulsions sur les quatre pas prédits.

**Vérifications finales :** quatre tests du prédicteur, sept tests de projection/
commande, un replay chargé ciblé, Clippy strict, format et contrôle du diff passent.
La suite complète n'est pas déclarée verte : l'échec hybride et les quatre échecs
TypeScript historiques restent ouverts. Aucun seuil n'est relâché.

[Analyse et audit natif](../evidence/prone-roll-native-command-contract-2026-10-11/analysis.json),
[registre des trois tentatives](../evidence/prone-roll-native-command-contract-2026-10-11/native-ledger.json),
[preuves de caps/cibles](../evidence/prone-roll-native-command-contract-2026-10-11/actual-cap-position-proof.json),
[patch de régression conservé](../evidence/prone-roll-native-command-contract-2026-10-11/retained-contract-regression.patch).

[Décharge puis recontact](../evidence/prone-roll-native-command-contract-2026-10-11/temporal-contact-audit.png),
[comparaison mesurée](../evidence/prone-roll-native-command-contract-2026-10-11/comparison.png),
[trois poses natives](../evidence/prone-roll-native-command-contract-2026-10-11/milestones.png),
[vidéo locale](../evidence/prone-roll-native-command-contract-2026-10-11/native.mp4).
La vidéo de 3,70 s montre le cas 02 terminé : 20 images natives/s, sans interpolation,
dernière image à 13,450 s, arrêt mesuré à 13,45417 s. Aucun visuel du cas 03
n'est inventé. Aucun nouvel appel Library ; aucune publication.
