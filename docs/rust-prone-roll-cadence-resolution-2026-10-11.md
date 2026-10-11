# Cadence courte : verdict physique — 11 octobre 2026

La définition exacte du troisième essai est maintenant évaluée jusqu'à un
**arrêt physique**, sans campagne de paramètres : **13,27083 s simulées,
3 185 pas, stagnation originale de 3 s**. Le bassin termine à **45,0677°**, le
torse à **54,3880°**. Le bras droit porte encore **53,2067 N** et le torse
**283,8669 N**. L'équilibre à 75° n'est pas franchi ; aucun succès à répéter,
aucune phase suivante qualifiée. Cette clôture remplace l'incertitude sur le
troisième cas du [rapport précédent](rust-prone-roll-native-command-contract-2026-10-11.md).

HEAD `c597959f46f37715035349c8cd0a699b6822111a`, branche
`investigate/physical-support`. Aucun push, merge ou publication. Récupération
désactivée et non qualifiée, anatomie et critères physiques inchangés.

## Reprise exacte et budget

La tentative précédente n'avait laissé aucun état physique récupérable. Une
relance de la même chute complète était donc nécessaire ; les témoins déjà
prouvés n'ont pas été rejoués. Les huit fichiers de contrôle, projection et QP
sont identiques aux sources archivées du troisième cas. Le code d'exécution du
prédicteur natif est également identique. Seuls la garde murale du harness et un
journal d'observation après intégration changent. Les **2 404 premiers hashes
corporels** sont identiques au cas précédent à cadence de 50 ms ; la divergence
commence à la première décision supplémentaire, comme attendu.

Horizon et cadence : quatre pas, soit 16,7 ms. Les douze vitesses, correction
±0,25 rad/s, sondes de 0,02 rad/s, objectif, QP, clipping cumulé, cibles de position,
gains, caps, contacts et limites restent identiques. Le compteur de progrès de
la tâche commence à 10 s comme dans le troisième cas ; dernier progrès réel à
**10,27083 s**, puis trois secondes sans le degré de progrès original.

Le coût précédent projetait jusqu'à 450 s pour 900 décisions à son maximum
mesuré. Budget réservé : **600 s natives + 60 s de sauvegarde**, sans compilation
concurrente. Le verdict arrive après **118,109 s murales**. Le journal persistant
est sauvegardé à chaque décision et chaque seconde simulée. Aucun arrêt mural,
aucune optimisation des commandes, aucun second essai après cet échec physique.

## Obstacle mesuré

La normale de la chaîne droite reste sous 3 N pendant cinq pas seulement,
soit **20,833 ms**. Son dégagement géométrique complet n'est jamais établi après
10 s. Le haut du bras finit à **3,508 mm du sol**, dans la peau de contact native
de 4 mm : il reste le pivot porteur. Sur la dernière seconde, sa charge moyenne
vaut 51,707 N et celle du torse 283,919 N ; le moment global Z obtenu par
`dH/dt` vaut −0,000155 N·m. Le système revient à un appui immobile avant le jalon.

Dans la fenêtre commune 10–12,04583 s, les deltas moyens face à la commande
conservée sont : bras −0,466 N, torse −14,163 N, main gauche +18,482 N, cuisse
gauche −21,761 N et cuisse droite +17,885 N. L'accélération COM X change de
−0,000437 m/s² et le moment global Z de −0,000166 N·m. Le transfert soutenu vers
le torse demandé par l'objectif n'est donc pas réalisé, malgré la cadence courte.
Ce verdict borne ce candidat ; il ne prouve pas l'impossibilité d'autres
trajectoires et ne justifie aucun relâchement de seuil.

Les **785 prochains pas** contrôlés correspondent bit à bit aux 25 poses/vitesses
et à toutes les impulsions moteur. Cette garantie concerne le prochain pas.
Sur 196 horizons complets comparables, les prévisions terminales à quatre pas
présentent jusqu'à **5,996 N d'écart sur la chaîne droite** et **17,337 N sur le
torse** ; écart absolu moyen de chaîne droite 0,379 N. La préparation du pas réel
est répétée entre les intégrations, alors que l'horizon conserve les moteurs
installés. Des variations de quelques ulp des cibles de vitesse reconstruites
sont visibles dans les traces ; leur responsabilité complète dans ces écarts
n'est pas isolée. Aucune égalité à quatre pas n'est revendiquée.

197 décisions, dont 134 corrections acceptées par amélioration native fraîche ;
au plus 19 évaluations par décision. Le certificat QP reste celui de la
proposition avant clipping, comme dans la définition reprise. Les 242 instantanés
communs conservent exactement les cibles articulaires et les caps des 24 joints.
Norme vectorielle moteur maximale : **0,988 du plafond alloué**. Tête/nuque à
0 N après 10 s, assistance directe au bassin nulle, 25 corps immobiles et
COM/capture dans l'enveloppe réelle à l'arrêt. Le verdict historique de recontact
reste faux ; aucun reset du monde ni du compteur de qualification.

## Hors ligne et livraison

La décision coûte en moyenne **371 ms**, au maximum 985 ms, pour un intervalle
simulé de 16,7 ms : environ 22 fois ce délai en moyenne. L'outil permet
l'exploration physique hors ligne ; ce cas ne fournit ni trajectoire de relevage
réussie ni contrôleur temps réel livrable. Le budget administratif était suffisant
pour obtenir le verdict, sans optimisation supplémentaire ni autre architecture.

Les prototypes et le journal expérimental sont retirés du code actif ; six
sources de départ et la régression native déjà conservée sont restaurées
exactement. **Quatre tests du contrat, sept tests de projection/commande, Clippy
strict, format et contrôle du diff passent.** La suite complète n'est pas déclarée
verte ; l'échec hybride et les quatre échecs TypeScript historiques restent ouverts.

[Définition inchangée](../evidence/prone-roll-cadence-resolution-2026-10-11/same-definition-proof.json),
[analyse](../evidence/prone-roll-cadence-resolution-2026-10-11/analysis.json),
[registre](../evidence/prone-roll-cadence-resolution-2026-10-11/native-ledger.json),
[journal natif](../evidence/prone-roll-cadence-resolution-2026-10-11/case-01/progress.jsonl),
[caps et cibles](../evidence/prone-roll-cadence-resolution-2026-10-11/actual-cap-position-proof.json).

[Comparaison mesurée](../evidence/prone-roll-cadence-resolution-2026-10-11/comparison.png),
[trois poses natives](../evidence/prone-roll-cadence-resolution-2026-10-11/milestones.png),
[vidéo locale de 3,50 s](../evidence/prone-roll-cadence-resolution-2026-10-11/native.mp4).
70 images natives à 20 images/s, sans interpolation ; dernière image à 13,250 s,
arrêt physique à 13,27083 s. Aucun nouvel appel Library, aucune publication.
