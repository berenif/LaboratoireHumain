# Observation native des impacts — 11 octobre 2026

**Cause du grand écart linéaire isolée : la lecture historique ne conserve que
la dernière résolution CCD du pas, alors que les résolutions précédentes ont
réellement appliqué leurs impulsions.** Une observation en lecture seule, avant
le renouvellement des contacts, retrouve leur somme. Elle ne change pas la chute
ni les couples ; aucun relevage n’est obtenu. P1 reste non franchie : bilan
angulaire et borne indépendante d’arrondi encore à certifier.

Le travail reprend le plan autorisé à 21:18 UTC, depuis `9d68ab2`, sur
`investigate/physical-support`. Le checkout était propre ; aucun `AGENTS.md` ou
`.agents/skills/SKILL.md` trouvé dans le dépôt. `main` reste à `c597959`. La
récupération reste désactivée par défaut et non qualifiée. Les critères, gains,
plafonds, contacts, masses et anatomie restent inchangés.

## Intervention et portée

Le fork Rapier possède une fonctionnalité `solver-step-observation`, désactivée
par défaut. Deux callbacks exposent uniquement des références immuables : juste
avant chaque résolution CCD, puis après l’écriture des impulsions/vitesses et
avant le clamp CCD, l’avancement des poses et la nouvelle détection des contacts.
Aucun champ de l’état physique sérialisé, calcul de contrainte ou amorçage changé.
La provenance et le diff se trouvent dans `rust/vendor/solver-step-observation*`.

Le banc expérimental `recovery/planner/contact_observation.rs`, sous `cfg(test)`
et `experimental-solver-step-observation`, enregistre impulsions normales et
friction, quantité de mouvement des 25 corps, gravité et forces utilisateur,
durée et ordinal de chaque résolution. Les totaux accumulés du solveur sont lus
une fois par résolution ; ils ne sont pas additionnés à chaque itération PGS.
Le rapport indique toujours `angular_balance_verified=false`.

Le domaine est explicitement refusé en présence de sommeil, amortissement
linéaire, axes verrouillés, multibody, corps supplémentaire mobile, autre collider,
interaction ou environnement mobile. Le profil mesuré conserve son amortissement
angulaire de 0,52 : il reste à comptabiliser pour le bilan angulaire.

`whole_history.rs` conserve les observations du pas physiquement intégré même
si le contrôle d’intégrité refuse avant l’observateur public. Son rapport final
ajoute les hashes complets des corps, articulations, narrow phase et compteurs,
y compris à ce pas refusé. La collecte n’alimente pas encore le contrôleur.
Une impulsion passée ne certifie pas un appui actuellement persistant.

## Résultats mesurés

| Histoire | Ancienne charge verticale | Somme des résolutions CCD | Bilan vitesse/gravité | Résidu vertical à cet impact |
|---|---:|---:|---:|---:|
| Référence à 1,375 s | 42,929 N | 6 503,863 N | 6 503,863 N | 1,224×10⁻⁶ N·s |
| Contrôleur témoin local à 1,441667 s | 443,180 N | 2 568,086 N | 2 568,086 N | 1,273×10⁻⁷ N·s |

Dans la référence, la première résolution dure environ 2,873 ms et applique
26,920532 N·s verticaux, la suivante 1,294 ms et 0,178899 N·s. La dernière lecture
ne contient plus la première impulsion. Cela explique le pic d’écart d’environ
6 460,9 N ; il ne s’agit pas d’une force reconstruite ou injectée dans le bassin.
Les petites composantes verticales des impulsions tangentielles sont incluses.

Les résidus linéaires maximum par résolution sur toute la référence sont
`[0,962; 3,188; 2,309]×10⁻⁶ N·s` en X/Y/Z. Sur le témoin de 26 s :
`[1,515; 6,703; 2,697]×10⁻⁶ N·s`. Ces valeurs sont des observations ; aucune
nouvelle tolérance n’est choisie pour les transformer en certificat.

La référence compte 1 172 pas natifs, dont le dernier refusé, et 1 176 résolutions
CCD. Quatre pas ont deux résolutions. Elle échoue exactement comme le témoin
archivé à 4,883333 s : pied gauche/droit, pénétration 5,0944 mm pour la limite
originale de 5 mm. Le contrôleur local conserve 6 240 pas, 6 244 résolutions CCD,
quatre pas doubles, zéro récupération. Ce contrôleur comporte le correctif local
de support du checkpoint `dcbcdfa` ; ce n’est pas le `main` pristine.

Les deux paires observation activée/désactivée ont des lignes numériques
strictement identiques au témoin archivé : contacts résolus, CoM/vitesse et
accélération, couples livrés/caps, stabilité, compteurs et empreintes physiques.
Les états initiaux, finaux et refus sont identiques. Le rejeu final ajoute la
comparaison des hashes complets au pas refusé. Les résidus sont calculés en `f64`
à partir des masses/vitesses natives ; le diagnostic historique utilisait une
vitesse CoM agrégée en `f32` et des forces finales seulement.

## Contrôles et budget

`scripts/run-native-recovery-contract.py` réserve atomiquement chaque histoire
avant lancement, sous verrou exclusif, avec SHA du binaire/programme et reçu UTC.
Un crash ou timeout conserve sa réservation ; un lancement incertain interdit
la reprise automatique. Le lanceur impose 32 histoires techniques au maximum,
26 s chacune, 30 min murales depuis le premier lancement, un processus mono-thread,
aucune validation idle et aucun lancement de recherche. Les réservations ne sont
pas remises à zéro. Les compteurs globaux et l’empreinte du registre historique
sont préservés. Le stockage applique une borne conservatrice de 2 Gio non compressés, avec
512 Mio réservés pour le prochain cas. Le registre et son répertoire sont fsync.

Les cinq tests Python d’admission des preuves ne font aucune intégration :
ils refusent une physique modifiée, y compris les hashes finaux du monde refusé,
un intervalle dupliqué, un pas refusé manquant et vérifient la somme unique des impulsions. Cinq tests supplémentaires refusent les budgets expirés, épuisés, modifiés ou
incertains, sans lancer de monde. Les builds, Clippy et format sont
séparés des histoires intégrées. Aucun test complet vert n’est revendiqué :
l’échec d’articulation hybride et les quatre échecs TypeScript historiques restent
hors de ce jalon.

Le registre compte **six histoires techniques**, **17 168 pas natifs de 1/240 s**
(71,533 s physiques), 37 440 pas réservés conservés, zéro recherche et zéro
validation finale. Les quatre premiers utilisent le kernel `01af8583`, les deux
derniers le kernel `21a40388`, avec domaine explicitement vérifié et hashes finaux.
Clippy strict passe avec et sans la fonctionnalité ; build release, format et
dix unitaires Python passent. Aucune compilation concurrente avec les histoires.

Le reçu final, les comptes exacts et hashes sont dans
`docs/recovery-impulse-boundary-results-2026-10-11.json` et le registre local
`evidence/native-hybrid-v1-2026-10-11/ledger.json`. Les vérifications de neutralité
sont des tests physiques déterministes d’observation ; elles ne prouvent ni un
transfert d’appui, ni un handoff, ni une récupération.

## Porte suivante, toujours fermée

Une revue indépendante en lecture seule confirme la frontière et les signes.
Elle identifie la suite structurelle nécessaire : journal indépendant des
impulsions avec Jacobiennes/bras de levier natifs, puis journal des opérations
numériques et géométriques. Le moment physique doit utiliser une origine fixe,
les CoM réellement validés et les inerties mondiales correspondantes.

Le callback actuel voit des vitesses déjà amorties avec les poses/propriétés
d’inertie encore au début de l’intervalle et `next_position` intégrée. Croiser un
point géométrique courant avec une impulsion ne restituerait pas forcément le
couple réellement appliqué. Les lignes de limites/verrouillage articulaires et
les éventuelles opérations gyroscopiques ou plafonds doivent être comptabilisées,
ainsi que le transport d’inertie et le clamp CCD. Le chemin de friction simplifiée
possède en plus un couple pur de torsion non rapporté ; le cas actuel utilise
Coulomb, mais aucune généralisation silencieuse n’est permise.

La borne d’arrondi doit être déduite des opérations et magnitudes natives, figée
avant les rejeux de certification ; les défauts de conservation des contraintes
internes doivent rester visibles séparément. Tant que cela et la frontière
terminale d’origine ne sont pas vérifiés, pas de recherche de transfert.
Le défaut de lecture expliqué n’établit ni la faisabilité d’une route pieds seuls,
ni la nécessité d’une main/genou. Graphe, retour d’état et recherche ne sont pas
encore livrés.

## Traces et visuels locaux

- `evidence/native-hybrid-v1-2026-10-11/` : registres, reçus, binaires, JSONL par
  pas et résolution, poses natives 20 Hz, rapports et analyses reproductibles.
- `reference-impulse-boundaries.png` : impact de référence et résidus ; légende
  explicite « trajectoire échouée, physique inchangée ; P1 non certifiée ».
- `controller-impulse-boundaries.png` : confirmation sur le contrôleur témoin.
- La capture et vidéo natives déjà archivées dans
  `evidence/recovery-coordinated-retarget-2026-10-11/` illustrent la même chute,
  dont les états sont ici reproduits exactement. Aucun nouveau relevage illustré.

Les graphiques sont produits uniquement depuis les traces enregistrées, sans
pas physique supplémentaire. Aucune publication de l’archive volumineuse, aucun
merge ou changement de `main`.
