# Chaîne physique C → appuis des bras → tentative de redressement

Le correctif d'appui obtient **les deux mains réellement porteuses** et permet
l'exécution de la montée du tronc. **La tête n'est pas déchargée et le brace
échoue dans les trois cas**. Aucun passage ultérieur genou/pied avancé, relevage
ou station debout n'est qualifié. Récupération de production désactivée.

HEAD `c597959f46f37715035349c8cd0a699b6822111a`, branche locale
`investigate/physical-support`. Aucun push, merge ou publication. Les changements
locaux historiques de production sont préservés ; ce suivi ne prétend pas
posséder les corrections non publiées d'un autre environnement.

## Exécutions réelles et jalon inchangé

Chaque cas recommence la chute C dans un monde neuf, puis continue ce **même
monde** depuis son repos intégré à 2,016667 s. Aucun snapshot de repos rechargé,
clone vivant, reset ou effort direct au bassin. Les 484 premiers pas physiques
sont identiques à la référence C dans chaque cas.

| Cas | Commande | Arrêt physique | Main gauche/droite à l'arrêt | Tête/torse à l'arrêt | Gain torse / UpY | Brace |
|---|---|---:|---:|---:|---:|---:|
| 01 | Références originales + compensation gravitaire validée | 13,104 s, stagnation de la pose droite | 0,49 / 0 N | 144,13 / 228,31 N | 0,92 mm / −0,173 | 0 s |
| 02 | Appuis corrigés depuis le torse mesuré, clearance native | 15,692 s, stagnation pendant montée | 72,09 / 61,09 N | 226,48 / **0 N** | 19,79 mm / −0,373 | 0 s |
| 03 | Cas02 + orientation mondiale prévue via réactions des hanches | 15,313 s, stagnation pendant montée | 58,15 / 57,00 N | 134,71 / 174,07 N | 6,67 mm / −0,285 | 0 s |

Les six tâches de préparation sont exécutées au cas02. La phase
`loaded-trunk-rise` commence à 10,204167 s. L'appui actuel **et** persistant des
deux mains dépasse le seuil original de 3 N pendant 2,708333 s consécutives,
de 12,987501 à 15,691668 s ; au cas03, 2,154167 s. Cela établit un appui réel,
pas la stabilité du corps entier ni le déchargement de la tête.

Le critère brace conserve simultanément : tête/nuque/torse actuels et persistants
sous3N, torse UpY≥0,40 et gain≥0,12m, bassin>0,28m, deux mains et un membre
inférieur porteurs, membres≥80% du poids, COM/capture dans leur hull réel,
vitesses originales des25corps, maintien0,55s. Aucun pas ne remplit même le seul
critère de déchargement tête/nuque/torse. La charge minimale de tête observée
après entrée est51,70N au cas02 et47,45N au cas03. Le passage genou/pied reste
conditionné au jalon et n'est donc pas exécuté.

## Cause de l'échec de pose des mains et correction conservée

Au cas01, la référence articulaire empiète de8,181mm sur l'enveloppe des skins
natives torse–UpperArm (4+4mm), malgré seulement0,181mm d'intersection géométrique,
compatible avec l'ancienne garde anatomique5mm. Les forces réelles torse–bras
atteignent110,146N à droite et109,083N à gauche. Le bilan gravité+moteurs+contacts
des six axes proximaux droits ferme à0,00448Nm. Aucun cap saturé ni butée libre
n'explique cet équilibre terminal. Le résidu maximal sur **tous** les axes
est0,0955Nm ; ne pas présenter la fermeture proximale comme universelle exacte.

De plus, les mêmes angles appliqués au torse réellement roulé placent la main
droite à32,633mm de la cible, encore en l'air, et la gauche sous le sol.
Le correctif conserve les positions d'appui originales et calcule leurs
références articulaires depuis le torse mesuré, avec séparation au moins égale
aux skins natives. Il se fond dans la durée de land déjà prévue. Les vitesses
cibles dérivent de ces références, puis reçoivent la compensation gravitaire
validée. Maximum12itérations algébriques bornées par bras/pas, angles bornés,
aucun changement des gains, masses, contacts, friction ou plafonds.

La compensation reste celle validée au checkpoint précédent : couples de
gravité des descendants via COM/masses natifs et Jacobien quaternion exact,
transmis comme biais de vitesse `−gravité/D` aux moteurs existants. Elle commence
dès la préparation gauche, puis s'applique aux deux bras dès la préparation
droite. Aucune attente diagnostique supplémentaire.

Le défaut distinct de progression de phase est aussi corrigé : la progression
de l'ancien objectif ne doit pas initialiser le compteur de stagnation du
suivant. Le test de transition et l'arrêt3s restent inchangés.

## Nouveau refus du redressement et essai rejeté

Au cas02, les hanches suivent les angles cibles à0,045rad près, mais le bassin
s'éloigne de l'orientation mondiale prévue. Cette orientation et sa vitesse
existent dans le programme géométrique ; l'exécuteur original n'en utilise que
les angles relatifs. Erreur mondiale terminale0,8895rad, projections de main
non convergées à0,318/0,378m depuis le torse réellement orienté.

Une omission de commande est donc établie. Son rôle **suffisant** dans la chute
du tronc n'est pas établi. Le cas03 teste une transmission de la référence
mondiale par réactions internes des deux moteurs de hanche, partage1/2 et
Jacobien natif. Les cas02/03 ont2 448pas physiques strictement identiques avant
le premier changement de commande. Le signe restaurateur sans écriture des
corps est vérifié unitairement. L'erreur mondiale diminue à0,7754rad, mais le
redressement régresse et le torse revient en appui. **Correction rejetée et
retirée du code actif**, conservée uniquement dans `candidate-03`.

À l'arrêt du cas03, les réactions mondiales X réellement livrées au parent par
les deux hanches valent+44,58 et−11,92Nm ; les références modifiées ne donnent
pas une autorité globale indépendante sur la racine. Les appuis et contraintes
articulaires déterminent le résultat. COM : vitesse[0,000838;−0,000184;−0,000060]
m/s, accélération[0,01791;−0,04499;−0,00274]m/s², tous25corps sous les seuils de
vitesse à l'arrêt, mais tête/torse chargés et hull de capture refusé. Un équilibre
immobile chargé au sol n'est pas un redressement.

Les deux projections géométriques du terminal virtuel convergent sous0,035mm,
avec clearance torse–bras25,42/26,57mm et sans intersection des bras examinés.
Cela ne prouve aucune trajectoire dynamique. Depuis les positions natives
refusées, les projections restent hors portée. Le prochain problème est une
commande coordonnée de pose mondiale, transmission des efforts par les appuis
et maintien de portée, avec contacts réellement résolus. Ni l'impossibilité
universelle d'une séquence pieds seuls ni la nécessité universelle d'un appui
particulier n'est prouvée ici ; la chaîne effectivement exécutée utilise mains
et membres inférieurs et reste insuffisante.

## Coûts, intégrité et vérifications

Budget fixé avant essai :4cas maximum,25s physiques/cas,100s cumulées,
150s natives/cas,600s cumulées. Garde héritée120s conservée. Un amendement explicite
avant cas03 autorise la seconde intervention structurelle après diagnostic.
Le quatrième cas est réservé à répéter un jalon réussi ; aucun succès complet,
donc aucun quatrième essai ni recherche de gains.

Exécution totale : **3cas,44,108333s physiques /198,024381s natives**.
Compilation, analyses JSON et rendus séparés. Présélection géométrique totale
80/96itérations, dont58 rejetées avec normales non orientées ; ces résultats
rejetés sont conservés et n'appuient aucun succès. L'ancien corridor201positions
reste une preuve géométrique séparée.

Les traces incluent positions/vitesses25corps, COM/accélération, contacts normaux
et tangents sol/corps, paramètres et impulsions réellement livrés des deux bras,
limites et caps. Dans les trois cas : caps installés≤760Nm et anatomie≤1,
aucun couple livré dépassant son cap, aucun effort direct bras ou bassin,
aucune garde d'intégrité déclenchée. Maxima hérités du préfixe : pénétration
sol16,372mm, erreur limite0,008017rad ; auto-intersection mesurée0. La contrainte
libre de limite peut produire des impulsions pendant la montée ; on ne prétend
pas que toutes les articulations sont libres de butée sur tout l'essai.

Code actif final : **288sources et binaire release exactement identiques au
candidate02 physiquement exécuté**, SHA binaire
`cb5139096f41eea167302cf16ed38a9f7dee6ecede9a90677ca4124acb973bb4`.
Après restauration et recompilation explicite :4unitaires passent,5bancs natifs
ignorés par défaut, Clippy strict et format passent. Un premier lancement après
restauration avait réutilisé un binaire périmé à cause des timestamps copiés ;
son log est marqué rejeté, puis une recompilation81s a produit le binaire exact
candidate02. Le log rejeté n'est pas une preuve du code final.

Aucune suite complète verte annoncée : le test hybride d'articulation et les
quatre échecs TypeScript historiques ne sont ni relâchés ni déclarés résolus.
Les tests ignorés collectent les résultats : leur `ok` signifie que le banc a
écrit sa trace, **pas** que l'objectif physique a réussi.

## Livrables et Library

Dossier autonome : `evidence/prone-chain-gravity-2026-10-11/`. Il contient plans
et ledger, entrées originales, référence C, sources/binaires exacts des3cas et
source finale, traces JSON complètes, diagnostics, unités/lint, captures/vidéos,
manifestes de SHA et reçus Library. L'archive de livraison conserve les essais
rejetés ; aucun déploiement ou publication publique.

| Média confirmé | Identifiant Library |
|---|---|
| Capture cas01, refus avant montée | `libfile_a5907330f2848191a57ad8f518dc9e63` |
| Capture cas02, deux mains porteuses mais tête chargée | `libfile_d41535bc27fc8191ac6c1275961d1dfa` |
| Vidéo cas02, chute complète puis montée refusée | `libfile_00b2d65b22cc8191b9551d00180d8841` |
| Capture cas03, intervention rejetée | `libfile_6626ca8ba3908191a1bece558dade405` |
| Vidéo cas03, chute complète puis montée refusée | `libfile_f816e60ba2388191811b69b2cfb5f5f1` |

Vidéo cas02 :315états enregistrés,20fps,15,75s. Cas03 :308états,20fps,15,40s.
Aucune interpolation de pose ni nouvelle physique. Les légendes montrent les
charges actuelles et l'échec du jalon. Rapport et archive : identifiants dans
les reçus de livraison confirmés et la réponse finale.

Checkpoint précédent déjà confirmé : vidéo
`libfile_70da0f152c2c81919f43de6534b7571a`, image
`libfile_50b5d288a1548191b3a22321df4a9e9d`, archive
`libfile_e980a9e6943081919e6b8603ed1bf3ff`, rapport
`libfile_c1676439e98881919cef14a7ff59ff1b`. Archive précédente immuable,
SHA`8bc96cfe8b59bbe6525af5add3a5363ba41a53821cb91498c226f1822624f531`.
