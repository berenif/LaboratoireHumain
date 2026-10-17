# Décision après les 25 programmes refusés — 2026-10-11

Base `c597959f46f37715035349c8cd0a699b6822111a`, branche locale
`investigate/physical-support`. Revue de sources et de traces archivées ; aucune
nouvelle intégration physique, campagne de gains ou modification du contrôleur
dans ce tour. Les modifications et corrections existantes sont conservées.

## Décision

Tester une **descente préparée avec les mains**, puis un parcours explicite
**mains + tibias → mains + pied avancé + tibia opposé → mains + deux pieds →
deux pieds → debout**. Le premier travail Sol doit prouver l'acquisition de
l'appui intermédiaire depuis la pose accroupie réelle. Ne pas commencer par
optimiser de nouveau les huit paramètres de la famille précédente.

Ce choix est une nouvelle hypothèse de contrôle, pas la conclusion que
l'anatomie impose les mains ou les genoux. La faisabilité dynamique de ce
parcours complet reste à démontrer. Une descente contrôlée vers les mains et
les pieds pourrait finalement permettre de sauter l'étape tibias ; cette
branche ne sera choisie qu'après mesure d'un transfert réellement soutenu.

## Observations établies

1. Les 25 évaluations distinctes sont 23 variantes couplées et deux graines
   « pieds d'abord ». Les deux tours de coordonnées ont sélectionné la graine
   couplée. Il ne s'agit ni de 25 stratégies de contact, ni d'une exploration
   exhaustive des trajectoires pieds seuls.
2. Le banc refuse toute charge persistante hors des pieds : il exclut aussi
   une main ou un tibia correctement posé. Ce refus est adapté à la famille
   pieds seuls ; il ne doit pas être interprété comme une contrainte anatomique.
3. Les variables portent sur cinq décalages sagittaux et trois durées. Les
   bras, la région lombaire, l'asymétrie gauche/droite, le choix des contacts et
   l'emplacement d'un pied avancé ne sont pas recherchés. Les 41 références
   de remontée ont exactement les mêmes coordonnées de bras.
4. Dans les candidats réellement évalués, les cibles de recentrage sont
   40,49–54,24° à la hanche et 46,79–59,40° au genou. Les limites anatomiques
   respectives sont 110° et 140°. Ces nombres décrivent les **consignes**,
   pas les angles effectivement réalisés. Une descente profonde ou un
   demi-agenouillement n'a pas été testé par cette famille.
5. La référence basse est déjà à 20° de dorsiflexion, 17,99° lombaires et
   14,95° thoraciques. Certains décalages positifs de cheville/tronc sont
   écrêtés à leur limite. Une partie de l'espace de paramètres ne crée donc
   pas de nouvelle consigne dans la direction recherchée.
6. Le banc exige avant toute remontée 0,55 s de calme de tous les corps,
   deux talons chargés et le centrage mesuré. Cette pause en position basse
   est un choix supplémentaire de la famille. Le contrôleur original vérifie
   le calme pour la transition debout/ordinaire ; il ne prouve pas qu'une
   remontée doit obligatoirement partir d'un équilibre bas immobile.
7. Toutes les variantes échouent avant la remontée. Le score privilégie alors
   le plus petit pic d'appui parasite avant le progrès d'appui. Son « meilleur »
   résultat classe des chutes ; il n'identifie pas un voisinage prometteur de
   trajectoires réussies. Ne pas poursuivre ce score dans cette situation.

Ces observations sont reproductibles avec
`evidence/strategy-review-2026-10-11/audit.py` et les empreintes de `audit.json`.
Les résultats physiques originaux restent dans le rapport hors ligne.

## Ce que le mannequin et le contrôleur permettent

Le modèle fait 72,2 kg (poids 708,282 N). Les membres supérieurs sont dynamiques,
articulés, motorisés et capables de contacts natifs au sol. Le coefficient de
friction des mains/tibias est 1,2, celui des pieds 4,0 ; le banc ne doit en
modifier aucun. Il n'existe pas de segment « genou » : l'appui correspondant
utilise une région de surface du tibia. Un contact quelconque du tibia ne doit
pas être renommé « genou » sans vérifier son point local.

| Articulation gauche, axe x | Intervalle anatomique | Couple maximal |
|---|---:|---:|
| Ceinture scapulaire | −10 à 30° | 32 Nm |
| Épaule | −35 à 120° | 70 Nm |
| Coude | 0 à 145° | 55 Nm |
| Poignet | −45 à 55° | 18 Nm |
| Lombaires | −10 à 18° | 100 Nm |
| Thorax | −10 à 15° | 90 Nm |
| Hanche | −20 à 110° | 150 Nm |
| Genou | 0 à 140° | 165 Nm |

Les autres axes restent ceux du modèle : épaule et ceinture à trois axes,
pronation de l'avant-bras à ±80°/15 Nm et deuxième axe du poignet à
−15…30°/14 Nm. Toutes les articulations partagent toujours le plafond 760 Nm.
Le faible couple du poignet impose de planifier une charge près de sa chaîne
de transmission ; l'existence de moteurs n'est pas une preuve de force utile.

`reaching::arm_targets` parcourt les cinq segments de la ceinture à la main.
Son IK, à racine thoracique mesurée, demande une **position de point**, avec
six itérations et bornage articulaire. Il ne planifie ni l'orientation de la
surface de main, ni les forces, ni le déplacement futur du thorax, ni les
collisions de tout le corps. Une solution IK n'est donc pas un appui acquis.
La représentation de coordonnées n'est pas Euler : conserver l'inversion
`2 atan2(q_i, q_w)` de `endpoint_pose`.

Le contrôleur sait détecter et conserver une ancre de main réelle. Mais :

- `Controller::new` peut sélectionner `Stand` dès les pieds chargés, le tronc
  assez droit et le bassin au-dessus de 0,5 m. Dans `Stand`, il conserve les
  mains **déjà** ancrées ; il ne prépare pas de nouvelle prise d'appui.
- Hors `Stand`, une nouvelle cible sol n'est demandée que lorsqu'un proxy de
  portée est déjà satisfait. Ce proxy mesure 0,5034 m entre les centres du
  bras supérieur et de la main au repos. Sur sol plat, avec les offsets
  actuels, le centre du bras doit être sous 0,5082 m. Ce n'est pas une borne
  anatomique exacte de toute la chaîne.
- Sur les états réels du témoin, ce prédicat devient vrai à 1,325 s à gauche,
  le premier contact de main ≥3 N arrivant à 1,333 s. À droite : 1,329 puis
  1,354 s. La tête touche à 1,467 s. C'est un calcul du prédicat sur les états
  archivés, **pas** l'affirmation qu'il a été exécuté pendant `Stand`.
- La vitesse de progression ordinaire des consignes est bornée à 1 rad/s
  avec un facteur commun à toutes les articulations. Une fenêtre de 8 ms ne
  permettrait qu'environ 0,008 rad de nouvelle progression. Il faut préparer
  les bras avant que le point au sol soit atteignable, en visant d'abord un
  point intermédiaire atteignable dans l'espace.
- Le tronc hors roulade revient essentiellement vers les références ; les
  objectifs bras et jambes sont traités séparément. Les réserves d'effort
  fondées sur les charges sont actuellement calculées pour les jambes en
  `Stand`/handoff, pas pour les chaînes d'appui des bras. Mesurer cette limite
  avant de décider d'une extension de l'allocation.
- Les poids de bras affichés à 0,06 en `Stand` ne constituent pas à eux seuls
  une preuve de manque de couple : l'allocation finale par demande native
  peut remplacer ces plafonds préparatoires. Comparer demandes, caps installés
  et impulsions réellement délivrées ; ne pas augmenter un poids à l'aveugle.

Sources principales : `recovery.rs` (`Controller::new`, `targets`,
`supported_transfer`, `support_effort_reserves`, `observe_recovery`),
`reaching.rs`, `lib.rs` (colliders et allocation), et `offline_trajectory.rs`
(`offset`, `command`, `observe_step`, `score`).

## Hypothèse causale et trajectoire complète visée

**Hypothèse :** la représentation précédente ne peut pas préparer un appui
antérieur avant que le centre de masse s'échappe. Une descente coordonnée
prépare les surfaces de main pendant que les pieds portent encore, acquiert
une base d'appui plus longue, dissipe le mouvement par les vrais contacts,
puis permet des transferts successifs avec une jambe mobile à la fois.

| Étape | Commande et appuis visés | Preuve attendue avant transfert |
|---|---|---|
| Préparation | Depuis l'accroupi réel, flexion hanche/genou et trajectoire des bras vers des points intermédiaires atteignables ; pas de tentative de poser immédiatement une main hors de portée | Géométrie de toute la trajectoire admissible ; vitesse, demandes et charges observées |
| Descente avec mains | Amener les surfaces de main au sol avant tête/torse ; pieds encore autorisés et chargés | Contacts mains réels, persistants et sans adhérence artificielle ; action sur le mouvement mesurée |
| Base basse | Poser les régions de tibia prévues, mains maintenues, pointes éventuellement conservées | Charges et calme sur la base déclarée ; tête/torse libres ; pas de simple survie après impact |
| Pied avancé | Décharger une jambe, déplacer son pied sous le bassin ; conserver mains et tibia opposé | Appuis restants porteurs et centrage calculé en excluant **toute** la jambe libérée |
| Deux pieds | Transférer vers pied avancé + mains, libérer le tibia opposé, amener le second pied | Même preuve avant libération ; pieds posés par leurs surfaces réelles |
| Retrait des mains | Transférer la charge vers les deux pieds avant de lever les mains | Charges bilatérales, appuis restants ≥80 % du poids, charge libérée ≤25 %, COM/capture dans le polygone réel avec marge 3 mm |
| Remontée et retour ordinaire | Étendre seulement sur un transfert soutenu, puis handoff existant | Critères debout originaux, échéance 25 s, observation stable 1 s, puis essais 30 s/caps/rotations/cycles séparés |

L'anticipation et l'asymétrie sont de nouvelles variables de **trajectoire et
de contacts**, sans variation de gains. La géométrie prospective ne peut
écrire aucune pose ou vitesse dans le monde physique. Aucun contact n'est
une contrainte artificielle : il doit apparaître par les colliders natifs.

Les seuils de `Brace/Kneel/PlantLead` et le refus de bassin bas en `Stand`
doivent être audités lors du raccordement au contrôleur. Leur simple présence
ne fournit pas cette trajectoire. Une descente volontaire ne doit pas être
étiquetée `Stand`, puis masquée après son refus. Le plan doit posséder
explicitement son mode et ses consignes ; le chronomètre et les critères de
qualification ne sont jamais réinitialisés. Si un mode ne peut être raccordé
aux critères conservés, rapporter ce blocage avant une modification de contrat.

## Première expérience discriminante à confier à Sol

**Résultat ultérieur :** l'A/B est maintenant exécuté. B acquiert de vrais appuis
de mains mais les deux variantes échouent sur appui tête après le choc ; aucune
stabilité intermédiaire. Voir le
[rapport natif, budgets réellement utilisés et preuves](rust-hand-support-experiment-2026-10-11.md).
Le protocole ci-dessous reste conservé comme spécification préalable.

**But limité :** acquérir et stabiliser la base basse depuis l'accroupi,
avant de coder le parcours de remontée entier. Aucun nouveau test natif n'a
été lancé dans cette revue : la portée des échecs et le manque de préparation
des bras sont déjà établis par les sources et les traces.

1. Construire **un seul** corridor prospectif de descente vers mains/tibias,
   avec surface de main orientée et positions de contact identifiées. Employer
   les articulations complètes et la géométrie canonique. Vérifier limites,
   sol, auto-collision et continuité à chaque échantillon ; relever aussi les
   marges aux butées du coude/poignet. Une pose terminale sans chemin ne suffit pas.
   Les poses canoniques prone/half-kneel existantes peuvent amorcer ce calcul
   virtuel, jamais remplacer l’état physique courant. Les
   forces admissibles/friction/couples du terminal sont une présélection
   statique, clairement séparée de la preuve native.
2. Geler la descente des jambes et du tronc, les gains, l'allocation par demande
   et les durées. Exécuter A avec les bras de référence, B avec la préparation
   anticipée et la commande de surface de main. Même état initial canonique ;
   aucune transplantation à un état déjà au sol. Même ensemble d'appuis permis
   dans A et B pour que l'observateur ne crée pas la différence.
3. Limiter A et B à **3 s physiques chacun**. Si B passe le critère ci-dessous,
   faire une répétition fraîche puis un essai à 60° depuis sa construction
   normale. Maximum quatre essais, 12 s physiques, 120 s murales de simulation.
   Aucun balayage de gains ou série de nouvelles graines après refus. La
   préparation géométrique est séparée, bornée à 60 s/200 évaluations ; si elle
   échoue, rapporter « corridor non trouvé dans ce budget », pas « impossible ».

**Mesures et réussite du sous-problème :**

- Déclarer les surfaces mains/tibias/pieds permises par phase et observer à
  240 Hz leurs points, normales, charges normales/tangentielles, glissement et
  persistance. Les 3 N/0,05 s, distance de contact, normales et limites
  d'intégrité restent ceux du contrat. Garder le refus de tout appui tête/torse.
- B doit atteindre, au plus tard à 3 s, des mains et tibias prévus réellement
  porteurs, une charge totale sur les appuis déclarés ≥80 % du poids et un
  maintien continu de 0,55 s avec tous les corps ≤0,22 m/s et ≤0,65 rad/s.
  Mesurer COM/capture dans le polygone des **contacts résolus autorisés**, marge
  3 mm. Ce test conservateur d'appui intermédiaire ne qualifie pas le relevage.
- Mesurer l'évolution de vitesse/accélération du COM, du moment cinétique,
  l'impulsion horizontale apportée par les mains, les caps et impulsions
  articulaires. Le simple élargissement d'un polygone dessiné ne suffit pas.
  Une première touche à forte charge suivie d'une chute ne passe pas.
- B passant et A échouant établit l'intérêt causal de la préparation des bras
  dans **ce corridor**. Si les deux passent, la préparation anticipée
  n'est pas nécessaire dans ce corridor, sans que cela prouve l'inutilité des
  contacts de mains. B échouant n'élimine pas toutes les trajectoires avec mains.

**Refus et diagnostic :** conserver les gardes natives de finitude,
d'intégrité, limites articulaires/couples et absence d'assistance ; refuser un
appui non déclaré, une perte de tous les appuis autorisés pendant 0,20 s ou
l'absence du terminal à 3 s. Le compteur pieds-bilatéraux du banc précédent
est un invariant de sa famille, pas le compteur d'absence de tout support :
le nouveau mode doit compter les mains/tibias déclarés, sans modifier la durée
0,20 s. Classer séparément : corridor géométrique introuvable, IK/consigne non
suivie, moteur saturé, main mal orientée, glissement, contact trop tardif ou
transfert instable. Photographier le premier événement causal et l'état final.

Ne corriger l'allocation des bras que si les mesures prouvent un plafonnement
incohérent avec le propriétaire des consignes. Ne remplacer ni une force de
contact manquante par une assistance au bassin, ni un retard par un reset.

## Livraison et état de validation

Reprise proposée avec Sol : extension opt-in du banc par modes de contact,
corridor unique bras/tronc/jambes, paire A/B et répétition conditionnelle.
Pas de nouveau modèle coûteux nécessaire pour cette expérience définie.
Réutiliser l'intégrateur natif, l'allocation cohérente, les diagnostics et
les correctifs validés ; les essais rejetés restent hors du contrôleur livré.

Cette revue ne change aucun fichier Rust, contrat, modèle ou test. Elle ajoute
le plan, l'audit déterministe et `strategy-audit.png`, qui représente le témoin
archivé en chute. Les résultats unitaires antérieurs ne sont pas de nouvelles
preuves physiques. La récupération reste désactivée par défaut et non qualifiée.
