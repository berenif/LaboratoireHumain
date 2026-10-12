# Revue logicielle indépendante des briques de coordination — 12 octobre 2026

Cette revue porte sur les nouveaux `whole_body_tasks.rs`,
`coordinated_runtime.rs` et `support_state.rs` dans l'arbre de travail de la
branche `investigate/physical-support`. Elle n'exécute aucun test, intégrateur,
prévision native, calcul physique ou logiciel tiers. Les compilations et tests
effectués séparément par le parent doivent être cités par leurs reçus, pas par
cette lecture. Aucun budget de physique n'est consommé ici.

**Verdict de portée : ces fichiers constituent des briques logicielles
testables, pas encore un contrôleur de transfert admis à l'exécution.**
Le solveur commun traite effectivement les 46 taux dans un même problème et
place les tâches dures dans les contraintes. L'échec de projection ou de
convergence retourne un refus. Le compilateur produit ensemble les positions
et les vitesses, sans commande de racine, et vérifie les quatre commandes f32
d'une décision. Le suivi matériel conserve le même point local et archive
explicitement les remplacements en pivot. Ces propriétés sont pertinentes ;
aucune ne suffit à établir des réactions de contact ou un relevage natif.

## Défauts transmis au responsable des deux autres modules

Les constats suivants décrivent la version initialement lue. Le parent est
responsable des corrections et de leur validation ; ce réviseur n'a pas
modifié ces deux fichiers.

1. **Calculs non finis masqués dans la projection.** Des coefficients, poids et
   cibles tous finis peuvent produire un gradient infini. `project` ne vérifiait
   ni son entrée, ni ses corrections, ni toutes les opérations intermédiaires.
   Un `clamp` peut ramener une entrée infinie sur une borne finie, puis une
   différence `Inf - Inf` devient NaN. `f64::max` ignore un opérande NaN : le
   contrôle de variation pouvait alors annoncer une convergence. Exemple de
   famille : ligne constante `1e-100`, poids `1e100`, cible `1e300`. Sa courbure
   calculée reste finie mais le résidu pondéré déborde. Correction demandée :
   refus explicite des intermédiaires non finis, y compris gradient, entrée et
   corrections de projection, produit scalaire, norme, déplacement et résidu.

2. **Bornes invalides masquées avant validation.** `decide` intersectait les
   bornes fournies avec les bornes natives au moyen de `max/min`, avant de
   laisser `Problem::solve` valider. Une borne NaN peut ainsi être remplacée
   silencieusement par une borne valide. Correction demandée : valider le
   problème original avant l'intersection.

3. **Amplitude différentielle et largeur.** `from_samples` vérifiait chaque
   amplitude mais pas la finitude de leur différence. Les amplitudes
   `[-1e308, +1e308]` donnent une largeur infinie, puis une dérivée nulle et
   finie : un faux échantillon pourrait être accepté. Une variation effective
   entre deux taux totaux admis dans `[-1,1]` ne peut pas dépasser 2 rad/s.
   Corrections demandées : largeur finie, amplitude admissible, puis traçage
   ultérieur des taux de référence et des commandes réellement compilées.
   L'API numérique seule ne prouve pas la provenance des échantillons.

4. **Débordement de l'horloge.** Le dernier multiple de quatre représentable,
   `u64::MAX - 3`, franchit le contrôle de cadence mais fait déborder
   `native_substep + 4`. Correction demandée : calcul de fin avec `checked_add`
   avant la boucle de validation. La commande individuelle utilisait déjà un
   incrément vérifié.

5. **Validation du modèle trop tardive.** `initialize` appelait le validateur
   de commandes avant le contrôle des 25 segments. Un modèle plus grand peut
   indexer hors des tableaux de commandes avant d'être refusé. Correction
   demandée : contrôle du modèle en premier ; vérifier également l'unicité
   des coordonnées et la finitude/l'ordre de leurs limites.

6. **Distinguer contraintes de taux et tâches physiques prédites.** La
   validation de quatre commandes f32 vérifiait l'anatomie et le slew, pas les
   tâches dures avec les taux effectivement émis après arrondi. Autour de
   `q=1`, un taux demandé de `0.1` peut devenir environ `0.099993` après
   l'arrondi de la position. Cet écart dépasse `1e-8` et compte pour une égalité
   de taux exacte. Si une tâche contraint le taux réel, il faut vérifier son
   résidu après compilation. Si sa ligne prédit une sortie physique, sa
   tolérance dépend du modèle de réponse et doit être vérifiée par une
   prévision native : le résidu algébrique n'est pas ce certificat.

## Durcissement effectué dans `support_state.rs`

La même lecture a motivé des corrections limitées à ce fichier : refus des
sommes de charge et du point de capture non finis malgré des entrées finies ;
calcul des normes par `hypot` ; refus des produits géométriques débordants
dans le test du polygone ; refus d'une dérive matérielle non finie avant toute
mutation de la dernière mesure. Un remplacement de pivot invalide ne doit
pas ajouter une nouvelle ancre avant de découvrir l'erreur de l'ancienne.

Deux tests synthétiques supplémentaires couvrent le débordement avec entrées
finies et l'absence de mutation après une mesure matérielle refusée. Ils
s'ajoutent aux neuf tests initiaux : séparation CCD/support, exclusion de la
main libérée, maintien de la tâche jusqu'à preuve, conservation du point
matériel, persistance propre des deux talons, durée complète de stabilité,
observation des 25 corps et polygone avec marge. Aucun de ces tests n'intègre
la physique. Leur exécution reste à établir dans le reçu du parent.

## Ce qui manque avant un runtime physique interprétable

- Un adaptateur natif validé doit produire les **contacts actuellement
  sélectionnés et encore présents**, leurs points locaux, la charge mesurée
  et l'état des 25 corps. La somme des impulsions des intervalles CCD reste
  un journal séparé. L'ancienneté globale du segment ne peut pas devenir
  l'ancienneté d'un nouveau talon.
- La référence complète doit inclure l'acquisition, les pivots/remplacements,
  le recentrage et une continuation admissible vers debout. Une séquence de
  commandes compilable n'est pas une trajectoire d'appuis admissible.
- `Problem` est assemblé à l'extérieur et `ResponseMap::require_current`
  contrôle seulement l'âge et une signature de mode. Cette interface ne
  démontre pas que le problème utilise les lignes de cette carte, leurs
  unités, leurs taux de référence ou une région de confiance. Ces liens
  doivent être construits et journalisés avant un essai causal.
- L'âge maximal de 24 sous-pas et la signature de mode sont des motifs de
  refus utiles ; ils ne suffisent pas à vérifier la qualité prédictive au
  sein d'un même mode. Il manque la comparaison prédiction/réalisation et la
  règle de refus lorsqu'une carte devient inutilisable.
- Le rollout doit copier l'état natif **et** la mémoire, réexécuter le même
  propriétaire de commandes et la même allocation à leurs cadences, sans
  planification récursive. Sa première étape doit retrouver la réalisation
  sous la même commande. Les prévisions et sondes doivent toutes réserver
  leurs sous-pas dans le registre avant exécution.
- Le contrôle local ne modifie ni le compteur de récupération ni ses seuils.
  Son éventuel succès reste distinct de debout en 25 s, de la seconde de
  stabilité, du handoff et des 24 poses/cycles/impacts de qualification.

En l'absence d'une route d'appuis admise, refuser de lancer B constitue la
bonne fermeture de cette préparation. Ce refus ne prouve ni l'impossibilité
anatomique, ni l'inutilité d'un retour global, ni la nécessité universelle des
mains ou des genoux.

## Clôture des constats avant admission native

Les refus non-finis/NaN, la validation des tâches dures, les débordements
synthétiques et la cohérence des références f32 ont été corrigés puis testés.
La carte native courte reste une infrastructure séparée ; le propriétaire
exécutable reconstruit ses tâches géométriques et copie sa mémoire explicite.
La continuation numérique de huit secondes a maintenant ses 1 921 inspections
natives dans chacun des deux repères. La revue finale du propriétaire est
archivée dans `owner-final-review.md`.

Les 40 tests Rust ciblés de préparation passent ; aucun de ces tests n'intègre
le monde physique. Le chemin de production passe `cargo check --lib --locked`.
Ces contrôles ne remplacent pas les gates de parité native et de transfert.
La suite globale Rust/TypeScript n'a pas été relancée : ses échecs historiques
ne sont ni corrigés ni présentés comme des résultats de ce prototype.
