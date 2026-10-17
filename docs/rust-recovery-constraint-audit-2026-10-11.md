# Audit causal du refus QP à 37,5 ms — 2026-10-11

Travail local sur `investigate/physical-support`, référence `c597959f46f37715035349c8cd0a699b6822111a`. Aucune publication. Récupération désactivée par défaut et non qualifiée.

## Décision

Le refus historique à 37,5 ms vient de deux contraintes numériques parasites introduites par la réduction des contacts redondants. Il ne prouve ni l'impossibilité d'un appui pieds seuls, ni un manque de couple, ni la nécessité d'un appui main/genou. Reprendre sous Sol avec une construction indépendante des contraintes de corps rigide, avant tout autre réglage de commande.

Deux points collants distincts sur un même avant-pied imposent cinq contraintes indépendantes : trois vitesses à une ancre et deux composantes de vitesse angulaire perpendiculaires à la ligne des contacts. Les quatre contacts résolus (deux par avant-pied, aucun talon) apportent donc dix contraintes, auxquelles s'ajoutent six équations de bilan mécanique. La construction originale en f32 puis Gram–Schmidt conserve pourtant 18 lignes : elle amplifie deux directions d'arrondi, de valeurs singulières 2,5001e-8 et 9,0874e-9.

## Preuves reproductibles

Dossier : `evidence/constraint-audit-2026-10-11/`.

- `exact-witness/0009.json` : état, matrices, cinématique complète, contacts, limites et objectif au refus.
- `exact-replay-comparison.json` : les 25 états de segments finaux et les deux snapshots préfixes sont identiques au rapport historique. Pour respecter strictement 760 Nm, l'excès d'arrondi plus 0,00001 Nm est retiré du plafond gauche de cheville AngX avant intégration ; cette correction ne change aucun état corporel enregistré. Chaque somme installée est vérifiée en f64. Sommes aux deux snapshots : 759,999991963 et 759,999896348 Nm, sur les 46 axes actionnés (hors sentinelles des axes verrouillés).
- `analytic_contacts.py` reconstruit en f64 les contraintes cinématiques indépendantes par corps rigide. Il conserve les six équations de bilan, les quatre contacts collants et toutes les inégalités articulaires, de friction et de couple. Il n'augmente aucun seuil.
- `analytic-contact-audit.json`, première entrée : certificat de Farkas du système réduit original, résidu dual 2,5101e-11, second membre −1,0000000000014, multiplicateurs d'inégalité non négatifs. Le système indépendant à 16 lignes est faisable : résidu d'égalité 9,77e-15, marge d'inégalité minimale 6,71e-7. Cette résolution indépendante resserre même les inégalités normalisées de 1e-6. Les quatre vitesses de contact reconstruites sont inférieures à 2,33e-15 m/s ; budget alloué 759,998730373 Nm. Ces vitesses sont prédites par le modèle, pas mesurées après intégration.
- `fixed-state-rust.json` et `fixed-state-build.log` : **même solveur Rust**, même état, même objectif et mêmes inégalités, ancienne formulation refusée, formulation indépendante résolue. Résidu d'égalité 7,82e-14, violation maximale −6,86e-14. Un test ignoré dédié passe ; zéro intégration physique.
- `verify_primary.py` vérifie indépendamment les témoins et le certificat ; `constraint_audit.rs` et `constraint-audit-test-binary` conservent le diagnostic hors du code final.

Reproduction des calculs : `python3 evidence/constraint-audit-2026-10-11/analytic_contacts.py`, puis `python3 evidence/constraint-audit-2026-10-11/verify_primary.py` (NumPy/SciPy). Reproduction Rust sans compilation : définir `LH_AUDIT_ROOT` au chemin absolu de ce dossier puis lancer `constraint-audit-test-binary recovery::support_probe::constraint_audit::solve_fixed_state --ignored --exact --nocapture`. Le source archivé peut être réattaché comme enfant du module de test `support_probe` pour recompilation avec Rust 1.89.0, Cargo offline/locked/release.

## Pipeline réel et limites de l'audit

Le témoin historique utilise un pas direct à 20 passes PGS. Le pipeline `advance_tick_observed` utilise 32 passes en chute/récupération (lib.rs, sélection avant `step_with_events`). Ce nombre n'est pas le nombre de sous-pas temporels : le solveur vendored utilise ici 20 itérations solveur + 5 itérations additionnelles de corps, soit 25 sous-pas temporels. La commande reste mise à jour au pas externe de 1/240 s.

`pipeline-kinematics/0014.json` est une observation du vrai pipeline, sans installer la commande QP. À 58,33 ms la formulation indépendante est également faisable là où Rust refuse. Cela confirme la pertinence du défaut au pipeline ; ce n'est pas un essai de la commande corrigée à 32 passes.

Les autres diagnostics sont exploratoires : la marge répartie (`direct20`, `direct32`) et la réparation sur la tête (`historical-replay`) changent le chemin physique et ne reproduisent pas le refus historique. Les LP sur les lignes brutes presque dépendantes donnent parfois des résultats contradictoires selon le présolveur. Les fichiers `independent-audit.json` et `certificates.json` ne constituent donc pas une preuve générale d'impossibilité mécanique. Utiliser le certificat réduit et le témoin analytique explicitement vérifiés ci-dessus. D'autres états peuvent présenter de vraies incompatibilités du mode collant ou des plafonds individuels ; leur résolution reste ouverte.

## Test causal suivant, borné, pour Sol

1. Construire les contraintes de contact en f64 par corps rigide : ancre + sous-espace angulaire indépendant déterminé uniquement par la géométrie des points. Dédupliquer les points confondus. Ne pas supprimer une contrainte physique sur la seule petitesse d'un résidu numérique.
2. Rejouer l'état figé à 37,5 ms avec objectif, inégalités et solveur inchangés. Exiger résolution et validation de toutes les vitesses ponctuelles originales ; conserver un test de non-régression indépendant du contrôleur.
3. Conserver une allocation native dont la somme f64 des plafonds f32 réellement installés ne dépasse jamais 760 Nm, avec marge d'arrondi retirée avant la physique.
4. Faire une seule comparaison bornée de 0 à 100 ms depuis l'accroupi réel, via le pipeline natif à 32 passes : ancien modèle contre formulation indépendante, aucune autre modification. Deux répétitions déterministes. Arrêter au premier refus/invariant violé et exporter l'état complet.
5. Comparer contacts réellement résolus (impulsions, talons/pointes), accélération du centre de masse, couples demandés/livrés, dérive, limites et collisions. Si le refus suivant persiste, auditer ce nouvel état avant prolongation ; aucune recherche de gains aveugle.

Ce jalon n'établit ni transition au talon, ni recentrage, ni remontée, ni stabilité finale. La question pieds seuls contre appui intermédiaire reste ouverte. Les précédents maintiens préparés à 7 s et les poses géométriques ne qualifient pas le relevage.

## Vérifications et livraison

Le diagnostic contrôleur temporaire est retiré du code final. Les corrections de support mesuré du checkpoint précédent sont conservées. Aucun seuil, anatomie, assistance au bassin ou reset ajouté. Aucun test complet workspace/TypeScript/WASM/navigateur n'est déclaré passant ; les échecs de référence signalés restent hors du périmètre de ce jalon. Les corrections locales de l'autre environnement n'ont pas été importées.

Vérifications finales : 25 tests filtrés `recovery::` passent dans le binaire archivé (4 ignorés), le test causal ignoré est exécuté séparément et passe ; Clippy du source final `-p lh-sim --tests -- -D warnings`, formatage et `git diff --check` passent. Ce filtrage n’est pas la suite complète.
