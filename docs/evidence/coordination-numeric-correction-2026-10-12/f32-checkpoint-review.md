# Revue historique du contrat numérique f32 — 12 octobre 2026

Revue en lecture seule des sources et documents Git, destinée à la correction
statique après clôture du prototype. Aucun test, FK, monde natif, compilation ou
rejeu physique exécuté. Aucun registre modifié. Les numéros de ligne ci-dessous
désignent **les versions au SHA indiqué**, indépendamment des corrections en
cours dans l'arbre de travail.

Le précédent exact existe : l'ordre des opérations `f32` de la commande doit
être identique dans le prédicteur et dans l'exécution. Il ne constitue pas un
algorithme déjà validé pour convertir une solution continue en commandes
discrètes respectant toutes les contraintes communes. Le prototype conserve
heureusement le refus après conversion ; son manque est la construction d'une
commande admissible et le diagnostic numérique précis du refus.

## Sources retrouvées et portée exacte

| SHA | Fichier et lignes dans ce commit | Contrat retrouvé |
| --- | --- | --- |
| `dcbcdfabaace5f3fbe91c944cb99efe5687276ac` | `docs/rust-prone-roll-horizon-equivalence-2026-10-11.md`, 8–26 et 30–45 | `(base + ancien_delta) + incrément` et `base + (ancien_delta + incrément)` diffèrent de 1–2 ULP sur quatre vitesses. La même reconstruction rétablit les quatre pas exacts ; réémettre les valeurs figées ne suffit pas. |
| même SHA | `rust/crates/sim/src/recovery/native_predictor.rs`, 150–216, 287–298, 375–408, 424–489 | `predict_reprepared_horizon` réinstalle les commandes absolues ultérieures ; positions, gains, caps, modèles et masques sont protégés. `restore_motor_command` conserve les impulsions vivantes. La régression reconstruit les vitesses en `f32`, contrôle les hash indépendants, corps, impulsions et charges. |
| même SHA | `docs/rust-prone-roll-native-command-contract-2026-10-11.md`, 35–50 et 63–68 ; `native_predictor.rs`, 268–283 | Un delta nul conserve la commande sans conversion ni notification d'édition ; une cible de vitesse `ForceBased` n'est pas un couple moyen. Le certificat QP antérieur portait sur la proposition **avant** clipping ; une validation native fraîche contrôlait le résultat. |
| même SHA | `rust/crates/sim/src/recovery/trajectory_program.rs`, 104–122, 126–150, 165–185 et 192–207 | Toute la grille `f32` à 240 Hz est validée, intérieurs compris ; `q` et `dq` viennent du même Hermite absolu. Horloge identique → mêmes bits, sans curseur ni delta cumulé. Le contrat ne disait pas `dq = (q[n] - q[n-1]) / dt`. |
| `891b4a4acf74afd3d9c5c7d18b826296b4cc3e1d` | `rust/crates/sim/src/recovery/support_probe/getup_ab.rs`, 272–305 et 488–493 | Bornes de position/slew rendues représentables vers l'intérieur avec `next_up`/`next_down`. Sans offset effectif, position/vitesse nominales préservées ; assertions bit à bit. Il s'agit d'un autre contrat, nominal + offset, avec clipping de vitesse, sans certificat des tâches du nouveau solveur. |
| `dcbcdfabaace5f3fbe91c944cb99efe5687276ac` | `docs/rust-native-control-2026-10-11.md`, 17–30 et 94–99 | Optimisation de réponse native bornée ; correction antérieure d'une asymétrie QP de `3,64e-12` par calcul d'un triangle puis miroir, sans élargir les seuils. Petits pas Newton définis positifs résolus au lieu d'être refusés pour leur seule longueur. |

Les documents du 11 octobre donnent `c597959...` comme HEAD de leur session,
mais les modifications étaient alors locales. Leur **commit de conservation**
est `dcbcdf...` ; attribuer ces sources nouvelles au contenu de `c597959...`
serait incorrect. Les nombres de tests et résultats dynamiques dans ces documents
sont historiques, pas revérifiés par cette revue.

Les prototypes de l'optimiseur natif avaient été retirés avant ce commit
(`rust-native-control`, lignes 28–30 ; `horizon-equivalence`, lignes 107–109).
Le dossier local `evidence/` et les snapshots annoncés `source-aligned/` ne sont
pas présents dans cet environnement. Les recherches `git log --all` par chemin
et contenu n'ont retrouvé aucun ancien fichier nommé `native_response*` ou
`coordinated_compiler*`. Le comportement documenté et l'API de reconstruction
conservée sont consultables ; le corps de cet ancien optimiseur n'est pas une
implémentation récupérée ni auditée ici.

## Écart introduit par le prototype clos

Référence figée : `b6e711d3dc58909979295df81437e6a5579c41db`.

Dans `planner/coordinated_runtime.rs`, lignes 261–286, le solveur produit 46
vitesses `f64`, puis vérifie les quatre commandes effectivement émises.
`command`, lignes 307–340, effectue :

```text
q = f32(f64(q_précédent) + vitesse_f64 × f64(dt_f32))
éventuel recul de q d'un ULP pour respecter le slew
dq = f32(f32(q - q_précédent) / dt_f32)
```

Les vitesses émises sont donc quantifiées en fonction de la position précédente,
et ne sont généralement pas les vitesses de la solution `f64`. Le recul d'un ULP
protège une borne scalaire de déplacement ; il ne protège pas les combinaisons
linéaires des tâches communes. À des angles d'ordre un radian, une fraction
d'ULP de position divisée par environ `1/240 s` peut produire une erreur de
vitesse d'ordre `1e-5 rad/s`, supérieure au seuil numérique `1e-8` du problème.
Ce constat est analytique, sans mesure nouvelle sur le cas clos.

`whole_body_tasks.rs`, lignes 5–9 et 106–111, confirme le type `f64` et le refus
au-delà de `1e-8`. Le contrôle des quatre commandes est correct et doit rester.
Le test de `coordinated_runtime.rs`, lignes 357–383, prouve la définition de
`dq` et les horloges sur un cas ; il ne prouve pas la faisabilité de chaque
commande quantifiée pour une solution proche d'une face active.

Autre différence explicite : `whole_body_controller.rs`, lignes 390–405,
remplace aussi le `dq` Hermite nominal du témoin A par la différence finie des
positions émises. A est bien le témoin du nouveau protocole ; il n'est pas un
rejeu bit à bit du contrat Hermite antérieur. Une restitution du vieux `dq`
modifierait ce témoin et ne serait pas une simple correction d'arrondi.

L'écart établi est ainsi une **absence de garantie entre proposition et
commande représentable**, pas une preuve de divergence copie/réel. Le précédent
du 11 octobre concernait deux séquences de commandes différentes. Le prototype
partage déjà son compilateur entre copie et réel. Ses refus de convergence
restent par ailleurs distincts du refus après compilation : l'historique ne
prouve pas que leur cause est identique.

## Règle transférable et conditions de correction

1. Fixer une seule définition opérationnelle de `q/dq`, son association
   arithmétique et sa mémoire ; la faire utiliser par la décision, les sondes,
   toutes les prévisions et l'exécution réelle.
2. Évaluer les contraintes sur les valeurs effectivement représentables, à
   chacun des quatre sous-pas. Conserver le seuil et les refus originaux.
   Un arrondi indépendant ou un clipping ne transmet pas le certificat QP.
3. Construire une proposition avec une marge d'arrondi démontrée pour les
   lignes concernées, ou résoudre/rechercher explicitement une commande dans
   l'ensemble représentable. Une marge doit resserrer l'admission, jamais
   élargir les critères. Une égalité sans intérieur peut exiger une méthode
   différente ; l'existence d'une solution représentable n'est pas acquise.
4. Journaliser avant refus la ligne fautive, ses bornes, le résidu `f64`, les
   quatre `q/dq` émis et leurs résidus. Vérifier statiquement les limites, les
   angles négatifs, les faces actives, le zéro et les répétitions d'horloge.
5. Ne transférer aucune qualification physique des anciens essais. La parité
   native ancienne est locale, sous ses commandes, états et moteurs ; une
   correction numérique nouvelle ne prouve ni talons acquis ni relevage.

Cette revue propose des obligations de correction, pas un nouveau budget ni
une admission physique. Elle ne modifie aucun source, test ou registre.
