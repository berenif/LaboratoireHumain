# Référence archivistique du protocole de parité

[expected.json](expected.json) fige les états du préfixe aux sous-pas 1–12 et les commandes numériques certifiées aux sous-pas 12–15. Son SHA-256 est `afcd7278f05b1d08e9434507042f77c5e0a24a1d5c235ba68b84d8ec0ae05ab8` ; taille : 1 633 929 octets. Cette référence ne contient aucun résultat du nouveau protocole physique.

Le [script d'extraction](extract_expected.py) et le [reçu](extraction-receipt.json) conservent les empreintes des sources, la correspondance gzip/brut et les douze contrôles d'égalité entre anciens cas. Reproduction puis contrôle sans réécriture :

```sh
python3 docs/evidence/coordination-native-parity-2026-10-13/extract_expected.py
python3 docs/evidence/coordination-native-parity-2026-10-13/extract_expected.py --check
```

## Interface pour le worker

```text
schema: 1
prefixFeedbackScale: 1.0
prefixFrames: [12 anciennes lignes JSON, nativeSubstep 1..12]
prefixBodyAndMotorBits: [12 tableaux de 2341 u32]
cases: [
  { feedbackScale: 0|0.5|1|1.5, iterations: 8|8|12|12,
    commands: [{ nativeSubstep: 12..15,
                 positions: [[f32;3];25], velocities: [[f32;3];25] };4] }
]
```

`prefixFrames` reprend les lignes 25–36 originales de `002-B.states.jsonl`, groupe `preview-0-2`. Le facteur de feedback vaut donc déjà 1, sans modification du owner. Tous les champs sont conservés : snapshot, support, moteurs, cibles, persistance, diagnostics et owner. À chaque sous-pas, les quatre anciens groupes sont identiques après exclusion du seul `kind` et normalisation de `owner.feedbackScale`. L'extracteur vérifie cette égalité sur la représentation JSON canonique, qui conserve les zéros signés.

`cases[*].commands` provient exclusivement de `certified-commands.json:cases[*].audit.guardedAttempt.attemptedCommands`. Chaque cas est marqué `actualF32CommandsCertified=4`, `replayedCommandBitsEqual=true` dans sa source ; son `previousQ` correspond au owner du pas 12. La tentative `directAttempt`, refusée, n'est pas utilisée. Les 75 positions et 75 vitesses de chaque commande sont conservées, root et axes verrouillés compris ; toutes sont exactement représentables en f32.

## Comparaisons exactes

Les bits de corps et moteurs sont rangés, pour chaque frame, dans l'ordre suivant :

1. Les 25 corps `support.bodies`, segments 0–24 : position xyz, rotation xyzw, vitesse linéaire xyz, vitesse angulaire xyz, soit 325 u32.
2. Les 24 joints `motors`, segments 1–24 ; pour chacun, ses six moteurs dans l'ordre archivé. Pour chaque moteur : `target_pos`, `target_vel`, `stiffness`, `damping`, `max_force`, `impulse`, `delivered_abs_impulse`, **`delivered_impulse` signé**, puis `delivered_angular_impulse_1` xyz et `delivered_angular_impulse_2` xyz. Soit 14 u32 par moteur et 2 341 u32 au total.

Cette liste étend les anciennes 2 197 valeurs de `native_plan_preview::parity_frame` en ajoutant l'impulsion signée, placée après l'impulsion absolue. Le layout complet figure dans le JSON. Les autres champs moteur, dont erreurs livrées et modèle ForceBased, restent présents dans `prefixFrames` pour comparaison séparée.

Comparer en bits f32 les corps, moteurs et commandes. Pour les autres nombres, notamment `owner.rates` et l'arithmétique de support, conserver f64 et comparer aussi leur représentation si la parité revendiquée est bit à bit. Une simple égalité numérique `0.0 == -0.0` ne suffit pas : les 48 anciennes lignes contiennent 144 zéros négatifs. Aucun quaternion ne doit être renormalisé ou assimilé à son opposé.

Seul `kind` est une métadonnée de trace explicitement différente. Les anciens `ccdJournalCollected=false` et `ccd=[]` signifient qu'aucun journal n'a été collecté dans ces previews ; ils ne prouvent pas l'absence d'intervalles CCD. Comparer le nouveau journal séparément, en déclarant cette différence. Tout nouveau champ de diagnostic owner absent de l'ancienne version doit être explicitement identifié et contrôlé nul pendant le préfixe, puis exclu de la projection de parité ancienne.

## Horloge et limites des preuves

Après le pas physique 12, `owner.previousStep=11`, `owner.nextSubstep=12`, `owner.last.commandSubstep=11`, et le snapshot a `tick=2`, `sequence=13`. La commande suivante est celle de `configure(12)`, au tick 3/sous-pas 0. Le owner du préfixe est l'état accepté avant cette tentative ; il ne faut pas le remplacer par l'ancien `terminalOwner` après refus, déjà modifié.

Les anciennes archives B s'arrêtent au pas 12 pour chaque preview : zéro pas dans l'histoire réelle, quatre copies de douze pas. Elles ne fournissent **aucun état physique B aux pas 13–16**. Les commandes 12–15 sont une preuve numérique antérieure et ne préjugent ni de leurs effets physiques ni du succès local. La présente extraction effectue **0 construction de monde, 0 intégration, 0 FK et 0 requête géométrique** ; elle ne consomme aucun des 44 pas physiques autorisés au nouveau protocole.
