# Audit indépendant des preuves de parité native

Audit du 13 octobre 2026 après clôture du lot
`coordination-numeric-parity-serialization-2026-10-13T10:06Z`.
Les opérations de cet audit sont des lectures de fichiers, décompressions,
comparaisons de valeurs/bits, vérifications de sommes et d'empreintes. Elles
n'exécutent **aucun worker natif, aucune construction de modèle ou de monde,
aucune FK, aucun pas Rapier**. La seule écriture est cette note, hors manifeste.

**Verdict : les preuves établissent la parité annoncée sur les quatre pas
comparés de chacun des quatre cas.** Les commandes numériques archivées sont
bien celles installées dans les moteurs, et les états copie/réel comparés sont
identiques. Ce verdict ne constitue ni une acquisition des talons ni un relevage.

## Comptage et intégrité des preuves

Le registre porte `closed-success`, sans motif de refus. Il réserve 44 pas,
en rapporte exactement 44, ne rembourse aucun pas, et mesure
`1.4777865719988768` seconde, sous le plafond de 120 secondes. Son intervalle
commence à `2026-10-13T10:22:22.085649+00:00` et sa clôture est journalisée à
`10:22:23.563447+00:00`. Le processus lié est le PID 6288, un thread,
sans enfant déclaré par le superviseur.

Le rapport natif confirme simultanément :

- 44 admissions, 44 pas achevés, borne minimale = borne maximale = 44 ;
- 44 réservations locales et zéro pas réservé restant à intégrer ;
- préfixe commun de 12 pas, puis quatre cas de 4 pas copiés + 4 pas réels ;
- les quatre portes `integrity_ok`, `prefix_matches_archive`,
  `parity_verified`, `diagnostic_present` vraies.

Les 55 lignes de comptage durable comprennent les 44 nombres de pas de 1 à 44
dans l'ordre, avec 11 répétitions de fin de tick. Elles ne représentent donc
pas 55 intégrations. Les traces contiennent 12 observations du préfixe,
16 observations prédites, 16 réelles et 11 états après finalisation de tick.
Le journal stderr du worker est vide.

L'enveloppe SHA-256 du registre est valide. Les **326 fichiers gelés** du
manifeste et les **7 fichiers protégés** de la préparation ont été relus :
leurs empreintes sont inchangées. Les contenus bruts du rapport et de la
trace, directement lus ou décompressés, correspondent aux empreintes du
registre. Aucun ancien registre n'a été modifié par l'audit.

| Pièce | SHA-256 vérifié |
| --- | --- |
| Manifeste | `179bfe3cb58819cd2ba1b73642969c0eb2a343a052dee83ae89e2850e0830356` |
| Registre, fichier complet | `191d3216953bfb4aca5b7abcea277454d7d97ad5b4ca72e28ffbc260399af250` |
| Rapport natif brut | `91af43ee6a6ee5420b381ddefb5d722876304b25f403154d6e326fe4dbe0924c` |
| Trace native brute | `88802a9348cc7e8ff074b3396d6261e71e43026018168f972f748b3b5f891766` |
| Binaire lié par le superviseur | `83bddf95897acc6015de7d2947b04ee2f5aaf2eb49b4dcb85ba0a67764f95e1b` |
| Source `numeric_parity_worker.rs` | `ac89b99890d1842205bb1071967697a221c7fa2b929fd253d54e08035ccd04e0` |

## Comparaisons refaites indépendamment sur les archives

Les comparaisons n'ont pas seulement relu les booléens du rapport. Elles ont
parcouru les observations et installations enregistrées :

1. **12 états du préfixe** correspondent aux anciennes archives : tableaux
   u32 de corps/moteurs égaux, puis comparaison récursive des champs après la
   projection explicitement gelée. Cette projection exclut les métadonnées
   de trace et journaux CCD distincts ; le nouveau `numericAttempt` est exigé
   nul avant retrait. Les flottants sont comparés par leurs bits, zéros signés
   compris.
2. **32 commandes installées** — quatre commandes × quatre échelles × deux
   chemins — correspondent bit à bit aux commandes certifiées archivées aux
   indices 12–15. Les 75 positions/vitesses stockées, les cibles du simulateur
   et la mémoire de Recovery concordent avec cet oracle indépendant.
3. **1 472 installations d'axes moteurs** ont été vérifiées : 32 × 46 axes.
   Les `target_pos` et `target_vel` effectivement relus dans les joints sont
   ceux de l'oracle. `dq` est exactement la différence finie f32 de q sur dt.
   Les moteurs sont `ForceBased`, leurs axes canoniques sont activés, le
   plafond vectoriel est actif et les gains correspondent aux formules
   canoniques avec l'ordre d'arrondi f32 du profil gelé.
4. Les plafonds finaux relus respectent chaque plafond anatomique. La somme
   f64 des 46 `max_force` varie de **401,6021146774292 à
   759,999991953373 Nm**, donc reste ≤ 760 Nm. La vérification utilise les
   moteurs après éventuelle réparation ULP, pas seulement les caps journalisés
   avant cette réparation.
5. **16 paires d'installations copie/réel**, **16 paires d'états après pas** et
   **4 paires d'états après finalisation de tick** sont égales. La comparaison
   inclut le JSON exact du monde, corps, moteurs, contacts/charges/persistance,
   horloges, propriétaire, `modeSignature` et la mémoire Recovery complète.
   Chaque état final porte `nativeSubstep=16` et `tick=4`.

Les certificats du premier tick contraint sont effectivement présents, avec
quatre commandes certifiées par cas et la tentative finale
`certified guarded exact f32 commands` :

| Facteur | Itérations | Erreur dure déclarée | Résidu du gradient projeté |
| --- | ---: | ---: | ---: |
| 0 | 8 | 1,7347 × 10⁻¹⁸ | 6,4889 × 10⁻⁹ |
| 0,5 | 8 | 5,5511 × 10⁻¹⁷ | 9,3625 × 10⁻⁹ |
| 1 | 12 | 5,5511 × 10⁻¹⁷ | 1,7328 × 10⁻⁷ |
| 1,5 | 12 | 1,1102 × 10⁻¹⁶ | 1,7785 × 10⁻⁷ |

Ces valeurs respectent les seuils gelés de `1e-8` et `1e-6`. Les tentatives
directes refusées sont conservées comme diagnostics ; elles ne sont pas
utilisées comme oracle des commandes installées.

## Conservation effective du bit 127

Les mondes sont archivés avec l'encodage explicite
`serde-json-text-preserving-u128-v1`. Leur champ `text` a été comparé exactement
entre copie et réel, puis lu avec des entiers Python arbitrairement grands pour
contrôler les valeurs, sans conversion en flottant.

Le premier état achevé contient notamment :

```text
step1/narrow_phase/body_solver_color_masks/19
= 170141183460469231731687303715884105728
= 2^127
```

L'audit retrouve cette valeur exacte 56 fois dans les 29 textes de mondes
distincts conservés. Le défaut précédent de passage forcé dans un `Value`
borné à u64 est donc effectivement dépassé sur un état natif contenant le
bit 127, au-delà du seul test synthétique. Aucune comparaison ne tronque cette
valeur à u64 ni ne l'arrondit en f64.

## Portée et limites physiques

La preuve est limitée à la fixture native `landed-crouch-left`, orientation
0 rad, telle qu'archivée. Ce lot ne simule aucun nouvel impact.

Chaque cas compare **4 / 240 s = 16,67 ms** depuis le même checkpoint au pas 12.
Le préfixe commun représente 50 ms ; chaque branche s'arrête à 66,67 ms depuis
la fixture initiale. Les 44 pas sont un coût total de calcul sur ces branches,
pas une histoire continue de 44 pas.

Les quatre diagnostics locaux finaux portent `passed=false`. Les deux talons
ont une charge courante nulle (encodée `-0.0` dans les traces) et aucune
persistance qualifiée. Les maxima de vitesse finale vont de 0,436 à 0,454 m/s
et de 3,43 à 3,97 rad/s selon le cas : cette courte vérification n'établit pas
la stabilité locale. Elle ne démontre ni l'impossibilité de l'objectif causal,
ni une préférence physique entre les quatre facteurs.

Les journaux CCD du chemin réel sont conservés séparément ; le chemin copié
n'en collecte pas. L'égalité prouvée porte sur les états persistants natifs et
leurs contacts/impulsions comparés, pas sur une égalité de journaux CCD absents
ni sur tous les espaces de travail omis par la sérialisation. Aucun bilan
angulaire global supplémentaire n'est qualifié.

**Le résultat permet de poursuivre la préparation d'un prochain jalon causal ;
il ne vaut pas succès local, continuation longue, relevage après impact ou
qualification des 24 poses/cycles/impacts.** Aucun nouveau calcul physique ni
crédit de budget n'est autorisé par cet audit.
