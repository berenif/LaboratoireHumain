# Provenance et portée des seuils de transfert

**80 % de charge restante, 25 % de charge libérée et un retrait de 3 mm sont des gardes conservatrices du contrôleur et des protocoles gelés. Ce ne sont pas des constantes anatomiques ni des conditions nécessaires de tout mouvement dynamique.** Leur conservation reste obligatoire dans la famille déjà figée ; leur origine algorithmique ne permet pas de les changer après un échec. Aucun seuil n’est modifié ici.

Audit en lecture seule sur le commit `b9809dfdc2db1a5317379e7914c220d4e856fdbc`, avec `git log -S/-G`, `git blame`, lectures des objets Git et des documents. Zéro monde, `Model` ou `Simulation` construit, zéro FK, solveur ou intégration native. Les exemples chiffrés ci-dessous sont de simples conséquences algébriques, pas de nouvelles observations.

## Chaîne de provenance vérifiable

| Objet | Première provenance pertinente retrouvée | Statut établi par cette source |
| --- | --- | --- |
| Charge du membre libéré ≤25 % | Document du 26 septembre, ajouté au commit `493d887afe044838f2615102fc882aaea8c53aad`, [contrat lignes 76–85](https://github.com/berenif/LaboratoireHumain/blob/493d887afe044838f2615102fc882aaea8c53aad/docs/physics-standing-contract-2026-09-26.md#L76-L85) et [cohérence corporelle lignes 54–65](https://github.com/berenif/LaboratoireHumain/blob/493d887afe044838f2615102fc882aaea8c53aad/docs/body-physical-coherence-acceptance.md#L54-L65) | Garde gelée du pas intentionnel, avec charge restante **52 %**, pendant 0,10 s ; une charge prédite ne suffit pas. Ce document ne demande pas 80 %. |
| Capture à 3 mm dans le support | Migration Rust du 2 octobre, commit `186b747d89ec634a024ba4d7cc6c3c6da7c674f1`, [stepping lignes 285–298](https://github.com/berenif/LaboratoireHumain/blob/186b747d89ec634a024ba4d7cc6c3c6da7c674f1/rust/crates/sim/src/stepping.rs#L285-L298) | Valeur déjà présente dans une garde algorithmique de pas. Cette version intermédiaire utilisait alors 65/35 % ; elle ne justifie pas les seuils ultérieurs. |
| Pas strict : 52/25 % et 3 mm | Commit `61f640aeaae1280ce9252a4954ead02f80f65446`, [document du 9 octobre lignes 30–36](https://github.com/berenif/LaboratoireHumain/blob/61f640aeaae1280ce9252a4954ead02f80f65446/docs/rust-strict-stepping-2026-10-09.md#L30-L36) | Le pas strict restaure/documente 52/25 %, 3 mm et 0,10 s, avec des preuves distinctes de décollage et de réception. |
| **Récupération : 80/25 % et CoM + capture à 3 mm** | Ajout du contrôleur de récupération au commit `27ea010c3d856d00805ce9266feebc94cf7b77f1`, [fonction lignes 234–294](https://github.com/berenif/LaboratoireHumain/blob/27ea010c3d856d00805ce9266feebc94cf7b77f1/rust/crates/sim/src/recovery.rs#L234-L294) | `released_load > 0.25*weight` refuse à la ligne 243 ; `remaining_load < 0.8*weight` refuse à la ligne 266 ; CoM et capture sont testés à 0,003 m aux lignes 293–294. Les trois apparaissent ensemble dans cette fonction. Le commit décrit un contrôleur expérimental encore non qualifié. |
| Diagnostic local expérimental : 80 % et 3 mm | Proposition du 12 octobre au commit `e3b4ed3b2ddaac5450c2846ec12c736a1287082e`, [lignes 299–319](https://github.com/berenif/LaboratoireHumain/blob/e3b4ed3b2ddaac5450c2846ec12c736a1287082e/docs/coordination-architecture-review-2026-10-12.md#L299-L319) | Ces valeurs deviennent explicitement des critères du diagnostic local de talons/recentrage ; les seuils doivent rester identiques entre A et B. Ce diagnostic ne remplace pas le terminal de récupération. |
| Extraction pure pour le planner | Commit `b6e711d3dc58909979295df81437e6a5579c41db`, [support_state lignes 8–19](https://github.com/berenif/LaboratoireHumain/blob/b6e711d3dc58909979295df81437e6a5579c41db/rust/crates/sim/src/recovery/planner/support_state.rs#L8-L19) | Reprise explicite des constantes 0,8, 0,25 et 0,003. Ce commit ne fournit pas une nouvelle justification physique de leurs valeurs. |

Les numéros de lignes ci-dessus appartiennent aux **commits liés**, pas au fichier courant : par exemple les lignes 243/266/293–294 du premier transfert sont aujourd’hui 275/298/325–326.

Le JSON général [recovery-contract-v1.json, lignes 5–41](https://github.com/berenif/LaboratoireHumain/blob/27ea010c3d856d00805ce9266feebc94cf7b77f1/rust/crates/sim/data/recovery-contract-v1.json#L5-L41), ajouté dans le même commit, fige notamment 3 N, une normale 0,65, 12 mm, les persistances 0,05/0,30/0,55 s, les vitesses, le délai de récupération et l’absence d’assistance. **Il ne contient aucun des trois nombres de transfert audités.** Cela distingue leur emplacement et leur rôle ; cela ne les rend pas facultatifs dans un protocole qui les a ensuite gelés.

Aucun verbatim utilisateur primaire imposant le choix initial **80/25/3 mm** n’a été retrouvé dans les sources conservées examinées. Les commits et documents prouvent une provenance d’implémentation, puis un gel documentaire. Le nom de l’auteur d’un commit ne prouve pas qui a choisi chaque constante. Il serait donc excessif d’écrire « l’utilisateur a demandé ces trois nombres », comme d’affirmer qu’il ne les a jamais approuvés. Les instructions actuelles de conserver les critères ne sont pas contestées.

## Ce que les quantités calculées signifient

Dans le [transfert courant](https://github.com/berenif/LaboratoireHumain/blob/b9809dfdc2db1a5317379e7914c220d4e856fdbc/rust/crates/sim/src/recovery/planner/support_state.rs#L340-L385), le groupe retiré est exclu avant le calcul de support restant. Deux métriques différentes sont comparées au poids :

- `released_load_n` additionne la charge des segments retirés **après leur filtre de persistance de douze pas** ; avant cette persistance, `persistent_load` renvoie zéro.
- `remaining_current_load_n` additionne les points courants porteurs des segments restants admis dont la charge est persistante. Le hull provient de ces mêmes points courants. Le filtre de présence, l’ensemble de points et celui des segments peuvent différer de la somme précédente.

La [charge courante d’un point](https://github.com/berenif/LaboratoireHumain/blob/b9809dfdc2db1a5317379e7914c220d4e856fdbc/rust/crates/sim/src/recovery/planner/native_observation.rs#L153-L194) est la composante verticale de son impulsion sélectionnée divisée par le pas natif englobant. Le code [measurement lignes 13–18](https://github.com/berenif/LaboratoireHumain/blob/b9809dfdc2db1a5317379e7914c220d4e856fdbc/rust/crates/sim/src/measurement.rs#L13-L18) précise que ces points appartiennent au dernier intervalle CCD : ce n’est pas la somme des impulsions de tous les intervalles. Le journal CCD séparé sert aux bilans ; une impulsion antérieure ne certifie pas un contact encore présent.

Par conséquent, ces deux champs ne constituent pas automatiquement une partition synchrone de toute la réaction verticale. Ils ne doivent pas être sommés et comparés à `mg` comme un bilan de quantité de mouvement exact. La persistance confirme une histoire de charge de segment, pas l’identité persistante de chaque point ; le suivi matériel ajoute cette seconde information.

La capture utilisée est exactement `c + v*sqrt(max(c_y-h_support,0.2)/9.81)`. Le test conserve ses coordonnées x/z et impose, pour **CoM et capture**, une distance intérieure d’au moins 0,003 m à chaque arête du hull. Un point ou une ligne est donc toujours refusé comme polygone de support de cette garde. [Formule de capture](https://github.com/berenif/LaboratoireHumain/blob/b9809dfdc2db1a5317379e7914c220d4e856fdbc/rust/crates/sim/src/recovery/planner/support_state.rs#L183-L188), [construction du hull et retrait](https://github.com/berenif/LaboratoireHumain/blob/b9809dfdc2db1a5317379e7914c220d4e856fdbc/rust/crates/sim/src/recovery/planner/support_state.rs#L601-L640).

## 80 % et 25 % : aucune contradiction générale

Écrivons `R` pour la réaction restante, `L` pour celle du membre retiré et `W=mg`, **dans un exemple idéalisé où ce sont des forces verticales complètes sur le même intervalle**. Les inégalités sont `R≥0,8W` et `L≤0,25W`. La seconde est un maximum, pas un minimum : il n’y a donc aucune obligation de totaliser 105 %.

- À l’équilibre statique, avec ces deux groupes seuls, `R+L=W`. La première inégalité impose déjà `L≤0,2W`, plus strict que 25 %. Les répartitions 80/20 et 100/0 satisfont les deux. La borne 25 % est alors redondante, pas contradictoire.
- Avec accélération verticale et aucun autre effort externe, `R+L=W+m*a_y`. Le couple 80/25 correspondrait à `a_y=+0,05g` ; 80/0 à `a_y=−0,2g`. Le test n’impose donc pas à lui seul l’équilibre statique.
- Si le total des forces porteuses est inférieur à 80 % du poids, aucune partition ne passe la garde de charge restante. Une phase dynamique de décharge peut ainsi être exclue sans violation de la mécanique. Si le total est supérieur au poids, un pourcentage rapporté à `W` n’est pas une part d’un total égal à 100 %.

Ces exemples ne décrivent pas les forces des anciennes traces : les différences d’observation ci-dessus interdisent cette extrapolation. Le refus des deux retraits depuis B16 reste un refus de **cette garde dans ces états**. Il ne démontre ni une impossibilité humaine du transfert, ni qu’une jambe doive toujours porter 80 % du poids pendant tout mouvement.

## Nécessité matérielle et approximation de contrôle

Les exigences matérielles de fond sont des contacts réellement présents quand ils sont invoqués, des réactions compressives compatibles avec friction et géométrie, les limites articulaires et efforts disponibles, la cohérence des impulsions et du mouvement, et une histoire continue des appuis. Les contraintes internes des moteurs ne peuvent pas remplacer une réaction externe manquante. Un appui retiré ne peut pas prouver lui-même le soutien restant ; une cible ne crée pas une charge.

En revanche, les valeurs exactes 80 %, 25 % et 3 mm ne découlent pas de ces exigences. Une analyse mécanique générale devrait considérer `ΣF = m*a_c` et `Σ(r_i−c)×F_i + τ_ext = dH_c/dt`, avec gravité, efforts externes, réaction normale, friction et moment cinétique. Le test scalaire de charge et la projection horizontale ne résolvent pas ces équations avec les capacités des moteurs.

Sous hypothèses statiques usuelles — sol plan, gravité seule, pas d’accélération ni variation de moment cinétique — le CoM projeté doit appartenir au support réalisable. En dynamique, accélération, évolution du moment cinétique, pivot et changement de contacts peuvent déplacer les relations pertinentes. Le point de capture du code introduit une approximation pendulaire à hauteur instantanée bornée ; il ne modélise ni toute l’articulation, ni la friction, ni les couples saturés, ni les changements de hauteur et d’appui à venir. CoM/capture dans le hull est donc une garde conservatrice de cette route, **pas une preuve suffisante générale de faisabilité**, et son échec n’est pas une preuve nécessaire d’échec de toute route dynamique.

La marge 3 mm est une réserve géométrique fixée. Aucune dérivation depuis une borne d’incertitude physique ni calibration universelle n’est fournie dans les sources citées. Le refus des hulls dégénérés est utile pour cette famille de transferts ; il ne démontre pas qu’un pivot sur une arête soit physiquement impossible.

## Conséquence pour le prochain candidat statique

Conserver les nombres et les rapporter sous un nom de **garde de transfert de la famille**, distinct du contrat moteur, de la faisabilité dynamique et du diagnostic local. Les étapes géométriques peuvent préparer décharge et recentrage ; leurs réactions souhaitées ne satisfont jamais cette garde mesurée. L’ancien diagnostic local conserve ses 80 %, sa marge et son maintien ; son résultat ne doit pas être renommé qualification du relevage.

Si un candidat propose un transfert dynamique qui ne passe pas cette garde, le présent audit ne l’autorise pas. Il faut formuler explicitement une autre classe de transition et ses preuves prospectives avant un nouveau protocole ; il ne faut ni abaisser discrètement les seuils, ni rebaptiser la charge, ni utiliser les appuis futurs pour certifier le départ. L’admission native, les bornes de force et les registres restent inchangés.
