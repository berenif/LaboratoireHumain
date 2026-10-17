# Prochain jalon causal : proposition conditionnelle, aucune nouvelle physique

**Recommandation actuelle : corriger et diagnostiquer le refus logiciel avant toute histoire de transfert.** Après une vraie parité native réussie, préparer une seule paire A/B sous le même gel, avec une continuation conditionnelle déjà décrite et chiffrée. Ne pas lancer d'emblée répétition, orientation supplémentaire ou variante de relevage. Cette note est une lecture de sources et de reçus ; elle n'exécute aucun test, monde physique, FK, logiciel tiers ou réglage.

## Situation au moment de la note

Le renouvellement autorisé porte sur **44 pas natifs et 120 secondes murales**, pas 120 pas. Le [registre](native-run/physical-ledger.json) est désormais `closed-failure` après envoi du permis, avec `actualNativeStepsKnown=null` et 44 pas réservés sans remboursement. Le [stderr](native-run/worker.stderr.log) indique un panic `number out of range` à `numeric_parity_worker.rs:408`. Ce message et l'absence de reçu complet ne permettent ni de déclarer zéro intégration, ni de déclarer la parité. La cause précise et le nombre éventuellement exécuté relèvent de l'audit du worker ; ils ne sont pas inférés ici. Le scénario effectivement rencontré est donc la branche « refus/non-preuve » ci-dessous.

Le [prototype du 12 octobre](../../coordination-prototype-result-2026-10-12.md) avait exécuté A pendant 4 s : chute, aucun talon postérieur chargé, aucun intervalle stable. B avait refusé ses quatre copies au pas 12, sans commande coordonnée dans une histoire réelle. La [correction numérique](../../coordination-numeric-correction-2026-10-12.md) fournit quatre commandes f32 admissibles par facteur sur cet état archivé ; elle ne prouve pas leurs effets physiques. La [route complète](../coordination-prototype-2026-10-12/continuation-review.md) de huit secondes est géométriquement préparée à 0° et 60°, sans preuve de charge.

## Les deux branches de décision

| Résultat de parité | Travail suivant justifié | Ce qui reste fermé |
|---|---|---|
| Préfixe divergent, panic, timeout, résultat manquant, commande/installateur/parité refusés | Identifier la première frontière fautive depuis les archives ; correction logicielle et tests purs sur données figées, dans la préparation autorisée ; nouvelle admission native uniquement dans un registre explicitement distinct | Toute prévision longue et histoire de transfert ; aucune conclusion sur impossibilité anatomique ou besoin de plus de couple |
| Les 12 états du préfixe, quatre cas et quatre pas copie/exécution satisfont tous les critères figés | Figer le prochain test causal A/B et son driver de continuation ; les 44 pas deviennent une preuve technique limitée au premier tick contraint | Aucune acquisition de talon, stabilité locale, anticipation à 480 pas ou récupération n'est encore démontrée |

Un correctif d'export ou de sérialisation ne change pas la famille de commande. Il ne crée pas une autorisation de réutiliser le lot clos. Un refus physique ultérieur sous commandes admises doit rester distinct d'un défaut de la preuve ou de l'outil.

## Paire causale à préparer après parité réussie

**Hypothèse testée :** la boucle commune des 46 coordonnées, avec sélection native des quatre facteurs déjà figés, permet d'acquérir des talons chargés et de continuer la même référence jusqu'au terminal original, là où le servo de référence seul ne le permet pas.

- **A :** référence commune, servo/installateur/allocation existants, coordination désactivée.
- **B :** même fixture, profil v24, référence, caps, gains, anatomie, friction, collisions et critères ; coordination activée. Facteurs `0 / 0,5 / 1 / 1,5` seulement, règle de sélection et de départage figée avant lancement.
- Une histoire continue depuis `landed-crouch-left` pour chaque bras. Aucun settle caché, redémarrage à 4 s, écriture de bassin ni transplantation d'un état prédit. Les copies de prédiction n'avancent jamais la mémoire de l'histoire réelle.
- Décisions communes à 60 Hz, installation native à 240 Hz, au plus un appel prédictif tous les 24 pas (10 Hz). Quatre candidats de 480 pas au plus par appel ; **zéro sonde courte** et aucune identification récursive dans les branches.
- Comparer les 24 premiers pas de la prévision sélectionnée aux 24 pas effectivement commis avant la replanification suivante, tant que les deux chemins ont la même politique. Ces pas existent déjà dans le rollout et l'histoire : leur comparaison et archivage n'ajoutent aucune intégration. Au-delà de cette fenêtre, les changements de sélection ultérieurs empêchent de réclamer une identité de toute la prévision de 2 s.

La paire isole la contribution de **l'ensemble coordination + sélection prédictive**, pas celle d'un seul membre, du nouveau solveur ou de la seule anticipation. Pas d'ablation ou de réglage après lecture du premier résultat. Si A et B réussissent, la référence suffit pour cette entrée et un gain propre de B n'est pas établi. B seul réussi est une preuve causale limitée à cette paire ; une répétition reste nécessaire avant une revendication de répétabilité.

## Préparer la continuation avant l'histoire locale

Conserver les six clés existantes : entrée 0 s ; acquisition géométrique 0,70 s ; recentrage 1,25 s ; maintien préparé jusqu'à 1,80 s ; cible debout 7 s ; cible constante jusqu'à 8 s. Ces dates sont des références de commande, pas des événements physiques acquis. Le programme porte SHA-256 `f14de3871ef9aba18083b36d10eeb56c8587c66a73b1ab2ec54e01830e06e59d`.

Avant toute admission de cette paire, livrer sans physique un driver distinct des fichiers gelés qui :

1. Distingue l'intervalle local observé, l'état courant à 4 s et le terminal de récupération ; aucune variable `local_success` ne peut déclencher un compteur de relevage.
2. Préserve q/dq, horloges, commandes précédentes et points matériels du vrai préfixe dans la continuation. Une ancienne réussite locale mémorisée ne suffit pas à autoriser un raccord après disparition des appuis : contacts actuellement chargés, identité des patches, dérive et intégrité sont relus sur l'état courant.
3. Laisse le terminal original gérer `Stand → Handoff → Ordinary → Upright`. Le propriétaire expérimental se retire lorsque le handoff observé lui retire la commande ; le planificateur cesse alors ses recherches, sans neutraliser la station ordinaire ni ses observateurs.
4. Arrête l'histoire au premier refus de support, d'intégrité, de solveur, de représentation, de parité ou de budget. Un échec de continuation conserve le succès local éventuel comme résultat intermédiaire, jamais comme relevage.

Le driver actuel de transfert est limité en dur à 960 pas et 40 appels. Il n'est pas un exécuteur de continuation de 26 s. Modifier seulement un délai ou recopier sa variable `passed` serait insuffisant. La préparation doit aussi vérifier par lecture/tests purs le passage du propriétaire et la conservation des critères ; les copies intégrées éventuelles restent payantes.

**Passage à 4 s, identique dans A et B :** continuer seulement si le succès local a été acquis **et** les appuis actuellement porteurs, le raccord de commande et l'intégrité restent admissibles. Sinon arrêter ce bras à 4 s et conserver son échec local ; cela n'empêche pas d'exécuter l'autre bras causal déjà prévu lorsqu'aucun refus global d'intégrité ou de budget ne ferme le lot. Cette décision ne remet aucun compteur ni état à zéro.

## Critères mesurés, tous distincts

**Jalon local :** au moins 132 échantillons natifs consécutifs satisfaisant toutes les conditions, dont le début est strictement avant 2 s. Chaque talon doit être un contact natif du hindfoot avec `local_z < −0,02 m`, charge ≥3 N et ses propres 12 pas de persistance. Support admis ≥80 % du poids ; aucun appui non-pied persistant ≥3 N ; CoM et capture dans le support observé avec 3 mm de marge ; les 25 corps à ≤0,22 m/s et ≤0,65 rad/s. Rapporter aussi les charges non-pieds plus petites. L'histoire locale de 4 s doit finir sans refus. Ce verdict est un latch d'intervalle, pas une certification de l'état terminal à 4 s.

**Continuation :** appui debout et équilibre réellement observés, bassin >0,93 m au-dessus du support, verticalité et limites originales ; settle de 0,30 s (72 pas), handoff de 0,60 s (144 pas), puis observation stable sous contrôle `Ordinary`. Exiger `recoveries == 1` et `Motion::Upright` au plus tard à 25 s, puis les **240 pas réels supplémentaires** de stabilité du banc original. La seconde de cible constante entre 7 et 8 s ne remplace aucun de ces intervalles.

Dans le code courant, `Stand` est demandé à la fin du programme, à 8 s. Si toutes les conditions physiques étaient déjà remplies, l'enchaînement nominal pourrait donc donner handoff vers 8,3 s, contrôle ordinaire vers 8,9 s, premier `Upright` vers 9,9 s et observation supplémentaire terminée vers 10,9 s. C'est un repère de préparation des horloges, **pas une prédiction physique**, avec frontières exactes à vérifier en pas entiers.

Le banc de fixture réserve jusqu'à 25+1 s. Cela ne change pas les critères des cycles/impacts dont le maintien doit être inclus dans leur délai original. Cette entrée stockée ne simule aucun nouvel impact ; elle ne qualifie ni 24 fixtures, ni cinq cycles, ni idle strict ou robustesse à 60°.

## Coûts proposés, aucun crédit implicite

Le prochain lot long n'est pas à lancer tant que les 44 pas de parité ne sont pas prouvés. Pour sa préparation, le choix économe recommandé est **une seule paire A/B avec continuation conditionnelle**, avec un plafond supplémentaire explicite de **100 appels B**. Ils restent à 10 Hz maximum, horizon 480, quatre candidats ; ils s'arrêtent dès retrait observé du propriétaire. Si le propriétaire est encore actif au 101e appel, le protocole refuse avant cet appel : aucune baisse de cadence, conservation arbitraire du dernier facteur ou variante de secours pour économiser des pas. Ce plafond correspond à la route de 8 s plus 2 s de fenêtre de planification ; il restreint cette expérience sans relâcher le contrat de récupération à 25 s.

Avec l'appel initial à t=0 puis les indices 0, 24, 48, etc., le 100e appel est à **9,9 s (pas 2 376)** ; le 101e serait à **10,0 s (pas 2 400)** et doit être refusé avant réservation/exécution. L'appel initial est inclus dans les 100.

| Périmètre éventuel | Histoires réelles maximales | Prévisions maximales | Total de pas natifs |
|---|---:|---:|---:|
| Paire locale A/B seule, 4 s chacune | 1 920 | `40 × 4 × 480 = 76 800` | **78 720** |
| Paire recommandée, continuation déjà conditionnée, ≤26 s chacune | 12 480 | `100 × 4 × 480 = 192 000` | **204 480** |
| Ancienne forme locale à quatre histoires, A/B/B répété/B60 | 3 840 | `3 × 40 × 4 × 480 = 230 400` | **234 240** |
| Paire complète si l'on recherchait jusqu'à 25 s à 10 Hz sans plafond100 | 12 480 | `250 × 4 × 480 = 480 000` | **492 480** |

Les deux premières lignes sont **des alternatives**, pas deux lots automatiquement enchaînés. Dans la paire recommandée, le jalon local et sa continuation utilisent les mêmes instances réelles sous la même réservation ; les 12 480 pas incluent tous les préfixes. Un arrêt local libère du travail non exécuté, pas un crédit remboursable. Si un lot local est clos avant décision de continuation, toute nouvelle histoire complète doit repartir de sa fixture et repayer son préfixe.

La dernière ligne montre pourquoi compter seulement 6 240 pas réels pour B masquerait le coût de l'anticipation. La troisième ligne est une borne documentaire, pas la recommandation immédiate : répéter et tourner la fixture consommerait des histoires supplémentaires et ne remplacerait toujours pas la continuation physique.

Pour la paire recommandée, proposer un plafond mural indépendant de **2 h**, un processus natif et un thread, arrêt au premier plafond. C'est une borne d'exécution, pas une estimation de durée ou une garantie que 100 appels seront réalisables. Toute vérification intégrée nouvelle s'ajoute par un nombre explicitement réservé avant lancement ; aucune réserve technique générique n'autorise des essais libres. Les comparaisons de traces, lectures, rendus et corrections purement numériques ne sont pas des sous-pas physiques.

Les trois lots de coordination déjà réservés représentent **3 856 + 44 + 44 = 3 944 pas**, avec consommations connues/inconnues conservées séparément. Une paire à 204 480 porterait ce total conservateur à **208 424 réservés**, si le parent décide de conserver un plafond cumulatif de famille à 300 000. Cette addition est un contrôle de coût proposé ; **elle ne rouvre aucun ancien registre et ne transforme aucun solde en permission**. Le registre futur doit être distinct et borner exactement son scope.

La préparation renouvelée déclare 23 989 s / 144 000 s et 9 173 / 20 000 inspections réservées, échéance originale du 17 octobre à 20:14 UTC. Cette note ne modifie pas ces registres et ne revendique aucune nouvelle inspection. Les débits de préparation ne sont pas interchangeables avec le compteur physique.

## Preuves à conserver et conclusion admissible

À chaque sous-pas : commandes et propriétaires, installation native réelle, caps, livraison vectorielle, états des 25 corps, CoM/vitesse, contacts et impulsions, points matériels et dérive, états des critères locaux et terminaux. Archiver la première divergence avec la fenêtre prédite correspondante. Garder journal CCD et contact actuellement porteur distincts ; aucun `angularBalanceCertified=false` ne devient un bilan angulaire certifié par simple absence d'erreur.

Le reçu final doit séparer : **parité technique**, **transfert local**, **continuation réellement exécutée**, **terminal original acquis**, **seconde supplémentaire tenue**, **qualification générale non effectuée**. Une parité réussie permet de tester la commande ; une réussite locale permet de constater le transfert ; seule l'histoire mesurée complète permet de constater ce relevage particulier. À l'état actuel, le refus du worker bloque la première de ces promotions.
