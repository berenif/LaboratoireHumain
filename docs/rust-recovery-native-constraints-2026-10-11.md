# Comparaison native bornée des contraintes — 2026-10-11

Branche locale `investigate/physical-support`, HEAD `c597959f46f37715035349c8cd0a699b6822111a`. Aucun push, merge ou publication GitHub. Aucun changement de seuil, anatomie, assistance au bassin ou reset. Récupération désactivée par défaut et non qualifiée.

## Résultat de la preuve de 100 ms

**La borne de 100 ms n'est atteinte par aucun candidat.** Le pipeline réel intègre 16 pas, soit 66,67 ms, puis le témoin et la formulation corrigée/dédupliquée refusent la demande suivante. Ce sont des arrêts du modèle de commande, pas des chutes observées à cette date. Les contrôles natifs d'intégrité passent pour tous les pas réellement exécutés. Aucun relevage, transfert ou stabilité finale n'est établi.

Le témoin n'est plus le diagnostic direct à 20 passes : `advance_tick_observed` sélectionne réellement **32 passes PGS** avant chaque pas. Il conserve les contacts, mesures, contrôles d'intégrité et observation de mouvement d'origine. Un crochet uniquement compilé dans les tests remplace les moteurs actionnés juste avant `step_with_events`, après leur configuration et la sélection PGS. Il n'écrit aucun état corporel. Ce crochet et les contrôleurs expérimentaux sont retirés du code final et archivés dans ce dossier.

La même fonction de commande, les mêmes objectifs, inégalités, points natifs et allocation de couple sont utilisés dans les deux branches. Seule la construction des égalités de contact change. La marge d'arrondi est retirée du plafond gauche de cheville AngX avant physique ; toutes les sommes f64 des plafonds f32 réellement installés sont vérifiées ≤760 Nm. Le témoin exécuté après la dernière compilation reproduit exactement sa première trace physique.

| Mesure native sur les pas exécutés | Témoin | Contraintes indépendantes + déduplication |
|---|---:|---:|
| Temps réellement intégré | 66,67 ms | 66,67 ms |
| Plafonds installés maximaux | 759,999991857 Nm | 759,999991835 Nm |
| Somme maximale des couples livrés absolus | 656,284 Nm | 664,124 Nm |
| Vitesse linéaire maximale des segments | 0,7591 m/s | 0,6773 m/s |
| Vitesse angulaire maximale des segments | 6,1407 rad/s | 5,0158 rad/s |
| Charge maximale des arrière-pieds | 107,94 N, un seul pas à droite | 0 N |
| Dernière accélération CdM avant Z | +0,1927 m/s² | −0,2358 m/s² |
| Dernière accélération CdM latérale X | −2,7526 m/s² | −2,6862 m/s² |

Les vitesses transitoires dépassent les réserves de stabilité finale (0,22 m/s et 0,65 rad/s). Cela ne démontre pas que ces réserves sont inatteignables plus tard ; aucun état stable final n'est observé. Le contact d'arrière-pied ponctuellement chargé du témoin ne prouve pas un appui talon persistant. Le candidat corrigé ne produit aucun contact d'arrière-pied chargé pendant cette fenêtre.

Deux répétitions de chaque variante donnent des rapports **byte-identiques** ; `repetition-comparison.json` contient leurs SHA-256. Les entrées et états initiaux sont identiques. Le moteur ne dépasse aucun plafond installé par axe dans les traces.

## Première formulation et contacts quasi confondus

La première construction indépendante s'arrête après 13 pas (54,17 ms). Elle traite deux coordonnées droites distantes de seulement **7,45e-9 m** comme deux points différents, et crée deux contraintes angulaires artificielles. Le problème figé redevient faisable sous toutes les limites lorsque ces deux lignes sont retirées.

La formulation finale retient toutes les variables de force et les contraintes d'ancre. Elle filtre seulement les directions de géométrie inférieures à la précision arithmétique de leurs coordonnées f32 : `8 * EPSILON_f32 * max(|point| + |centre_de_masse_du_corps|)`. Cette borne numérique ne change aucun seuil de contact ou qualification. Elle complète la déduplication demandée dans le protocole et permet trois pas supplémentaires, jusqu'à 66,67 ms. La première variante reste archivée comme essai rejeté.

## Classification du nouveau refus à 66,67 ms

1. Le modèle propose sept contacts : trois par avant-pied, dont des doublons, et un arrière-pied droit. Ce dernier est à **charge nulle**, à **4,586 mm au-dessus du sol**, mais reçoit une tâche de vitesse collante nulle. Les six contacts chargés se trouvent sur les avant-pieds, pas sur les talons.
2. La cinématique native ne correspond pas exactement à l'arbre rigide utilisé par le modèle à 52 coordonnées. À cet état, les résidus `v_corps - J*qdot` des avant-pieds atteignent **0,0851/0,0996 m/s** en translation et **2,545/2,792 rad/s** en rotation. Des résidus existent aussi dans le témoin et dans l'ancienne observation passive du pipeline ; ce n'est pas une preuve nouvelle de défaillance générale de Rapier. Ils indiquent que l'hypothèse de contraintes articulaires instantanément satisfaites ne suffit pas à prédire cette transition native.
3. Les 19 égalités indépendantes par corps ont une direction presque dépendante (valeur singulière **1,285e-8**) mais un second membre associé **0,3408**. La dépendance implique principalement la vitesse latérale de l'arrière-pied et de l'avant-pied droits. La réduction l'amplifie. `kinematic-compatibility.json` mesure cette incompatibilité dans le modèle ; ce vecteur presque nul n'est **pas** un certificat exact d'infaisabilité.
4. Même en retirant la tâche collante de l'arrière-pied séparé et en fixant sa force à zéro, le système restant est réellement infaisable sous les contacts compressifs et plafonds individuels. Le problème obtenu est bien conditionné (plus petite valeur singulière **0,0753**). `after-speculative-contact-removal.json` contient un certificat de Farkas : multiplicateurs non négatifs, résidu dual **1,531e-12**, second membre **−1**. Supprimer les limites articulaires ou le seul plafond global ne le résout pas ; retirer les enveloppes de couple ou les plafonds individuels le rend faisable. Les termes dominants concernent la compression des contacts de pointe et les plafonds d'avant-pied. Ces suppressions servent uniquement à classifier le système figé ; aucune n'est installée dans la physique ou utilisée pour qualifier un essai.

Le refus est donc lié au **mode qui immobilise simultanément tous les contacts**, aux vitesses natives hors de la cinématique supposée et aux enveloppes locales pendant ce mode. Il ne prouve pas une impossibilité de tout mouvement pieds seuls ou une insuffisance universelle du budget de 760 Nm. Un changement de gains ou du nombre de passes ne répondrait pas à ce diagnostic.

## Discriminateur pieds seuls, limité à l'état figé

`frozen-contact-modes.json` garde les mêmes variables de contact, friction et limites articulaires/de couple. Les points libérés reçoivent une force nulle et une condition de séparation normale ; l'arrière-pied séparé peut approcher le sol sans fermer son écart au pas suivant. Le mode qui garde les deux contacts arrière des avant-pieds et libère les anciennes pointes ainsi que l'arrière-pied séparé est faisable.

Le premier témoin LP est presque en chute libre et ne constitue pas un soutien utile. Un second calcul ajoute une condition plus exigeante : **réaction normale totale au moins égale au poids**, soit 708,282 N pour 72,2 kg. `frozen-weight-bearing-mode.json` fournit un témoin qui la satisfait sous les limites originales : budget alloué **474,614 Nm**, résidu d'égalité **4,221e-14**, violation maximale **3,031e-15**. Cela établit une possibilité instantanée de soutien pieds seuls dans le modèle. Les appuis restent sur l'arrière des avant-pieds ; aucun appui talon, polygone de stabilité qualifié ni chemin dynamique complet n'est démontré.

Ce mode n'est pas installé dans un nouvel essai dynamique : le protocole borné échoue, et la classification est livrée avant une nouvelle décision de commande. Les calculs à état figé ne peuvent remplacer l'essai natif de cette transition.

## Livraison de couple et prédiction native

`physical-metrics.json` compare les demandes au **vecteur d'impulsion réellement appliqué au corps enfant**, obtenu par différence des compteurs intégrés du pipeline. Le signe du scalaire brut du moteur ne représente pas directement le signe du couple enfant ; `request-delivery-comparison.json` le marque explicitement comme exploratoire. Aucune inversion de commande n'est déduite de ce scalaire.

Sur le candidat corrigé, l'écart entre le moment demandé et livré atteint **26,01 Nm** (cheville gauche à 8,33 ms), RMS **6,11 Nm** sur les moments articulaires. L'accélération CdM prédite et mesurée diffère jusqu'à **3,418 m/s² après les trois premiers pas**. Le contrôleur identique du témoin présente aussi des écarts. Ces mesures justifient d'auditer les réactions de contraintes et la livraison discrète native avant de tirer une conclusion physique d'une seule faisabilité QP.

## Décision et reprise précise

**Ne pas prolonger ce candidat vers une remontée : il échoue au jalon de 100 ms.** Le prochain correctif structurel doit représenter les contacts unilatéraux et leur transition de mode, sans imposer le collage des points séparés ou devenus séparants, et tenir compte des résidus natifs de contraintes articulaires dans les accélérations/reactions prédites. Définir la sélection par compression, séparation et friction physiques, sans programmer les indices de contact du témoin comme une réussite forcée.

1. Repartir de `independent-dedup-a/0016.json`, qui conserve le nouvel état complet, ses matrices et points natifs. Vérifier la prédiction des moments enfants et des vitesses de contraintes à un pas avant une nouvelle trajectoire.
2. Conserver les 32 passes, anatomie et caps f32 installés dont la somme f64 reste ≤760 Nm. Construire un mode unilatéral cohérent et vérifier les conditions de séparation/charge sur toutes les lignes natives, y compris les doublons.
3. Rejouer une seule comparaison bornée à 100 ms depuis le même accroupi, avec deux répétitions. Exiger des appuis réellement résolus et des prédictions de livraison justifiées avant extension à la préparation, au pivot talon, au recentrage puis à la remontée.
4. Poursuivre les critères originaux de persistance et de stabilité finale. Si le mode pieds seuls échoue physiquement malgré une formulation valide, comparer ensuite un appui intermédiaire main/genou dans les mêmes limites. Sa nécessité n'est pas démontrée à ce jalon.

## Reproduction et état de l'environnement

Le message de déconnexion du parent ne correspondait pas à une perte d'accès durable ici : les outils locaux ont réussi à 00:55:38 UTC, la compilation a terminé, et les nouveaux essais ont été exécutés et sauvegardés. Aucun essai à état inconnu n'a été relancé.

Le binaire archivé `pipeline-test-binary` contient le crochet de test et peut exécuter `recovery::support_probe::pipeline_probe::native_comparison --ignored --exact --nocapture`. Fournir `LH_CONTACT_ROWS=original` ou `independent-dedup`, `LH_CAP_REPAIR=ankle`, `LH_STATIC_PLAN` vers `static-plan.json` inclus, et deux chemins nouveaux `LH_PIPELINE_OUT`/`LH_QP_DUMP`. Le test refuse un rapport existant et s'arrête au premier refus. Le source `pipeline_probe.rs` et les deux lignes du crochet dans le reçu `test-hook.patch` permettent de recompiler sous Rust 1.89.0 avec Cargo offline/locked/release. Le test de harness qui passe signifie que le rapport est produit ; le champ `failure` fait autorité sur le résultat physique.

Les scripts `analyse.py`, `contact_modes.py`, `metrics.py`, `render.py` et `verify.py` exploitent les rapports sauvegardés. `analytic_contacts.py` est une dépendance copiée de l'audit précédent, importée pour ses fonctions ; ne pas lancer son ancien `main` dans ce nouveau dossier.

Les suites complètes workspace/hybride/TypeScript/WASM/navigateur ne sont pas déclarées passantes. Les corrections locales de l'autre environnement ne sont pas importées. L'archive contient tous les essais rejetés hors du code final.

Vérifications finales : 24 tests ciblés `recovery::` passent (3 diagnostics ignorés), Clippy du source final `-p lh-sim --tests -- -D warnings`, formatage et `git diff --check` passent. Les tests du harness borné passent car ils sauvegardent les refus ; leurs rapports indiquent l’échec physique du jalon. `verify.py` vérifie les répétitions, caps et les deux preuves figées faisant autorité. Le visuel et la vidéo ralentissent/illustrent uniquement les 66,67 ms exécutées.
