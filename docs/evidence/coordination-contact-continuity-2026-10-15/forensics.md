# Continuité des contacts : examen statique de l’essai clos

Le garde a refusé une **perte de soutien chargé au point matériel retenu gauche**. Les archives ne prouvent pas si le point brut a disparu, a été désélectionné, ou a reçu une impulsion nulle. Elles excluent cependant un simple remplacement par un triangle chargé équivalent : le point matériel immuable reste **69.992785 mm hors de l’enveloppe chargée** de son avant-pied à 78. Une règle prudente de continuité régionale refuserait donc encore ce cas.

## Ce qui est conservé, et ce qui manque

| Branche | Sous-pas | Archives disponibles |
|---|---|---|
| Préfixe technique | 1–36 | Poses, vitesses, contacts locaux/monde chargés, normale verticale, forces équivalentes, journaux CCD |
| Deux paires techniques, copies et contreparties | 37–40 pour chaque branche | Mêmes observations ; mondes bruts aux horloges 36–40 et checkpoint final40 |
| Préfixe principal | 1–36 | Mêmes observations complètes ; histoire séparée |
| Première prévision | 37–78 | CoM et diagnostic après chaque pas ; proposition et points XZ de support issus de l’observation précédente |
| Prévision, état préservé | 77 | Observation complète dans la décision conservée après le rejet atomique |
| Prévision, état refusé | 78 | Observation complète et géométrie actuelle ; **pas de monde brut/manifold/points non sélectionnés** |

Le résultat contient 24 tranches de mondes bruts, toutes limitées aux paires techniques. Les `identityBytes` de la mémoire de prévision ne sont que des longueurs : on ne peut en extraire les données manquantes à 78. La prévision compacte ne fournit pas les normales, impulsions individuelles et points locaux de chaque pas intermédiaire. Le JSON compagnon conserve toutes les lignes avec leur branche et leurs horloges distinctes, sans remplir ces lacunes par interpolation.

Le total reste130 sous-pas déjà exécutés :36 préfixe technique +16 vérifications +36 préfixe principal +42 prévision. **Zéro commande de transfert a été exécutée après A36 dans l’histoire principale.** Aucun sous-pas, monde, FK ou test n’a été ajouté par cet examen.

## Identité du point, identité du manifold et charge

Dans les mondes techniques 36–40, les quatre segments de pied conservent leurs indices sélectionnés, coordonnées locales brutes et couples d’identifiants géométriques. Les avant-pieds utilisent les indices `[0,4,3,1]`, les arrière-pieds `[4,3]`. Aucun renouvellement n’est constaté sur cette courte fenêtre. Cela ne renseigne pas la transition77→78, dont le monde brut n’a pas été conservé.

Les préfixes complets montrent une absence transitoire antérieure du même coin matériel gauche. Il porte 85,06281 N au 27, disparaît des contacts chargés au 28 alors que l’avant-pied porte toujours 212,06144 N, puis réapparaît au 29 avec 21,10173 N. Les deux préfixes présentent ce même motif. L’identification rétrospective emploie le point local qui sera retenu à A36 ; le garde de ce candidat n’était pas encore actif aux pas27–29. Cette observation interdit de confondre toute absence d’un point chargé avec une impossibilité de soutien. Elle ne prouve toutefois ni un renouvellement de manifold, ni un retour hypothétique au 79, ni l’innocuité d’une tolérance temporelle nouvelle.

Le code local de Rapier définit `contact_id` comme un index dans le manifold courant, avec un bit de nouveauté ; ce n’est pas un identifiant matériel universel. Les points peuvent être réduits à un sous-ensemble et triés ; Parry rapproche des données par identifiants de caractéristiques ou positions. Les ancres du solveur, les points géométriques bruts, les coordonnées liées au CoM et les coordonnées locales corporelles n’ont pas la même définition. L’observateur actuel transforme les **ancres courantes du solveur** dans le repère du corps. Une comparaison d’indices ou de coordonnées appartenant à des représentations différentes ne constitue pas une preuve de glissement.

Un exemple concret interdit de sommer aveuglément le cache : le point non sélectionné d’indice2 garde une impulsion normale de 0,8868289 N·s à gauche et0,88085604 N·s à droite pendant 36–40. Ces valeurs ne sont pas des charges courantes. Le chemin d’observation existant les exclut correctement ; la continuité ne doit pas les réintroduire pour remplir un polygone d’appui.

## Transition décisive77→78

Le point gauche conservé est localement environ `[-0.045,-0.045,+0.060]`m. À77 il porte 65,341455 N ; à 78, le segment reste chargé à 85,552933 N via trois autres points. Le plus proche est à 90,000026 mm. Le point n’est pas couvert par leur triangle : distance locale3D à l’arête la plus proche **69,992785 mm** ; distance XZ de l’ancre monde initiale à l’enveloppe courante **69,816090 mm**. Ce dernier calcul et celui du point matériel courant sont volontairement distincts.

La dérive tangente du point gauche reste0,186918 mm, sous le garde de 10 mm. Sa hauteur dérivée des poses est−3,929819 mm, et sa vitesse verticale instantanée passe de−0,012204 m/s à 77 à+0,098154 m/s à 78. Ces données sont compatibles avec une décharge et un mouvement ascendant, mais ne prouvent pas pourquoi le solveur/filtre n’expose plus ce point. Elles n’autorisent pas à qualifier l’absence de simple renouvellement normal, ni de décollement du pied entier.

Les trois contacts gauches à 78 ont une normale verticale1.0 et leurs coordonnées localesY diffèrent du point retenu d’au plus7,45 nanomètres. L’exclusion de l’enveloppe est donc latérale et importante, pas un résidu de plan de quelques nanomètres. Le point droit garde son contact chargé direct ; il n’a pas besoin d’une exception régionale.

Dans les 689 contacts des 90 observations complètes, état préservé77 compris et branches répétées incluses, les normales verticales sont comprises entre0.9999998808 et1.0. Le résidu maximal à la face localeY=−45 mm est53.942 nanomètres. Ce constat concerne les points **sélectionnés et chargés conservés**, pas ceux absents ni les contacts intermédiaires non archivés. Le JSON détaille les changements de population sur tous les sous-pas disposant d’observations complètes, et signale les intervalles non consécutifs.

## Règle proposée, sans implémentation

1. Conserver une identité matérielle indépendante du solveur : point local et ancre monde immuables, dérive tangente mesurée, garde10 mm inchangé.
2. Distinguer géométrie et charge : normale/séparation et contact géométrique d’une part ; impulsions sélectionnées réellement actives, charge résultante et centre de charge d’autre part. Une impulsion nulle n’efface pas l’identité matérielle ; elle ne prouve pas non plus que le point soutient le corps.
3. Pour une tâche explicitement **régionale**, accepter éventuellement l’absence d’un échantillon exact si un triangle non dégénéré de contacts chargés du même segment et de la même surface couvre toujours le point inchangé, avec normale, plan, séparation et bilan global des efforts compatibles. La couverture ne fabrique pas une force ponctuelle interpolée.
4. Pour un **pivot ponctuel chargé**, conserver l’exigence plus forte ou effectuer une transition d’appui explicite. Ne pas convertir silencieusement cette tâche en région et ne pas déplacer son ancre.
5. Hors enveloppe, séparation, données contradictoires ou couverture insuffisante : refuser. Le cas78 reste refusé avec cette règle.

Pour départager les causes lors d’un futur protocole autorisé, l’archive devra distinguer points géométriques, points sélectionnés, impulsions nulles/positives, motifs de filtrage, identifiants de corps/surfaces et de sous-formes, ancres courantes, séparation normale et intervalles CCD. Aucun nouveau test physique n’est proposé comme déjà autorisé ici.

## Reproduction et limites

Exécuter `python3 docs/evidence/coordination-contact-continuity-2026-10-15/forensics.py` ne lit que les archives et sources locales et réécrit ses deux livrables. Le script emploie la bibliothèque standard Python et Git en lecture seule ; il ne charge ni le moteur natif ni le modèle. Les transformées rigides utilisent les poses/vitesses mesurées déjà archivées, sans arbre cinématique ni intégration. Les SHA256 des archives et sources consultées figurent dans le JSON ; les quatre preuves d’entrée contrôlées restent inchangées.

La provenance du code est fixée explicitement à `sourceCodeCommit=da4575170c6ae9b0858c0851a6f1ed15fe1ce858`. Toutes les sources du dépôt, y compris Rapier inclus dans le dépôt, sont lues par `git cat-file blob` à ce commit ; les empreintes ne proviennent pas du worktree en cours de modification. La référence Parry externe conserve son chemin absolu et son SHA256 imposé. Les quatre archives d’entrée doivent correspondre à leurs empreintes historiques exactes avant toute lecture analytique ; aucun fichier manquant ou différent n’est remplacé silencieusement. Cette correction de provenance ne modifie aucune colonne calculée à partir des archives.

Les forces de contact sont des impulsions du dernier intervalle sélectionné divisées par le pas natif ; elles ne sont pas des forces instantanées ni nécessairement le total d’un pas comportant plusieurs intervalles CCD. Les centres de charge calculés n’incluent pas les moments de friction et ne remplacent pas une validation complète du torseur de support. Aucun relevage, impossibilité anatomique ou nécessité universelle d’appuis supplémentaires ne découle de cet essai.
