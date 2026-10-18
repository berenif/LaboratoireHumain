# Dix pas alternés de 18 cm : état provisoire

Essais du 18 octobre 2026, dans le prototype natif optionnel. **Aucune séquence de dix pas suivie de cinq secondes calmes n'est qualifiée.** L'admission prospective d'appui qualifie un pilote avec repos final et permet six pas continus ; le glissement global arrête ensuite la séquence.

## Essais arrêtés, preuves conservées

- **Pilote 1, durées raccourcies** : un pas terminé, arrêt pendant la levée du deuxième à **11,720 s / 9 376 intégrations**. Capture −8,150 µm ; glissement chargé gauche/droite 4,588 / 4,708 mm.
- **Pilote 2, suivi d'appui 6D renforcé** : gains position/rotation 200/30, poids 60. Arrêt sur `right_hip_y` pendant la première levée à **4,88875 s / 3 911 intégrations** ; marge articulaire −0,000050133 rad. Aucun pas terminé.
- **Candidat 3, contraintes COM/capture mesurées** : trois pas terminés de **181,968 / 182,012 / 181,509 mm**. Arrêt au transfert du quatrième à **24,93125 s / 19 945 intégrations** : glissement **10,000211 / 9,376005 mm**, dépassant la borne globale inchangée de 10 mm par pied.
- **Candidat 5, relais direct après réception chargée** : un pas acquis, puis refus de sortir de la table FK vérifiée des bras au transfert suivant, à **8,650 s / 6 920 intégrations**. Aucun écrêtage ou élargissement de la table.
- **Candidat 6, relais direct et garde de plage des bras** : deux réceptions de **181,939 / 181,995 mm**, à 7,13875 et 13,28125 s. Arrêt au transfert du troisième à **16,3875 s / 13 110 intégrations** : dépassement de `right_hip_x` de **0,0000887095 rad**. Glissement 8,070 / 7,796 mm. L'audit strict rejette l'essai.

- **Candidat 7, coordination d'accélération bras/cuisses dans le même QP** : arrêt dès la première levée à **4,88125 s / 3 905 intégrations**, sur `right_hip_y` (−0,0000167491 rad). La réception initiale n'est pas atteinte. Cette coordination reste explicitement optionnelle, désactivée par défaut.
- **Candidat 8, transfert après recentrage sur les deux appuis mesurés** : pas terminés de **182,059 / 182,020 mm** à **8,66125 / 15,3325 s** ; troisième réception de **181,180 mm** à **20,475 s**. Arrêt pendant son recentrage à **21,6125 s / 17 290 intégrations**, sur le glissement global **10,000964 / 9,679699 mm**. À l'arrêt, réserve articulaire +0,019906 rad et marge COM/capture +84,522 mm. Le troisième pas n'est pas terminé.

Sous-total vérifié : **74 457 intégrations** pour ces sept essais seulement. Les **19 branches de diagnostic sur checkpoint, 13 355 intégrations**, sont comptées séparément, relectures dynamiques et erreurs préparatoires intégrées incluses. Ce n'est pas un total de toutes les expériences ou des tests historiques.

## Distance et vitesse physiques

« 180 mm » désigne l'avance du pied mobile depuis sa propre position réelle au début du pas, selon le cap courant ; la cible est 182 mm pour obtenir au moins 180 mm mesurés. Cela ne désigne ni la séparation devant l'autre pied, ni l'avance du COM. À l'arrêt du candidat 6, le milieu des pieds a avancé de **181,399 mm**, le COM de **118,113 mm**. Dix déplacements alternés ainsi définis ne signifient pas automatiquement 1,8 m parcouru.

Les références sont raccourcies en **secondes physiques**, sans accélérer la vidéo : levée 0,8 s, translation 1,2 s, descente 1 s, contre 2 s chacune au témoin. Le premier transfert reste à 4 s. Les essais segmentés utilisent ensuite 2 s de transfert, 1,8 s de stabilisation et au moins 0,25 s stables. Le relais direct utilise 3 s de transfert et enchaîne après 100 ms de réception réellement chargée ; stabilisation et repos sont réservés au dernier pas. Cette coordination ne qualifie pas à elle seule une marche complète plus rapide.

Le candidat 8 utilise un recentrage de 1,52 s vers le milieu des pieds réellement chargés, plus 35 mm suivant le cap, puis un transfert de 2 s. Le choix du recentrage borne la vitesse/accélération de COM du premier chemin conservé à 0,196 m/s et 0,396 m/s² ; ce calcul hypothétique n'est pas une preuve physique. Le relais exige 80 échantillons de charge, capture, réserve articulaire et convergence mesurées, sans phase de repos intermédiaire. Les deux relais sont audités indépendamment (101 et 145 échantillons prêts). L'intervalle réellement terminé de 6,67125 s correspond à **8,99 pas/minute**, aux longueurs ci-dessus. Aucun maintien calme final n'a été acquis.

La trajectoire de buste incliné vers l'avant et une marche d'aspect humain ne sont toujours pas qualifiées ; la posture de base hanches/genoux et le balancement opposé restent conservés.

## Intégrité et vérification

MuJoCo 2.1.3 reste à **800 Hz**, le QP de couple à **200 Hz**. Dans les essais au modèle sélectionné original, modèle et frottements sont conservés ; les comparaisons d'impédance ci-dessous déclarent séparément leur modification et leurs caches. Preset, réglages par défaut, moteurs à 95 % et seuils d'appui, collision, articulation, capture et glissement restent inchangés. Aucun effort de racine, pose installée ou reset physique entre les pas. Les reçus indiquent un code inchangé pendant chaque essai, avec empreintes distinctes entre variantes.

Levée et réception exigent toujours 80 échantillons qualifiés ; la fin exige **4 000 échantillons calmes**. Le dépassement de cheville hérité jusqu'à 63,75 ms reste distinct des nouveaux dépassements. L'audit reconstruit indépendamment le hull COM/capture ; une marge négative ou indéfinie échoue. Les contraintes QP sur les faces courantes figées ne constituent pas un certificat de viabilité dynamique. La piste d'amortissement tangentiel a été abandonnée sans gain de séquence validé. Les corrections locales de réponse native, y compris le suivi de l'accélération atteinte du QP et les restaurations de faisabilité, ont échoué sur la capture physique ; aucune n'est intégrée au contrôleur de production.

Le relais par recentrage évite le précédent échec articulaire du troisième transfert, mais n'élimine pas le glissement : transfert et recentrage représentent 73,46 % du total gauche et 75,34 % du total droit du candidat 8. L'analyse de 16 checkpoints distingue saturation du cône de Coulomb et dérive du contact souple : à l'arrêt, les pieds chargés à 224/175 N glissent à 1,163/1,328 mm/s alors que les contacts n'utilisent au maximum que 4,94/6,44 % du cône pyramidal (μ effectif 1). Cela appuie la dérive du contact régularisé plutôt qu'une saturation terminale ; des transitoires non sauvegardés ne sont pas exclus. MuJoCo 2.1.3 décrit explicitement cette dernière possibilité ; les mesures par contact ont été vérifiées sans intégration ou mutation du modèle natif. Tout essai ultérieur d'`impratio` doit être séparé et déclaré, avec modèle/cache liés à ses paramètres ; il ne qualifie pas le modèle original inchangé. Voir [les options 2.1.3](https://mujoco.readthedocs.io/en/2.1.3/XMLreference.html#option) et [la formulation des contraintes](https://mujoco.readthedocs.io/en/2.1.3/computation.html#constraint-solver).

- **30 tests d'audit réussis**, dont relais direct et préservation du mode historique : 200 échantillons stables acceptés, 199 refusés.
- **10 tests contrôleur réussis**, dont parité exacte des couples par défaut avec `0ffb1bb`.
- Preuve antérieure : **63 cas, 61 réussis, 2 ignorés** faute d'anciens assets natifs. Les journaux agrégés distincts de 72 et 73 cas sont conservés séparément. Aucun succès de CI complète n'est revendiqué.

Les régressions actuelles réussissent en deux ensembles disjoints : **48 tests contrôleur** et **70 tests d'audit/admission** (52 + 18), sans intégration physique de validation de marche. Les anciennes séries et leurs deux ignorés historiques restent séparés.

Le manifeste conserve contrôles exacts, compteurs, réceptions et SHA-256. Les audits 7/8 reconstruisent l'intégrité native, l'opposition des bras et les contributions de glissement par phase. Le diagnostic de contacts actif et son futur résultat restent séparés de ces essais arrêtés.


Les preuves compactes de ce point d'étape sont dans
[le manifeste](../evidence/getup-source-cadenced-2026-10-18/manifest.json).
Les journaux bruts, bibliothèques, XML et caches restent locaux, hors commit.
Le point d'étape ne promeut aucun réglage et ne clôt pas la demande de dix pas.

## Comparaisons explicites d'impédance, sans changement de μ

Le recentrage original a été rejoué sur 1 216 intégrations : qpos, qvel et glissement correspondent **bit à bit** au journal conservé. Une première relecture de 389 intégrations s'était arrêtée sur une tolérance numérique diagnostique ; reproduire exactement la soustraction du temps global a éliminé cette différence, sans changer les seuils physiques.

Deux modèles isolés ont ensuite reçu chacun un cache froid lié à son XML, aux mêmes qpos/qvel initiaux. Seul `impratio` change ; μ = 1, moteurs, géométrie, solveur, `solref`/`solimp`, cadence et gardes restent identiques. **Aucun cache complet du modèle original n'est importé dans les modèles modifiés. Aucun preset n'est promu.**

- **impratio 2** : quatre pas de 182,049 / 182,009 / 181,559 / 181,993 mm terminés à 8,64375 / 15,3075 / 21,9675 / 28,6325 s, soit environ 9,00 pas/minute après le premier. Arrêt au cinquième transfert à 29,43125 s / 23 545 intégrations : glissement 10,000106 / 8,774740 mm. Réserve articulaire terminale +0,019856 rad, capture +79,671 mm. Le gain causal sur la dérive est insuffisant pour dix pas.
- **impratio 8, avant correction d'admission** : arrêt dès la première levée à 4,5175 s / 3 614 intégrations sur capture −2,653 mm. Réserve articulaire +0,010968 rad ; glissement seulement 0,589 / 0,484 mm. Aucun pas terminé. Le changement de topologie d'appui est diagnostiqué ci-dessous.

Les audits indépendants conservent le refus d'identité du modèle original. Leurs indicateurs séparés de qualification expérimentale restent **faux** pour les deux profils. Les 27 159 intégrations de ces deux comparaisons et les 1 605 intégrations des deux relectures sont des sous-totaux distincts. Aucun essai à impratio 5 n'a été créé.

Le lanceur optionnel est `source_contact_impedance_probe.py`, ses caches restent séparés et son inspecteur en lecture seule se reconstruit avec `build_source_contact_inspection.sh`. Les **79 tests spécifiques** (22 lanceur, 53 audit, 4 inspecteur) réussissent sans intégration physique. Les [preuves compactes](../evidence/getup-source-cadenced-2026-10-18/contact-impedance-comparisons.json) distinguent tous les profils. La vidéo livrée montre les quatre pas du profil 2 à **1× : 858 images, 30 images/s, 28,600 s**, sans queue d'échec ni maintien inventé. Elle ne représente pas dix pas ni cinq secondes calmes finales.

## Admission prospective sur l'appui réel, impratio 8 explicite

L'échec précédent change en 1,25 ms de quatre contacts à trois : la marge passe de +12,107 à −2,653 mm alors que le point de capture ne bouge que de 43,8 µm et reste +12,871 mm dans l'ancien quadrilatère. La charge d'appui reste 401,6–402,2 N. Aucun échantillon n'est ignoré et aucune garde de capture n'est assouplie.

Une réserve positive après **n'importe quelle** perte d'un coin n'est pas possible dans un rectangle : les quatre triangles survivants n'ont pas d'intérieur commun. Le nouveau contrôleur traite donc un risque unique sélectionné causalement, sans prétendre couvrir toute perte de contact.

Les deux premiers pilotes géométriques s'arrêtent sans lever le pied : **3 200 intégrations / 4 s**, puis **4 802 / 6,0025 s**. Le coin le moins enfoncé sous double appui exige une correction de 21,34 mm, au-delà de la borne de 12 mm. Attendre la géométrie post-déchargement avant d'autoriser ce même déchargement créait une dépendance circulaire.

`source_robust_support_steps.py` copie alors le contrôleur et résout le QP original avec son déchargement proposé, uniquement pour prévoir le coin à risque. La pression normale minimale prédite doit désigner le même coin que la tendance de dégagement à 100 ms calculée par la réponse native pure. Ce calcul n'applique aucun contrôle, n'intègre aucun état et laisse les octets, le modèle et le compteur natifs inchangés. Les points du triangle conservateur restent les **trois supports réels actuels distincts**, pas des contacts futurs prédits.

La décision exige 21 observations sur les 100 ms précédentes, au moins 80 % d'accord, les trois supports présents et une charge d'appui d'au moins 80 % du poids. La référence COM reçoit une correction C2 bornée : 12 mm maximum, 25 mm/s, 0,15 m/s², au moins une seconde. La réserve cible vaut 4 mm ; le COM et la capture mesurés doivent rester à au moins **3 mm dans le triangle réel pendant les 80 échantillons d'admission**. Les contraintes prédictives s'ajoutent aux gardes physiques d'origine. La correction s'annule continûment au recentrage ; aucune pose ou dérivée physique n'est réinitialisée.

- **Pilote prospectif qualifié, un pas** : avance réelle **181,978 mm**, réception chargée à **8,1275 s**, puis **5 s calmes** ; total **14,64875 s / 11 719 intégrations**. Glissement 1,367 / 1,555 mm. La correction de 6,073 mm dure 1 s ; les 642 échantillons de levée restent à au moins **3,686 mm** dans le triangle réel. Les 21 checkpoints de prévision, QP proposés, pressions, 33 composantes de réponse native et jets des extrémités sont reproduits exactement sans intégration. Seul l'indicateur expérimental de ce pilote est vrai ; le modèle original et la qualification dix pas restent faux.
- **Tentative continue de dix** : **six pas terminés**, de 181,961 / 182,014 / 181,216 / 181,987 / 181,412 / 181,996 mm, aux temps 9,64875 / 17,28375 / 24,9175 / 32,5525 / 40,18625 / 47,82375 s. Arrêt au septième transfert à **49,4625 s / 39 570 intégrations**, sur le glissement global **10,000058 / 7,849188 mm**. Aucun repos final n'est acquis. Les **126 prévisions natives/QP** sont reproduites exactement ; admissions, opposition des bras, articulations, moteurs, collisions et continuité passent indépendamment. La capture minimale mesurée vaut **3,035 mm**. La séquence échoue bien sur le glissement global et l'inachèvement, pas sur un désaccord d'audit.

L'intervalle terminé est d'environ **7,634 s, soit 7,86 pas/minute**. Le mouvement est physique ; la correction ajoutée après le transfert nominal ralentit cependant cette version par rapport aux 9,00 pas/minute de la vidéo du profil 2. Elle améliore l'admission d'appui, pas la vitesse par rapport à cette vidéo.

Transfert et recentrage représentent **69,48 %** du glissement des deux pieds, réception **16,11 %**, démarrage **5,92 %**. Ces sommes servent uniquement à l'attribution ; la limite reste 10 mm par pied sur toute la séquence. Au dernier état, les charges réelles valent 5,254 / 396,368 N. Le calcul frais simultané utilise au maximum 76,90 / 1,56 % du cône pyramidal effectif μ = 1 ; les huit contacts portent au moins 1,229 N. La dérive du contact régularisé est compatible avec cet état terminal, sans exclure une saturation transitoire plus tôt.

Sous-total de ces quatre essais : **59 291 intégrations**, distinct des comparaisons précédentes. Les **27 tests du contrôleur prospectif**, **48 régressions contrôleur** et **61 tests d'audit** passent dans des journaux séparés ; ce n'est pas une CI complète. Preset et chemins par défaut restent inchangés. L'impédance 8 est explicite, avec le même μ, les mêmes moteurs, géométries et gardes ; aucun réglage supplémentaire n'est exploré ici. Le buste en avant et l'allure humaine demandés restent un travail distinct, à revoir sur la vidéo avant implémentation.

Les [preuves compactes d'admission prospective](../evidence/getup-source-cadenced-2026-10-18/robust-support-checkpoint.json) conservent les refus, le pilote qualifié et l'échec continu distincts. Les payloads natifs compressés, journaux et bibliothèques restent locaux, hors commit.
