# Clôture P1 — 11 octobre 2026

**P1 n’est pas validée. Le budget technique est fermé, sans renouvellement ni
recherche.** Le défaut de lecture linéaire aux impacts est expliqué, le diagnostic
angulaire et une partie de la frontière terminale sont mieux délimités. Aucun
relevage physique, handoff physique positif ou qualification produit acquis.
La récupération reste désactivée par défaut ; critères, anatomie, contacts,
plafonds et gains inchangés. `main` reste à `c597959`.

## Compteurs et arrêt

Le registre existant compte huit histoires, 19 512 pas de 1/240 s, soit 81,3 s
physiques, et conserve 49 920 pas réservés. Les deux dernières histoires sont
les mêmes commandes/fixture, avec observation angulaire activée puis désactivée.
Aucune nouvelle seed, topologie ou recherche. Le registre des 96 histoires
antérieures conserve son SHA `b61f17b180afece7db1e3a7d541edeaf24094f358edd2bb7e1362797d8b600e4`.

Échéance absolue : **22:05:43,209632 UTC**. Le dernier processus natif se termine
à **22:05:40,406431 UTC**, avant cette limite. La fermeture administrative du
registre est écrite à 22:05:52,102956 ; cette écriture tardive n’est pas une
intégration supplémentaire. Aucun processus physique ne reste actif et aucun
nouveau calcul physique n’est lancé après l’échéance.

## Ce que l’observation angulaire apporte

Une troisième frontière en lecture seule prélève les poses réellement avancées,
après clamp CCD et mise à jour des propriétés de masse. Le journal distingue :

- moment avant/après résolution à géométrie de référence fixe ;
- gravité et efforts utilisateur connus ;
- moments des contacts Coulomb à partir des impulsions normales/tangentielles
  rapportées et des **bras de levier natifs figés**, sans point géométrique inventé ;
- correction inverse d’amortissement **estimée**, explicitement distincte d’une
  livraison native enregistrée ;
- transport géométrique mesuré après poses/clamp/propriétés de masse.

Cette estimation ne remplace pas le journal indépendant des opérations internes.
Les lignes articulaires, leurs Jacobiennes variables, les opérations exactes
avant/après amortissement et les éventuels plafonds restent à comptabiliser.
Le code refuse la friction simplifiée pour ce diagnostic ; son couple pur de
torsion ne peut être silencieusement omis. Aucune force n’est inférée depuis
le mouvement pour être injectée au mannequin.

Sur 1 176 résolutions de la référence, aucune frontière post-pose manquante.
Maximum du résidu incomplet en X/Y/Z :
**[0,190156 ; 0,021430 ; 0,016706] N·m·s**. Maximum du transport géométrique :
**[0,218277 ; 0,017029 ; 0,067871] N·m·s**. Correction inverse d’amortissement
estimée maximale : **[0,003369 ; 0,001054 ; 0,002871] N·m·s**.

Le maximum apparaît au pas 329, soit 1,370833 s. Ces colonnes mettent en évidence
l’importance des frontières et du transport ; elles ne démontrent pas à elles
seules un défaut mécanique du solveur, ni la fermeture du bilan. Les résidus
restent visibles et ne sont absorbés dans aucune tolérance.

Les 1 171 lignes admissibles et les hashes complets du pas refusé restent
strictement identiques entre les deux dernières histoires, et au témoin archivé
pour les données disponibles. Contacts, accélérations CoM, couples, stabilité,
états et compteurs inchangés. Même refus pied/pied à 4,883333 s, même zéro
récupération. Le résidu linéaire maximal reste 3,188×10⁻⁶ N·s vertical.

## Borne numérique et terminal

**Aucune borne native indépendante certifiée.** Les états aux extrémités et
les impulsions accumulées ne bornent pas les additions/multiplications `f32`
intermédiaires du solveur. Il manque leurs operands/magnitudes et leur effet,
y compris lignes de limites/verrouillage, amortissement, gyroscopie/plafonds
lorsqu’actifs, intégration géométrique et clamp. Le dossier est figé avant les
rejeux angulaires ; aucune tolérance n’est déduite du résidu observé.

L’unitaire original
`measured_handoff_has_matching_native_rates_and_cannot_count_during_blend`
passe : 143 demandes moteur gardent Handoff et compteur nul, la 144e passe à
Ordinary avec corps inchangés. **Zéro intégration physique** dans ce test ; ce
résultat ne prouve pas un handoff réel sous charge.

La revue statique confirme les durées 72/144/240/6 000 sous-pas pour settle,
handoff, maintien et limite d’essai. Les timers `f32` hérités franchissent 0,20 s
à la 49e addition et 3 s à la 721e ; les remplacer implicitement par des compteurs
entiers changerait leur frontière. `enter()` remet stagnation et meilleur progrès
à zéro : le futur runtime ne doit pas utiliser une nouvelle phase de plan pour
renouveler artificiellement la stagnation. Pendant handoff, `update()` retourne
avant l’évolution de certains timers ; la perte de support est refusée
immédiatement ailleurs. La limite 25 s appartient aux essais, pas à un arrêt
interne de `observe_recovery`.

Restent à démontrer : settle chargé à 71/72, maintien à 239/240 et interruption
finale, ancres absentes, perte de support pendant handoff, membre isolé trop
rapide, horloge globale et stagnation lors du changement de propriétaire moteur.
La garde historique `routeStanding` sous 0,60 m reste distincte du terminal
original ; elle n’est pas une condition générale d’une route pieds seuls.

## Sauvegarde et vérifications

Le checkpoint antérieur `2e3c9f1` a été poussé et vérifié. Le présent jalon ajoute
uniquement observation, preuves et médias ; aucun adaptateur terminal ou
planificateur de transfert activé. Build release et Clippy strict avec la
fonctionnalité passent avant les derniers rejeux. Le nouveau Clippy sans
fonctionnalité n’a pas été réexécuté ; celui de `2e3c9f1` passait. Dix unitaires
Python et format/diff passent, sans monde intégré. Suite complète non exécutée ;
échec hybride et quatre échecs TypeScript historiques toujours non résolus.

Le parent signale une déconnexion à 22:06 UTC. Dans cette session, lecture/écriture
locale restent observées à 22:07:29 pour cette sauvegarde ; aucune restauration
de la connexion du parent n’est revendiquée. La livraison Library prise en charge
échoue sur le réseau **avant transfert**. Aucune pièce jointe Library créée.
Les graphiques sont donc sauvegardés dans le dépôt :

- [Référence et impulsions CCD](media/native-hybrid-v1-2026-10-11/reference-impulse-boundaries.png)
- [Confirmation sur le témoin](media/native-hybrid-v1-2026-10-11/controller-impulse-boundaries.png)
- [Diagnostic angulaire non certifié](media/native-hybrid-v1-2026-10-11/angular-diagnosis.png)

Le reçu détaillé est `recovery-p1-closure-results-2026-10-11.json` ; les traces,
registres et binaires restent dans `evidence/native-hybrid-v1-2026-10-11/`.

## Prochaine décision proposée, sans lancement

Un lot d’instrumentation indépendant doit enregistrer les impulsions avec leurs
Jacobiennes natives et les opérations numériques, puis geler la borne avant
les rejeux de certification. Le travail statique et son examen sont estimés à
3–5 journées, avec incertitude ; cela ne promet ni fermeture de P1 ni relevage.

Pour cette seule vérification future, **proposition à décider séparément** :
huit histoires maximum de 26 s, 49 920 pas natifs maximum, 30 min murales de
vérification après instrumentation et borne gelées, un processus mono-thread.
Ce n’est pas une prolongation, réinitialisation ou reprise du registre fermé.
Aucun budget n’est créé ou autorisé implicitement ici. Recherche toujours
interdite avant P1 validée ; qualification générale reste distincte.
