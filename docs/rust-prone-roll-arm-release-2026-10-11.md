# Roulis natif : bras libéré, pivot scapulaire non traité

2026-10-11, branche locale `investigate/physical-support`, HEAD
`c597959f46f37715035349c8cd0a699b6822111a`, conforme à la référence fournie.
Le tracking `origin/main` demeure absent : aucun nouvel état distant n'est affirmé.
Aucun push, merge ou déploiement. Récupération de production désactivée et non qualifiée.

La nouvelle tâche **libère réellement le haut du bras et les liens distaux**,
mais l'appui se reporte sur la ceinture scapulaire droite. La chaîne entière
reste libre pendant **2,254 s** ; le roulis atteint **42,24° au bassin / 51,80°
au torse**, tête/nuque déchargées. À **8,608 s**, la ceinture porte **256,25 N**
et la garde de recontact de 0,20 s arrête le candidat. Le jalon latéral de 75°
n'est pas acquis. Aucune répétition ni phase suivante n'est exécutée.

Le candidat est encore en mouvement à cet arrêt : il ne faut pas annoncer une
nouvelle stagnation ni une régression déduite de ses angles finaux à une durée
plus courte. Le prototype refusé est retiré du code actif et conservé avec son
binaire exact. Le témoin, ses traces et son déchargement antérieur restent intacts.

## Comparaison à temps physique égal

Le témoin utilise le binaire exact du précédent roulis avec pression. Il
reproduit ses **3 186 pas / 13,275 s**, positions et vitesses des 25 corps
identiques bit à bit. Seules les durées murales de projection enregistrées
peuvent différer ; elles ne font pas partie de l'état physique.

Le candidat repart de la même histoire C→repos ventral→trois tâches de paume
gauche. **1 469 pas / 6,120833 s** sont communs avant la divergence de commande.
Ni reconstruction, reset, translation ou rotation imposée au bassin, ni pose
mondiale optimisée : chaque mouvement provient des moteurs bornés et contacts natifs.

| Mesure au pas commun de 8,608333 s | Témoin, main seule | Candidat, chaîne droite |
|---|---:|---:|
| Roulis réel bassin / torse vers la droite | 41,59° / 50,25° | 42,24° / 51,80° |
| Tête / nuque | 0 / 0 N | 0 / 0 N |
| Bras supérieur droit | 149,20 N | 0 N |
| Ceinture scapulaire droite | 80,76 N | 256,25 N |
| Avant-bras / torsion / main droits | 0 N | 0 N |
| Paume gauche | 75,81 N | 66,37 N |
| Torse | 97,55 N | 68,40 N |
| Moment cinétique global Z | −0,0827 kg·m²/s | −0,7192 kg·m²/s |
| Énergie cinétique | 0,0806 J | 1,1365 J |
| Accélération du COM X/Y/Z | −0,0754 / −0,0227 / +0,0092 m/s² | −0,0948 / −0,0445 / −0,0719 m/s² |
| Tous les 25 corps sous les vitesses originales | oui | non |

Le témoin poursuit ensuite jusqu'à sa stagnation originale à 13,275 s,
46,10°/61,87° finaux. Le candidat n'a aucune trace après 8,608333 s ; aucun
résultat à 13,275 s ne lui est attribué.

![Comparaison native et charges réellement résolues](../evidence/prone-roll-arm-release-2026-10-11/comparison.png)

Les vues du haut utilisent les dernières poses natives disponibles avant
l'arrêt court : 8,600 s pour le témoin et 8,608 s pour le candidat. Les courbes
et le tableau utilisent les vrais pas ; aucune pose interpolée n'est ajoutée.

![Préparation, tête déchargée, puis recontact scapulaire](../evidence/prone-roll-arm-release-2026-10-11/milestones.png)

[Vidéo native complète du candidat](../evidence/prone-roll-arm-release-2026-10-11/native.mp4) :
174 images, 20 images/s, 8,7 s de lecture pour 8,608 s physiques. Les images
initiale et d'arrêt supplémentaires expliquent l'écart de moins de 0,1 s.
Vitesse nominale ×1, aucune interpolation ni nouvelle intégration physique.

## Intervention annoncée avant exécution

Les contacts à libérer sont ceux du sol avec les cinq liens droits : ceinture
scapulaire, bras supérieur, avant-bras, torsion et main. La paume gauche et les
cuisses conservent leur rôle de support ; le tronc peut recevoir un appui
intermédiaire. Le passage de préparation conserve la main haute ≥0,12 m et
ajoute l'absence de charge actuelle/persistante ≥3 N ainsi que la clearance
des skins natives de chacun des cinq liens.

La demande de pression gauche reste celle du précédent essai : nominalement
150 N, traduite par −JᵀF en références de moteurs avec réduction uniforme selon
leurs plafonds. Elle n'est pas une force appliquée directement à un corps et
n'est pas identifiée à la réaction réelle de la paume. Gains, frottements,
articulations, anatomie, plafonds et allocation agrégée de 760 Nm restent inchangés.

Le moyen articulaire est contrôlé avant l'essai sur les poses enregistrées.
La rotation X de la ceinture ajoute une course que l'épaule seule, auparavant
en butée d'extension −35°, n'offre plus. Une seule référence est choisie :
ceinture (0,35; 0,23; −0,10), épaule (−0,50; −0,15; 0,20), coude X 0,35,
torsion et poignet neutres, en radians. Deux chemins descriptifs de 101 poses
respectent les limites, le sol et l'auto-collision ; ils gardent la racine
enregistrée immuable. **Cette géométrie ne prouve pas le pivot dynamique.**

La tâche mondiale de main droite est remplacée par cette référence liée au
torse, avec projection des points bas de toute la chaîne depuis le torse natif
MESURÉ. Les coordonnées articulaires seules peuvent changer. La demande de
clearance vaut les skins natives (4 mm) + 5 mm supplémentaires ; la séparation
torse–bras existante est conservée. Le nombre d'itérations et les corrections
sont bornés. Le nouveau contrôle des jacobiens de sommets valide leur convention.

Une charge réapparue sur un lien droit pendant 0,20 s refuse cette stratégie
de dégagement. Après 0,55 s de déchargement tête/nuque, leur recharge persistante
pendant 0,20 s refuse aussi la tâche. Ces exigences supplémentaires ne remplacent
ni n'abaissent le jalon latéral ou les critères Brace/Stand originaux.

## Cause du refus et portée du diagnostic

**Le haut du bras n'est plus le frein porteur.** À l'arrêt ses sommets sont
à 7,79 mm du sol, au-dessus des skins de 4 mm ; sa charge et celles des liens
distaux sont nulles. La ceinture descend à 1,02 mm, et ses deux points chargés
portent 256,25 N. Leur moment normal autour du COM suivant Z vaut **+38,24 Nm**,
opposé aux **−39,64 Nm** de la paume gauche. Les autres appuis et la friction
participent au bilan. Les moments normaux utilisent les points effectivement
résolus ; le moment de friction au centroïde reste une estimation.

Tête/nuque sont déchargées sur 1,171 s de pas enregistrés pendant le roulis,
avec déchargement persistant acquis à 7,992 s et conservé jusqu'au refus.
Cela ne constitue pas un appui latéral stable : le candidat est encore mobile,
son maximum des sous-critères de support sans angle n'est que 0,0083 s et
le jalon complet vaut 0 s. Sur les 0,55 s finales, les charges moyennes sont
torse 327,34 N, paume gauche 53,69 N, ceinture droite 35,61 N, cuisses
75,80/212,31 N ; tête/nuque et chaîne distale droite 0 N. La montée de charge
scapulaire est récente, donc sa valeur instantanée n'est pas un équilibre statique.

**La projection ne satisfait plus ses contraintes à cette pose.** À la
dernière commande, ses propres prédictions placent la ceinture à −0,42 mm et
le bras à −5,44 mm, au lieu des 9 mm demandés. Recalculées depuis le torse
mesuré après le pas, elles valent −0,47 et −5,59 mm. La cible Y de la ceinture
est à sa borne +15°. Résoudre sans traiter activement les bornes, puis clamper,
laisse un résidu et produit une référence invalide. Le candidat ne peut pas
être déclaré validé parce que certains liens réels restent plus hauts que
leurs références grâce à leur état dynamique et aux contacts.

Un diagnostic géométrique supplémentaire, après le refus, examine uniquement
la ceinture sur son domaine articulaire complet au torse final FIXE : grille
101³ avec reste Lipschitz borné. Aucun nouveau candidat, gain ou pas natif.
Il trouve une hauteur minimale de 10,43 mm pour ce lien et borne son maximum
à 11,84 mm. **Il ne prouve donc pas une impossibilité de dégagement anatomique.**
Mais ce coin du domaine, appliqué avec les autres angles inchangés, place la
main à −43,92 mm sous le sol : libérer la ceinture seule ne libère pas la chaîne.
Le choix de posture initiale et la projection à bornes non actives sont insuffisants
pendant l'évolution réelle du torse. Ce diagnostic n'est pas une nouvelle trajectoire.

La suite structurelle doit coordonner ceinture, épaule et coude avec leurs
bornes actives, et refuser une référence ayant encore un résidu de clearance,
ou définir explicitement un pivot scapulaire puis son transfert de charge.
Augmenter des gains ou supprimer un contact ne corrige pas une référence
géométriquement invalide. Aucun de ces changements suivants n'est exécuté ici.

L'énergie mécanique passe de 109,02 à 119,18 J pendant le roulis, pic cinétique
2,68 J ; le COM se déplace réellement de (−0,0209; 0,1539; 1,1718) m à
(0,1037; 0,1667; 1,1260) m. Tous les pas gardent forces/couples directs au
bassin à 0 et des couples livrés sous leurs caps. L'axe Y gauche reçoit
27,45 Nm sur 28 Nm anatomiques ; le ratio élevé à droite concerne un cap
alloué de 4,91 Nm, pas une saturation démontrée de son plafond anatomique de 28 Nm.

## Route, vérifications et reproduction

La route [documentée au précédent checkpoint](rust-prone-roll-2026-10-11.md)
reste C→repos ventral→paume gauche→roulis→latéral/dorsal→mains/pieds
plantés→accroupi supporté→Stand. Le nouveau résultat précise le transfert
scapulaire à résoudre avant latéral/dorsal. Seules les trois premières phases
ont des pas natifs ; les suivantes restent conditionnées au jalon puis à sa
répétition depuis l'histoire complète. Les textes/légendes primaires HumanUP
déjà lus servent de référence de route, pas de validation de nos contacts.
[Article primaire](https://arxiv.org/html/2502.12152v1). Aucune vidéo externe403
n'a été visionnée et aucune restriction n'a été contournée.

![Route et nouveau point de transfert restant ouvert](../evidence/prone-roll-arm-release-2026-10-11/route-phases.png)

Budget annoncé : trois cas de 14 s et un transfert éventuel de 18 s, chacun
120 s murales ; maximum 60 s physiques / 480 s natives. **Deux histoires
consommées : 21,883333 s physiques / 71,878704 s murales natives.** Cas3/4
non lancés après refus du jalon. Compilation, géométrie, diagnostic et rendu
sont distincts du budget d'intégration native. Le contrôle géométrique sur
un lien ajouté après refus est consigné, sans alimenter une nouvelle commande.

**Cinq unitaires ciblés passent sur le candidat**, dont le nouveau test des
jacobiens de sommets. Après retrait, **quatre unitaires ciblés passent sur les
sources remises à l'identique**, Clippy strict et formatage aussi. Les six puis
cinq tests ignorés ne comptent pas comme passés. L'exécution du test ignoré
produit des traces même lorsque la physique refuse : sa sortie Rust `ok`+n'est pas une réussite physique. **Aucune suite complète verte annoncée** ;
les échecs préexistants articulation hybride et quatre TypeScript ne sont pas
résolus par ce travail.

Preuves locales dans `evidence/prone-roll-arm-release-2026-10-11/` :

- `PLAN.md`, `native-ledger.json`, `run_case.py` : contacts/moyens définis,
  budget, appels explicites et conditions interdisant une répétition refusée.
- `reference-replay.json`, `case-01/summary.json`, `case-02/case.json`,
  `analysis.json` : déterminisme, contacts, accélération COM, énergie,
  progression, valeurs instantanées et moyennes.
- `kinematic-audit.json`, `arm-reference.json`, `girdle-clearance-bound.json`,
  `projection-diagnosis.json` : portée des contrôles géométriques séparés
  des histoires physiques ; aucune liberté mondiale du bassin.
- `source-delta/` : seulement **deux fichiers nouveaux** par rapport au premier
  roulis, `land_arm_tracking.rs` et `prone_roll_experiment.rs`, à replacer sous
  `rust/crates/sim/src/recovery/support_probe/` sur sa base exacte.
- `comparison-binary` : SHA256
  `f69f9484d68a02d41af9e1e3450d5419e933c3c2d0df3d7ea54f6b5b8d05950c`.
- `source-manifest.json`, `active-source-restored.json`, logs unitaires/Clippy/
  format : reconstruction et retrait vérifiables ; sources du début de session
  remises à l'identique.
- `render.py`, `video-metadata.json`, `media-validation.json` : fonctions de
  dessin précédentes réutilisées, états natifs, métadonnées et inspection.

La nouvelle archive est un complément. Elle réutilise
`/workspace/LaboratoireHumain-prone-roll-2026-10-11.zip`, SHA256
`6c3e8bee76c2c1d759564815c211bfb9fda8cc332b1849314a65c91ff2467fb1`,
et sa base historique Library `libfile_28c3e675dd78819193626fa3492e60c2`.
Ni le dépôt complet, ni les intrants historiques, ni le binaire témoin ne sont
dupliqués. Les fichiers antérieurs demeurent intacts. Le nouveau paquet indique
explicitement ce qu'il contient et ce qu'il réutilise.

Les livrables locaux sont complets. Le dépôt Library n'est affirmé qu'après
réception d'identifiants confirmés ; l'état est consigné séparément dans les
reçus de livraison. Seul le transport officiel de la
[compétence Library](skill://plugin_connector_1p_1b8ff8edfc1481918b252c8277e23125/library/SKILL.md)
est autorisé : « Use the bundled prepared-upload helper below. » Aucun envoi
direct ou contournement ne remplace son éventuel échec réseau.
