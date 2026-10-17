# Levée gauche : audit de commande et correction gravitaire vérifiée

La première levée gauche reprend avec une **compensation de gravité par les
moteurs articulaires existants**. Une comparaison appariée, avec 1 268 pas
natifs strictement identiques avant l’intervention, donne **99,948 mm**
d’erreur finale pour A et **24,350 mm** pour B. B franchit le seuil inchangé
de **50 mm après 187,5 ms**, puis y reste pendant les 76 derniers pas
(0,3167 s). Aucun passage latéral, nouvel appui ou relevage du torse n’est
commandé. La récupération demeure désactivée par défaut et non qualifiée.

Ce résultat établit la suffisance de cette correction pour la tâche bloquée
dans cet état reproduit. Il ne prouve pas la réussite d’un brace, d’une
séquence complète, ni la nécessité universelle d’un appui main/genou.

Le travail reste local sur `investigate/physical-support`, à
`c597959f46f37715035349c8cd0a699b6822111a`, correspondant à la référence main
fournie. Aucun push, merge ou publication. Les modifications de production
préexistantes sont conservées ; aucune nouvelle modification de production
n’est apportée par cet audit. Les corrections locales non présentes,
décrites dans la délégation, ne sont pas revendiquées.

## Chaîne IK → coordonnées → moteur → état réel

L’audit exploite d’abord le code et la trace originale. Il ne lance aucune
optimisation, nouvelle IK, recherche de trajectoire ou campagne de gains.

- IK, mesure et solveur utilisent les mêmes coordonnées :
  `q_i = wrap(2 atan2(r_i, r_w))`, avec quaternion relatif entre les deux
  repères articulaires. L’inverse est le quaternion normalisé
  `[tan(qx/2), tan(qy/2), tan(qz/2), 1]`.
- La vérification numérique du Jacobien a une erreur maximale de
  `1,65e-10`. La reconstruction FK du bras mesuré diffère de la main
  réellement enregistrée de `8,94e-8 m`. Ces contrôles numériques restent
  distincts de l’intégration physique.
- Les cibles du programme, leurs petits décalages capturés à l’entrée et
  les commandes enregistrées correspondent à `1,34e-7 rad` près. Aucune
  cible n’est hors limites. Les repères natifs et les repères canoniques
  ont des rotations identiques et des écarts d’ancrage d’au plus `3,58e-9 m`.
- Les paramètres réellement installés sont les cibles/rates attendus,
  les gains du profil figé et le modèle `ForceBased`. Le parent natif
  correspond au parent canonique. Le hook intervient après la commande
  ordinaire et avant l’intégration ; aucune réécriture ultérieure de la
  cible n’apparaît avant le solveur.
- Aucune impulsion de limite sur un axe libre du bras ne participe au
  dernier équilibre. Les couples livrés, reconstruits depuis les
  impulsions angulaires natives, ont le signe restaurateur attendu.

**Aucun défaut de repère, signe ou mapping n’est établi.** Dans la pose
réelle du torse à la stagnation, la même référence angulaire place
algébriquement la main à 20,71 mm de la cible originale. Ce calcul explique
pourquoi la cible peut satisfaire les 50 mm sans nouvelle trajectoire ; il
n’est jamais envoyé comme pose aux corps.

## Équilibre mesuré, y compris le contact entre corps

A reste immobile autour de 100 mm d’erreur pendant l’extension de 0,5 s.
Un **contact réel torse–bras supérieur gauche** fournit 4,819 N normaux et
1,746 N tangentiels au dernier pas. Les deux points résolus coïncident ;
l’impulsion tangentielle de la manifold est comptée une seule fois.

| Axe libre | Gravité généralisée | Moteur transmis | Contact | Résidu statique |
|---|---:|---:|---:|---:|
| Ceinture scapulaire gauche X | −12,8556 Nm | +12,7638 Nm | +0,09085 Nm | −0,00090 Nm |
| Bras supérieur gauche X | +12,5781 Nm | −12,5053 Nm | −0,07180 Nm | +0,00097 Nm |

Sur tous les axes libres du bras, le résidu gravité + moteur + contact est
inférieur à **0,00486 Nm**. Omettre les contacts entre corps laisse un
résidu allant jusqu’à 0,348 Nm et donne une attribution incomplète.

La commande antérieure est un ressort/amortisseur. À vitesse nulle et
écart articulaire nul, elle ne fournit aucun effort moteur stationnaire.
Dans cet équilibre, les ressorts se déforment pour supporter la gravité
et le contact ; cette déflexion rend la première tâche insuffisamment
suivie. Les caps disponibles seuls et le remplacement algébrique d’angles
dans l’ancien rapport ne suffisaient pas à prouver cette cause.

Le contact n’est pas nié : le déplacement du point matériel vers la même
référence projette **−1,807 mm** sur l’ancienne normale sortante. Une
ouverture selon cette normale n’était donc pas établie. La géométrie
complète de la cible est séparée de 10,311 mm, au-delà des skins cumulés
de 8 mm ; le chemin inchangé présente une compression de skin allant
jusqu’à 0,202 mm, sans violation de la garde de collision. A seul
n’excluait pas un blocage par contact. B conserve tous ces contacts et
teste physiquement la compensation, sans déplacer la cible.

L’ancienne inférence textuelle incorrecte d’ouverture normale est
conservée dans `rejected-contact-normal-inference.json`, puis corrigée
dans le calcul et le rapport. Elle n’a déclenché aucune troisième expérience.

## Intervention unique et comparaison appariée

Chaque cas reconstruit uniquement le départ original C, rejoue C puis
la même première levée dans le même monde. Il ne recharge jamais un repos,
ne clone aucun monde vivant, ne modifie aucune pose/vitesse, et conserve
les handles et la génération de l’assemblage. La référence A reproduit
exactement les champs physiques, contacts, momentum, commandes et
observations du témoin archivé sur ses 1 268 pas. A et B sont ensuite
strictement identiques sur **tous les champs enregistrés** de ce préfixe.

À `1268/240 = 5,283333 s`, A garde sa consigne. B calcule, pour chacun des
dix axes libres des cinq segments du bras gauche :

```
tau_g = J^(-T) R_joint^T sum_descendants[(COM_world − ancre_world) × m g]
delta_motor_target_velocity = −tau_g / damping_installed
```

Masses, centres de masse, repères et Jacobien proviennent de l’état natif
mesuré avant le pas. L’audit vérifie que chaque biais enregistré correspond
exactement à la gravité du pas précédent et à l’amortissement inchangé.
Ce biais est une manière d’ajouter un effort au moteur `ForceBased` ; il
n’impose **aucune vitesse de corps** et n’applique aucun effort externe.

Toutes les cibles de position sont identiques dans A et B, pendant toute
l’expérience. Gains, bornes anatomiques, solveur, allocation et plafonds
restent ceux d’origine. Les caps instantanés alloués peuvent changer avec
la nouvelle demande, **sous la même règle**, sans dépasser les limites
anatomiques ou le total de 760 Nm. La comparaison valide la correction
avec cette allocation existante ; elle n’isole pas un budget instantané
artificiellement figé.

| Mesure réellement intégrée | A, consigne inchangée | B, gravité compensée |
|---|---:|---:|
| Erreur main au départ commun | 99,939 mm | 99,939 mm |
| Erreur main finale | 99,948 mm | **24,350 mm** |
| Hauteur main finale | 160,83 mm | **241,14 mm** |
| Premier passage sous 50 mm après intervention | Aucun | **187,5 ms** |
| Contact torse–bras final chargé | Oui | Non |
| Charge tête finale | 138,47 N | **143,50 N** |
| Charge torse finale | 218,58 N | 212,03 N |
| Accélération COM maximale, extension (norme) | 0,0151 m/s² | 0,5202 m/s² |
| Énergie cinétique maximale, extension | 0,00000242 J | 0,08509 J |
| 25 corps sous les seuils de vitesse au dernier pas | Oui | Oui |
| Brace / temps qualifié | Non / 0 s | Non / 0 s |

La compensation déclenche effectivement un transitoire : le couple moteur
scapulaire X reconstruit passe de 12,77 à environ 24,35 Nm au premier pas
différent. Le contact torse–bras se charge d’abord puis se décharge ; son
dernier pas chargé est à 5,3375 s. La levée franchit les 50 mm à 5,470833 s.
Ce mouvement et ces contacts sont observés dans les états intégrés, pas
dans une géométrie substituée.

Le refus original de stagnation de 3 s est conservé exactement à son
instant original dans les deux cas. L’extension diagnostique de 0,5 s
observe la réponse après ce refus ; elle ne l’efface pas et ne sauve aucune
qualification. La première tâche est maintenue même après franchissement
des 50 mm. La tête est encore davantage chargée à la fin de B : **la
correction du suivi n’établit pas un support intermédiaire sûr**.

## Gardes, coût et portée du code conservé

Les deux cas restent sous les gardes originales : séparation d’ancrage
maximale 4,026 µm ; pénétration du sol maximale 16,372 mm contre la garde
originale de 80 mm ; erreur articulaire structurelle maximale 0,008017 rad
contre 0,06 rad ; auto-pénétration non exclue 0 contre 5 mm. Ces maxima
comprennent C et le snapshot final. Les plafonds installés atteignent au
plus 760 Nm et le ratio anatomique au plus 1. Aucun couple livré ne dépasse
son cap installé. Effort/couple direct bassin : **0 N / 0 Nm** ; aucun
effort/couple direct sur les corps du bras sélectionné.

Budget déclaré : deux cas maximum, 6 s physiques par cas, 12 s cumulées,
90 s natives cumulées, garde murale de 45 s par cas. Réalisé :

- A : 5,783333 s physiques ; 21,179425 s natives.
- B : 5,783333 s physiques ; 17,109026 s natives.
- Total : **11,566667 s physiques ; 38,288451 s natives**. Aucun troisième
  cas, répétition, nouvelle trajectoire ou balayage de gains.
- Compilation initiale : 101,985 s, distincte du budget natif. La compilation
  finale des unitaires prend 105 s selon Cargo ; leur exécution prend
  0,05 s. Clippy final prend 5,36 s. Ces checks ne sont pas des rollouts.

Le correctif reste **exclusivement dans un banc `cfg(test)` opt-in**.
L’entrée diagnostique exige `reference` ou `gravity`, refuse d’écraser un
dossier de preuves, s’arrête à 1 388 pas, et conserve le refus original.
Le helper de compensation prend `&Simulation` et ne modifie que les
consignes de vitesse des moteurs avant leur installation limitée.

Les phases latérales/de montée rejetées ou non vérifiées et leur ancien
test complet sont retirés du code vivant après la comparaison. Seule la
première tâche originale subsiste dans le contrôleur de diagnostic ; les
sources et le binaire **exactement utilisés par A/B** restent figés dans
`source-frozen/` et `audit-release-binary`. Les sources nettoyées finales,
leur diff et les checks sont distingués dans `source-final/`. Ce nettoyage
ne constitue pas un troisième essai physique ni une revalidation native
du binaire final.

Clippy strict et le format passent. Deux tests ciblés du banc vérifient
l’installation de consignes/caps et la propriété des corps sans écriture
de leur état. Ils sont distincts du test diagnostique ignoré et de la
preuve physique A/B. **Aucune suite complète verte n’est annoncée** :
l’échec d’articulation hybride et les quatre échecs TypeScript signalés
sur le témoin Linux ne sont pas résolus ni réévalués ici.

## Livrables et suite encore ouverte

Les preuves se trouvent dans
`evidence/left-hand-command-audit-2026-10-11/` : témoins/manifestes,
`native-reference/case.json`, `native-gravity/case.json`, bilans statiques,
`paired-analysis.json`, scripts de lecture, sources figées et logs.
`left-hand-audit-actual.png` compare le départ commun et les états finaux ;
`left-hand-audit-actual.mp4` montre uniquement les 0,5 s réellement intégrées
de A/B, ralenties ×8, sans interpolation de poses. Légendes : **levée
reprise, tête/torse encore chargés, aucun relevage qualifié**.

L’archive locale prévue est
`/workspace/LaboratoireHumain-left-hand-command-audit-2026-10-11.zip`.
Les reçus Library privés et les empreintes de livraison accompagnent les
fichiers. L’archive originale refusée reste intacte :
`libfile_3368bcae77b081918b259c4ac5a50e23`, SHA-256
`d307172f754e88b5ee10440736b79448ccbc0f9600ddb5355341477cacdd2afd`.

Le prochain obstacle est l’acquisition d’appuis utiles sans charge tête,
puis le relevage du torse. Aucun passage extérieur, appui de main, transfert
genou/pied ou levée debout n’est validé par cet audit. La faisabilité d’un
relevage pieds seuls et la nécessité d’appuis intermédiaires restent ouvertes.
Il faut traiter ces tâches après ce résultat causal, en gardant les
critères de support, anatomie, couple et qualification originaux.
