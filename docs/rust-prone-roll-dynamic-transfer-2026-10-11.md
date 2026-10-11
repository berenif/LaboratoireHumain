# Transfert dynamique bras droit → torse — 11 octobre 2026

Le transfert articulaire produit une redistribution mesurée, mais ne franchit
pas l'équilibre. Le candidat atteint **43,8124° bassin / 55,3551° torse**, soit
**+0,1114° / +0,2220°** face au contrôle à transfert nul corrigé. Les deux
s'arrêtent à **12,04583 s** sur les **3 s de stagnation originales**. Le jalon
latéral75° reste non acquis ; aucune répétition de réussite ni phase suivante.
La fenêtre effective d'action est10–12,04583 s : le compteur de progrès global
n'a pas été remis à zéro à l'entrée du transfert. Cela borne la conclusion à ce
candidat et cette fenêtre, sans démontrer une impossibilité générale du relevage.

HEAD `c597959f46f37715035349c8cd0a699b6822111a`, branche locale
`investigate/physical-support`. Aucun push/merge/publication. Récupération
désactivée et non qualifiée ; contrat, anatomie, profils et seuils inchangés.

| Cas natif | Temps physique | Résultat |
| --- | --- | --- |
| 01 maintien témoin | 12,04583 s | 2 891 hashes corporels identiques à l'archive précédente ; stagnation |
| 02 première conversion, transfert nul | 10,75833 s | Torse sans appui résolu ; génération de commande refusée, conversion rejetée |
| 03 conversion implicite corrigée, transfert nul | 12,04583 s | 43,7010° / 55,1331°, appuis et vitesses stables ; stagnation |
| 04 même conversion, transfert demandé | 12,04583 s | 43,8124° / 55,3551°, appuis et vitesses stables ; stagnation |

Toutes les histoires partent de la chute C complète, sans reset de corps ou
d'impulsions. Cas03 et04 partagent les2 400 pas du maintien jusqu'à10 s et les
2 401 premiers pas entre eux. Le verdict de recontact historique reste faux.
Le refus de dégagement9 mm à8,6625 s reste refusé : aucun itéré non certifié
n'est installé. Le maintien de commande antérieur prépare l'équilibre observé.

Le signe a été vérifié dans la convention native : roulis utile négatif autour
de l'axe longitudinal. À10 s, transférer une normale bras→torse donne
`[+0,05394 ; 0 ; −0,12424]` N·m/N, direction utile également contrôlée dans le
monde natif. Le couple généralisé enfant a le signe opposé à l'impulsion de ligne
rapportée ; les impulsions réellement délivrées au corps enfant corroborent
cette convention. Tous les24 joints sont audités, avec contraction native des
huit coins de la boîte et norme vectorielle allouée. La marge conservatrice à10 s
est **2,274 N**, limitée par la ceinture droite X :6,586 N·m délivrés,
plafond effectif6,672 N·m après contraction, plafond alloué6,837 N·m,
anatomique32 N·m. Le plafond anatomique n'est pas saturé.

La commande construit `−JᵀΔF` depuis les deux contacts réellement résolus, sur
les moteurs du bras, de la ceinture, du torse et des lombaires. Elle conserve les
positions de référence, gains, plafonds et modèles moteurs. Elle convertit les
couples mesurés à l'entrée plus cette variation en cibles de vitesse, sans force
appliquée aux corps. Le premier inverseur omettait le terme natif `dt×K` et son
contrôle nul perdait l'appui destinataire ; cet arrêt est un refus de candidat,
pas une stagnation physique. La correction emploie `D+dt×K` et une régression
vérifie l'équation de la ligne ForceBased. Le contrôle nul corrigé reste stable,
mais ses charges diffèrent du maintien : cette conversion n'est pas un maintien
identique. Le cas04 est comparé aux deux témoins, sans attribuer sa différence
totale face au maintien au seul transfert.

Rampe1 s vers4 N ; maximum réellement **admis comme demande :2,3246 N**.
Entre11 et11,99583 s, les deltas face au contrôle nul sont :

| Mesure native moyenne | Delta transfert − contrôle nul |
| --- | --- |
| Normale haut du bras droit | −7,379 N |
| Normale torse | +14,256 N |
| Normale ceinture droite | −4,619 N |
| Moment normal global Z | −0,08655 N·m |
| Moment global Z observé par `dH/dt` | −0,00913 N·m |

Les autres appuis changent aussi : la redistribution n'est pas un échange isolé
de deux normales. La demande admise moyenne vaut1,990 N ; son modèle statique
prévoit−0,24789 N·m de moment normal Z, davantage que le changement résolu.
L'estimation de moment de friction au centroïde oppose+0,05090 N·m de deltaZ ;
elle n'est pas un moment exact par point. Le moment cinétique natif établit la
faible réponse globale, indépendamment de cette estimation.

La livraison de couple explique pourquoi la demande virtuelle ne vaut pas
réalisation du transfert prévu. Sur la dernière demi-seconde, la ceinture X
demande en moyenne6,665 N·m, mais délivre5,266 N·m ; son delta réellement livré
face au contrôle nul vaut−0,788 N·m, alors que le modèle demande un incrément
positif. Le bras X demande−8,626 N·m et délivre−6,833 N·m. Les limites de couple
et vectorielles sont respectées ; le maximum vectoriel mesuré sur les24 joints
du candidat vaut0,963 du plafond alloué. Ce défaut de suivi est mesuré, sans
prétendre isoler toutes les réactions de limite articulaire ou de contact.

L'équation native emploie les vitesses résolues en fin de pas, tandis que la
conversion utilise les vitesses observées avant le pas. L'écart sous contraintes
et contacts est donc un mécanisme explicatif issu du code, **pas une mesure
directe de chaque réaction**. La suite ciblée serait de corriger le suivi des
couples réellement délivrés et des normales mesurées, en gardant un contrôle
nul identique au maintien ; aucune nouvelle posture arbitraire n'est justifiée
par ces données. Ce candidat n'est pas conservé comme contrôleur de relevage.

Tête/nuque0 N après10 s, forces/couples directs au bassin0 sur toutes les traces.
Critères de vitesse et enveloppe COM/capture réels satisfaits aux arrêts physiques.
Sources expérimentales retirées ; six sources de départ restaurées exactement,
projection pure précédente conservée. Onze unitaires candidat corrigé passent,
sept après retrait. Clippy strict candidat et final, format final et diff-check
passent ; suite complète non relancée ni déclarée verte. Échec hybride et quatre
échecs TypeScript historiques toujours non résolus.

Budget clos : **quatre histoires,46,89583 s physiques /144,20681 s natives**,
sous125/600 s annoncés après correction du contrôle nul. Aucun cinquième essai.
[Traces, audits et registre](../evidence/prone-roll-dynamic-transfer-2026-10-11/analysis.json),
[budget vectoriel](../evidence/prone-roll-dynamic-transfer-2026-10-11/vector-audit.json).

[Comparaison](../evidence/prone-roll-dynamic-transfer-2026-10-11/comparison.png),
[trois étapes](../evidence/prone-roll-dynamic-transfer-2026-10-11/milestones.png),
[conversion rejetée](../evidence/prone-roll-dynamic-transfer-2026-10-11/rejected-conversion.png),
[vidéo native locale de2,30 s](../evidence/prone-roll-dynamic-transfer-2026-10-11/native.mp4).
46 images natives20 images/s, ×1 nominal, sans interpolation ; dernier frame
décodé inspecté. Archive complémentaire locale :
`/workspace/LaboratoireHumain-prone-roll-dynamic-transfer-2026-10-11.zip`.
Library inchangée : aucune tentative sur le transport réseau bloqué.
