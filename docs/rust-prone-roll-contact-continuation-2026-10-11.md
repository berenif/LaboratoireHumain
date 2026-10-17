# Continuation physique après recontact — 11 octobre 2026

Le recontact ne termine pas immédiatement le mouvement. La continuation atteint
43,60° de roulis du bassin et 54,83° du torse, contre 42,19°/51,71° à l'arrêt
historique. Le bras droit devient un appui porteur, tête et nuque restent à 0 N,
puis le corps se stabilise pendant 3,3125 s sans atteindre le jalon latéral de
75°. L'arrêt final est une stagnation physique de 3 s sous la dernière commande
maintenue. Il ne prouve pas l'échec de toutes les trajectoires possibles.

HEAD vérifié : `c597959f46f37715035349c8cd0a699b6822111a`, branche locale
`investigate/physical-support`. Aucun push, merge ni publication. Récupération
de production désactivée par défaut et non qualifiée. Anatomie, contrat figé,
profil de récupération, limites de couple et critères finaux inchangés.

Deux histoires natives seulement ont été exécutées, depuis le début de la chute C,
sans reset en cours. Le témoin corrigé est le cas 03 du
[rapport de projection bornée](rust-prone-roll-bounded-projection-2026-10-11.md).
Son verdict de sous-test reste faux, avec le même recontact de 0,20 s à 8,629 s.
Les diagnostics de continuation enregistrent ce verdict et poursuivent le monde
natif ; ils ne remplacent aucune qualification.

| Histoire | Pas / temps physique | Arrêt | Roulis bassin / torse |
| --- | --- | --- | --- |
| Référence corrigée antérieure | 2 071 / 8,62917 s | Recontact du sous-test | 42,1925° / 51,7114° |
| Cas 01 : commandes programmées identiques | 2 079 / 8,66250 s | Nouvelle référence non certifiée | 42,4712° / 52,3040° |
| Cas 02 : dernière commande conservée après ce refus | 2 891 / 12,04583 s | Stagnation physique originale de 3 s | 43,5963° / 54,8271° |

Le cas 01 ajoute huit pas au-delà du recontact. La projection refuse ensuite un
résidu maximal de 25,53 µm, sans installer sa référence ni son itéré. Ce refus
local ne démontre aucune impossibilité anatomique globale. Les vérifications
géométriques hors monde sont archivées comme diagnostics, sans insertion de pose.

Le cas 02 rejoue exactement les 2 079 premiers pas du cas 01, puis conserve les
paramètres de la dernière commande effectivement installée : positions, vitesses,
raideurs, amortissements, plafonds, modèle moteur et masque moteur. Les impulsions
du solveur et l'historique d'impulsion restent vivants. Aucune position ou vitesse
de corps n'est écrite. Le contrôle en lecture seule de cette commande vérifie
bounds, non-pénétration du sol et collisions natives ; il ne certifie pas la tâche
de dégagement à 9 mm, toujours refusée. Ce maintien diffère explicitement du suivi
de la future trajectoire programmée.

Les 2 071 hashes de corps de la référence corrigée sont reproduits. Les 173
instantanés de commande comparables ont les mêmes cibles et vitesses ; la cadence
du dernier instantané diffère à l'ancien arrêt. Les métadonnées de phase sont
identiques hors temps mural de projection. Durant le maintien, 68 instantanés
montrent des paramètres, demandes et plafonds identiques, y compris les plafonds
réellement installés dans les joints. Forces et couples externes au bassin : 0.
Voir [preuves de comparaison](../evidence/prone-roll-contact-continuation-2026-10-11/analysis.json)
et [audit des plafonds](../evidence/prone-roll-contact-continuation-2026-10-11/retention-cap-audit.json).

Au terme du cas 02, les 25 corps satisfont les vitesses de stabilité originales,
COM et capture restent dans l'enveloppe des contacts réellement chargés. Les
charges résolues finales sont : torse 298,76 N, main gauche 53,04 N, bras droit
50,96 N, cuisse gauche 70,35 N et cuisse droite 235,36 N. Ceinture et chaîne
distale droites : 0 N ; tête/nuque : 0 N sur toute la continuation. L'énergie
cinétique vaut 4,83 × 10⁻⁵ J. Le dernier progrès d'au moins 1° date de 9,04583 s.
La stabilité partielle de 3,3125 s est calculée sans la porte d'angle et ne vaut
pas acquisition du latéral : aucun des deux corps n'atteint 75°. Aucune répétition
conditionnelle, phase suivante ou réussite Stand n'est donc déclarée.

Le bilan global explique le freinage. Entre 8,6625 et 9,19583 s, `dH/dt` moyen
autour de Z vaut **+0,240 N·m**, opposé au roulis droit négatif. Sur la dernière
seconde, il vaut **−0,00126 N·m**, cohérent avec un équilibre partiel. Les moments
normaux moyens autour de Z se compensent : main gauche −31,35 N·m, cuisse gauche
−14,66, cuisse droite +13,20, bras droit +10,33 et torse +22,51. Leur somme est
+0,0297 N·m. Le moment de friction calculé au centroïde n'est qu'une estimation ;
le verdict dynamique repose sur le moment cinétique total natif.

La correction causale proposée est de transférer une fraction de la charge
normale du bras droit vers le **contact du torse déjà résolu**, par les moteurs
anatomiques et tous leurs ancêtres. Les deux points mesurés donnent, par newton
transféré, Δmoment normal `[+0,05489 ; 0 ; −0,12429]` N·m : sens favorable au
roulis, avec peu de moment avant/arrière. Le transfert vers la cuisse gauche
aurait +0,85434 N·m/N avant/arrière ; cette alternative est écartée hors code.
L'enveloppe des points réellement chargés sans le bras garde 66,71 mm de marge
COM et 66,67 mm pour la capture, à cet instant uniquement.

L'audit statique donne **4,73 N comme borne initiale conservatrice selon les
plafonds scalaires actuellement alloués**, limitée par la ceinture droite X :
6,659 N·m délivrés / 6,837 N·m alloués, pour un plafond anatomique de 32 N·m.
Il s'agit donc d'une faible marge d'allocation, pas d'une saturation anatomique.
Cette borne ne prouve ni le budget vectoriel simultané, ni le transfert dynamique,
ni le dégagement complet des 50,96 N, ni le relevage. La suite utile est un essai
borné de redistribution de charge avec mesure de la livraison, des frottements
et du moment global, sans hausse de pression de paume ou de plafond anatomique.
Voir [audit de transfert](../evidence/prone-roll-contact-continuation-2026-10-11/transfer-audit.json).
Aucun contrôleur de transfert nouveau n'a été installé dans cette session.

Les prototypes de continuation et de maintien sont archivés hors code actif.
Les six sources de départ ont été restaurées exactement ; le correctif pur de
projection bornée et sa régression restent conservés. Le candidat passe huit
unitaires ciblés, dont la préservation des impulsions lors du maintien. Après
retrait, sept unitaires ciblés passent. Clippy strict sur toutes les cibles,
format et `git diff --check` passent. La suite complète n'a pas été relancée et
n'est pas déclarée verte : l'échec d'articulation hybride et les quatre échecs
TypeScript du témoin restent non résolus.

Le budget est clos : deux histoires, 20,70833 s physiques cumulées et 53,64531 s
murales natives, sous les plafonds 75/360 s. Les sources, binaires, entrées
historiques référencées et traces déterministes permettent la revue, sans nouvel
essai de réglage. [Registre](../evidence/prone-roll-contact-continuation-2026-10-11/native-ledger.json).

Captures vérifiées : [comparaison](../evidence/prone-roll-contact-continuation-2026-10-11/comparison.png),
[trois étapes](../evidence/prone-roll-contact-continuation-2026-10-11/milestones.png).
La [vidéo locale](../evidence/prone-roll-contact-continuation-2026-10-11/native.mp4)
contient 242 images natives, 20 images/s, 12,10 s, sans interpolation, vitesse
nominale ×1. La dernière image décodée a été inspectée ; les légendes distinguent
le refus géométrique, le maintien de commande et la stagnation physique.

L'archive complémentaire locale est
`/workspace/LaboratoireHumain-prone-roll-contact-continuation-2026-10-11.zip`.
Elle contient les deux nouvelles traces, les deux binaires, les différences de
source, audits et médias ; les anciens témoins et leurs entrées ne sont pas
dupliqués. Manifeste interne avec hashes et contrôle CRC. Library reste bloquée
par son erreur officielle de transport réseau déjà archivée ; aucune nouvelle
tentative, aucun identifiant ni nouvelle version cloud. Aucun secret demandé.
