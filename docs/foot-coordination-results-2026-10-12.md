# Résultat physique du jalon de coordination des pieds

**Aucun relevage.** Le témoin B est reproduit à l'identique ; le candidat C
modifie bien la dynamique à 54,17 ms mais n'obtient pas de talon porteur. Le
registre est fermé après les deux histoires prévues, sans réglage intermédiaire.
La récupération reste désactivée par défaut et non qualifiée. L'essai C est
rejeté et retiré de l'arbre de code final ; son code figé est conservé dans le
commit `67211c7091939b9c029e99a2f489c28bf4cf8e78` et son binaire dans
l'archive locale de preuves.

| Mesure native | Témoin B | Coordination C |
|---|---:|---:|
| Pas physiques | 5 013 | 1 222 |
| Dernier temps physique | 20,8875 s | 5,0917 s |
| Compteur de récupération original | 0 | 0 |
| Premier arrêt | programme fini, observateur original refuse Stand | auto-collision avant-pied gauche / pied droit : 5,071 mm, limite 5 mm |
| Première charge sous un point postérieur du talon | jamais | jamais |
| Première charge sur l'avant de l'arrière-pied | 0,100 s | 0,100 s |
| Hauteur arrière de semelle à 50 ms, pied gauche | 46 mm | 46 mm |
| Vitesse avant du CoM à 1 s | 0,865 m/s | 0,853 m/s |
| Hauteur du bassin à 2 s | 0,160 m | 0,168 m |
| Cap anatomique maximal installé / permis | 1,000 | 1,000 |

Le témoin du nouveau binaire a les mêmes 5 013 lignes physiques et les mêmes
empreintes finales des corps, moteurs, contacts et compteurs que B archivé.
La première divergence B/C est à 0,05417 s. Les points réellement choisis par
le solveur donnent un contact d'avant-pied dès 0,00417 s ; les arrière-pieds
touchent à 0,100 s, mais **leur contact est à l'avant de la semelle**. Aucun
point sous la partie postérieure (`z local < −0,02 m`) ne porte d'impulsion
normale positive dans l'une ou l'autre histoire. L'arrière de la semelle reste
environ 14 mm au-dessus du sol pour B et 18 mm pour C à 0,5 s. Dès 1 s,
les deux arrière-pieds ont perdu leur charge et le CoM a avancé d'environ
30 cm. Sur 0,1–1,0 s, l'accélération avant moyenne est environ 1,00 m/s²
dans les deux versions. La pose légèrement meilleure de C à 2 s ne produit
aucune transition hors de Brace.

Les moteurs montrent une contrainte réelle, sans gain de force caché. Pour C,
l'avant-pied gauche atteint au moins 95 % de son cap de ligne native sur
590 des 1 221 pas observés, l'avant-pied droit sur 385 ; leurs caps sont
35 Nm. La demande de ressort brute peut atteindre 259 Nm à gauche, tandis
que l'impulsion délivrée reste limitée. B sature aussi souvent ces axes
(606 et 530 pas sur le même horizon). La friction effective aux contacts de
pied reste 4 et les points d'avant-pied atteignent ponctuellement la borne
de Coulomb ; la vitesse tangentielle résiduelle maximale mesurée dans C
atteint 1,26 m/s à l'avant-pied droit. Cette valeur maximale décrit la chute,
pas un glissement continu d'un même point matériel. Les vitesses et impulsions
sont lues aux frontières du solveur ; elles ne sont pas des couples
généralisés certifiés à l'enfant.

**Décision.** Cette commande des pieds seuls n'établit pas un pivot soutenu.
L'échec ne prouve pas que toute commande utilisant seulement les pieds serait
impossible. Il justifie de traiter un appui transitoire main ou genou comme
l'hypothèse suivante à étudier, avec un nouveau budget explicite ; aucun tel
essai n'a été lancé dans ce jalon. Les seuils d'anatomie, d'auto-collision,
de couple et de qualification n'ont pas été relâchés.

Les traces locales sous `evidence/foot-coordination-2026-10-12` incluent le
registre fermé, les lignes à 240 Hz, les frontières du solveur, les commandes,
les points de friction et les empreintes. Les
[reçus versionnés](evidence/foot-coordination-2026-10-12/index.json), le
[résumé calculé](evidence/foot-coordination-2026-10-12/analysis.json)
et les rapports natifs accompagnent le commit. Les
[captures](media/foot-coordination-2026-10-12/foot-coordination-milestones.png),
la [courbe des appuis](media/foot-coordination-2026-10-12/foot-coordination-support.png)
et la [vidéo](media/foot-coordination-2026-10-12/foot-coordination-recorded.mp4)
utilisent exclusivement les poses enregistrées. La vidéo dure 5,30 s, lit
106 images à 20 i/s environ quatre fois plus vite que le temps physique,
et fige chaque sujet après son propre arrêt, avec sa légende. Elle ne crée
aucun nouveau pas physique.

Préparation figée à 12:21:57 UTC, avant l'échéance de 19:39 ; simulation
commencée à 12:22:04 et terminée à 12:22:50, dans sa fenêtre de cinq
minutes. Deux réservations de 6 240 pas ont consommé le plafond de 12 480 ;
6 235 pas ont effectivement été calculés. Un seul processus à un thread
par histoire. Les trois anciens registres restent inchangés. Les six tests
Rust ciblés sans intégration, huit tests d'admission, la compilation Release
et Clippy passent. Les essais géométriques antérieurs ne sont pas une preuve
de stabilité physique. La suite complète n'a pas été relancée : un test
d'articulation hybride et quatre tests TypeScript échouaient déjà sur le
témoin Linux.
