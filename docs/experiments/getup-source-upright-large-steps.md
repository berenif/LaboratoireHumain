# Grandes amplitudes et posture plus verticale

Essai du 18 octobre 2026. La base sélectionnée et l'application restent
inchangées. Les nouvelles commandes sont expérimentales et optionnelles.

## Suite : réception de 24 cm

Une correction ultérieure de recherche de contact a qualifié un pas unique de
241,926 mm, avec réception chargée et cinq secondes calmes. Le
[rapport de correction](getup-source-contact-seek.md) conserve ses preuves et
ses limites. Les résultats ci-dessous décrivent les essais antérieurs. La
posture verticale ne constitue plus la direction de travail demandée.

## Conclusion

- Le témoin à bras opposés réalise **182,112 mm**, puis cinq secondes calmes.
  L'audit indépendant valide mouvement, limites après l'entrée, appuis,
  capture et suivi des bras. Ce témoin fléchit cependant fortement le bassin.
  Un rejeu avec le code final et les bornes FK maximales vérifiées confirme
  ce résultat, avec cinq secondes calmes et les mêmes marges arrondies.
- Le test de 240 mm atteint **241,601 mm de translation**, mais n'acquiert
  aucun appui du pied mobile avant l'échéance de réception. Il échoue.
- Le prototype plus vertical a terminé un mouvement de 122,160 mm et cinq
  secondes calmes, mais son audit révèle une sortie du support de capture,
  une nouvelle excursion articulaire et un suivi des bras insuffisant.
  **Ce mouvement n'est pas validé.**
- La version finale conservatrice s'arrête avant de lever le pied : la
  réserve mesurée de cheville reste insuffisante dans le temps de transfert
  admis. Aucun grand pas en posture verticale n'est actuellement qualifié.

Ces résultats établissent les limites du contrôleur et de ses échéances dans
ce scénario, pas une impossibilité physique ou une amplitude anatomique maximale.

## Posture et jeu autorisé

L'objectif statique favorise l'axe vertical du torse et du bassin, tout en
conservant leurs orientations axiales. Un balayage fini de cinq poids compare
inclinaison et effort réservé. Le poids 1 conserve une référence motorisée
admissible, avec environ 10° de torse et 3° de bassin. Cette inclinaison sert de
jeu d'équilibre ; elle n'est pas une installation de pose.

Le QP suit ces références avec une bande souple de 1,5°. Les deux corps ont
une enveloppe d'inclinaison de 12°, vérifiée aussi sur chaque échantillon réel
à 800 Hz. Une barrière prédictive dans le QP ne remplace pas cette vérification.
En fin de mouvement, la cible revient vers la verticale. Les cinq secondes
calmes exigent en plus un bassin et une flexion relative torse/bassin ≤5°.
Tangage, roulis et inclinaison totale sont mesurés séparément dans le monde ;
la flexion relative du tronc est aussi publiée.

## Résultats mesurés

| Essai | Témoin 180 mm | Vertical 120 mm diagnostic |
|---|---:|---:|
| Avance finale | 182,112 mm | 122,160 mm |
| Échantillons physiques | 15 312 | 15 314 |
| Fin calme | 5,000 s | 5,000 s |
| Capture minimale | +11,074 mm | −63,623 mm |
| Réserve articulaire pendant la marche | +0,3179° | −0,1671° |
| Glissement chargé gauche/droite | 2,395 /2,487 mm | 3,228 /4,568 mm |
| Effort moteur maximal | 93,696 % | 95,000 % |
| Inclinaison maximale torse/bassin en marche | 9,235° /32,163° | 9,399° /5,507° |
| Flexion relative maximale en marche | 31,005° | 7,308° |
| Suivi des bras dans les tolérances antérieures | oui | non |
| Qualification indépendante du mouvement | oui | non |

Dans le diagnostic vertical, le suivi maximal gauche atteint 0,661° au lieu
des 0,5° admis ; le bras droit, dont l'excursion est petite, dépasse aussi la
tolérance de rapport d'amplitude. Ces seuils ne sont pas élargis.

Le témoin 240 mm s'arrête à 12,13875 s /9 711 intégrations : pied mobile à
0 N, pied porteur à 400,68 N, garde au sol mobile +0,091 mm. La proximité du
sol n'est pas une réception chargée. L'audit conserve +8,560 mm de capture,
+0,3179° de réserve articulaire pendant la marche, et aucun nouveau
dépassement. Une séquence continue 120/180/240 mm a
également acquis les deux premières réceptions, mais pas la troisième ; elle
n'est pas une séquence complète avec cinq secondes finales qualifiées.

## Cause et correction conservatrice

Le diagnostic vertical sort du support de capture à 9,2475 s, pendant la
descente et avant le premier contact du pied mobile : ce pied porte 0 N et
reste 14,91 mm au-dessus du sol. Le polygone courant est défini par quatre
points d'appui effectivement chargés. La cheville droite ne conserve alors
que 0,666° de réserve. Son dépassement ultérieur atteint −0,167° à 9,8525 s.
La marge de capture minimale survient à 9,8875 s, avec les deux pieds chargés
à 142/250 N ; ce n'est pas un polygone vide ou un appui supposé.

La correction finale impose une réserve cible de cheville-y de 0,06 rad et
une réserve réellement mesurée de 0,035 rad avant décharge. La référence
statique reste admissible à 91,013 % d'effort minimal, sous le seuil inchangé
de 92,5 %. Le transfert réel termine à seulement 1,304° de réserve droite,
sous les 2,005° demandés : arrêt à 6,0025 s, avant levée. Ce blocage est une
admission/échéance du contrôleur ; la faisabilité statique ne prouve pas que
le transfert dynamique l'atteint.

La version finale arrête aussi immédiatement un nouveau dépassement
articulaire après les 63,75 ms héritées, ou une capture courante indéfinie ou
négative. Les anciens journaux restent explicitement disqualifiés.

## Amplitudes des bras et intégrité physique

La table initiale ±0,45 rad était trop courte. L'inversion FK vérifie une
extension asymétrique, sans écrêtage, à réserve articulaire originale de
0,02 rad : arrière droit −0,620325 et gauche −0,529279 rad, avant +0,80 rad.
Les bornes arrière sont à 10⁻⁵ rad à l'intérieur des frontières déterminées
par bissection. Les articulations et les collisions conservent leurs limites.
La cible sagittale reste exactement l'opposé du changement réellement mesuré
de la cuisse du même côté, avec gain −1.

Seule l'enveloppe expérimentale de demande/reach est portée à 302/300 mm.
Cela n'élargit ni les limites articulaires, ni les moteurs à 95 %, ni le
glissement global à 10 mm, ni les conditions d'appui/capture/collision.
Le moteur original MuJoCo 2.1.3 intègre à 800 Hz ; le QP de couple agit à
200 Hz. Tous les rayons restent actifs. Aucune force ou correction de pose de racine après initialisation,
augmentation de puissance, modification de frottement ou réinitialisation
entre les pas d'une séquence n'est utilisée.
La capture instantanée reste le proxy LIP existant, pas un certificat de
viabilité dynamique.

Le dépassement initial de cheville hérité reste présent jusqu'à 63,75 ms.
Le critère strict « toutes les articulations valides dès le premier
échantillon » demeure faux, y compris pour le témoin qualifié après l'entrée.
Il n'y a aucune revendication de robustesse aux perturbations ou d'économie
énergétique.

## Reproduction et preuves

Depuis la racine, installer les dépendances légères épinglées dans
scripts/getup-step/requirements.txt et scripts/requirements-balance-mvp.txt.
Puis, avec OPENBLAS_NUM_THREADS=1 et PYTHONDONTWRITEBYTECODE=1 :

```sh
.venv/mvp/bin/python scripts/getup-step/restore_source_step_assets.py
.venv/mvp/bin/python scripts/getup-step/source_upright_large_steps.py \
  --output NEW_EMPTY_DIRECTORY --distances .18 --reference
.venv/mvp/bin/python scripts/getup-step/source_upright_large_steps.py \
  --output ANOTHER_EMPTY_DIRECTORY --distances .12
.venv/mvp/bin/python scripts/getup-step/audit_source_upright_large_steps.py \
  NEW_EMPTY_DIRECTORY/run/generation-001 --output AUDIT.json
```

La restauration est liée aux empreintes originales du moteur, du XML, du
helper et de l'état q/qvel. Le rejeu témoin 30 mm retrouve exactement les
journaux physiques et les commandes historiques. Les assets, bibliothèques,
caches et journaux bruts restent locaux, hors commit.

Les preuves compactes sont dans
docs/evidence/getup-source-upright-large-2026-10-18. Le manifeste conserve
tous les essais, y compris ceux arrêtés ou interrompus ; aucun total exact
d'intégrations n'est revendiqué pour les processus interrompus. Les contrôles
ciblés comprennent 29 nouveaux tests réussis. Le contrôle de sept modules
exécute 63 cas : 52 passent, 11 sont ignorés faute d'anciens helpers v5/v6.
Ces 11 cas passent séparément avec le helper v9 retenu ; cela ne valide pas
les anciens binaires absents. Les sources restent inchangées pendant le
contrôle final. Un premier échec de construction de grille dans le test
(arrondi de l'extrémité flottante) est conservé, avec sa correction utilisant
les extrémités exactes ; aucun seuil physique ni écrêtage n'a été ajouté.
La découverte large nécessitant Torch n'a pas été relancée et n'est pas
déclarée réussie. Aucun changement sur main ni déploiement.
