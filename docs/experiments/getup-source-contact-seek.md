# Réception chargée du pas de 24 cm

Correction du 18 octobre 2026, dans le prototype natif optionnel. La base
sélectionnée et les réglages de l'application restent inchangés.

## Résultat qualifié, sur le code final

Un pas gauche demandé à 240 mm réalise **241,926 mm**, acquiert un appui réel,
puis tient **5,000 s calmes**. Les 15 476 intégrations représentent 19,345 s
physiques. L'audit indépendant valide le mouvement, les bras et la cohérence FK.

- Réception qualifiée à 10,34375 s, après 100 ms d'appuis mesurés :
  pied mobile 21,905 N, pied porteur 379,210 N ; avance 241,575 mm
- Capture minimale : +8,560 mm
- Réserve articulaire minimale pendant la marche : +0,31789°
- Effort moteur maximal : 94,0497 %, sous le plafond inchangé de 95 %
- Glissement chargé gauche/droite : 2,613 /3,167 mm
- Rapport amplitude réelle bras/cuisse : 0,99058 /1,00238 ; erreurs maximales
  de suivi 0,138° /0,172°, dans les tolérances antérieures
- Aucun vol, appui non pédestre, avertissement ou interpénétration propre

Ce résultat reste un pas segmenté, avec une forte adaptation du bassin. En
marche, l'inclinaison du bassin atteint environ 32,16°. L'inclinaison du torse
est surtout latérale ; ce n'est pas encore la marche humaine à buste légèrement
penché vers l'avant demandée ensuite. Aucun multistep supplémentaire n'a été
lancé après cette nouvelle direction.

Le dépassement de cheville hérité à l'entrée persiste jusqu'à 63,75 ms. La
validité stricte de toutes les articulations dès le premier échantillon reste
fausse. Après cette entrée, chaque échantillon est contrôlé, sans nouveau
franchissement articulaire ni capture négative.

## Cause et correction

Le précédent essai de 24 cm terminait sa descente avec 0 N sur le pied mobile,
malgré une garde au sol de seulement +0,091 mm. La position réelle du pied
restait environ 1,114 mm au-dessus de sa cible finale : une petite erreur de
suivi dans le compromis du QP suffisait à empêcher le contact. Allonger la
phase seule ne corrige pas cette erreur statique.

La correction cherche le contact par une petite descente supplémentaire de la
cible, après la descente nominale et seulement à proximité du sol avec l'autre
pied réellement chargé. La recherche freine quand l'effort mesuré atteint
12 N. Sa profondeur est bornée à 3 mm, sa vitesse à 3 mm/s et son accélération
à 0,03 m/s². Le rejeu final n'utilise que **0,330 mm** de correction. La limite
de profondeur est précédée d'un freinage discret ; les appels répétés au même
temps natif conservent le même jet de cible. Le freinage continue sans saut
lors du passage en stabilisation.

Ce sont des références de mouvement du pied. Seuls les couples articulaires
du QP sont appliqués. Les charges de réception proviennent du moteur physique,
avec les rayons de sol et les contacts d'origine. Les seuils d'appui à 3 N,
l'échéance de réception, moteurs, frottements, limites, collisions, capture et
glissement ne sont pas assouplis. Aucune force de racine, installation de pose,
réinitialisation entre pas ou modification du modèle n'est ajoutée.

## Posture : diagnostic conservé, travail suspendu

Un diagnostic séparé identifie une cheville insuffisamment suivie pendant le
transfert vertical antérieur. Une tâche de cheville-y limitée au transfert
atteint l'admission existante à 4,13625 s avec des réserves de 3,150° /8,159°.
Cette petite tâche reste dans l'expérience optionnelle ; **elle ne qualifie
aucun pas vertical complet**. Les essais de posture ont été arrêtés à la
nouvelle demande de plan de marche avec flexion avant du buste. Une référence
statique couplée aux bras a aussi été étudiée, sans modification du planificateur.

## Vérification et reproduction

Les huit modules ciblés exécutent 63 cas : 61 réussissent et deux sont ignorés
car leur ancien helper v6 est absent. Ces deux cas réussissent séparément sur
le helper v9 retenu ; cela ne valide pas le binaire historique absent. Les sept
nouveaux tests couvrent bornes, freinage, répétitions temporelles et tâche de
cheville isolée. La parité exacte du contrôleur non étendu est vérifiée dans
les six phases, avec et sans évitement propre. Un premier échec de test dû à
un compteur historique fixé à 6400 est conservé : le cache froid restauré a un
compteur 0, et le test vérifie désormais l'invariance du compteur capturé avant
le QP. Aucun seuil physique n'a été changé. La CI complète n'est pas déclarée
réussie.

Après restauration des assets locaux épinglés, depuis la racine :

```sh
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv/mvp/bin/python scripts/getup-step/source_upright_large_steps.py \
  --output NEW_EMPTY_DIRECTORY --distances .24 --reference
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv/mvp/bin/python scripts/getup-step/audit_source_upright_large_steps.py \
  NEW_EMPTY_DIRECTORY/run/generation-001 --output AUDIT.json
```

Les preuves compactes sont dans
[ce dossier](../evidence/getup-source-contact-seek-2026-10-18/manifest.json).
Le manifeste conserve les essais arrêtés, les diagnostics non qualifiés et les
empreintes. Les journaux bruts, XML, bibliothèques et caches restent locaux.
Le code n'a pas changé pendant le rejeu final. Aucun changement sur main,
promotion de réglage ou déploiement n'est inclus.
