# Comparaison de coordination tibia cheville et avant pied

Le propriétaire autorise le seul jalon de commande du pied le 12 octobre 2026
à 11:39 UTC, après le diagnostic `0a0db45`. Ce diagnostic est poussé et son SHA
distant vérifié. Friction, géométries, anatomie, forces, gains moteurs, contacts,
caps et critères physiques restent ceux des essais A/B. Aucun nouveau segment,
coefficient de friction, verrouillage ou assistance au bassin.

## Hypothèse figée avant exécution

À 50 ms le tibia mesuré ne permet pas de poser le talon par une simple correction
locale de cheville : celle-ci est presque à 20°. Une commande simultanée des
hanches, genoux, chevilles et avant-pieds peut faire évoluer la chaîne entière
autour de l'appui réel. Cette hypothèse porte sur un chemin physiquement soutenu,
pas sur un placement de poses ou un gain de score articulaire.

Le témoin reprend **B du jalon précédent**, programme et retour d'état identiques.
Le candidat C conserve le programme nominal et son calendrier et ajoute un seul
ensemble de tâches coordonnées sur les huit axes sagittaux de jambe :

- inclinaison sagittale arrière-pied et avant-pied vers zéro ;
- inclinaison du tibia dans la plage compatible avec la cheville existante ;
- hauteur de la ligne arrière de semelle vers le vrai sol, autour des points
  porteurs de la même chaîne réellement présents.

Ces tâches ne s'activent que dans les fenêtres d'intention pied du programme
et avec un appui de pied persistant réel. Le candidat maintient dans sa commande
les contacts présents qui sont aussi prévus par le programme ; les contacts
passifs tête/tronc restent résolus par Rapier mais ne sont pas artificiellement
maintenus par la commande. Une intention absente ne crée aucun contact.
La dernière intention est retenue pendant le terminal, sans remettre l'horloge
globale ou celle de stagnation à zéro.

Les Jacobiennes sont calculées depuis les poses et chart quaternion natives.
Elles produisent des cibles articulaires ; la rotation libre de la racine reste
une inconnue dynamique, sans commande de racine ni certificat inverse-dynamique.
La récupération physique est décidée indépendamment par le vérificateur original.

Paramètres déterminés avant les deux histoires : gain angulaire dérivé du minimum
des rapports raideur/amortissement effectifs cheville/avant-pied (7,5/s dans le
profil), vitesse de tâche limitée à 0,25 rad/s, poids angulaires `4 × demi-longueur²`
des segments existants, descente du talon limitée à `0,25 × bras horizontal talon/appui`
m/s. Les limites de cible originales de 1 rad/s et d'anatomie et l'offset expérimental
de 0,12 rad sont conservés. C anticipe leurs intersections dans la résolution ;
une intersection vide est un refus explicite, sans clipping cachant une impossibilité.
La préférence de correction reste 0,25 rad/s. Si la variation nominale arrive à
une butée, la projection annule uniquement la part nécessaire de cette variation ;
elle peut dépasser cette préférence interne, tout en respectant la vitesse totale
originale et la cible anatomique. Cette propriété est vérifiée sans intégration.

## Prérequis et mesures

Un seul producteur installe les moteurs. Dès le handoff, la propriété revient
définitivement au contrôle natif. Settle, handoff 143/144, maintien et interruptions
gardent le comportement original ; phase, talon et score ne peuvent attribuer
une récupération. Le délai global de 25 s, perte de support et stagnation de 3 s
restent continus. Aucune réinitialisation ou copie de monde pendant une histoire.

Le journal en lecture seule est complété aux frontières déjà établies du solveur :
μ effectif, impulsion normale/tangente, point et bras natifs figés, vitesse de point
avant et après résolution pour chaque point sélectionné. Ce sont des frontières
de résolution, pas un suivi certifié du même point matériel à travers les changements
de manifold. La demande moteur avant le pas, le cap et les impulsions réellement
livrées sont enregistrés à 240 Hz. Le signe de ligne native reste distinct du
couple généralisé sur l'enfant. Positions arrière du talon, contacts persistants,
acquisition/décharge, glissement, CoM et intégrité complètent le verdict.

La première histoire doit reproduire les 5 013 pas du témoin B archivé et ses
hashes physiques. Un défaut de neutralité ferme le jalon avant C. Les tests
préparatoires sont sans intégration ; aucune simulation de vérification cachée.
P1 reste non certifiée et la qualification générale n'est pas acquise.

La préparation a compilé le binaire Release avec l'observation du solveur et
passé Clippy sur les cibles Rust avec avertissements bloquants. Cinq tests
algébriques ciblés du propriétaire, le test original de handoff et huit tests
d'admission du registre ont passé sans intégration. Les tests algébriques
utilisent notamment un point de contact synthétique : ils vérifient des signes
et des bornes de commande, et ne prouvent aucun appui résolu. Les journaux sont
dans le nouveau répertoire `evidence/foot-coordination-2026-10-12`.

## Budget et arrêt

Le registre distinct est `evidence/foot-coordination-2026-10-12/ledger.json`.
Préparation : **8 heures au maximum depuis l'accord de 11:39 UTC**, donc arrêt
au plus tard à 19:39 UTC si les prérequis ne sont pas prêts. Les anciens registres
restent fermés, vérifiés par SHA et non réutilisés.

**Deux histoires au plus, chacune ≤26 s, 12 480 pas natifs réservés au total,
un processus mono-thread, cinq minutes murales depuis le premier lancement.**
Toute intégration d'un test ou d'une copie consomme ce plafond. B puis C, avec
le même binaire, source et paramètres figés. Aucune retouche entre les histoires.
Pas de troisième histoire ou de prolongation pour un talon posé ou une chute
retardée. Un crash/timeout reste débité et bloque la reprise automatique.

Un succès exige le compteur original avant 25 s, handoff, maintien original puis
la seconde d'observation supplémentaire, même monde et mêmes handles. Un talon
chargé et un sous-jalon sont des diagnostics uniquement. Après le deuxième essai,
le jalon est fermé avec résultat physique, preuves et vidéo des états archivés,
quel que soit le résultat. Aucun rendu ne relance la physique.
