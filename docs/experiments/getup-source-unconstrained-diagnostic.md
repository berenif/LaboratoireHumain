# Dix pas avec les seuils d'arrêt en observation

Essai diagnostique demandé le 19 octobre 2026. **Dix avances alternées de pied de 18 cm et cinq secondes calmes finales sont enregistrées.** Cela ne qualifie pas la marche sous les critères précédents : le glissement cumulé dépasse 10 mm et la réserve COM/capture passe brièvement sous 3 mm.

## Seuils d'arrêt désactivés, contrôleur conservé

Le meilleur contrôleur existant reste à 800 Hz natifs, avec ses couples actualisés à 200 Hz. Ses contraintes de planification et ses admissions de phase mesurées restent actives. Les seuils de glissement, contact, collision, articulation et capture signalent les écarts sans arrêter cet essai optionnel.

- **66 687 intégrations**, soit **83,35875 s physiques**
- Dix avances mesurées de **181,022 à 182,014 mm**
- **5,000 s calmes consécutives** selon le critère de base, vérifié indépendamment sur les 4 000 dernières lignes
- Le critère supplémentaire d’alignement du bassin et du tronc relatif à 5° échoue ; il ne fait pas partie du calme de base de ce contrôleur. La posture finale n’est donc pas qualifiée « bassin/tronc droits »
- Glissement chargé cumulé gauche/droite : **16,336 / 12,291 mm**
- Même franchissement de l'ancienne limite à **49,4625 s / 39 570 intégrations**, au septième transfert, puis poursuite debout
- Marge COM/capture minimale : **+2,9514 mm**, positive mais sous la réserve de conception de 3 mm
- Aucun appui non pédestre ni interpénétration mesurée ; inclinaison maximale du tronc **13,06°**

La petite excursion articulaire héritée au départ reste distincte de la marche. Aucun autre seuil d'arrêt d'exécution n'est signalé. Cela ne signifie pas que chaque critère de qualification est satisfait.

Le seuil cumulé de 10 mm interrompait donc cette séquence avant sa fin. Le dépasser ne fait pas disparaître le glissement. La cadence physique du contrôleur reste identique, environ **7,86 pas/minute** entre les fins des pas intermédiaires ; cet essai ne démontre pas une marche plus rapide. Les 18 cm décrivent le déplacement de chaque pied depuis sa propre position initiale. Le déplacement horizontal du COM vaut environ **0,859 m**, et non 1,8 m.

## Interruption de calcul et continuité

Le plafond de 1 200 secondes de calcul a coupé le premier processus à **79,900 s physiques / 63 920 intégrations**, pendant le maintien final, après **1,54125 s calmes**. Le checkpoint natif complet a été conservé.

La continuation importe exactement qpos/qvel et le payload natif, sans remise en pose ni effort de racine. La table des bras retrouve exactement son reçu. L'historique de dérivation des bras, la recherche de contact, la correction d'appui et la cible q0 d'entrée au maintien sont reconstruits depuis les journaux/checkpoints. **Un aller-retour exact de l'objet contrôleur complet n'est pas prouvé.** Les compteurs décoratifs de durée de charge sont reconstruits sur la fin du journal ; ils n'interviennent pas dans les couples ni dans le critère de repos. Les 66 687 compteurs physiques enregistrés restent continus.

La source nettoyée permet un plafond explicite de 120 à 3 600 secondes, à 1 800 par défaut. Le lanceur de reprise conserve et vérifie les preuves antérieures avant de continuer.

## Planification et admissions relâchées

Un second essai retire les inégalités de capture, arrêt prédictif articulaire, distance au sol, auto-évitement, cône de forces virtuelles et déchargement. Les phases suivent leur durée nominale. L'égalité de dynamique et la plage réelle de commande moteur [-1,1] restent dans le QP ; la réserve de planification précédente à 95 % est retirée. Les engrenages, forces moteur, gravité, μ, collisions et limites articulaires du modèle natif restent inchangés.

La première préparation de référence, puis la préparation sans intégration du deuxième cycle, conservent des vérifications héritées dans la version réellement exécutée. Elles n'ont pas arrêté ce résultat. La source nettoyée retire les vérifications résiduelles de changement de cycle ; cette correction ultérieure ne remplace pas le résultat physique enregistré.

- Capture hors support à **7,11625 s**
- Glissement cumulé supérieur à 10 mm à **7,39375 s**
- Deux pieds déchargés à **7,43875 s**
- COM sous 65 cm à **7,7725 s**, appui non pédestre à **7,87375 s**
- Poursuite diagnostique, puis interruption volontaire après la chute devenue claire : dernier échantillon complet à **14,7525 s / 11 802 images natives**

Le premier contact de réception chargé est présent pendant 146,25 ms, mais aucun cycle stable complet n'est qualifié. Les deux cycles de commandes commencés ne sont pas deux pas réussis. Le dernier état enregistré a un COM à 17,83 cm, un tronc incliné à 98,67° et un glissement cumulé de 92,58 / 65,30 cm. Le checkpoint final n'a pas été sauvegardé lors de cette interruption ; d'éventuelles intégrations terminales non journalisées ne sont pas chiffrées.

La vidéo de neuf secondes montre uniquement les premières positions réellement intégrées, à **1×**, 180 images à 20 images/s. Elle montre la chute, sans interpolation ni fin figée ajoutée. La variante modifie à la fois les contraintes d'optimisation et les admissions ; elle n'isole pas la contribution de chacune.

## Portée et vérifications

Les deux variantes utilisent explicitement le modèle sélectionné hanches/genoux avec `impratio=8`, comme le meilleur essai antérieur. Elles ne qualifient pas le modèle GetUp original inchangé. Aucun état physique n'est réinstallé entre les pas. Aucun effort de racine n'est appliqué. Les limites articulaires souples peuvent être franchies dans la chute même si leurs paramètres natifs restent présents.

Quatre tests ciblés et 27 régressions du support prospectif réussissent. Le test natif ciblé résout sans intégrer et vérifie l'identité des octets et du compteur, les commandes finies/bornées et le résidu de dynamique. Les changements ultérieurs de métadonnées, statut de sortie et préparation de cycle sont distingués des sources archivées réellement exécutées. Ce n'est pas une CI complète de l'application.

Le [résumé compact](../evidence/getup-source-unconstrained-2026-10-19/summary.json) conserve les compteurs, placements, qualifications et empreintes. Journaux bruts, checkpoints, bibliothèques, XML, poids et vidéos restent dans les sorties locales ignorées. Aucune modification de main, aucun déploiement et aucune activation par défaut.

## Modification demandée ensuite : arrêt actif à 20 mm

Après cet essai, la limite de glissement cumulé de marche passe explicitement à **20 mm par pied**. L'arrêt redevient actif dans le contrôleur multi-pas normal : **20 mm exactement est autorisé ; la première valeur strictement supérieure arrête la simulation**. Les cumuls ne sont ni remis à zéro entre les pas, ni additionnés entre les deux pieds.

La configuration du contrôleur cadencé et ses validateurs utilisent ce même critère. Les anciens reçus à 10 mm gardent leur propre seuil ; les critères distincts des sondes à un pas et de stabilisation ne sont pas modifiés. Le mode diagnostique sans arrêts reste une option explicitement séparée.

Le journal existant de 66 687 lignes est rejoué sans nouvelle intégration contre le seuil de 20 mm. Quatre tests de borne/historique, dix tests du contrôleur cadencé, quatre tests diagnostiques et trois régressions ciblées de l’audit réussissent ; leurs résultats sont conservés avec les preuves. Ses maxima 16,336 / 12,291 mm sont inférieurs au nouveau seuil. Cette relecture porte sur le seul critère de glissement : **elle ne qualifie pas toute la marche**, dont la réserve COM/capture minimale reste 2,9514 mm pour une cible de 3 mm. La vidéo reste celle de l'essai diagnostique décrit ci-dessus, et non celle d'un nouveau lancement sous le seuil de 20 mm.
