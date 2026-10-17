# Diagnostic causal A36 — une exécution distincte

Autorisation parent explicite conservée dans `parent-authorization-provenance.json`.
La base physique est `a5c8ff36af4dfbe784277aebc91021040b947a52`. Le seul changement
Rust de cette préparation est le mode diagnostic et ses gardes dans
`transfer_worker.rs`; aucun réglage physique ni politique de transfert ne change.

## Hypothèse et décision attendue

Expliquer la disparition du point matériel gauche au pas 78 : point supprimé ou
renouvelé dans le manifold, point désélectionné, impulsion courante nulle ou rejet
par un prédicat de l'observateur. La géométrie brute conservée dans Rapier peut
précéder la dernière intégration ; la décision privilégie les contacts sélectionnés,
leurs impulsions, leurs ancres courantes et les motifs de filtrage. Une charge
globale positive ou une faible dérive ne suffit pas à maintenir ce point.

Comparer aussi la transition 27–29, les pas 76–78, les deux préfixes et les
branches de parité. Si la dynamique n'atteint pas 78, expliquer l'arrêt observé
sans nouvel essai. Les seuils de charge, distance et dérive restent inchangés.
Le point situé 69,993 mm hors enveloppe ne peut être accepté par élargissement.

## Plafonds et ordre figés

1. Claim durable exclusif et ventilation de l'unique réserve globale de **568**
   sous-pas : 52 techniques, 516 pour le préfixe et la prévision diagnostiques.
2. Rejouer les 36 pas archivés, comparés exactement après exclusion du seul
   champ de télémétrie ajouté et des deux différences de schéma déjà admises.
3. Deux nouvelles paires de parité de quatre pas chacune, soit 16 intégrations
   supplémentaires, copies et branches réelles techniques comprises.
4. Rejouer le préfixe principal de 36 pas, puis **une** prévision de 480 pas au
   maximum, arrêtée au premier refus. Aucun pas réel de transfert après A36.
5. Archiver les observations de support et frontières de contact, y compris
   l'endpoint courant. Fermer sans remboursement, reprise, variante ou renouvellement.

Le plafond de **120 secondes murales** comprend démarrage, calcul et arrêt.
Le démarrage est limité à 30 secondes dans ce même délai. Les permis natifs
expirent au plus tard à début+113 secondes ; sept secondes restent pour l'arrêt.
Tous les sous-pas passent par le journal natif avant/après intégration. Les appels
interrompus restent des intervalles incertains tant qu'un reçu ne les réconcilie pas.

Le garde d'endpoint exige observation et télémétrie complètes, ainsi qu'un
échantillon diagnostique à la même horloge. En cas de refus natif à N, cet
échantillon peut rester à N−1 : un refus secondaire de preuve d'endpoint sera alors
conservé avec le refus original. Les poses, support et télémétrie courants N déjà
archivés ne sont pas réétiquetés N−1.

## Intégrité, autorisation et portée

Le manifeste couvre sources, binaire dédié, inputs inchangés, protocole,
réservation et preuves de tests. Les 42 pièces historiques sont figées dans ce
nouveau manifeste sans être modifiées. Son empreinte lie l'autorisation à usage
unique. Un audit indépendant et un contrôle d'absence de claim/processus
précèdent le lancement par le responsable uniquement.

Les premières compilations restent archivées mais ne qualifient pas le candidat
final. En particulier `release-final-receipt.json` a des empreintes source
avant/après différentes et est inadmissible. Seul `release-admitted-receipt.json`
avec les sources stables `811df09b…` qualifie le build dédié ; celui-ci n'a pas
été exécuté lors de sa construction. Douze tests purs du worker et 54 tests
purs/inertes du superviseur passent. La suite globale n'est pas revendiquée.

Une fermeture sur refus ne vaut ni perte totale du pied, ni impossibilité
anatomique. Aucun résultat diagnostique ne vaut acquisition des talons,
transfert réussi ou relevage. La continuation complète reste obligatoire en
au plus 25 secondes après impact, suivie d'une seconde de stabilité réelle puis
de la qualification distincte des 24 poses/cycles/impacts. La récupération reste
désactivée. Toute correction après résultat sera uniquement statique dans la
fenêtre de préparation existante, sans autre calcul physique autorisé ici.
