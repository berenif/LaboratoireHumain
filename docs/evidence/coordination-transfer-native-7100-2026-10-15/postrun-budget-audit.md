# Audit indépendant après l’essai A36 à 7 100 secondes

**L’essai unique est clos sur refus de la première prévision, sans relance ni remboursement.** Audit en lecture seule du 2026-10-15T14:26:44.913470+00:00. Aucun nouveau worker, test, build, monde, FK ou sous-pas n’a été exécuté pour cet audit.

Le worker 5506 a commencé sous supervision le **15 octobre à 14:21:07,491954 UTC**. Le registre est fermé à **14:21:17,868896 UTC**, après **10,376937300003192 s** mesurées sur le chronomètre monotone, sous le plafond de 7 100 s. Le PID est absent lors de l’audit ; aucune autre invocation native du candidat n’est détectée. Le claim unique demeure présent et interdit une nouvelle tentative.

| Décompte natif réconcilié | Sous-pas réalisés |
| --- | ---: |
| Préfixe technique | 36 |
| Deux paires copie/réel, quatre pas par branche | 16 |
| Préfixe de l’histoire réelle | 36 |
| Première prévision, 480 demandés | 42 |
| **Total exact** | **130** |

Les journaux indépendants de porte native contiennent 52 admissions et 52 complétions techniques, puis 78 admissions et 78 complétions pour la réserve histoire/prévisions. Deux emplacements de tick abandonnés sont conservés, sans intégration et sans remboursement. Le reçu natif, le registre, les six événements chaînés par empreinte et le résultat du superviseur concordent. Le transcript contient un seul `ready`, une seule commande `transfer-run` et un seul résultat. Les archives des deux paires de parité sont exactement égales entre copie et réel, pour l’entrée, quatre images, le checkpoint final et le périmètre du candidat.

La première prévision avance de l’horloge 36 à **78**, soit **42 pas / 0,175 s**, puis refuse la conservation d’un contact retenu ou le dépassement de son enveloppe de dérive. Le diagnostic exact est conservé dans le JSON. Les champs génériques `measured: 0.0` et `threshold: 0.0` ne constituent pas une mesure physique de dérive. Aucun horizon complet n’est admis. L’histoire réelle reste au préfixe A36 : **zéro pas réel de transfert après A36**, aucun succès local, relevage ou seconde stable démontré. `integrity_ok=false` représente ici le refus du protocole natif ; les empreintes d’archives et la comptabilité passent leur contrôle.

La trace conserve 130 états après intégration et un endpoint répété à l’horloge 78, soit 131 états et trois événements. L’endpoint ne compte pas comme un 131e sous-pas. Une vidéo du préfixe réel et de la prévision peut être tirée de ces états, avec identification explicite des branches ; elle ne montre pas une continuation réelle admise.

Le débit reste **131 096 sous-pas**, dont **130 966 non exécutés mais non remboursés**. Ajouté aux 7 844 historiques, le cumul débité est **138 940 / 300 000**, laissant arithmétiquement **161 060**. Ce solde n’autorise aucun autre essai. Les quatre anciens emplacements d’histoire et le nouveau successeur unique restent consommés et clos. La somme des temps documentés devient **32,328294294003192 s** ; avec la provision historique inchangée de 74 s, le rapprochement prudent vaut **106,328294294003192 s**. Ce sont des comptes après arrêt, sans réouverture d’autorisation ni remise à zéro.

Les **345 fichiers de l’ancien manifeste**, les **358 du nouveau** et les **14 preuves historiques protégées** sont à nouveau vérifiés et inchangés. Le binaire conserve `63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5`. Certains liens `/proc/PID/exe` de services sans rapport restent refusés ; cette limite ne change pas le constat d’absence du PID 5506 et aucune lecture refusée n’est contournée. Le JSON joint donne les empreintes de chaque reçu et les contrôles précis.
