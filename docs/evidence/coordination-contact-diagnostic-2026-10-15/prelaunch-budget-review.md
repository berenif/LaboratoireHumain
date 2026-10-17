# Réservation distincte du diagnostic de contact A36

**Le diagnostic unique de 568 sous-pas maximum et 120 secondes murales entre dans les soldes globaux existants.** Le budget est réservé une seule fois. Le lancement reste conditionné au manifeste neuf, à son binaire, à l'autorisation formelle liée à leurs empreintes et au contrôle final d'absence de doublon.

La [provenance parent](parent-authorization-provenance.json) conserve le texte exact transmis par le parent via `codex_delegation`, thread `01a12799-cefe-7764-8aa3-db5db4c1d0f3`. Il s'agit de la décision explicite du parent rapportée par root, et non d'une citation directe attribuée au propriétaire. Elle autorise un seul diagnostic causal lié à la correction `a5c8ff36af4dfbe784277aebc91021040b947a52`, sans remboursement ni renouvellement ; elle interdit de rouvrir l'essai clos ou d'élargir arbitrairement un seuil pour accepter le point situé à 69,993 mm hors surface.

## Réservation et consommation

| Compteur | Avant | Nouvelle réservation | Après |
|---|---:|---:|---:|
| Sous-pas natifs débités, tous usages | 138 940 | 568 | **139 508 / 300 000** |
| Temps physique prudent engagé | 106,328294294003192 s | 120 s | **226,328294294003192 / 7 200 s** |
| Préparation engagée | 68 089 s | 0 s | **68 089 / 144 000 s** |

Restes non engagés : **160 492 sous-pas**, **6 973,671705705996808 secondes physiques prudentes**, **75 911 secondes de préparation**. Le créneau de préparation déjà débité reste **14:12:53–17:12:53 UTC le 15 octobre**, avec échéance globale inchangée au **17 octobre, 20:14:00 UTC**. Ce bilan ne renouvelle aucun créneau.

Les **568 sous-pas** se ventilent en **52 techniques** (36 de préfixe et 16 de parité), puis **516 pour le diagnostic** (36 de préfixe et au maximum 480 de prévision). Aucun pas réel de transfert au-delà de l'entrée A36 n'est autorisé. Le superviseur peut nommer la seconde allocation `history` dans son schéma ; elle correspond à ces mêmes 516 sous-pas. Les allocations du registre d'exécution sont une ventilation de la réservation parent, **pas un second débit de 568**. Identifiant commun : `contact-diagnostic-distinct-2026-10-15-568-single`.

À l'écriture de cette réservation, le diagnostic a exécuté **zéro sous-pas** et son temps physique mesuré est **zéro**. Les réservations restent débitées après un arrêt précoce. La provision de 120 secondes n'est pas une durée déjà mesurée. Le plafond comprend démarrage, calculs, arrêt et clôture : démarrage au maximum 30 secondes à l'intérieur des 120 ; fin des intégrations au plus tard à départ +113 secondes ; arrêt et clôture au plus tard à départ +120 secondes. Les dates UTC et monotones seront consignées par le superviseur avant lancement.

Les anciennes consommations réellement connues restent distinctes : **2 238 sous-pas connus cumulés**, exactitude globale non certifiée ; borne reconstruite par les archives **2 238–2 239**. Le dernier essai A36 a exécuté exactement **130 sous-pas** et reste `closed-refused`, avec ses **131 096 sous-pas réservés** intégralement débités. Aucun champ historique inconnu n'a été réécrit.

## Vérifications avant gel

Au **2026-10-15T15:29:03.533322+00:00**, les **six registres clos** lus directement donnent **7 844 + 131 096 = 138 940** sous-pas débités. Les **42 pièces protégées** restent identiques, dont les anciens registres, binaires, manifestes, autorisation et preuves du dernier essai clos. Les quatre documents sources du bilan correspondent exactement à leurs objets Git dans la correction de base. Le [rapprochement des durées](wall-source-reconciliation.json) recalcule les fenêtres historiques à partir des registres : **21,951356994 + 10,376937300003192 = 32,328294294003192 s** documentées, auxquelles s'ajoute l'allocation prudente historique inchangée de **74 s**. Ni cette allocation ni la nouvelle provision de 120 s ne sont présentées comme du temps mesuré.

Aucun claim du nouveau diagnostic, aucun nouveau registre physique et aucun processus candidat visible n'étaient présents lors du contrôle. Ce constat est ponctuel et doit être renouvelé après le gel, immédiatement avant lancement. La lecture des métadonnées accessibles de `/proc` ne constitue pas un inventaire privilégié de tous les processus ; aucun accès refusé n'a été contourné.

Le [registre de réservation](budget-reservation.json), SHA256 `be1a9bb8a01485629944a6f0e52419cf6741e44961d89fc0222793b278ad65a1`, et la [revue détaillée](prelaunch-budget-review.json), SHA256 `c3c47516941594c0184ee6ae65757d6644f44ab8e3dfa95eb58e1193dade30d6`, sont des pièces neuves. Aucun ancien registre n'a été modifié. Audit effectué par lectures, calculs arithmétiques et empreintes uniquement : **zéro physique, FK, monde, assemblage, test ou compilation**.
