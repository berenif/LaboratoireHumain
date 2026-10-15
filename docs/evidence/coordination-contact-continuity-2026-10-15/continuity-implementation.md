# Continuité matérielle : certificat statique conservateur

Cette modification ne relance pas le protocole A36 clos. Elle ajoute une preuve géométrique recalculable pour les tâches `Hold` des segments `Hindfoot` et `Forefoot`. Elle ne fabrique pas une force ponctuelle à l’ancien point, ne remplace pas les règles de dérive, d’appui ou de budget et ne qualifie aucun mouvement.

## Interface

`MaterialPatch.current_contact_present` reste le diagnostic original : au moins un contact actuellement chargé est à la distance locale admise du point matériel acquis. La règle directe, son seuil de normale et la tolérance fournie par l’appelant restent inchangés.

`MaterialPatch.reference_surface: Option<AcquiredMaterialSurface>` est fixé uniquement par `MaterialTracker::acquire`. Il conserve le point local, l’ancre monde, le sous-pas d’acquisition, le plan matériel local orienté et les trois indices témoins de l’acquisition. Il n’est jamais reconstruit par `observe`. Les anciens JSON sans ce champ donnent `None` et n’obtiennent aucun repli surfacique.

La méthode pure `material_continuity(observation, tolerance)` retourne `DirectLoadedPoint`, `EquivalentLoadedSurface` ou `Refused`, avec le véritable plus proche contact chargé, ses indices, les sommets témoins, la charge courante, les exclusions et la raison d’un refus. `verify_current_support` recalcule cette preuve et retourne son résultat booléen ; cette méthode est la frontière à employer par les consommateurs disposant de l’observation.

`observe` remplit le cache diagnostique `continuity_evidence`, sans modifier le point acquis ou son ancre. Ce cache est sérialisable mais exclu de la désérialisation : un JSON ne peut pas importer un certificat. `has_current_support` est réservé aux consommateurs internes sans observation, après validation : point direct présent, ou cache `EquivalentLoadedSurface` avec mode courant `Hold`, le même identifiant de patch, le même segment et le même sous-pas observé. Un cache `DirectLoadedPoint` ne peut pas contourner un `current_contact_present=false`. Une observation nouvelle doit être revalidée ; un cache ancien n’est jamais une permission native.

## Conditions du certificat surfacique

À l’acquisition, il faut un triangle non colinéaire de contacts observés, cohérents en coordonnées locales et monde, comprenant le point sélectionné. Tous les sommets actuellement chargés à au moins 3 N doivent être coplanaires. Le plan est orienté vers le haut dans la pose acquise. Aucun plan n’est créé pour `Pivot`, `Unload`, `Free` ou un segment autre qu’un pied.

En l’absence de correspondance directe, le certificat exige :

- la même référence matérielle et le même plan de sol qu’à l’acquisition ;
- la normale acquise, transformée par la pose courante, encore orientée vers le haut selon le seuil existant `MIN_NORMAL_Y` ;
- le point matériel original à au plus 12 mm du plan de sol ;
- au moins trois contacts non colinéaires du même segment et du plan matériel acquis, chacun actuellement chargé à **au moins 3 N** ;
- des normales de contact de sol plat, `upward_normal >= 1 - 1e-6`, et une cohérence local/monde à `1e-6 m` ;
- le point original inclus dans un triangle de ces contacts, donc dans leur enveloppe convexe, avec seulement `1e-6 m` de tolérance numérique sur le plan et les arêtes.

Les 12 mm de correspondance ou de proximité du sol ne dilatent jamais l’enveloppe chargée. Le seuil de 3 N par sommet est un choix conservateur de ce certificat, pas une loi universelle de contact. Il impose au moins 9 N répartis sur trois sommets ; une charge persistante de segment ou un sommet presque déchargé ne suffit pas.

La preuve géométrique peut rester vraie pendant un glissement. `observe` continue donc de mesurer la dérive du **même** point depuis la **même** ancre. Le garde de 10 mm existant doit rester appliqué séparément par l’adaptateur. Le test synthétique de dérive à 11 mm vérifie que cette mesure n’est pas remise à zéro.

## Régression sur l’archive 36 → 78

`continuity-extract-archive.py` lit uniquement `native-trace.jsonl.gz` du lot clos et extrait deux observations déjà calculées dans `continuity-archive-36-78.json`. Il n’instancie aucun modèle, scène ou monde et n’appelle ni FK ni worker. L’empreinte de l’archive source est inscrite dans le fichier extrait ; un fichier extrait différent existant n’est jamais écrasé.

Au sous-pas prédit 78, le contact acquis du pied avant gauche est absent. Le plus proche contact encore chargé est à **90,000025928 mm** en coordonnées locales. Le sommet faible porte **0,989888184 N** : seulement deux sommets atteignent 3 N. Le certificat réel est donc refusé pour nombre insuffisant de sommets chargés admissibles.

Le point acquis est aussi situé à **69,992784709 mm** de l’arête opposée de l’ancien triangle, hors de celui-ci. Un test séparé remplace les trois charges par 5 N dans une copie synthétique de l’observation, exclusivement pour isoler cette condition géométrique : le refus devient `OriginalPointOutsideLoadedHull`. Cette copie ne décrit aucun nouvel état physique calculé et ne change pas l’archive.

## Vérification ciblée

Vingt et un tests purs sont préparés dans `material_continuity_tests.rs`. Ils couvrent un point intérieur entouré de trois nouveaux points, les archives et données anciennes, le cache, les modes et rôles, le plan, les normales, les charges, la colinéarité, la frontière de l’enveloppe, le point levé, les coordonnées incohérentes, le retournement et les ancres/dérives.

Filtre pour la compilation et l’exécution coordonnées par le responsable :

```sh
CARGO_HOME=/workspace/lh-cargo RUSTUP_HOME=/workspace/lh-rustup \
/workspace/lh-cargo/bin/cargo +1.89.0 test --manifest-path rust/Cargo.toml \
-p lh-sim --lib --locked support_state::material_continuity::tests:: -- --test-threads=1
```

Ce sous-travail prépare les tests sans lancer Cargo de manière concurrente. Le journal central de vérification détermine leur résultat effectif. L’extraction JSON et l’écriture de code ont utilisé **zéro sous-pas natif, zéro monde et zéro FK**. La télémétrie optionnelle `native_contact_boundary` est explicitement diagnostique et n’intervient pas dans le certificat.
