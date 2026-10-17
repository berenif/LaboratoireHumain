# Diagnostic de contact A36 — 568 sous-pas, 120 secondes

Ce superviseur prépare une seule opération native `contact-diagnostic`, explicitement distincte du lot A36 clos. Sa base physique et ses règles de contact sont celles du commit `a5c8ff36af4dfbe784277aebc91021040b947a52`. Le correctif du worker ajoute le bornage diagnostique, une compatibilité strictement additive de télémétrie pour le préfixe archivé et l’obligation d’un endpoint documenté. Il ne change ni politique, ni seuil, ni référence articulaire, ni anatomie, ni friction, ni contrat de couple.

Le responsable crée le manifeste, l’autorisation parent liée à ce manifeste et le gel final. Il vérifie le solde global, les pièces historiques et l’absence d’une opération déjà lancée. **Ce module et ses tests ne lancent aucun worker physique et ne créent aucune autorisation réelle.** L’import et le mode par défaut restent une inspection sans claim.

## Opération et limites exactes

- 52 sous-pas techniques : préfixe de 36, puis deux comparaisons indépendantes de quatre pas copiés et quatre pas réels.
- 516 sous-pas de diagnostic : second préfixe de 36 puis une seule prévision de 480 sous-pas au maximum.
- **568 sous-pas tous usages au maximum**, débités sans remboursement. Les deux réservations techniques ventilent l’unique débit global de 568 ; elles ne créent pas un second débit global.
- **120 secondes murales** pour le lot lancé : démarrage inclus, au plus 30 secondes ; expiration des permis natifs à 113 secondes ; sept secondes réservées à l’arrêt et à la fermeture, incluses dans les 120.
- Zéro continuation réelle après le second préfixe, zéro deuxième prévision, zéro variante, zéro renouvellement et zéro relance.

Le mode `contact-diagnostic` impose aussi ses limites dans le worker : permis exacts 52 et 516, durée native restante au plus 113 secondes. À la fin de l’unique prévision, il retourne **avant** certification, admission ou consommation d’une fenêtre de transfert réel. Le mode historique `transfer-run` demeure distinct et exige ses anciens montants ; des permis de 568 ne permettent pas de l’employer.

Un refus de parité ou de prévision ferme immédiatement le lot. Les sorties conservent les compteurs exacts ou leurs bornes en cas d’interruption. Une opération envoyée dont le reçu manque ne redevient jamais « zéro pas ». Aucun emplacement inutilisé n’est remboursé.

## Préfixe et endpoint

Le préfixe est comparé bit à bit, nombres signés et mémoire de commande compris. Les seules projections de compatibilité sont la vieille étiquette `kind`, l’absence ou nullité de `owner.transfer`, et le nouveau champ diagnostique exactement situé en `support.native_contact_boundary`. Ce dernier doit être présent, typé, en schéma 1, à l’horloge native courante, avec les 25 segments ordonnés dans chaque nouvelle observation comparée. Aucun champ de `MaterialPatch`, charge, contact, couple, q/dq, horloge ou mémoire historique n’est supprimé.

Chaque sous-pas prédit archive son `support` complet. L’endpoint archive aussi le support de l’état natif atteint, y compris si le contrôleur refuse. Pour déclarer le diagnostic complet, son observation, sa télémétrie et `LocalSupportSample` doivent porter exactement le sous-pas de cet endpoint. Si un refus du contrôleur laisse le diagnostic local au pas précédent, le support et la télémétrie courants restent archivés ; le reçu conserve le refus initial de la prévision ainsi que la raison de refus de l’endpoint. Une mémoire locale ancienne ne devient pas une observation courante.

Les identifiants de manifolds et de points sont des indices locaux à l’observation. Les points étroits et leurs impulsions non sélectionnées peuvent provenir d’un cache ; ils ne sont pas des identités matérielles ni des appuis certifiés. La sélection courante, les impulsions et les exclusions doivent être interprétées avec les poses et les distances archivées.

## Interface de gel

Le module `run_diagnostic.py` expose :

- `PROTOCOL = contact-diagnostic-a36-568-120s-2026-10-15` ;
- `MANIFEST_SCHEMA = laboratoirehumain.contact-diagnostic-manifest.v1` ;
- `AUTHORIZATION_SCHEMA = laboratoirehumain.contact-diagnostic-authorization.v1` ;
- identifiant d’autorisation préfixé par `contact-diagnostic-distinct-` ;
- `LIMITS`, `PREREQUISITES`, `ROLES` et `required_code_paths()` comme contrat exact du gel.

Le manifeste conserve les clés `schema`, `protocolId`, `authorizationId`, `binaryPath`, `outputPath`, `inputs`, `files`, `prerequisites`. `outputPath` est un nouveau sous-répertoire direct de ce dossier. Une autorisation approuvée doit lier le SHA-256 exact du manifeste et les limites exactes. Un claim exclusif empêche tout second lancement, même sous un autre nom de sortie.

Les anciennes pièces sont vérifiées à leur commit de gel approprié ; leurs anciens manifestes ne sont jamais réécrits pour les faire correspondre aux sources corrigées. Le nouveau manifeste fige les sources corrigées réelles, le worker diagnostique final, son binaire distinct, les validations et les mêmes entrées physiques. Les 14 pièces historiques référencées par la préparation originale restent également vérifiées par le superviseur.

Construction séparée, sans exécution, à effectuer par le responsable après revue et stabilisation des sources :

```sh
CARGO_HOME=/workspace/lh-cargo RUSTUP_HOME=/workspace/lh-rustup \
CARGO_TARGET_DIR=/workspace/lh-contact-diagnostic-target \
/workspace/lh-cargo/bin/cargo +1.89.0 test --manifest-path rust/Cargo.toml \
-p lh-sim --test coordination-transfer-worker --features coordination-transfer-worker \
--release --locked --no-run --message-format=json
```

L’ancien binaire reste protégé dans son répertoire d’origine. La commande ci-dessus ne constitue pas une exécution physique.

Inspection seule, une fois le nouveau manifeste créé :

```sh
python3 docs/evidence/coordination-contact-diagnostic-2026-10-15/run_diagnostic.py \
  --manifest /chemin/absolu/manifest.json
```

Le responsable seul ajoute `--authorization /chemin/absolu/authorization.json --execute` après gel, audit et vérification de l’absence de doublon. Le superviseur ne génère aucune autorisation de lui-même.

## Vérification

`supervisor-tests-final.log` rapporte **54 tests réussis** : 51 tests purs avec worker mémoire et trois tests utilisant de minuscules scripts Python temporaires inertes. Les tests couvrent les échéances, le PID, les réservations, le claim exclusif, les interruptions et comptes incertains, l’intégrité, le refus d’une deuxième prévision ou d’un pas réel supplémentaire, et la séparation des résultats de diagnostic et de relevage. Aucun binaire du projet, monde, FK ou sous-pas natif n’est exécuté par cette suite.

Le résultat natif a quatre critères : `integrity_ok`, `technical_parity_passed`, `diagnostic_completed`, `no_real_transfer`. Un diagnostic complet ne vaut ni transfert local réussi, ni relevage, ni qualification. Le résultat conserve donc l’absence d’autorisation de transfert réel et l’absence de qualification des 24 cas.
