# Audit final du budget — 12 octobre 2026

**Le lot est clos, avec 1 024 sous-pas intégrés documentés et 3 856 sous-pas définitivement réservés.** Aucun plafond n’a été atteint. L’arrêt provient du refus des quatre prévisions de B. Cet audit lit seulement les preuves archivées : aucune simulation, FK, inspection de monde ni modification du registre.

| Opération | Réservés | Réalisés inscrits au registre | Réalisés dans les rapports | Statut au registre |
|---|---:|---:|---:|---|
| Vérification A/B, copies et originaux | 16 | 16 | 16 | `completed` |
| Histoire A | 960 | 960 | 960 | `completed` |
| Histoire B réelle | 960 | Non renseigné | 0 | `reserved`, dans un protocole clos |
| Premier appel de planification B | 1 920 | 48 | 4 × 12 = 48 | `failed-reported` |
| **Total** | **3 856** | **1 024 connus** | **1 024** | **`closed-native-failure`** |

Le [rapport B final](native-run/002-B.json), produit après le résultat de planification refusé, indique `actualNativeSteps=0`, `predictionNativeSteps=48`, `workerActualAllOperations=1024` et `workerReservedAllOperations=3856`. Son résultat n’a pas été admis séparément par le parent : le registre conserve donc la réservation B en attente, sans nombre réalisé. **Cet audit rapproche le zéro archivé ; il ne l’inscrit pas rétroactivement au registre.** Le zéro n’ajoute rien au total connu, d’où l’accord numérique des deux sommes malgré cette différence de statut.

Les **2 832 pas réservés mais non intégrés documentés** restent débités : 960 pour B et 1 872 pour ses prévisions. `consumedLocalReservations=64` dans le [rapport de prévision](native-run/002-B-planning-000.json) compte les réservations internes par ticks de quatre pas, y compris les ticks refusés avant intégration. Ce nombre n’est ni 64 intégrations ni un débit supplémentaire aux 1 920 déjà réservés. Chaque branche rapporte exactement 12 intégrations, soit 0,05 s. Aucune sonde courte, aucun second appel B, aucune répétition B ni histoire à 60° n’a été lancée.

La vérification de parité sur 16 pas a été admise avant A. A achève ses 960 pas, soit 4 s, mais son diagnostic local échoue. B reste à son état initial réel : les quatre prévisions sont refusées au tick 3, avant le pas suivant, pour violation d’une contrainte de commande ou non-convergence du solveur dans son nombre d’itérations fixé. Aucun `continue-history` n’apparaît dans le [journal de protocole](native-run/protocol.ndjson). Le [résultat du lanceur](native-run/run-result.json) est `failed-closed` ; stderr confirme l’abandon demandé par le parent. Ce lot n’établit donc pas une comparaison causale A/B complète, un relevage ou une impossibilité anatomique.

La première réservation native date de **20:53:12.695276 UTC** ; l’échéance commune demeure **22:53:12.695276 UTC**. La fermeture est enregistrée à **20:53:20.625705 UTC**, soit **7,930429 s** après activation. Les quatre permis présents dans le protocole partagent cette échéance et le PID 10913. L’admission des preuves natives et le reçu d’échec n’ont renouvelé ni les délais ni les crédits. Le plafond total reste 300 000 sous-pas, dont 21 120 pour les vérifications techniques ; seuls 16 de ces derniers ont été réservés.

La préparation conserve un débit de **7 789 s sur 144 000 s** et de **6 923 inspections sur 20 000**, avec 5 768 inspections déclarées réalisées, antériorité de deux inspections comprise. Son début déclaré est 20:14 UTC le 12 octobre et son échéance calendaire le 17 octobre à 20:14 UTC. Les réservations inutilisées n’ont pas été remboursées.

Vérifications documentaires effectuées : digest interne du registre valide ; trois hashes de reçus inscrits au registre concordants ; manifeste de preuve native valide ; **82 fichiers gelés sur 82 inchangés**. Les traces contiennent 16 lignes de vérification, 960 lignes A et 48 lignes B ; les dernières correspondent aux prévisions, pas à une histoire réelle B de 48 pas.

Empreintes SHA-256 de l’état audité :

- [Registre clos](budget/ledger.json) : `2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07` — identique avant et après cet audit.
- [Rapport B non admis](native-run/002-B.json) : `4312ead48bc75bcc9316e36facf44083afa397efad2c21e5b546d393c3a80108`.
- [Résultat du lanceur](native-run/run-result.json) : `4e1ec85d3359514c180abb029dc21003e1858911ebf7c8d36be4180acf60edcb`.

**Aucune réserve restante n’autorise une reprise : ce protocole est fermé.** Toute histoire complète ou qualification reste hors de ce lot clos.
