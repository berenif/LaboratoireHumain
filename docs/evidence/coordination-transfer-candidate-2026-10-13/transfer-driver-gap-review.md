# Écart entre le candidat statique et un driver natif borné

Revue en lecture seule du 13 octobre 2026. Sources examinées :
`transition_worker.rs`, `native_plan_preview.rs`, `transfer_adapter.rs`,
`whole_body_controller.rs`, points d'intégration de `lib.rs` et cibles Cargo.
Aucun test, compilation, monde, FK, moteur ou pas natif exécuté pour cette revue.
Seul ce document est créé. Les registres antérieurs restent clos.

**Verdict :** le candidat de politique/compilation/adaptation est concret et
testable statiquement. Il peut être figé comme candidat de préparation, avec ses
limites. Le pipeline d'exécution du nouveau lot n'est pas encore complet et ne
peut pas honnêtement être annoncé comme prêt à lancer. Il faut une branche de
driver substantielle, bien que bornée : admission technique propre au candidat,
préfixe vérifié, orchestration des reçus longs, comptabilité et terminaison.
Ce travail supplémentaire n'exige pas de calcul physique pour être écrit et
testé sur doubles/archives. Il doit précéder le gel final du binaire et une
demande d'exécution concrète ; le chiffrage proposé ci-dessous peut déjà être
discuté comme plafond, pas comme autorisation acquise.

## Briques existantes utilisables

- `Controller::enable_observed_transfer` attache l'adaptateur au compteur
  observé, vérifie le dernier q/dq installé et l'historique du propriétaire ;
  il ne reconstruit pas de monde et ne remet pas l'observateur à zéro.
- `StaticPreviewBranch::capture` clone le monde et les mémoires. Pour un
  adaptateur, la capture exige déjà le checkpoint extérieur de la fenêtre
  précédente et un état de récupération original valide.
- `preview_with_post_tick` réutilise `advance_tick_observed`. Il réserve quatre
  places avant le tick, arme seulement sa copie privée pour les quatre actions,
  recueille les observations et écrit les checkpoints après finalisation.
- `certify_transfer_horizon` est le seul constructeur de
  `NativeWindowReceipt`. Il vérifie la branche réellement exécutée, l'état
  original final, le diagnostic et l'horizon effectif ; une fixture JSON ou une
  résolution de QP ne peut pas construire cette capacité.
- `Controller::admit_transfer_window` puis `continue_transfer_window`
  consomment un reçu complet par tranches d'au plus 24 actions. Chaque action
  compare l'identité interne avant installation et la commande exacte ; chaque
  prolongation/replacement exige le checkpoint extérieur après tick.
- Le handoff conserve un moniteur passif ; le propriétaire ordinaire garde les
  moteurs. `TerminalClock` date le succès natif original, interdit sa perte et
  termine la seconde supplémentaire au premier tick entier après 240 pas,
  donc 240 à 243 observations supplémentaires, sans nouveau départ.
- L'ancien `IntegrationGate` dispose déjà des admissions/completions durables,
  du refus avant `world.step`, de l'échéance monotone/non renouvelable et des
  comptes incertains en cas d'interruption. `ActiveScope` restaure le scope
  extérieur après une prévision imbriquée. Ce mécanisme doit être réutilisé,
  pas remplacé par un compteur de callbacks après l'intégration.

Les garanties ci-dessus sont des interactions de code. La parité native de ce
nouveau propriétaire n'a pas été exécutée dans le jalon courant.

## Deux raccords d'admission encore bloquants

### Liaison du reçu de noyau au candidat

Au moment de la revue, `NativeKernelAdmission` ne conserve que
`initial_substep` et `verified_substeps`. Les paramètres `_admission` de
`preview`/`preview_with_post_tick` et `_kernel` de `certify_transfer_horizon`
ne vérifient pas de scope. Une comparaison de quatre pas depuis un autre
propriétaire ne doit pas pouvoir autoriser ce candidat.

Il faut un scope immuable explicitement contrôlé : identité du build/manifeste,
modèle/profil/flags physiques, programme et tâches de référence, version et mode
du nouvel adaptateur, fixture/préfixe et schéma des mémoires de décision. Le
driver doit établir le reçu dans ce même processus et vérifier le scope à
chaque utilisation. Garder séparées cette identité de famille et l'identité
dynamique complète d'entrée déjà portée par le reçu d'horizon : figer le hash
de tout l'état A36 comme scope de toutes les prévisions futures empêcherait
injustement de prévoir depuis A60, A84, etc. Inversement, un simple hash de
référence sans les paramètres d'actionnement ne lie pas le noyau testé.

La revue contradictoire confirme que ce raccord n'a **pas** encore été ajouté.

### Bootstrap de la parité du nouvel adaptateur

`verify_tick_with_post_tick` sait armer la copie privée pour quatre pas. Le
contrepartie réel technique, s'il passe par `actual_tick` avec l'adaptateur
désarmé, rencontre `Authorization::None` et refuse avant les moteurs. Aucun
reçu de 120–480 pas n'existe encore : l'utiliser comme prérequis serait circulaire.

Il faut une capacité **technique**, distincte du reçu matériel d'horizon,
limitée à l'instance, à son compteur exact, à quatre actions, au scope testé et
à une réservation durable déjà active. Elle expire après ce tick et ne peut
autoriser ni une histoire, ni une deuxième fenêtre, ni être sérialisée puis
rechargée comme preuve. Son champ doit rester distinct du bail réel ordinaire.
Les traces copie/contrepartie comparent l'entrée, les quatre états et le tick
finalisé ; le premier écart arrête les intégrations restantes.

Le correctif actuel de `coordination_decision_evidence` retire strictement
`transfer.authorization` et `transfer.nativeRecords` de la parité. Il résout
la différence légitime de pointeurs/états de preuve entre branches, mais **ne
donne pas** cette autorisation technique. Tous les champs décisionnels restent
dans la comparaison, y compris file de commandes, curseur, origine, raccord,
diagnostic, ancres et clock terminale. Ne pas élargir l'exclusion pour faire
passer artificiellement une divergence de q/dq ou de mémoire.

## Séquence minimale du futur driver

1. **Ouvrir un lot distinct.** Geler son manifeste, binaire, sources, entrée,
   opérations autorisées, cap total et échéance unique. Vérifier la capacité
   réelle du watchdog et la production de reçus avant le premier spawn ; les
   anciens accidents `/proc` et u128 ne doivent pas être découverts pendant
   une histoire. Sérialiser le monde directement en octets/texte exact, jamais
   via `json!(world)` qui perd le domaine des grands u128.
2. **Préfixe technique A36.** Créer la scène native d'origine en mode A, replay
   de 36 pas par le même noyau avec budget actif. Comparer l'entrée avec la
   fixture A36 figée et le q/dq réellement installé au pas 35, ainsi que les
   mémoires/horloges nécessaires. Les poses JSON seules ne restaurent pas
   contacts et caches. À cette frontière de tick, construire
   `ObservedCommandEntry`, obtenir `native_observation::observe`, puis appeler
   `enable_observed_transfer` sans modifier les corps. Préserver `start=0`.
3. **Admission technique fraîche.** Armer les deux côtés par la capacité
   technique bornée ; utiliser `NativeParityTrace::begin`, les quatre
   `record`, `record_post_tick`, `compare_prefix` après chaque état réel puis
   `native_kernel_admission_complete`. Archiver les commandes installées,
   caps/gains/`ForceBased` et les champs de scope. Aucun ancien reçu A/B ne
   remplace cette preuve. Définir précisément les deux ticks/cas techniques
   prévus avant de réserver leurs pas.
4. **Histoire unique depuis son propre A36.** Recréer la scène, rejouer son
   préfixe de 36 pas comptés dans le plafond réel, vérifier l'entrée, attacher
   l'adaptateur. Ne pas poursuivre discrètement le monde jetable de parité
   depuis A40/A44 en le présentant comme l'entrée A36 de l'histoire.
5. **Première prévision.** Après un permis de 480 pas, appeler `capture`, puis
   `preview_with_post_tick` avec le nouveau reçu de noyau scopé. Le premier
   horizon doit couvrir les 132 observations neuves du jalon : 120 pas depuis
   une entrée sans jalon ne suffisent pas. Il n'y a qu'un candidat, donc ni
   boucle des quatre échelles anciennes ni réglage opportuniste.
6. **Certifier puis admettre.** Si le run est complet et sans refus, conserver
   les nombres demandés/effectifs via `transfer_horizon_evidence` et appeler
   `certify_transfer_horizon(kernel,start,requested_steps)`. Le driver garde
   le reçu en mémoire ; le parent valide durablement son rapport avant la
   reprise. Le ACK ne renouvelle ni l'échéance ni le crédit de l'histoire.
   Appeler `admit_transfer_window` depuis la même entrée extérieure exacte.
   Si Rust oblige à extraire temporairement le propriétaire pour l'emprunt,
   le remettre même sur erreur, comme le fait déjà `configure_motors`.
7. **Avancer le réel en ticks.** Réserver chaque tick avant le noyau ; le hook
   `before_integration` revérifie permis et deadline après configuration et
   immédiatement avant `world.step`. `after_integration` comptabilise l'appel
   achevé avant les callbacks/journaux riches. Garder observations de support,
   diagnostic et compteurs même après le jalon et le handoff.
8. **Checkpoint, continuation ou nouvelle prévision.** À 24 pas, confirmer le
   checkpoint extérieur avant de capturer une nouvelle branche ou de prolonger
   le reçu existant. `continue_transfer_window` utilise seulement les états
   déjà prévus et n'accorde aucun crédit budgétaire. Figer la règle de choix
   entre re-prévision et consommation du reste du reçu **avant** le lot.
9. **Fermer au terminal ou au premier refus.** Vérifier l'état original
   `Upright`, récupération exactement une, observateur terminé et seconde
   supplémentaire achevée, puis le checkpoint final. Ne pas lancer un tick
   supplémentaire sous prétexte que le plafond réel laisse une marge. En cas
   de refus/parité perdue/deadline/compte incertain, archiver l'état partiel,
   fermer le lot et ne pas essayer une variante.

## Queue terminale et arrêt anticipé : pièges précis

Le reçu garde maintenant tout l'horizon certifié, de 120 à 480 pas, avec les
checkpoints tous les 24 pas et l'extrémité. Une prévision peut s'arrêter
proprement au premier tick de fin de surveillance terminale ; sa durée
**effective** doit néanmoins être d'au moins 120 pas pour obtenir le reçu.
Un refus, une panique ou un timeout ne constitue jamais un arrêt terminal réussi.

Reprévoir 480 pas tous les 24 pas jusqu'au dernier instant échouerait encore
lorsqu'il reste moins de 120 pas jusqu'à la fin terminale. Le driver doit alors
consommer la queue **déjà certifiée**, avec ses checkpoints, sans lancer un
nouveau rollout trop court. Cette décision se prend à partir du reçu et de
son extrémité connus, avant une nouvelle prévision ; elle ne peut pas servir
de repli après un refus. Une politique plus simple consommant systématiquement
tout le reçu par tranches de 24 est techniquement possible, mais serait un
choix de protocole à figer, pas une modification silencieuse de la cadence.

Les identités internes avant installation et les checkpoints extérieurs après
tick ne sont pas interchangeables : le noyau vide des accumulateurs entre ces
phases. `capture`, admission et continuation doivent utiliser les checkpoints
extérieurs sans muter la clock terminale ; le propriétaire observe cette clock
juste avant son identité interne. Les contrôles de capture/certification déjà
implémentés clonant `TerminalClock` respectent cette distinction.

## Comptage du lot proposé

Le plafond annoncé est arithmétiquement cohérent :

| Domaine | Réserve maximale |
| --- | ---: |
| Une histoire, **préfixe A36 inclus** | 6 244 |
| Au plus 260 prévisions, chacune réservée à 480 | 124 800 |
| Admission technique distincte | 52 |
| Total | **131 096** |

Une décomposition possible des 52 pas est un préfixe technique de 36, puis deux
ticks de comparaison avec chacun quatre pas copiés et quatre pas de contrepartie
technique, soit `36 + 2×(4+4)`. Les deux cas et leurs états de départ ne sont pas
spécifiés par ce seul nombre ; les définir dans le driver/manifeste. Si deux
préfixes techniques indépendants sont requis, 52 ne suffit plus. Aucun pas de
restauration, installation vérifiée par intégration ou sondage supplémentaire
n'est gratuit.

Avec l'origine conservée à zéro, le succès doit être observé au plus tard au
pas 6 000, puis la seconde terminale se ferme à son tick déterminé ; 6 244 est
un plafond conservateur, pas une permission de dépasser cette fin. Rebaser
l'origine à A36 repousserait le délai et changerait la portée du protocole.
L'entrée A36 ne contient pas à elle seule la preuve d'un délai depuis impact.

Les 260 appels sont un maximum, pas une quantité à consommer. Chaque appel
demande une réserve entière de 480 même si sa fin terminale arrive plus tôt.
Les pas effectivement achevés, places abandonnées et réservations consommées
restent distincts. Les copies refusées/partielles restent comptées ; une
admission interrompue rend le compte exact inconnu dans sa borne, pas zéro.
L'échéance maximale de deux heures reste unique et non renouvelable entre
préfixe, technique, copies et histoire.

## Pourquoi l'ancien worker ne suffit pas

`transition_worker::verify` impose 16 pas A/B depuis le seed, sans attachement
de la nouvelle politique. `planning` impose quatre propositions de 480 pas et
ne conserve que leur parité initiale de quatre pas. `history` impose la suite
A/B/B-repeat/B60, 960 pas par histoire, 40 appels et les anciennes conditions
de suivi. `accept_permit`/`count` portent encore le plafond 300 000. Aucun de
ces choix ne décrit l'histoire unique proposée ici.

L'intégration minimale doit donc fournir de nouvelles opérations/protocoles
explicitement sélectionnés, ou un worker séparé. Réutiliser le gate, le
comptage durable et la sérialisation est souhaitable ; changer seulement les
constantes de l'ancienne histoire ne branche pas les reçus longs ni le
bootstrap de parité. Si un nouveau binaire/feature est choisi, raccorder aussi
les hooks `before_integration`/`after_integration` de `lib.rs` : ils visent
actuellement `transition_worker` sous `cfg(test, coordination-worker)`. Les
deux cibles Cargo existantes utilisent le même main ; un nom de cible neuf
sans main/protocole distinct ne crée pas un driver neuf.

Avant exécution, tester sans monde les permis/PID/replays, les trois domaines
de comptes et leur somme, les refus de scope, les quatre pas techniques, les
ACK non renouvelables, la restauration des budgets imbriqués, la queue du
reçu terminal, le 261e appel refusé, les plafonds/échéances et les journaux
partiels. Puis figer le code du driver, le runner et leurs tests avec le
candidat. **La préparation actuelle établit un composant testable ; ces
raccords restent nécessaires pour livrer un lot natif effectivement lançable.**
