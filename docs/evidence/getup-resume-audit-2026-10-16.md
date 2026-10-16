# Reprise exacte indisponible — audit sans intégration

La continuation demandée est bloquée par la perte du monde physique arrêté, pas par l'autorisation de calcul. **Aucun nouveau sous-pas, aucune prévision et aucun rejeu n'ont été exécutés.** Le reliquat de 4 440 sous-pas et 141,444994836 s reste intact.

Le lanceur a fermé le navigateur dans son bloc `finally`, puis le superviseur a terminé les processus descendants. L'inspection des processus ne trouve aucun navigateur ou worker Node actif. Les archives contiennent les observations des 161 ticks, les références et le contrôleur sérialisé. Elles ne contiennent pas un checkpoint du monde Rapier : état interne des articulations, contraintes, contacts et caches du solveur. Le rapport `getup_report` exporte uniquement le contrôleur. Les fichiers WASM figés sont du code, pas la mémoire vivante de l'essai.

Recharger les positions, rotations et vitesses observées ne préserverait donc pas la continuité mécanique exacte. Une telle reconstruction n'est pas présentée comme une reprise.

Le rejeu n'a pas été lancé non plus : le préfixe observé coûte 3 740 sous-pas, soit **84,2 % du reliquat**. Il ne laisserait que 700 sous-pas, **0,875 s** de progression supplémentaire. La référence demande encore 0,95125 s à partir de l'état arrêté, avant toute observation finale de stabilité d'une seconde. Cela répond à la condition explicite de ne pas dépenser presque tout le reliquat pour un nouveau fragment incomplet.

Le résultat physique reste donc celui de l'[essai de 4 s](getup-penetration-retry-2026-10-16.md) : **0,39875 s de SLOW, 16 décisions, stabilité debout 0 s, aucun refus physique**, corps encore couché. Le cumul, premier lancement interrompu inclus, reste **3 760 sous-pas et 38,555005164 s de lancement physique**. Les anciens registres et les archives ont été revérifiés par empreintes.

La vidéo existante `LaboratoireHumain-Rapier-WEAK-SLOW-4s-2026-10-16.mp4` reste le clip pertinent : validé, déjà enregistré dans Library en version 0. Aucun nouveau film ne prétend montrer une continuation. L'[audit structuré](getup-resume-audit-2026-10-16.json) conserve les champs présents, les preuves de fermeture, les empreintes, le calcul de rejeu et l'identité Library confirmée.

Pour une continuation exacte ultérieure, il faudra conserver le worker vivant et arrêté entre les tranches, ou disposer avant fermeture d'un checkpoint complet avec restauration vérifiée. Ces mécanismes ne peuvent pas récupérer rétroactivement le monde perdu ; aucun nouveau protocole physique n'est lancé ici.
