# Rejeu visuel court du protocole

Ce contrôle produit une seule trajectoire Rapier de **3 secondes simulées à 60 Hz** après un appel à `requestStrike()`. Il clone et gèle les 181 snapshots, puis les transmet tels quels aux rendus WebGL et Canvas2D. Le ralenti à 0,25× est dérivé des vidéos à vitesse normale ; aucune simulation n’est relancée pour le ralenti.

## Exécution

Suivre d’abord la [configuration des contrôles navigateur](browser-verification.md) : Playwright, Microsoft Edge, `CODEX_MCP_NODE_PATH` et FFmpeg sont requis. `npm ci` seul n’installe pas ces outils. Les résultats historiques et les fichiers absents sont distingués dans [l’état courant](status.md) et [l’inventaire des preuves](evidence.md).

Dans un premier terminal, démarrer le serveur Vite local :

```text
node scripts/run-tool.mjs vite --host 127.0.0.1 --port 4182 --strictPort
```

Dans un second terminal configuré selon le guide, lancer :

```text
node --import tsx scripts/run-protocol-visual-replay.mjs evidence/protocol-visual-replay http://127.0.0.1:4182
```

Le script écrit une trace compressée, un résumé avec les empreintes des sources, et pour chacune des quatre combinaisons (`wide` 1440×900, `narrow` 390×844 ; WebGL, Canvas2D) : trois PNG (`initial`, `contact`, `final`), une vidéo normale et une vidéo à quart de vitesse. `impactFrame` identifie le premier contact Rapier mesuré. Les vidéos lentes reprennent les mêmes images ; elles ne constituent pas une nouvelle trajectoire physique.

Contrôler visuellement la position au repos, le cadrage, l’alignement avant contact, la frappe, la rétraction et la lisibilité du sujet au bord de la salle. Comparer WebGL et Canvas2D aux mêmes images clés. Le champ `largestFrameJump` indique si la capture vidéo a sauté des snapshots sous charge ; examiner les PNG clés même dans ce cas.

Une sortie non nulle signifie qu’il manque un impact, qu’une erreur navigateur ou un diagnostic non fini s’est produit, ou qu’un fichier source suivi a changé pendant l’enregistrement. Dans ce dernier cas, relancer le contrôle sur une source stable. Ces trois secondes ne prouvent pas le relevage autonome ni le critère de 25 secondes.
