#!/usr/bin/env python3
"""Render saved A36 values only: no simulation, FK, pose or task assembly."""

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
INPUTS = {
    "entry": (
        "docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json",
        "3bd00408a5fdb3df9f17feda430f5a1aad8334b17c244d3b119a09325891c433",
    ),
    "commands": (
        "docs/evidence/coordination-transfer-candidate-2026-10-13/archived-a36-command-result.json",
        "9ee5613ece4177588f4acfdd07c4adf9d77c6c380830b4f79690afa1af4ce521",
    ),
}


def digest(path):
    data = path.read_bytes()
    return {"path": str(path.relative_to(ROOT)), "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


def load_inputs():
    data, hashes = {}, {}
    for key, (relative, expected) in INPUTS.items():
        path = ROOT / relative
        hashes[key] = digest(path)
        if hashes[key]["sha256"] != expected:
            raise ValueError(f"Archived input hash mismatch: {relative}")
        data[key] = json.loads(path.read_text())
    return data, hashes


def fr(value, digits=1):
    return f"{value:.{digits}f}".replace(".", ",")


def main():
    data, inputs = load_inputs()
    entry, result = data["entry"], data["commands"]
    observed = entry["archivedNativeRow"]["support"]
    proposal = result["proposal"]
    segments = [s for s in observed["segments"] if s["segment"] in (18, 19, 23, 24)]
    if observed["native_substep"] != 36 or proposal["native_substep"] != 36:
        raise ValueError("This figure is restricted to the saved A36 observation")
    contact_xz = [[c["world_point_m"][0], c["world_point_m"][2]]
                  for s in segments for c in s["contacts"]]
    if contact_xz != proposal["target_support_points_xz_m"]:
        raise ValueError("Proposal support points differ from saved observed contacts")
    heels = proposal["heel_intents"]
    heel_loads = [max(0.0, h["region_load_n"]) for h in proposal["heel_observations"]]
    com = observed["center_of_mass_m"]
    capture = proposal["measured_capture_xz_m"]
    target = proposal["com_capture_target_xz_m"]
    palette = {18: "#176b89", 19: "#45a5ba", 23: "#7d558c", 24: "#b185bc"}
    intent = "#cc5b21"
    ink = "#243340"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "axes.labelcolor": ink, "text.color": ink,
                         "xtick.color": ink, "ytick.color": ink,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none", "svg.hashsalt": "archived-a36"})
    fig = plt.figure(figsize=(15, 10), facecolor="white")
    grid = fig.add_gridspec(2, 2, width_ratios=(1.12, 1), height_ratios=(1, 1),
                           left=0.07, right=0.97, top=0.82, bottom=0.18,
                           hspace=0.48, wspace=0.28)
    ax = fig.add_subplot(grid[:, 0])
    loads_ax = fig.add_subplot(grid[0, 1])
    heels_ax = fig.add_subplot(grid[1, 1])
    fig.text(0.07, 0.95, "A36 · Appuis observés et transfert proposé", fontsize=22, weight="bold")
    fig.text(0.07, 0.907, "Archive à 0,150 s — état dynamique ; diagnostic local non satisfait",
             fontsize=13, color="#566672")
    fig.text(0.07, 0.866,
             "Contacts porteurs présents · Talons sans charge · Recentrage et abaissement demandés, non réalisés",
             fontsize=12, color=ink)

    # Coordinates below are copied from JSON and converted from m to cm.
    for s in segments:
        points = [c["world_point_m"] for c in s["contacts"]]
        ax.scatter([p[0] * 100 for p in points], [p[2] * 100 for p in points],
                   color=palette[s["segment"]], s=60, edgecolor="white", linewidth=0.6,
                   zorder=3)
    anchors = proposal["material_obligations"]
    ax.scatter([p["world_anchor_m"][0] * 100 for p in anchors],
               [p["world_anchor_m"][2] * 100 for p in anchors], marker="s", s=140,
               facecolors="none", edgecolors=ink, linewidth=1.2, zorder=4)
    ax.scatter(com[0] * 100, com[2] * 100, marker="o", s=82, color=ink,
               edgecolors="white", linewidth=0.8, zorder=6)
    ax.scatter(capture[0] * 100, capture[1] * 100, marker="x", s=82,
               color=ink, linewidth=1.7, zorder=7)
    ax.annotate("Centre de masse", xy=(com[0]*100, com[2]*100), xytext=(3.2, 1.0),
                fontsize=10, arrowprops={"arrowstyle": "-", "color": ink})
    ax.annotate("Point de capture", xy=(capture[0]*100, capture[1]*100), xytext=(-13.5, 8.1),
                fontsize=10, arrowprops={"arrowstyle": "-", "color": ink})
    ax.scatter(target[0] * 100, target[1] * 100, marker="*", s=200, color=intent,
               edgecolor="white", linewidth=0.6, zorder=5)
    ax.annotate("Cible de recentrage\n(intention)", xy=(target[0]*100, target[1]*100),
                xytext=(3.5, -3.7), fontsize=10, color=intent,
                arrowprops={"arrowstyle": "-", "color": intent})
    ax.annotate("", xy=(target[0]*100, target[1]*100), xytext=(com[0]*100, com[2]*100),
                arrowprops={"arrowstyle": "->", "linestyle": "--", "color": intent,
                            "linewidth": 1.4, "shrinkA": 7, "shrinkB": 9})
    for heel, side, offset in zip(heels, ("G", "D"), (-2, 0)):
        point = heel["observed_point_world_m"]
        ax.scatter(point[0]*100, point[2]*100, marker="D", s=85,
                   facecolors="white", edgecolors=intent, linewidth=1.5, zorder=5)
        ax.annotate(f"Talon {side}\n0 N", xy=(point[0]*100, point[2]*100),
                    xytext=(point[0]*100+offset, point[2]*100-3.3), ha="center",
                    fontsize=10, color=intent)
    ax.set(title="A. Projection au sol des valeurs archivées", xlabel="x (cm)", ylabel="z (cm)")
    ax.set_xlim(-18, 18)
    ax.set_ylim(-29, 12)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#e6eaed", linewidth=0.7)
    ax.text(0.02, 0.04, "Points talon : projections, pas des contacts porteurs.\n"
            "CoM et capture à l’intérieur : diagnostic archivé.",
            transform=ax.transAxes, fontsize=9, va="bottom", color="#566672")

    values = [s["persistent_load_n"] for s in segments]
    labels = ["Arrière-pied G\n18", "Avant-pied G\n19", "Arrière-pied D\n23", "Avant-pied D\n24"]
    bars = loads_ax.bar(range(4), values, color=[palette[s["segment"]] for s in segments], width=0.64)
    for bar, value, s in zip(bars, values, segments):
        loads_ax.text(bar.get_x()+bar.get_width()/2, value+7, f"{fr(value)} N", ha="center", fontsize=11)
        loads_ax.text(bar.get_x()+bar.get_width()/2, 10, f"{s['loaded_substeps']} pas\nconsécutifs",
                      ha="center", va="bottom", fontsize=9, color="white", weight="bold")
    loads_ax.set_xticks(range(4), labels, fontsize=9)
    loads_ax.set_ylim(0, 285)
    loads_ax.set_ylabel("Charge verticale archivée (N)")
    loads_ax.set_title("B. Quatre segments portent déjà une charge", loc="left")
    loads_ax.grid(axis="y", color="#e6eaed", linewidth=0.7)
    loads_ax.set_axisbelow(True)
    loads_ax.text(0, 1.03, "Contacts avant des arrière-pieds 18/23 ; charge des talons : 0 N.",
                  transform=loads_ax.transAxes, fontsize=9, color="#566672")

    heights = [h["height_above_observed_support_m"]*1000 for h in heels]
    vertical_intents = [h["world_velocity_mps"][1]*1000 for h in heels]
    heels_ax.axhline(0, color=ink, linewidth=1.2)
    for index, (height, load, velocity) in enumerate(zip(heights, heel_loads, vertical_intents)):
        heels_ax.plot([index, index], [0, height], color="#b8c3ca", linewidth=1.2)
        heels_ax.scatter(index, height, marker="D", s=95, facecolors="white", edgecolors=intent,
                         linewidth=1.6, zorder=5)
        heels_ax.text(index, height+3.3, f"{fr(height, 2)} mm", ha="center", weight="bold", fontsize=12)
        heels_ax.annotate("", xy=(index+0.18, 4), xytext=(index+0.18, height-2),
                          arrowprops={"arrowstyle": "->", "color": intent, "linestyle": "--", "linewidth": 1.5})
        heels_ax.text(index+0.22, 12, f"{fr(velocity)} mm/s\ndemandés", fontsize=9, color=intent, va="center")
        heels_ax.text(index, -3.1, f"Charge talon : {fr(load, 0)} N", ha="center", fontsize=10)
    heels_ax.set_xticks([0, 1], ["Talon gauche · 18", "Talon droit · 23"], fontsize=10)
    heels_ax.set_xlim(-0.45, 1.65)
    heels_ax.set_ylim(-5, 38)
    heels_ax.set_yticks([0, 10, 20, 30])
    heels_ax.set_ylabel("Point talon au-dessus du support (mm)")
    heels_ax.set_title("C. Talons encore au-dessus du support", loc="left")
    heels_ax.grid(axis="y", color="#e6eaed", linewidth=0.7)
    heels_ax.set_axisbelow(True)

    legend = [Line2D([0], [0], marker="o", color="none", markerfacecolor=palette[19],
                      markeredgecolor="none", label="Contact observé"),
              Line2D([0], [0], marker="s", color="none", markeredgecolor=ink,
                      markerfacecolor="none", label="Ancre matérielle retenue"),
              Line2D([0], [0], marker="*", color=intent, linestyle="--",
                      label="Intention de commande"),
              Line2D([0], [0], marker="D", color="none", markeredgecolor=intent,
                      markerfacecolor="white", label="Point talon archivé")]
    fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(0.52, 0.115), ncol=4,
                frameon=False, fontsize=10)
    fig.text(0.07, 0.077,
             "Lecture causale : conserver les appuis avant, acquérir les talons puis stabiliser le recentrage avant la continuation.",
             fontsize=11, weight="bold")
    fig.text(0.07, 0.043,
             "Données A36 et commandes archivées uniquement. Aucun mouvement réussi, relevage ou appui futur démontré.\n"
             "Figure sans simulation, FK, reconstruction de pose ni nouvel assemblage. Les flèches indiquent des consignes, pas une trajectoire.",
             fontsize=9, color="#566672", linespacing=1.6)
    png = HERE / "a36-support-and-intent.png"
    svg = HERE / "a36-support-and-intent.svg"
    fig.savefig(png, dpi=180, metadata={"Title": "A36 — observation archivée et intentions de transfert"})
    fig.savefig(svg, metadata={"Title": "A36 — observation archivée et intentions de transfert",
                               "Date": None})
    plt.close(fig)
    extracted = {
        "nativeSubstep": 36, "timeS": observed["time_s"],
        "centerOfMassM": com, "capturePointXZ_M": capture, "recenterIntentXZ_M": target,
        "observedContacts": [{"segment": s["segment"], "contacts": s["contacts"]} for s in segments],
        "segmentLoadsN": [{"segment": s["segment"], "loadN": s["persistent_load_n"],
                           "consecutiveSubsteps": s["loaded_substeps"]} for s in segments],
        "heelPoints": heels, "heelLoadsN": heel_loads, "materialObligations": anchors,
        "localDiagnostic": proposal["measured_local_diagnostic"],
        "meaning": "Copied saved observations and intentions; unit conversion for display only. No physical result computed.",
    }
    values_path = HERE / "plotted-archived-values.json"
    values_path.write_text(json.dumps(extracted, indent=2, ensure_ascii=False)+"\n")
    receipt = {
        "schema": 1, "createdUtc": datetime.now(timezone.utc).isoformat(),
        "scope": "Plot saved A36 observation and saved command intention; no physical or geometric solver execution.",
        "inputs": inputs, "script": digest(Path(__file__)),
        "outputs": [digest(p) for p in (png, svg, values_path)],
        "validation": {"inputHashesMatchExpected": True, "entryAndProposalAtA36": True,
                       "proposalSupportPointsExactlyEqualObservedContactProjections": True,
                       "observedSupportContacts": len(contact_xz), "heelsLoaded": False,
                       "newNativeSubsteps": 0, "newWorlds": 0, "newFk": 0,
                       "newAssembleObservedCalls": 0, "testsExecuted": 0},
        "limitations": ["The heel world points and heights are copied from the saved proposal; their old algebraic derivation is not rerun.",
                        "No foot outlines or support hull are reconstructed. Only saved contact points are displayed.",
                        "Dashed arrows show intended targets or direction, not a predicted or observed path; vertical arrows do not encode duration or speed by length.",
                        "The archived command result belongs to its historical source state and does not validate current code or physics."],
    }
    (HERE / "render-receipt.json").write_text(json.dumps(receipt, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps({"rendered": [str(p.relative_to(ROOT)) for p in (png, svg)],
                      "contacts": len(contact_xz), "heelHeightsMm": heights,
                      "nativeSubsteps": 0, "fk": 0, "assembleObserved": 0}))


if __name__ == "__main__":
    main()
