#!/usr/bin/env python3
"""Assemble the Library report from reviewed repository documents; no physics."""
from pathlib import Path
import argparse, hashlib, json, re, subprocess
parser=argparse.ArgumentParser()
parser.add_argument("--commit", required=True)
a=parser.parse_args()
if not re.fullmatch(r"[0-9a-f]{40}",a.commit):raise SystemExit("Exact commit required")
root=Path(__file__).resolve().parents[4]
tracked=set(subprocess.check_output(["git","ls-tree","-r","--name-only",a.commit],cwd=root,text=True).splitlines())
parts=[
 ("Résultat décisif et décision", "docs/coordination-local-pair-2026-10-13.md"),
 ("Revue du plan et architecture implémentée — historique du 12 octobre", "docs/coordination-prototype-2026-10-12.md"),
 ("Certificat causal après le dernier essai", "docs/evidence/coordination-local-pair-2026-10-13/postrun-causal-review.md"),
 ("Audit indépendant des mesures, moteurs et états", "docs/evidence/coordination-local-pair-2026-10-13/postrun-physical-audit.md"),
 ("Comptabilité et conservation des registres", "docs/evidence/coordination-local-pair-2026-10-13/postrun-budget-audit.md"),
 ("Décision de continuation complète", "docs/evidence/coordination-local-pair-2026-10-13/postrun-continuation-decision.md")]
records=[]
body=["# LaboratoireHumain — coordination, prototype et résultat décisif\n\n"
"Rapport consolidé du 13 octobre 2026. **Le prototype est implémenté et testé ; "
"cette famille est close sans acquisition des talons ni relevage.** Les quatre "
"problèmes prédictifs refusés sont contradictoires sous leurs contraintes "
"archivées, selon deux vérifications arithmétiques indépendantes. Ce résultat "
"porte sur la formulation locale, sans démontrer une impossibilité anatomique.\n\n"
+"Preuves publiées : [commit "+a.commit+"](https://github.com/berenif/LaboratoireHumain/commit/"+a.commit+"). "
"Branche `investigate/physical-support` ; `main` conservé à "
"`c597959f46f37715035349c8cd0a699b6822111a`.\n\n"
"Les sections historiques décrivent la préparation et ses hypothèses. "
"Le résultat et les décisions après clôture ci-dessous font autorité pour la suite. "
"Le rapport, la vidéo et les mesures sont disponibles dans le dépôt. "
"**Publication Library bloquée : erreur réseau du flux officiel avant toute création confirmée ; aucun nouvel identifiant ni version ne sont disponibles.** "
"Les liens vers des traces volumineuses renvoient à leur archive compressée "
"ou à son index de restitution sans perte.\n"]
for title,rel in parts:
 p=root/rel
 source_bytes=subprocess.check_output(["git","show",a.commit+":"+rel],cwd=root)
 text=source_bytes.decode()
 records.append({"path":rel,"sha256":hashlib.sha256(source_bytes).hexdigest()})
 text=re.sub(r"^- \[ \] Rapport Library.*\n", "", text, flags=re.M)
 def link(m):
  target=m.group(2)
  if re.match(r"(?:[a-z][a-z0-9+.-]*:|#)",target,re.I):return m.group(0)
  path,sep,frag=target.partition("#")
  resolved=(p.parent/path).resolve()
  try:rr=resolved.relative_to(root).as_posix()
  except ValueError:return m.group(0)
  if not resolved.exists():raise ValueError("Missing local link: "+str(resolved))
  if rr not in tracked:
   candidates=[resolved.with_name(resolved.name+".gz")]
   candidates += [parent/"archive-index.json" for parent in resolved.parents if parent==root or root in parent.parents]
   found=next((candidate for candidate in candidates if candidate.relative_to(root).as_posix() in tracked),None)
   if found is None:raise ValueError("Link absent from published commit: "+rr)
   rr=found.relative_to(root).as_posix()
   sep=""
  host="https://raw.githubusercontent.com/berenif/LaboratoireHumain/" if m.group(1).startswith("!") else "https://github.com/berenif/LaboratoireHumain/blob/"
  return m.group(1)+host+a.commit+"/"+rr+("#"+frag if sep else "")+")"
 text=re.sub(r"(!?\[[^\]]*\]\()([^\)]+)\)",link,text)
 body.append("\n---\n\n# "+title+"\n\n"+text)
out=Path(__file__).parent/"LaboratoireHumain-coordination-paire-finale-rapport-complet-2026-10-13.md"
out.write_text("\n".join(body))
receipt={"commit":a.commit,"sources":records,"path":str(out),"bytes":out.stat().st_size,"sha256":hashlib.sha256(out.read_bytes()).hexdigest(),"nativeIntegrations":0}
(Path(__file__).parent/"consolidation-validation.json").write_text(json.dumps(receipt,indent=2)+"\n")
print(json.dumps(receipt))
