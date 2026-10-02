import type { RenderQuality } from "../core/types";

export function QualityPicker({ quality, onChange, french = false }: {
  quality: RenderQuality; onChange: (quality: RenderQuality) => void; french?: boolean;
}) {
  return <label className="quality-picker">{french ? "Qualité" : "Quality"}
    <select aria-label={french ? "Qualité du rendu" : "Render quality"} data-testid="quality-picker"
      value={quality} onChange={event => onChange(event.target.value as RenderQuality)}>
      <option value="auto">Auto</option>
      <option value="low">{french ? "Basse" : "Low"}</option>
      <option value="high">{french ? "Haute" : "High"}</option>
    </select>
  </label>;
}
