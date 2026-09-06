import { defineConfig, globalIgnores } from "eslint/config";
import nextVitals from "eslint-config-next/core-web-vitals";
import nextTs from "eslint-config-next/typescript";


// Keep application composition in demo; subsystem implementations depend on core.
function layerBoundary(files, layers, packages = []) {
  return {
    files,
    rules: {
      "no-restricted-imports": ["error", {
        patterns: [
          {
            regex: "^(?:@/src/|(?:\\.\\./)+)(?:" + layers.join("|") + ")(?:/|$)",
            message: "Compose subsystems in src/demo and keep shared contracts in src/core.",
          },
          ...(packages.length ? [{
            group: packages,
            message: "This dependency belongs to a presentation or simulation adapter.",
          }] : []),
        ],
      }],
    },
  };
}
const eslintConfig = defineConfig([
  ...nextVitals,
  ...nextTs,
  layerBoundary(["src/core/**/*.{ts,tsx}"], ["character", "interaction", "scene", "ui", "demo"],
    ["react", "react/*", "react-dom", "react-dom/*", "three", "three/*", "@dimforge/*"]),
  layerBoundary(["src/character/**/*.{ts,tsx}"], ["interaction", "scene", "ui", "demo"],
    ["react", "react/*", "react-dom", "react-dom/*", "three", "three/*"]),
  layerBoundary(["src/scene/**/*.{ts,tsx}"], ["character", "interaction", "ui", "demo"],
    ["react", "react/*", "react-dom", "react-dom/*", "@dimforge/*"]),
  layerBoundary(["src/interaction/**/*.{ts,tsx}"], ["character", "scene", "ui", "demo"],
    ["react", "react/*", "react-dom", "react-dom/*", "three", "three/*", "@dimforge/*"]),
  layerBoundary(["src/ui/**/*.{ts,tsx}"], ["character", "interaction", "scene", "demo"],
    ["three", "three/*", "@dimforge/*"]),
  // Override default ignores of eslint-config-next.
  globalIgnores([
    // Default ignores of eslint-config-next:
    ".next/**",
    "out/**",
    "dist/**",
    ".sites-runtime/**",
    ".wrangler/**",
    "evidence/**",
    "build/**",
    "next-env.d.ts",
  ]),
  {
    files: ["components/ui/**/*.{ts,tsx}", "hooks/use-mobile.ts"],
    rules: {
      // These files are vendored verbatim from shadcn@4.17.0. Keep the
      // registry source intact while applying the stricter rules to Site code.
      "@typescript-eslint/no-unused-vars": "off",
      "react-hooks/purity": "off",
      "react-hooks/set-state-in-effect": "off",
    },
  },
]);

export default eslintConfig;
