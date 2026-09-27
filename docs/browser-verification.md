# Browser verification

Run from the repository root after `npm ci`, using Node 22.13.0 or newer. Browser runners have prerequisites beyond the project lockfile: Playwright, the browser channel they launch, and FFmpeg for the recovery/protocol video processing. See [current status](status.md) for acceptance scope.

## Install or select the tools

The protocol, recovery, and playground runners currently launch `channel: "msedge"`. They require Microsoft Edge installed on the host; installing Playwright's Chromium alone is insufficient. The interaction runner additionally supports `PLAYWRIGHT_CHANNEL=chromium`. Edge installation and platform requirements are described in the [official Playwright browser guide](https://playwright.dev/docs/browsers#google-chrome--microsoft-edge). Linux hosts also need the browser's system libraries.

To use a project-local tooling installation, install the Playwright version pinned by the repository's `physical-chain.yml` workflow into an ignored directory:

```sh
npm install --prefix .sites-runtime/browser-tools --no-save --package-lock=false playwright@1.63.0
```

In PowerShell, configure the runner session:

```powershell
$env:CODEX_MCP_NODE_PATH = (Resolve-Path '.sites-runtime/browser-tools/node_modules/playwright/cli.js').Path
node $env:CODEX_MCP_NODE_PATH --version
```

In Bash, use:

```bash
export CODEX_MCP_NODE_PATH="$PWD/.sites-runtime/browser-tools/node_modules/playwright/cli.js"
node "$CODEX_MCP_NODE_PATH" --version
```

Despite its historical name, these browser scripts use `CODEX_MCP_NODE_PATH` only as a location from which to resolve the `playwright` module; they do not execute it as Node. The existing CLI file above provides that location. Commands below use the active `node` executable. An existing bundled runtime can also supply this variable, provided Playwright resolves from its directory. Keep the environment in the terminal that runs the verification scripts.

If Edge is absent, Playwright can install it with `node "$CODEX_MCP_NODE_PATH" install msedge` in Bash or `node $env:CODEX_MCP_NODE_PATH install msedge` in PowerShell. This installs a browser into the operating system's default location and may require elevated rights; use an existing Edge installation when available. See the [installation guidance](https://playwright.dev/docs/browsers#google-chrome--microsoft-edge).

Install an FFmpeg executable using the [official download options](https://ffmpeg.org/download.html), and make `ffmpeg -version` succeed in the runner terminal. Alternatively, set `FFMPEG_PATH` to the absolute executable path; in PowerShell, verify it with `& $env:FFMPEG_PATH -version`. Video capture also uses the FFmpeg binary managed by Playwright:

```sh
node .sites-runtime/browser-tools/node_modules/playwright/cli.js install ffmpeg
```

The PowerShell recipe targets Windows. Bash works on supported macOS/Linux hosts with Edge and its dependencies installed. These instructions are setup requirements, not certification of every operating system or browser combination.

## Start the application

In a separate terminal, leave Vite running on port 5173; the full interaction runner currently hard-codes this address and imports Vite source modules:

```sh
node scripts/run-tool.mjs vite --host 127.0.0.1 --port 5173 --strictPort
```

The default `/` route is the protocol room. The playground is `/?mode=playground`; its interaction QA route is `/?mode=playground&qa=1`. These replay commands target the development server. Static Pages asset verification is a separate `npm run test:pages` check.

## Run the required selection

Use a fresh output directory for every run. The examples below use `review-01`; replace it before repeating a command. Outputs under `evidence/` are ignored by Git. Do not overwrite a previous failed or accepted run.

| Check | Command in the configured runner terminal | Scope |
| --- | --- | --- |
| Playground | `node scripts/verify-playground.mjs evidence/browser/review-01/playground` | Stations, controls, pause/reset, renderer switching, and mobile viewport |
| Interaction smoke | `node scripts/run-visual-replay.mjs quick review-01-quick` | Short interaction check; not complete recovery acceptance |
| Full interaction | `node scripts/run-visual-replay.mjs full review-01-full` | Longer interaction/fall/recovery checks in both renderers |
| Recovery snapshots/videos | `node --import tsx scripts/run-recovery-visual-replay.mjs evidence/browser/review-01/recovery` | Default fixture selection, two views, camera angles, normal and slow playback |
| Protocol snapshots/videos | `node --import tsx scripts/run-protocol-visual-replay.mjs evidence/browser/review-01/protocol-replay http://127.0.0.1:5173` | One 3 s trajectory; no autonomous-recovery acceptance |

For protocol UI verification, set its otherwise different default URL before running. PowerShell:

```powershell
$env:PROTOCOL_VISUAL_URL = 'http://127.0.0.1:5173/'
node scripts/verify-protocol-visual.mjs evidence/browser/review-01/protocol-ui
```

Bash:

```bash
PROTOCOL_VISUAL_URL=http://127.0.0.1:5173/ node scripts/verify-protocol-visual.mjs evidence/browser/review-01/protocol-ui
```

Other overrides are `PLAYGROUND_URL` (retain `?mode=playground`), `RECOVERY_REPLAY_URL`, and `PROTOCOL_REPLAY_URL`. The protocol replay's URL argument takes precedence over its environment variable. The full interaction runner has no URL override. Its output is under `evidence/visual-20260906/<run-name>` despite that historical parent name. Recovery fixture IDs can be supplied as a comma-separated second argument after the output directory; any filtered or shortened run must be labeled partial.

## Inspect and preserve results

Record command, exit code, source/configuration fingerprint, Node/Playwright/browser versions, selected fixtures, and output paths. Inspect the generated PNGs and videos in both renderers; verify that the requested renderer actually ran. A responsive page or a successful video encode does not demonstrate the 25-second recovery criterion or five completed cycles.

The recorded-snapshot replays share one physical trajectory across views and playback speeds. Synthetic touch events do not certify native touch, and software-rendered WebGL does not measure hardware GPU performance. Apply the [evidence preservation procedure](evidence.md#preserving-future-results) before reporting acceptance.
