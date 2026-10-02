// A repository dependency makes checks runnable without the Codex desktop runtime.
export { chromium } from 'playwright';

export function browserLaunchOptions(graphics = false) {
  const channel = process.env.RUST_BROWSER_CHANNEL ?? (process.platform === 'win32' ? 'msedge' : 'chromium');
  return {
    ...(channel === 'chromium' ? {} : { channel }),
    headless: true,
    ...(graphics ? { args: ['--enable-unsafe-webgpu'] } : {}),
  };
}
