import { spawnSync } from 'node:child_process';
import { rustCheckCommands, rustEnvironment } from './rust-toolchain.mjs';

if (process.argv.length !== 3) throw new Error('Usage: node scripts/run-rust-checks.mjs <test|lint>');
const commands = rustCheckCommands(process.argv[2]);
const env = rustEnvironment();
for (const args of commands) {
  const result = spawnSync('cargo', args, { env, stdio: 'inherit', windowsHide: true });
  if (result.error) console.error(result.error.message);
  if (result.status !== 0) {
    process.exitCode = result.status ?? 1;
    break;
  }
}
