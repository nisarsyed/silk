import assert from 'node:assert/strict';
import * as api from '@nisarsyed/silk';
import { exercise } from './exercise.mjs';
import { readFile, writeFile } from 'node:fs/promises';
const metadata = JSON.parse(await readFile(new URL('./node_modules/@nisarsyed/silk/package.json', import.meta.url)));
assert.equal((await exercise(api)).version, metadata.version);
assert((await readFile(new URL('./node_modules/@nisarsyed/silk/README.md', import.meta.url), 'utf8')).includes(`Silk ${metadata.version}`));
const binary = new URL(import.meta.resolve('@nisarsyed/silk/silk.wasm'));
await exercise(api, {wasmUrl: binary});
await writeFile(new URL('./corrupt.wasm', import.meta.url), 'invalid wasm');
for (const name of ['missing.wasm','corrupt.wasm']) {
  await assert.rejects(api.createSilk({wasmUrl:new URL(name, import.meta.url)}), error =>
    error.message.includes('asset URL') && error.cause !== undefined);
}
assert.throws(() => import.meta.resolve('@nisarsyed/silk/silk.mjs'), /not defined by "exports"/);
console.log('Installed Node consumer, types/runtime parity, ownership, binary override and load failures passed');
