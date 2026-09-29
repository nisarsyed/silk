import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';

const build=path.resolve(process.argv[2]??'build/render-study');
const value=JSON.parse(await fs.readFile(path.join(build,'provenance.json'),'utf8'));
const {provenanceJson}=await import(pathToFileURL(path.join(build,'provenance.mjs')));
assert.deepEqual(JSON.parse(provenanceJson),value);
assert.equal(value.schema,1);assert.equal(value.verification,'assembly-time');
assert.match(value.source.revision,/^[a-f0-9]{40}$/);assert.equal(typeof value.source.dirty,'boolean');
assert.match(value.source.sha256,/^[a-f0-9]{64}$/);
assert.equal(value.contract.sha256,value.source.files[value.contract.path]);
assert.match(value.contract.lastCommit,/^[a-f0-9]{40}$/);assert.ok(value.contract.declaredRevision>=1);
for(const required of ['runner.mjs','raylib.js','raylib.mjs','raylib.wasm','physics/study.wasm',
  'physics/driver.mjs','physics/build.json','benchmark/bench.wasm','benchmark/build.json'])assert.ok(value.artifacts[required]);
assert.equal(value.artifacts['provenance.json'],undefined);assert.equal(value.artifacts['provenance.mjs'],undefined);
for(const [name,expected] of Object.entries(value.artifacts)){
  const target=path.resolve(build,name);assert.ok(target.startsWith(build+path.sep));
  const bytes=await fs.readFile(target);assert.equal(bytes.length,expected.bytes,name);
  assert.equal(createHash('sha256').update(bytes).digest('hex'),expected.sha256,name);
}
for(const [kind,module] of Object.entries(value.modules)){
  assert.equal(module.kind,kind);assert.equal(module.source.sha256,value.source.sha256);
  assert.equal(module.sdkCommit,'5eb0bde7585670252e8ba05e9d361627bffd08b5');
  assert.match(module.compiler,/6\.0\.9/);assert.ok(module.compileCommands.length>0);
  for(const [name,expected] of Object.entries(module.artifacts))assert.deepEqual(
    value.artifacts[(kind==='physics'?'physics/':'')+name],expected);
}
assert.ok(Object.keys(value.modules.raylib.raylib.files).length>0);
console.log('Build provenance: delivered artifacts, module inputs, contract and exported envelope PASS');
