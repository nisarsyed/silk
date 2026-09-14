import type {SnapshotCopy} from '../../wasm/index.js';

export const circleSides = 32;
export const instanceLimit = 65536;
export const renderTiers = Object.freeze([256,512,1024,2048,4096,8192,16384,32768,65536]);
export const sceneRects = Object.freeze({
  pyramid: [-13,-1,13,22], rain: [-9,-1,9,25], chains: [-1,-1,29,16]
} satisfies Record<string, number[]>);
export type Rect = readonly [number,number,number,number];
export const background = '#101820';
export const colors = Object.freeze(['#7192a5', '#ebbd70']); // Static, moving body.
export const colorRgb = Object.freeze([[113/255,146/255,165/255],[235/255,189/255,112/255]]);
export interface Mesh { vertices: Float32Array; triangles: Float32Array }
export interface Batch { first: number; count: number; mesh: number; color: number }

/** Comparison data only. Source rows retain snapshot draw order; geometry uses
 * body slot indices, not packed rows. All storage is allocated during setup.
 * poses is four floats per instance: x, y, cos, sin. Fixed render-only replicas
 * retain the exact source transforms and have no per-frame uploads.
 */
export class DrawScene {
  readonly meshes: Mesh[] = [];
  readonly batches: Batch[] = [];
  readonly poses: Float32Array;
  readonly rows: Uint32Array;
  readonly offsets: Float32Array;
  readonly count: number;
  readonly frozen: boolean;
  readonly rect: Rect;
  revision = 0;
  constructor(snapshot: SnapshotCopy, name: keyof typeof sceneRects, instances?: number) {
    if (!(name in sceneRects)) throw new Error('Unknown comparison scene');
    if (instances !== undefined && (name !== 'rain' || !renderTiers.includes(instances)))
      throw new RangeError('Render-only requires rain and a frozen contract tier');
    this.frozen = instances !== undefined;
    const source: number[] = [];
    for (let row=0; row<snapshot.bodyCount; ++row) {
      const slot=snapshot.bodies.index[row];
      if ((!this.frozen || snapshot.bodies.type[row] !== 2) && snapshot.geometry.kind[slot] !== 0) source.push(row);
    }
    this.count = instances ?? source.length;
    if (!source.length || this.count < 1 || this.count > instanceLimit) throw new RangeError('Invalid instance count');
    this.poses = new Float32Array(this.count*4);
    this.rows = new Uint32Array(this.count);
    this.offsets = new Float32Array(this.count);
    const tiles = Math.ceil(this.count/source.length);
    const base=sceneRects[name];
    this.rect = [base[0]-16*(tiles-1),base[1],base[2]+16*(tiles-1),base[3]];
    const known = new Map<string,number>();
    for (let i=0; i<this.count; ++i) {
      const row=source[i%source.length], slot=snapshot.bodies.index[row];
      this.rows[i]=row; this.offsets[i]=32*(Math.floor(i/source.length)-(tiles-1)/2);
      const g=snapshot.geometry, kind=g.kind[slot], n=kind === 1 ? circleSides : g.count[slot];
      if ((kind !== 1 && kind !== 2) || n<3 || n>(kind===1?32:8)) throw new Error('Invalid shape');
      const vertices=new Float32Array(2*n);
      for (let v=0; v<n; ++v) {
        vertices[2*v]=kind===1 ? g.radius[slot]*Math.cos(v*2*Math.PI/n) : g[`x${v}` as 'x0'][slot];
        vertices[2*v+1]=kind===1 ? g.radius[slot]*Math.sin(v*2*Math.PI/n) : g[`y${v}` as 'y0'][slot];
      }
      if (!vertices.every(Number.isFinite)) throw new Error('Non-finite geometry');
      const key=vertices.join(',');
      let mesh=known.get(key);
      if (mesh === undefined) {
        mesh=this.meshes.length;
        const triangles=new Float32Array((n-2)*6);
        for (let v=1; v<n-1; ++v) {
          const at=(v-1)*6;
          triangles.set([vertices[0],vertices[1],vertices[2*v],vertices[2*v+1],vertices[2*v+2],vertices[2*v+3]],at);
        }
        this.meshes.push({vertices,triangles}); known.set(key,mesh);
      }
      const color=snapshot.bodies.type[row]===2?0:1, previous=this.batches.at(-1);
      if (previous && previous.mesh===mesh && previous.color===color) ++previous.count;
      else this.batches.push({first:i,count:1,mesh,color});
    }
    this.copyTransforms(snapshot);
  }
  copyTransforms(snapshot: SnapshotCopy): void {
    if (this.frozen && this.revision>0) throw new Error('Frozen render-only transforms cannot change');
    const b=snapshot.bodies;
    for (let i=0; i<this.count; ++i) {
      const row=this.rows[i], at=i*4;
      this.poses[at]=b.x[row]+this.offsets[i]; this.poses[at+1]=b.y[row];
      this.poses[at+2]=b.cos[row]; this.poses[at+3]=b.sin[row];
    }
    if (!this.poses.every(Number.isFinite)) throw new Error('Non-finite transform');
    ++this.revision;
  }
}

/** Equal world-unit scale on both axes; the remaining pixels are letterboxed. */
export function camera(rect: Rect, width: number, height: number) {
  if (!Number.isInteger(width) || !Number.isInteger(height) || width<1 || height<1 ||
      !rect.every(Number.isFinite) || rect[2]<=rect[0] || rect[3]<=rect[1]) throw new RangeError('Invalid camera');
  const scale=Math.min(width/(rect[2]-rect[0]),height/(rect[3]-rect[1]));
  return Object.freeze({scale, x:width/2-scale*(rect[0]+rect[2])/2, y:height/2+scale*(rect[1]+rect[3])/2});
}
