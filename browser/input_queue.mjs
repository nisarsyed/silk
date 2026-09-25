// Ordered pointer actions applied only at fixed-step boundaries. The small
// typed queue is local to the sole simulation owner on main or worker thread.
export const pointerQueueCapacity = 256;
export const pointerAction = Object.freeze({down:0,move:1,up:2,cancel:3});

export class PointerQueue {
  #sequence;
  #action;
  #pointer;
  #x;
  #y;
  #count = 0;
  #lastSequence = 0;
  #epoch = 1;
  #overflowed = false;
  #draining = false;
  #coalesced = 0;
  #droppedMoves = 0;
  #evictedMoves = 0;

  constructor(capacity=pointerQueueCapacity) {
    if (!Number.isInteger(capacity) || capacity < 4 || capacity > pointerQueueCapacity ||
        (capacity & (capacity-1)) !== 0) throw new RangeError('Invalid pointer queue capacity');
    this.capacity = capacity;
    this.#sequence = new Float64Array(capacity);
    this.#action = new Uint8Array(capacity);
    this.#pointer = new Int32Array(capacity);
    this.#x = new Float32Array(capacity);
    this.#y = new Float32Array(capacity);
  }

  get count() { return this.#count; }
  get epoch() { return this.#epoch; }
  get overflowed() { return this.#overflowed; }
  get statistics() { return {capacity:this.capacity,count:this.#count,coalesced:this.#coalesced,
    droppedMoves:this.#droppedMoves,evictedMoves:this.#evictedMoves,overflowed:this.#overflowed}; }

  reset(nextEpoch) {
    if (this.#draining || !Number.isSafeInteger(nextEpoch) || nextEpoch !== this.#epoch+1)
      throw new RangeError('Invalid pointer queue reset epoch');
    this.#epoch = nextEpoch;
    this.#count = 0;
    this.#lastSequence = 0;
    this.#overflowed = false;
    this.#coalesced = 0;
    this.#droppedMoves = 0;
    this.#evictedMoves = 0;
  }

  enqueue(epoch, sequence, action, pointerId, x, y) {
    if (!Number.isSafeInteger(epoch) || !Number.isSafeInteger(sequence) || sequence < 1 ||
        !Number.isInteger(action) || action < 0 || action > 3 ||
        !Number.isInteger(pointerId) || pointerId < -2147483648 || pointerId > 2147483647 ||
        !Number.isFinite(x) || !Number.isFinite(y) || Math.abs(x) > 8192 || Math.abs(y) > 8192 ||
        !Number.isFinite(Math.fround(x)) || !Number.isFinite(Math.fround(y)))
      throw new RangeError('Invalid pointer message');
    if (epoch !== this.#epoch) return 'stale';
    if (this.#draining) throw new Error('Cannot enqueue while applying pointer actions');
    if (this.#overflowed) return 'overflow';
    if (sequence <= this.#lastSequence) throw new RangeError('Pointer message sequence is not increasing');
    this.#lastSequence = sequence;
    const n = this.#count;
    if (action === pointerAction.move && n > 0 && this.#action[n-1] === action &&
        this.#pointer[n-1] === pointerId) {
      this.#sequence[n-1] = sequence;
      this.#x[n-1] = Math.fround(x);
      this.#y[n-1] = Math.fround(y);
      ++this.#coalesced;
      return 'coalesced';
    }
    if (n === this.capacity) {
      if (action === pointerAction.move) { ++this.#droppedMoves; return 'dropped-move'; }
      let firstMove = -1;
      for (let i = 0; i < n; ++i) if (this.#action[i] === pointerAction.move) { firstMove = i; break; }
      if (firstMove < 0) { this.#overflowed = true; return 'overflow'; }
      for (let i = firstMove; i < n-1; ++i) {
        this.#sequence[i] = this.#sequence[i+1]; this.#action[i] = this.#action[i+1];
        this.#pointer[i] = this.#pointer[i+1]; this.#x[i] = this.#x[i+1]; this.#y[i] = this.#y[i+1];
      }
      --this.#count;
      ++this.#evictedMoves;
    }
    const slot = this.#count++;
    this.#sequence[slot] = sequence;
    this.#action[slot] = action;
    this.#pointer[slot] = pointerId;
    this.#x[slot] = Math.fround(x);
    this.#y[slot] = Math.fround(y);
    return 'queued';
  }

  // The caller invokes this immediately before a C step; apply receives only
  // primitive values. A thrown action poisons this epoch to prevent replay.
  drain(apply) {
    if (typeof apply !== 'function') throw new TypeError('Missing pointer sink');
    if (this.#overflowed || this.#draining) throw new Error('Pointer queue requires reset');
    const count = this.#count;
    this.#draining = true;
    try {
      for (let i = 0; i < count; ++i)
        apply(this.#action[i],this.#pointer[i],this.#x[i],this.#y[i],this.#sequence[i]);
      this.#count = 0;
      return count;
    } catch (error) {
      this.#overflowed = true;
      this.#count = 0;
      throw error;
    } finally { this.#draining = false; }
  }
}
