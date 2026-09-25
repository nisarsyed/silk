// Host scheduling only. Physics always receives the same binary32 fixed step.
export const fixedStepSeconds = Math.fround(1/60);
export const catchUpStepsMax = 8;

export class FixedClock {
  #lastMs = null;
  #paused = false;
  #manual = 0;
  #debt = 0;
  #dropped = 0;
  #suspended = 0;
  #steps = 0;

  get paused() { return this.#paused; }
  get totalSteps() { return this.#steps; }
  get debtSeconds() { return this.#debt; }
  get droppedSeconds() { return this.#dropped; }
  get suspendedSeconds() { return this.#suspended; }
  get interpolation() { return this.#debt/fixedStepSeconds; }

  #time(nowMs) {
    if (!Number.isFinite(nowMs) || nowMs < 0 || nowMs > Number.MAX_SAFE_INTEGER ||
        (this.#lastMs !== null && nowMs < this.#lastMs)) {
      throw new RangeError('Invalid or backwards host clock');
    }
  }

  setPaused(paused, nowMs) {
    if (typeof paused !== 'boolean') throw new TypeError('Invalid pause state');
    this.#time(nowMs);
    if (paused === this.#paused) return false;
    if (this.#lastMs !== null) this.#suspended += (nowMs-this.#lastMs)/1000;
    this.#paused = paused;
    this.#lastMs = nowMs;
    if (!paused) this.#manual = 0;
    return true;
  }

  requestSingleStep() {
    if (!this.#paused || this.#manual === catchUpStepsMax) return false;
    ++this.#manual;
    return true;
  }

  reset(nowMs) {
    this.#time(nowMs);
    this.#lastMs = nowMs;
    this.#manual = 0;
    this.#debt = 0;
    this.#dropped = 0;
    this.#suspended = 0;
    this.#steps = 0;
  }

  // Returns the number of C steps to execute before this presentation. No
  // per-frame objects or wall-clock reads are needed in the scheduling path.
  tick(nowMs) {
    this.#time(nowMs);
    if (this.#lastMs === null) { this.#lastMs = nowMs; return 0; }
    const delta = (nowMs-this.#lastMs)/1000;
    if (this.#paused) {
      const manual = this.#manual;
      if (!Number.isSafeInteger(this.#steps+manual)) throw new RangeError('Host step count exhausted');
      this.#lastMs = nowMs;
      this.#suspended += delta;
      this.#manual = 0;
      this.#steps += manual;
      return manual;
    }
    const accumulated = this.#debt+delta;
    let possible = Math.floor(accumulated/fixedStepSeconds);
    if (!Number.isSafeInteger(possible)) throw new RangeError('Host elapsed range exhausted');
    if (possible*fixedStepSeconds > accumulated) --possible;
    let debt = accumulated-possible*fixedStepSeconds;
    if (debt >= fixedStepSeconds) { ++possible; debt -= fixedStepSeconds; }
    const steps = Math.min(catchUpStepsMax, possible);
    const dropped = this.#dropped+(possible-steps)*fixedStepSeconds;
    if (!Number.isSafeInteger(this.#steps+steps) || !Number.isFinite(dropped) ||
        debt < 0 || debt >= fixedStepSeconds) throw new RangeError('Invalid host step accounting');
    this.#lastMs = nowMs;
    this.#debt = debt;
    this.#dropped = dropped;
    this.#steps += steps;
    return steps;
  }
}
