/**
 * Deterministic, seedable PRNG (mulberry32).
 * Same seed always yields the same sequence, which makes planning reproducible.
 */
export class Rng {
  private state: number;

  constructor(seed: number) {
    // Force uint32.
    this.state = seed >>> 0;
    if (this.state === 0) this.state = 0x9e3779b9;
  }

  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

 /** Uniform float in [min, max). */
  uniform(min: number, max: number): number {
    return min + (max - min) * this.next();
  }
}
