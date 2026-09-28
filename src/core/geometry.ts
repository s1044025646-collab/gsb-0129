import type { CircleObstacle, Vec2 } from "./types.js";

export const TAU = Math.PI * 2;

export function sub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function add(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}
export function scale(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function dist(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function norm(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

/**
 * Signed shortest distance from a point to a finite segment AB.
 * Returns { d: signed-ish distance, t: parameter in [0,1] }.
 * Distance is always non-negative; callers compare with radius.
 */
export function pointSegmentDistance(p: Vec2, a: Vec2, b: Vec2): { d: number; t: number } {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const apx = p.x - a.x;
  const apy = p.y - a.y;
  const len2 = abx * abx + aby * aby;
  let t = 0;
  if (len2 > 1e-15) {
    t = (apx * abx + apy * aby) / len2;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
  }
  const cx = a.x + abx * t;
  const cy = a.y + aby * t;
  return { d: Math.hypot(p.x - cx, p.y - cy), t };
}

/**
 * Collision test between a capsule (segment AB inflated by linkHalfWidth)
 * and a circle obstacle. Touching exactly on the boundary counts as collision
 * (conservative planning).
 */
export function segmentCircleCollision(
  a: Vec2,
  b: Vec2,
  obs: CircleObstacle,
  clearance = 0,
): boolean {
  const { d } = pointSegmentDistance({ x: obs.x, y: obs.y }, a, b);
  return d <= obs.radius + clearance;
}

/** Clamp helper. */
export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/** Linear interpolation. */
export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/**
 * Wrap an angle into [-pi, pi).
 */
export function wrapAngle(a: number): number {
  let r = a % TAU;
  if (r >= Math.PI) r -= TAU;
  if (r < -Math.PI) r += TAU;
  return r;
}

/**
 * Normalize an angle into the periodic equivalence class closest to `ref`.
 * Useful for joint periodic-boundary handling: keeps interpolation short.
 */
export function nearestEquivalent(a: number, ref: number): number {
  const k = Math.round((ref - a) / TAU);
  return a + k * TAU;
}

export function degToRad(d: number): number {
  return (d * Math.PI) / 180;
}

export function radToDeg(r: number): number {
  return (r * 180) / Math.PI;
}
