export interface Vec2 {
  x: number;
  y: number;
}

export interface Circle {
  id: string;
  x: number;
  y: number;
  radius: number;
}

export interface Segment {
  ax: number;
  ay: number;
  bx: number;
  by: number;
}

export const TAU = Math.PI * 2;

export function degToRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI;
}

/** Normalize an angle into [0, 2*PI). */
export function normalizeAngle0To2Pi(a: number): number {
  let r = a % TAU;
  if (r < 0) r += TAU;
  return r;
}

/** Normalize an angle into (-PI, PI]. */
export function normalizeAnglePi(a: number): number {
  let r = normalizeAngle0To2Pi(a);
  if (r > Math.PI) r -= TAU;
  return r;
}

/** Angles equal modulo 2*PI. */
export function anglesEquivalent(a: number, b: number, tol = 1e-9): boolean {
  return Math.abs(normalizeAnglePi(a - b)) <= tol;
}

export function distance(ax: number, ay: number, bx: number, by: number): number {
  return Math.hypot(bx - ax, by - ay);
}

/**
 * Signed smallest angular difference b - a in (-PI, PI], handling the
 * +/-PI periodic boundary. Interpolation along this delta never takes the
 * "long way" around the circle.
 */
export function shortestAngleDelta(a: number, b: number): number {
  return normalizeAnglePi(b - a);
}

/**
 * Minimum distance from a circle to a line segment.
 * Segment endpoints are treated as part of the segment, so both joint
 * centres and the full link body are covered.
 */
export function segmentCircleDistance(seg: Segment, c: Circle): number {
  const vx = seg.bx - seg.ax;
  const vy = seg.by - seg.ay;
  const wx = c.x - seg.ax;
  const wy = c.y - seg.ay;
  const len2 = vx * vx + vy * vy;
  let t = len2 === 0 ? 0 : (wx * vx + wy * vy) / len2;
  t = Math.max(0, Math.min(1, t));
  const px = seg.ax + vx * t;
  const py = seg.ay + vy * t;
  return Math.hypot(c.x - px, c.y - py);
}

export function segmentHitsCircle(seg: Segment, c: Circle, margin: number): boolean {
  return segmentCircleDistance(seg, c) <= c.radius + margin;
}
