#!/usr/bin/env python3
"""MechFight / mechfight/1 survival-oriented bot. Standard library only."""
from __future__ import annotations

import json
import math
import os
import select
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def xy(v: Sequence[float]) -> Vec2:
    return (float(v[0]), float(v[1]))


def xyz(v: Sequence[float]) -> Vec3:
    return (float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)


def add2(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def mul2(a: Vec2, k: float) -> Vec2:
    return (a[0] * k, a[1] * k)


def norm2(a: Vec2) -> float:
    return math.hypot(a[0], a[1])


def unit2(a: Vec2) -> Vec2:
    n = norm2(a)
    return (a[0] / n, a[1] / n) if n > 1e-9 else (0.0, 0.0)


def dist3(a: Sequence[float], b: Sequence[float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = (float(a[2]) if len(a) > 2 else 0.0) - (float(b[2]) if len(b) > 2 else 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def point_in_ring(p: Vec2, ring: Sequence[float]) -> bool:
    """Even/odd point-in-polygon test for the map's flattened [x,y,...] rings."""
    n = len(ring) // 2
    if n < 3:
        return False
    inside = False
    x, y = p
    j = n - 1
    for i in range(n):
        xi, yi = float(ring[2 * i]), float(ring[2 * i + 1])
        xj, yj = float(ring[2 * j]), float(ring[2 * j + 1])
        if ((yi > y) != (yj > y)):
            cross_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < cross_x:
                inside = not inside
        j = i
    return inside


def point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return norm2((p[0] - a[0], p[1] - a[1]))
    t = clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den, 0.0, 1.0)
    q = (a[0] + t * dx, a[1] + t * dy)
    return norm2((p[0] - q[0], p[1] - q[1]))


def segments_distance(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> float:
    # Intersection test followed by the four endpoint-to-segment distances.
    def orient(p: Vec2, q: Vec2, r: Vec2) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    eps = 1e-9
    def on_segment(p: Vec2, q: Vec2, r: Vec2) -> bool:
        return (min(p[0], r[0])-eps <= q[0] <= max(p[0], r[0])+eps and
                min(p[1], r[1])-eps <= q[1] <= max(p[1], r[1])+eps)
    intersects = ((o1 > eps and o2 < -eps or o1 < -eps and o2 > eps) and
                  (o3 > eps and o4 < -eps or o3 < -eps and o4 > eps))
    if intersects or (abs(o1) <= eps and on_segment(a, c, b)) or \
       (abs(o2) <= eps and on_segment(a, d, b)) or \
       (abs(o3) <= eps and on_segment(c, a, d)) or \
       (abs(o4) <= eps and on_segment(c, b, d)):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


class WorldMap:
    """Small optional static obstacle model loaded once from /arena/v2/world.json."""
    def __init__(self) -> None:
        self.prisms: List[Dict[str, Any]] = []

    def load(self, map_dir: str) -> None:
        path = os.path.join(map_dir, "v2", "world.json")
        try:
            with open(path, "r", encoding="utf-8") as f:
                world = json.load(f)
            for p in world.get("prisms", []):
                ring = p.get("ring", [])
                if len(ring) < 6:
                    continue
                xs = ring[0::2]
                ys = ring[1::2]
                self.prisms.append({
                    "ring": ring,
                    "holes": p.get("holes", []),
                    "base": float(p.get("base", -1e9)),
                    "top": float(p.get("top", 1e9)),
                    "minx": min(xs), "maxx": max(xs),
                    "miny": min(ys), "maxy": max(ys),
                })
        except (OSError, ValueError, TypeError):
            self.prisms = []

    @staticmethod
    def _solid_at(p: Vec2, ob: Dict[str, Any]) -> bool:
        if not point_in_ring(p, ob["ring"]):
            return False
        return not any(point_in_ring(p, hole) for hole in ob.get("holes", []))

    def segment_clear(self, a: Vec2, b: Vec2, feet_z: float, clearance: float = 3.2) -> bool:
        # Treat a prism as blocking when it overlaps the mech's vertical body.
        body_top = feet_z + 9.0
        minx, maxx = min(a[0], b[0]) - clearance, max(a[0], b[0]) + clearance
        miny, maxy = min(a[1], b[1]) - clearance, max(a[1], b[1]) + clearance
        for ob in self.prisms:
            if ob["top"] <= feet_z or ob["base"] >= body_top:
                continue
            if ob["maxx"] < minx or ob["minx"] > maxx or ob["maxy"] < miny or ob["miny"] > maxy:
                continue
            if self._solid_at(a, ob) or self._solid_at(b, ob):
                return False
            ring = ob["ring"]
            n = len(ring) // 2
            # Outer wall and courtyard-hole walls both obstruct crossing.
            rings = [ring] + list(ob.get("holes", []))
            for r in rings:
                m = len(r) // 2
                for i in range(m):
                    c = (float(r[2 * i]), float(r[2 * i + 1]))
                    d = (float(r[2 * ((i + 1) % m)]), float(r[2 * ((i + 1) % m) + 1]))
                    if segments_distance(a, b, c, d) < clearance:
                        return False
        return True

    def steer(self, start: Vec2, goal: Vec2, feet_z: float) -> Vec2:
        if not self.prisms or self.segment_clear(start, goal, feet_z):
            return unit2((goal[0] - start[0], goal[1] - start[1]))

        # Try short detours just outside corners of nearby blocking prisms.
        direct = (goal[0] - start[0], goal[1] - start[1])
        reach = norm2(direct)
        minx, maxx = min(start[0], goal[0]) - 12, max(start[0], goal[0]) + 12
        miny, maxy = min(start[1], goal[1]) - 12, max(start[1], goal[1]) + 12
        candidates: List[Vec2] = []
        for ob in self.prisms:
            if ob["top"] <= feet_z or ob["base"] >= feet_z + 9.0:
                continue
            if ob["maxx"] < minx or ob["minx"] > maxx or ob["maxy"] < miny or ob["miny"] > maxy:
                continue
            r = ob["ring"]
            cx = sum(float(r[i]) for i in range(0, len(r), 2)) / (len(r) // 2)
            cy = sum(float(r[i]) for i in range(1, len(r), 2)) / (len(r) // 2)
            for i in range(0, len(r), 2):
                vx, vy = float(r[i]), float(r[i + 1])
                out = unit2((vx - cx, vy - cy))
                candidates.append((vx + out[0] * 5.0, vy + out[1] * 5.0))
        best: Optional[Vec2] = None
        best_cost = float("inf")
        for w in candidates:
            if norm2((w[0] - start[0], w[1] - start[1])) > max(100.0, reach + 20.0):
                continue
            if not self.segment_clear(start, w, feet_z) or not self.segment_clear(w, goal, feet_z):
                continue
            cost = norm2((w[0] - start[0], w[1] - start[1])) + norm2((goal[0] - w[0], goal[1] - w[1]))
            if cost < best_cost:
                best_cost, best = cost, w
        if best is not None:
            return unit2((best[0] - start[0], best[1] - start[1]))

        # Last-resort wall-following direction; server collision still slides along walls.
        d = unit2(direct)
        side1, side2 = (-d[1], d[0]), (d[1], -d[0])
        if self.segment_clear(start, (start[0] + side1[0] * 14, start[1] + side1[1] * 14), feet_z):
            return side1
        if self.segment_clear(start, (start[0] + side2[0] * 14, start[1] + side2[1] * 14), feet_z):
            return side2
        return mul2(d, -1.0)


class MechSurvivor:
    def __init__(self) -> None:
        self.config: Dict[str, Any] = {}
        self.world = WorldMap()
        self.scan_yaw = 0.0
        self.wander_angle = 0.0
        self.last_aim = [0.0, 0.0]
        self.last_target_id: Optional[int] = None
        self.tick_dt = 0.1

    def on_hello(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        self.config = msg.get("config", {}) or {}
        map_dir = (msg.get("map") or {}).get("dir", "/arena")
        self.world.load(map_dir)
        self.scan_yaw = 0.0
        return {"type": "ready", "name": "MechSurvivor-Pro"}

    def zone_state(self, obs: Dict[str, Any]) -> Tuple[Vec2, float, bool]:
        z = obs.get("zone", {}) or {}
        center = xy(z.get("center", [0.0, 0.0]))
        next_center = xy(z.get("next_center", center))
        radius = float(z.get("radius", 500.0))
        next_radius = float(z.get("next_radius", radius))
        t = float(obs.get("t", 0.0))
        start = float(z.get("shrink_start", 1e9))
        end = float(z.get("shrink_end", 1e9))
        urgent = False
        if end > start and start <= t <= end:
            a = clamp((t - start) / (end - start), 0.0, 1.0)
            center = (center[0] + (next_center[0] - center[0]) * a,
                      center[1] + (next_center[1] - center[1]) * a)
            radius = radius + (next_radius - radius) * a
            urgent = True
        elif t > end:
            center, radius = next_center, next_radius
        elif start - t < 25.0 and start < 1e8:
            # Prepare for the next circle, but only when outside its comfortable interior.
            p = self._position(obs)
            if norm2((p[0] - next_center[0], p[1] - next_center[1])) > next_radius - 25.0:
                center, radius = next_center, next_radius
                urgent = True
        p = self._position(obs)
        outside = norm2((p[0] - center[0], p[1] - center[1])) > radius - 18.0
        return center, max(0.0, radius), bool(urgent or outside)

    @staticmethod
    def _position(obs: Dict[str, Any]) -> Vec3:
        return xyz(obs.get("self", {}).get("pos", [0.0, 0.0, 0.0]))

    def _enemy_threat(self, me: Dict[str, Any], enemy: Dict[str, Any], d: float) -> float:
        epos = xyz(enemy.get("pos", [0, 0, 0]))
        mpos = xyz(me.get("pos", [0, 0, 0]))
        eaim = enemy.get("aim", [0.0, 0.0])
        bearing_to_me = math.atan2(mpos[1] - epos[1], mpos[0] - epos[0])
        facing = abs(wrap_angle(float(eaim[0]) - bearing_to_me)) if len(eaim) else math.pi
        threat = 0.0
        if facing < 0.55:
            threat += 18.0
        if enemy.get("charging"):
            threat += 18.0
        kinds = enemy.get("weapons", []) or []
        if "lightning" in kinds and d < 55:
            threat += 12.0
        if "mortar" in kinds and d < 120:
            threat += 8.0
        if "laser" in kinds and d < 180:
            threat += 8.0
        return threat

    def choose_target(self, obs: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        me = obs.get("self", {})
        my_pos = me.get("pos", [0, 0, 0])
        best = None
        best_score = float("inf")
        for e in obs.get("mechs", []) or []:
            if e.get("id") == me.get("id") or not e.get("pos"):
                continue
            kind = e.get("kind", "mech")
            if kind not in ("mech", "npc"):
                continue
            d = dist3(my_pos, e["pos"])
            hp = float(e.get("hp", 700.0))
            max_hp = max(1.0, float(e.get("max_hp", 1400.0 if kind == "mech" else 350.0)))
            hp_fraction = clamp(hp / max_hp, 0.0, 1.0)
            score = d + hp_fraction * 18.0 - self._enemy_threat(me, e, d)
            if kind == "npc":
                score += 24.0  # Drones are useful cubes, but players are the immediate placement threat.
            if e.get("id") == self.last_target_id:
                score -= 4.0  # Reduce target switching when scores are close.
            if score < best_score:
                best_score, best = score, e
        if best is not None:
            self.last_target_id = best.get("id")
        return best

    @staticmethod
    def _target_scale(target: Dict[str, Any]) -> float:
        return 1.0 + 0.05 * max(0.0, float(target.get("cubes", 0)))

    def _intercept_time(self, r: Vec3, v: Vec3, speed: float) -> float:
        # Earliest positive solution of |r + v*t| = speed*t.
        a = v[0]*v[0] + v[1]*v[1] + v[2]*v[2] - speed*speed
        b = 2.0 * (r[0]*v[0] + r[1]*v[1] + r[2]*v[2])
        c = r[0]*r[0] + r[1]*r[1] + r[2]*r[2]
        if abs(a) < 1e-9:
            t = -c / b if abs(b) > 1e-9 else 0.0
            return clamp(t, 0.0, 1.8)
        disc = b*b - 4.0*a*c
        if disc < 0.0:
            return clamp(math.sqrt(c) / max(speed, 1.0), 0.0, 1.8)
        root = math.sqrt(disc)
        roots = [t for t in ((-b-root)/(2*a), (-b+root)/(2*a)) if t > 0.0]
        return clamp(min(roots) if roots else math.sqrt(c) / max(speed, 1.0), 0.0, 1.8)

    def aim_at(self, me: Dict[str, Any], target: Dict[str, Any], weapon: Dict[str, Any]) -> Tuple[List[float], float, float]:
        mpos = xyz(me.get("pos", [0, 0, 0]))
        tpos = xyz(target.get("pos", [0, 0, 0]))
        vel = xyz(target.get("vel", [0, 0, 0]))
        kind = weapon.get("kind", "gun")
        my_stats = me.get("stats", {}) or {}
        muzzle_scale = float(my_stats.get("muzzle_height", 7.0)) / 7.0
        muzzle_h = (9.5 if kind == "mortar" else float(my_stats.get("muzzle_height", 7.0))) * (muzzle_scale if kind == "mortar" else 1.0)
        target_scale = self._target_scale(target)
        target_h = (1.75 if target.get("kind") == "npc" else 4.5) * target_scale
        muzzle = (mpos[0], mpos[1], mpos[2] + muzzle_h)
        aim_point = (tpos[0], tpos[1], tpos[2] + target_h)
        stats = weapon.get("stats", {}) or {}
        distance = dist3(mpos, tpos)

        if kind == "gun":
            speed = float(stats.get("speed", 252.0))
            r = (aim_point[0]-muzzle[0], aim_point[1]-muzzle[1], aim_point[2]-muzzle[2])
            flight = self._intercept_time(r, vel, speed)
            aim_point = (aim_point[0] + vel[0]*flight,
                         aim_point[1] + vel[1]*flight,
                         aim_point[2] + vel[2]*flight)
        elif kind == "mortar":
            speed = float(stats.get("speed", 72.0))
            gravity = float(stats.get("gravity", (self.config.get("weapons", {}).get("mortar", {}) or {}).get("gravity", 22.0)))
            # Iterate once on horizontal flight time, then solve the low ballistic arc.
            for _ in range(2):
                dx, dy = aim_point[0]-muzzle[0], aim_point[1]-muzzle[1]
                horizontal = math.hypot(dx, dy)
                dz = aim_point[2]-muzzle[2]
                if horizontal < 1e-3:
                    pitch = math.pi / 4.0
                    flight = 0.2
                else:
                    disc = speed**4 - gravity * (gravity*horizontal*horizontal + 2.0*dz*speed*speed)
                    if disc <= 0.0:
                        pitch = math.radians(43.0)
                    else:
                        tan_pitch = (speed*speed - math.sqrt(disc)) / (gravity*horizontal)
                        pitch = math.atan(tan_pitch)
                    flight = horizontal / max(1.0, speed * math.cos(pitch))
                aim_point = (tpos[0] + vel[0]*flight,
                             tpos[1] + vel[1]*flight,
                             tpos[2] + target_h + vel[2]*flight)
            dx, dy = aim_point[0]-muzzle[0], aim_point[1]-muzzle[1]
            yaw = math.atan2(dy, dx)
            return [yaw, clamp(pitch, -0.2, 1.25)], distance, flight
        else:
            # Laser is hitscan; lightning uses yaw but benefits from small lead while the torso turns.
            cur = me.get("aim", [0.0, 0.0])
            turn_rate = float(my_stats.get("turn_rate", 4.0))
            delay = min(0.35, abs(wrap_angle(math.atan2(aim_point[1]-mpos[1], aim_point[0]-mpos[0]) - float(cur[0]))) / max(turn_rate, 0.1))
            aim_point = (aim_point[0] + vel[0]*delay, aim_point[1] + vel[1]*delay, aim_point[2] + vel[2]*delay)

        dx, dy, dz = aim_point[0]-muzzle[0], aim_point[1]-muzzle[1], aim_point[2]-muzzle[2]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(dz, max(1e-6, math.hypot(dx, dy)))
        pitch = clamp(pitch, float(my_stats.get("min_pitch", -1.2)), float(my_stats.get("max_pitch", 1.2)))
        return [yaw, pitch], distance, 0.0

    def _after_turn_error(self, me: Dict[str, Any], aim: Sequence[float]) -> Tuple[float, float]:
        cur = me.get("aim", [0.0, 0.0])
        stats = me.get("stats", {}) or {}
        yaw_rate = max(0.1, float(stats.get("turn_rate", 4.0)))
        pitch_rate = max(0.1, float(stats.get("pitch_rate", yaw_rate)))
        dyaw = wrap_angle(float(aim[0]) - float(cur[0]))
        dpitch = float(aim[1]) - float(cur[1])
        return (max(0.0, abs(dyaw) - yaw_rate*self.tick_dt),
                max(0.0, abs(dpitch) - pitch_rate*self.tick_dt))

    def choose_weapon_slot(self, me: Dict[str, Any], target: Dict[str, Any], d: float) -> Optional[int]:
        weapons = me.get("weapons", [None, None]) or [None, None]
        kinds = target.get("weapons", []) or []
        best_slot, best_score = None, -1e9
        for slot in range(min(2, len(weapons))):
            w = weapons[slot]
            if not w:
                continue
            kind = w.get("kind", "gun")
            cd = float(w.get("cooldown", 0.0))
            charge = float(w.get("charge", 0.0))
            if kind == "laser" and charge > 0.0:
                return slot
            score = -100.0
            stats = w.get("stats", {}) or {}
            rng = float(stats.get("range", {"gun":230.0, "laser":280.0, "mortar":235.0, "lightning":45.0}.get(kind, 100.0)))
            if kind == "gun":
                ammo = int(w.get("ammo", 0))
                if ammo > 0 and d <= rng:
                    score = 45.0 + (8.0 if d < 100 else 0.0)
            elif kind == "laser" and d <= rng:
                score = 82.0
                if "mortar" in kinds:
                    score += 18.0
                if "lightning" in kinds and d < 50:
                    score -= 28.0
            elif kind == "mortar" and 14.0 < d <= rng:
                score = 68.0 + (20.0 if "lightning" in kinds else 0.0)
            elif kind == "lightning" and d <= rng:
                score = 72.0 + (24.0 if "laser" in kinds else 0.0)
                if float(me.get("energy", 0.0)) < 3.0:
                    score = -100.0
            if score > -90.0 and cd > 0.0:
                score -= min(35.0, cd * 18.0)
            if score > best_score:
                best_score, best_slot = score, slot
        return best_slot

    def _select_upgrade(self, obs: Dict[str, Any]) -> Optional[int]:
        offer = obs.get("offer")
        if not offer:
            return None
        choices = offer.get("choices", []) or []
        if not choices:
            return None
        weapons = [w.get("kind") for w in (obs.get("self", {}).get("weapons", []) or []) if w]
        def rank(c: Dict[str, Any]) -> int:
            label = (str(c.get("id", "")) + " " + str(c.get("name", "")) + " " + str(c.get("desc", ""))).lower()
            score = 0
            if any(k in label for k in ("armor", "damage taken", "resist")):
                score += 100
            if any(k in label for k in ("max hp", "max health", "health", "regen", "repair")):
                score += 90
            if any(k in label for k in ("walk", "move speed", "boost speed", "mobility", "accel")):
                score += 75
            if any(k in label for k in ("energy", "jet", "fuel")):
                score += 65
            if any(k in label for k in ("sensor", "vision", "radar", "range")):
                score += 60
            if "gun" in label and "gun" in weapons:
                score += 82
            if "laser" in label and "laser" in weapons:
                score += 88
            if "mortar" in label and "mortar" in weapons:
                score += 78
            if "lightning" in label and "lightning" in weapons:
                score += 78
            if any(k in label for k in ("damage", "rate", "cooldown", "reload")):
                score += 40
            return score
        return max(range(len(choices)), key=lambda i: rank(choices[i]))

    def _best_goal(self, obs: Dict[str, Any], zone_center: Vec2, zone_radius: float) -> Optional[Vec2]:
        me = obs.get("self", {})
        p = xyz(me.get("pos", [0, 0, 0]))
        my2 = (p[0], p[1])
        enemies = [e for e in (obs.get("mechs", []) or []) if e.get("kind") == "mech" and e.get("pos")]
        nearest_enemy = min((dist3(p, e["pos"]) for e in enemies), default=9999.0)

        def safe(q: Sequence[float]) -> bool:
            return norm2((float(q[0])-zone_center[0], float(q[1])-zone_center[1])) < max(0.0, zone_radius-12.0)

        # A contested supply drop is usually not worth the risk; capture only if alone or already holding it.
        my_id = me.get("id")
        for drop in sorted(obs.get("drops", []) or [], key=lambda d: dist3(p, d.get("pos", [1e9, 1e9, 0]))):
            if float(drop.get("lands_in", 999.0)) > 0.0:
                continue
            holder = drop.get("holder")
            if holder not in (None, my_id):
                continue
            dp = drop.get("pos")
            if not dp or not safe(dp):
                continue
            contest = any(dist3(e["pos"], dp) < 22.0 for e in enemies)
            d = dist3(p, dp)
            if not contest and d < 130.0 and (holder == my_id or nearest_enemy > 28.0):
                return (float(dp[0]), float(dp[1]))

        # Collect nearby cubes only when doing so does not mean walking into an immediate fight.
        cubes = []
        if nearest_enemy > 16.0:
            for c in obs.get("cubes", []) or []:
                cp = c.get("pos")
                if cp and safe(cp):
                    d = dist3(p, cp)
                    if d < 48.0:
                        cubes.append((d, cp))
        if cubes:
            cp = min(cubes, key=lambda x: x[0])[1]
            return (float(cp[0]), float(cp[1]))

        # Fill the empty secondary slot with the best nearby weapon.
        weapons = me.get("weapons", []) or []
        has_secondary = len(weapons) > 1 and weapons[1] is not None
        if not has_secondary and nearest_enemy > 22.0:
            preference = {"laser": 0, "mortar": 1, "lightning": 2}
            pickups = []
            for item in obs.get("pickups", []) or []:
                wp = item.get("pos")
                kind = item.get("kind")
                if wp and kind in preference and safe(wp):
                    d = dist3(p, wp)
                    if d < 55.0:
                        pickups.append((d, preference[kind], wp))
            if pickups:
                chosen = min(pickups, key=lambda x: (x[0] - 3.0*x[1]))
                return (float(chosen[2][0]), float(chosen[2][1]))
        return None

    def _projectile_evasion(self, obs: Dict[str, Any]) -> Tuple[Vec2, float]:
        me = obs.get("self", {})
        p = xyz(me.get("pos", [0, 0, 0]))
        v = xyz(me.get("vel", [0, 0, 0]))
        gravity = float((self.config.get("weapons", {}).get("mortar", {}) or {}).get("gravity", 22.0))
        evasion = (0.0, 0.0)
        danger = 0.0
        for shot in obs.get("projectiles", []) or []:
            sp = shot.get("pos")
            sv = shot.get("vel")
            if not sp or not sv:
                continue
            sp3, sv3 = xyz(sp), xyz(sv)
            mortar = shot.get("kind") == "mortar"
            best_d2, best_t, best_rel = 1e18, 0.0, (0.0, 0.0)
            for step in range(1, 11):
                t = step * 0.1
                sx = sp3[0] + sv3[0]*t
                sy = sp3[1] + sv3[1]*t
                sz = sp3[2] + sv3[2]*t - (0.5*gravity*t*t if mortar else 0.0)
                mx, my, mz = p[0]+v[0]*t, p[1]+v[1]*t, p[2]+v[2]*t
                dx, dy, dz = sx-mx, sy-my, sz-mz
                # Treat the mech as a vertical cylinder, not a point.
                if -1.0 <= dz <= 10.0:
                    d2 = dx*dx + dy*dy
                    if d2 < best_d2:
                        best_d2, best_t, best_rel = d2, t, (dx, dy)
            if best_d2 < 8.0*8.0:
                severity = (8.0 - math.sqrt(best_d2)) / 8.0
                away = unit2((-best_rel[0], -best_rel[1]))
                evasion = add2(evasion, mul2(away, severity * (1.0 + 0.8 * (1.0-best_t))))
                danger = max(danger, severity)
        return evasion, danger

    def _movement(self, obs: Dict[str, Any], zone_center: Vec2, zone_radius: float, urgent_zone: bool) -> Tuple[List[float], bool, bool]:
        me = obs.get("self", {})
        p = xyz(me.get("pos", [0, 0, 0]))
        my2 = (p[0], p[1])
        energy = float(me.get("energy", 0.0))
        goal = self._best_goal(obs, zone_center, zone_radius)
        to_zone = (zone_center[0] - p[0], zone_center[1] - p[1])
        zone_dist = norm2(to_zone)
        outside = zone_dist > zone_radius - 18.0

        if outside or urgent_zone:
            desired = unit2(to_zone)
            # If we're only preparing for the next circle, don't abandon a close safe loot target.
            if goal is not None and not outside and zone_dist < zone_radius - 35.0:
                desired = unit2((goal[0]-p[0], goal[1]-p[1]))
        elif goal is not None:
            desired = self.world.steer(my2, goal, p[2])
        else:
            # Patrol an inner orbit instead of standing still; this avoids camping at the edge.
            self.wander_angle = wrap_angle(self.wander_angle + 0.045)
            orbit_r = min(150.0, max(18.0, zone_radius * 0.38))
            orbit = (zone_center[0] + math.cos(self.wander_angle)*orbit_r,
                     zone_center[1] + math.sin(self.wander_angle)*orbit_r)
            if norm2((orbit[0]-p[0], orbit[1]-p[1])) > 8.0:
                desired = self.world.steer(my2, orbit, p[2])
            else:
                desired = unit2((-math.sin(self.wander_angle), math.cos(self.wander_angle)))

        # Repel from nearby enemy mechs. Keep a little separation rather than blindly chasing.
        repulse = (0.0, 0.0)
        closest = 9999.0
        for e in obs.get("mechs", []) or []:
            if e.get("kind") != "mech" or e.get("id") == me.get("id") or not e.get("pos"):
                continue
            ep = xyz(e["pos"])
            delta = (p[0]-ep[0], p[1]-ep[1])
            d = max(0.1, norm2(delta))
            closest = min(closest, d)
            if d < 85.0:
                strength = ((85.0-d)/85.0)**2
                repulse = add2(repulse, mul2(delta, strength / d))
        if norm2(repulse) > 0:
            desired = unit2(add2(desired, mul2(unit2(repulse), 1.9 if closest < 35.0 else 1.0)))

        evasion, projectile_danger = self._projectile_evasion(obs)
        if projectile_danger > 0.0:
            desired = unit2(add2(desired, mul2(unit2(evasion), 2.4)))

        move = unit2(desired)
        urgent = outside or projectile_danger > 0.25 or closest < 24.0
        boost_threshold = 8.0 if outside else 24.0 if urgent else 78.0 if goal is not None else 95.0
        boost = bool(norm2(move) > 0.05 and energy > boost_threshold)
        # Jump only for an imminent projectile close to the body; lateral evasion remains primary.
        jump = bool(me.get("grounded", False) and projectile_danger > 0.88 and closest > 8.0)
        return [move[0], move[1]], boost, jump

    def _aim_without_target(self, obs: Dict[str, Any]) -> List[float]:
        me = obs.get("self", {})
        actual = me.get("aim", self.last_aim)
        desired_yaw: Optional[float] = None
        # Damage bearing is the best immediate defensive cue.
        for event in reversed(obs.get("events", []) or []):
            if event.get("type") == "damaged" and event.get("bearing") is not None:
                desired_yaw = float(event["bearing"])
                break
        if desired_yaw is None:
            sounds = [s for s in (obs.get("sounds", []) or []) if s.get("bearing") is not None and float(s.get("age", 99.0)) < 0.8]
            if sounds:
                weight = {"mortar": 5, "laser": 4, "lightning": 4, "explosion": 3, "gun": 2, "npc_gun": 1, "jet": 1}
                sound = min(sounds, key=lambda s: (float(s.get("age", 0.0)), -weight.get(s.get("kind", ""), 0)))
                desired_yaw = float(sound["bearing"])
        if desired_yaw is None:
            if self.scan_yaw == 0.0:
                self.scan_yaw = float(actual[0])
            self.scan_yaw = wrap_angle(self.scan_yaw + 0.32)
            desired_yaw = self.scan_yaw
        return [wrap_angle(desired_yaw), 0.0]

    def decide(self, obs: Dict[str, Any]) -> Dict[str, Any]:
        me = obs.get("self", {}) or {}
        tick = int(obs.get("tick", 0))
        zone_center, zone_radius, urgent_zone = self.zone_state(obs)
        move, boost, jump = self._movement(obs, zone_center, zone_radius, urgent_zone)
        target = self.choose_target(obs)
        weapons = me.get("weapons", [None, None]) or [None, None]
        if len(weapons) < 2:
            weapons = list(weapons) + [None] * (2-len(weapons))
        fire = [False, False]
        reload_gun = False
        aim = self._aim_without_target(obs)
        selected_slot: Optional[int] = None
        target_distance = 1e9

        if target is not None:
            target_distance = dist3(me.get("pos", [0,0,0]), target.get("pos", [0,0,0]))
            selected_slot = self.choose_weapon_slot(me, target, target_distance)
            if selected_slot is None:
                selected_slot = next((i for i, w in enumerate(weapons[:2]) if w), None)
            if selected_slot is not None and weapons[selected_slot]:
                weapon = weapons[selected_slot]
                aim, _, _ = self.aim_at(me, target, weapon)
                yaw_err, pitch_err = self._after_turn_error(me, aim)
                aligned = math.hypot(yaw_err, pitch_err) < 0.14
                kind = weapon.get("kind", "gun")
                stats = weapon.get("stats", {}) or {}
                max_range = float(stats.get("range", {"gun":230.0, "laser":280.0, "mortar":235.0, "lightning":45.0}.get(kind, 100.0)))
                cooldown = float(weapon.get("cooldown", 0.0))
                if kind == "gun":
                    ammo = int(weapon.get("ammo", 0))
                    if ammo <= 0:
                        reload_gun = float(weapon.get("reload", 0.0)) <= 0.0
                    elif cooldown <= 0.0 and target_distance <= max_range and aligned:
                        fire[selected_slot] = True
                elif kind == "mortar":
                    if cooldown <= 0.0 and 15.0 < target_distance < min(max_range, 220.0) and aligned:
                        # Avoid own splash at close range.
                        fire[selected_slot] = True
                elif kind == "lightning":
                    yaw_only_ok = yaw_err < math.radians(13.0)
                    if cooldown <= 0.0 and target_distance <= min(max_range, 45.0) and yaw_only_ok and float(me.get("energy", 0.0)) >= 3.0:
                        fire[selected_slot] = True
                elif kind == "laser":
                    charge = float(weapon.get("charge", 0.0))
                    if charge > 0.0:
                        # Charge while tracking; release on a good line or near full charge.
                        fire[selected_slot] = not ((charge >= 0.48 and aligned and yaw_err < 0.07) or charge >= 0.72)
                    elif cooldown <= 0.0 and target_distance <= max_range:
                        fire[selected_slot] = True

        # A charging laser with no target must be released rather than charging indefinitely.
        for i, w in enumerate(weapons[:2]):
            if w and w.get("kind") == "laser" and float(w.get("charge", 0.0)) > 0.0 and target is None:
                fire[i] = False

        # Reload the arm gun early when it is nearly empty and there is no immediate shot.
        for i, w in enumerate(weapons[:2]):
            if w and w.get("kind") == "gun":
                ammo = int(w.get("ammo", 0))
                if float(w.get("reload", 0.0)) <= 0.0 and ammo <= 2 and (target is None or target_distance > 45.0):
                    reload_gun = True

        pick = self._select_upgrade(obs)
        action: Dict[str, Any] = {
            "tick": tick,
            "move": move,
            "boost": boost,
            "jump": jump,
            "aim": [float(aim[0]), float(aim[1])],
            "fire": fire,
            "reload": reload_gun,
        }
        if pick is not None:
            action["pick"] = pick

        # Take a weapon only when the secondary slot is empty and the pickup is in interaction range.
        if len(weapons) > 1 and weapons[1] is None:
            p = xyz(me.get("pos", [0,0,0]))
            best_pickup = None
            best_score = 1e9
            pref = {"laser": 0, "mortar": 1, "lightning": 2}
            for item in obs.get("pickups", []) or []:
                wp = item.get("pos")
                if not wp or item.get("kind") not in pref:
                    continue
                dxy = norm2((float(wp[0])-p[0], float(wp[1])-p[1]))
                dz = abs(float(wp[2] if len(wp)>2 else 0.0)-p[2])
                if dxy <= 4.0 and dz <= 4.0:
                    score = dxy + dz + pref[item["kind"]]*0.1
                    if score < best_score:
                        best_score, best_pickup = score, item
            if best_pickup is not None:
                action["take"] = int(best_pickup["id"])

        self.last_aim = list(action["aim"])
        return action


def encode(obj: Dict[str, Any]) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def main() -> None:
    bot = MechSurvivor()
    fd = sys.stdin.fileno()
    try:
        os.set_blocking(fd, False)
    except (AttributeError, OSError):
        pass
    buffer = bytearray()
    eof = False

    while True:
        if b"\n" not in buffer and not eof:
            ready, _, _ = select.select([fd], [], [])
            if not ready:
                continue
        # Read all immediately available bytes so we can collapse a backlog to the newest obs.
        while True:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                break
            except OSError:
                eof = True
                break
            if not chunk:
                eof = True
                break
            buffer.extend(chunk)
            if len(chunk) < 65536:
                # One more nonblocking read below catches any bytes already queued.
                continue
        messages: List[Dict[str, Any]] = []
        while True:
            nl = buffer.find(b"\n")
            if nl < 0:
                break
            raw = bytes(buffer[:nl])
            del buffer[:nl+1]
            if not raw:
                continue
            try:
                value = json.loads(raw)
                if isinstance(value, dict):
                    messages.append(value)
            except (ValueError, UnicodeDecodeError):
                continue
        if eof and buffer:
            try:
                value = json.loads(bytes(buffer))
                if isinstance(value, dict):
                    messages.append(value)
            except (ValueError, UnicodeDecodeError):
                pass
            buffer.clear()

        # Handle control messages in order; for a burst of observations, answer only the newest.
        obs_batch: List[Dict[str, Any]] = []
        should_exit = False
        for msg in messages:
            typ = msg.get("type")
            if typ == "hello":
                try:
                    print(encode(bot.on_hello(msg)), flush=True)
                except Exception as exc:
                    print("hello error: " + repr(exc), file=sys.stderr, flush=True)
                    print(encode({"type": "ready", "name": "MechSurvivor-Pro"}), flush=True)
            elif typ == "obs":
                obs_batch.append(msg)
            elif typ == "end":
                should_exit = True

        if obs_batch:
            observation = obs_batch[-1]
            try:
                answer = bot.decide(observation)
            except Exception as exc:
                print("decision error: " + repr(exc), file=sys.stderr, flush=True)
                me = observation.get("self", {}) or {}
                actual = me.get("aim", [0.0, 0.0])
                answer = {"tick": int(observation.get("tick", 0)), "move": [0.0, 0.0],
                          "boost": False, "jump": False, "aim": actual,
                          "fire": [False, False], "reload": False}
            try:
                print(encode(answer), flush=True)
            except (TypeError, ValueError):
                fallback = {"tick": int(observation.get("tick", 0)), "move": [0.0, 0.0],
                            "aim": [0.0, 0.0], "fire": [False, False]}
                print(encode(fallback), flush=True)

        if should_exit or eof:
            break


if __name__ == "__main__":
    main()
