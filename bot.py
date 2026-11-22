import sys
import os
import json
import math
import struct
import select


class ArenaMap:
    def __init__(self, map_info):
        self.terrain = None
        self.nx = 0
        self.ny = 0
        self.min_x = 0.0
        self.min_y = 0.0
        self.step = 1.0
        self.prisms = []
        self.blocked = set()

        if not map_info:
            return
        mdir = map_info.get("dir", "/arena")
        wpath = os.path.join(mdir, "v2", "world.json")
        tpath = os.path.join(mdir, "v2", "terrain.i16")

        try:
            with open(wpath, "r") as f:
                w = json.load(f)
            ti = w.get("terrain", {})
            self.nx = ti.get("nx", 1070)
            self.ny = ti.get("ny", 724)
            self.min_x = ti.get("minX", 0.0)
            self.min_y = ti.get("minY", 0.0)
            self.step = ti.get("step", 1.0)

            if os.path.exists(tpath):
                with open(tpath, "rb") as tf:
                    raw = tf.read()
                cnt = self.nx * self.ny
                if len(raw) >= cnt * 2:
                    self.terrain = struct.unpack(f"<{cnt}h", raw[: cnt * 2])

            for p in w.get("prisms", []):
                ring = p.get("ring", [])
                if len(ring) < 6:
                    continue
                xs, ys = ring[0::2], ring[1::2]
                prism = {
                    "min_x": min(xs),
                    "max_x": max(xs),
                    "min_y": min(ys),
                    "max_y": max(ys),
                    "base": p.get("base", -100.0),
                    "top": p.get("top", 100.0),
                    "ring": ring,
                    "holes": p.get("holes", []),
                }
                self.prisms.append(prism)
                self._rasterize_prism(prism)
        except Exception:
            pass

    def ground_z(self, x, y):
        if not self.terrain:
            return 0.0
        ix = max(0, min(self.nx - 1, int((x - self.min_x) / self.step)))
        iy = max(0, min(self.ny - 1, int((y - self.min_y) / self.step)))
        return self.terrain[iy * self.nx + ix] * 0.01

    @staticmethod
    def _in_poly(x, y, ring):
        inside = False
        n = len(ring) // 2
        j = n - 1
        for i in range(n):
            xi, yi = ring[2 * i], ring[2 * i + 1]
            xj, yj = ring[2 * j], ring[2 * j + 1]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi):
                inside = not inside
            j = i
        return inside

    def _point_in_prism(self, x, y, z, p):
        if z < p["base"] or z > p["top"]:
            return False
        if x < p["min_x"] or x > p["max_x"] or y < p["min_y"] or y > p["max_y"]:
            return False
        if not self._in_poly(x, y, p["ring"]):
            return False
        for h in p["holes"]:
            if len(h) >= 6 and self._in_poly(x, y, h):
                return False
        return True

    def _rasterize_prism(self, p):
        gs = 6.0
        gx0 = int(math.floor((p["min_x"] - 2.6) / gs))
        gx1 = int(math.ceil((p["max_x"] + 2.6) / gs))
        gy0 = int(math.floor((p["min_y"] - 2.6) / gs))
        gy1 = int(math.ceil((p["max_y"] + 2.6) / gs))
        for gx in range(gx0, gx1 + 1):
            cx = (gx + 0.5) * gs
            for gy in range(gy0, gy1 + 1):
                cy = (gy + 0.5) * gs
                gz = self.ground_z(cx, cy)
                if gz + 8.5 > p["base"] + 1.2 and gz < p["top"]:
                    if self._point_in_prism(cx, cy, gz + 3.0, p) or self._point_in_prism(cx, cy, gz + 7.5, p):
                        self.blocked.add((gx, gy))

    def is_blocked(self, x, y):
        if not self.blocked:
            return False
        return (int(math.floor(x / 6.0)), int(math.floor(y / 6.0))) in self.blocked

    def has_los(self, p1, p2):
        if not self.prisms and not self.terrain:
            return True
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 4.0:
            return True
        steps = min(28, max(4, int(dist / 6.0)))
        min_x, max_x = min(p1[0], p2[0]), max(p1[0], p2[0])
        min_y, max_y = min(p1[1], p2[1]), max(p1[1], p2[1])
        min_z, max_z = min(p1[2], p2[2]), max(p1[2], p2[2])

        cand = [
            p for p in self.prisms
            if p["max_x"] >= min_x and p["min_x"] <= max_x
            and p["max_y"] >= min_y and p["min_y"] <= max_y
            and p["top"] >= min_z and p["base"] <= max_z
        ]

        for i in range(1, steps):
            t = i / steps
            x = p1[0] + dx * t
            y = p1[1] + dy * t
            z = p1[2] + dz * t
            if self.terrain and z < self.ground_z(x, y) - 0.3:
                return False
            for p in cand:
                if self._point_in_prism(x, y, z, p):
                    return False
        return True


class SentinelBot:
    def __init__(self, hello):
        self.cfg = hello.get("config", {})
        self.map = ArenaMap(hello.get("map", {}))
        self.tick = 0
        self.t = 0.0
        self.alive = 16
        self.me = {}
        self.zone = {}
        self.enemies = {}
        self.npcs = {}
        self.pickups = []
        self.cubes = []
        self.crates = []
        self.drops = []
        self.projectiles = []
        self.sounds = []
        self.events = []
        self.offer = None
        self.last_pos = None
        self.stuck = 0
        self.strafe_sign = 1.0
        self.turn_sign = 1.0

    def update(self, obs):
        self.tick = obs.get("tick", 0)
        self.t = obs.get("t", 0.0)
        self.alive = obs.get("alive", 16)
        self.me = obs.get("self", {})
        self.zone = obs.get("zone", {})
        self.offer = obs.get("offer")
        self.pickups = obs.get("pickups", [])
        self.cubes = obs.get("cubes", [])
        self.crates = obs.get("crates", [])
        self.drops = obs.get("drops", [])
        self.projectiles = obs.get("projectiles", [])
        self.sounds = obs.get("sounds", [])
        self.events = obs.get("events", [])

        pos = self.me.get("pos")
        if pos and self.last_pos:
            if math.hypot(pos[0] - self.last_pos[0], pos[1] - self.last_pos[1]) < 0.25:
                self.stuck += 1
                if self.stuck % 8 == 0:
                    self.turn_sign *= -1.0
            else:
                self.stuck = 0
        if pos:
            self.last_pos = list(pos)

        if self.tick % 25 == 0:
            self.strafe_sign *= -1.0

        for m in obs.get("mechs", []):
            mid = m.get("id")
            if mid is None:
                continue
            m["seen_t"] = self.t
            if m.get("kind") == "mech":
                self.enemies[mid] = m
            elif m.get("kind") == "npc":
                self.npcs[mid] = m

        self.enemies = {k: v for k, v in self.enemies.items() if self.t - v.get("seen_t", 0) <= 4.0}
        self.npcs = {k: v for k, v in self.npcs.items() if self.t - v.get("seen_t", 0) <= 3.0}

    @staticmethod
    def d2(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def pick_upgrade(self, offer):
        choices = offer.get("choices", [])
        order = ["armor", "hp", "regen", "speed", "boost", "energy", "damage", "rate", "sensors"]
        best_i, best_s = 0, -1
        for i, c in enumerate(choices):
            cid = c.get("id", "").lower()
            s = 1
            for r, key in enumerate(reversed(order)):
                if key in cid:
                    s = r + 10
                    break
            if s > best_s:
                best_s, best_i = s, i
        return best_i

    def aim_at(self, tpos, tvel, th, wk, ws):
        pos = self.me["pos"]
        mh = self.me.get("stats", {}).get("muzzle_height", 7.0) + (2.5 if wk == "mortar" else 0.0)
        mx, my, mz = pos[0], pos[1], pos[2] + mh
        tx, ty, tz = tpos[0], tpos[1], tpos[2] + th * 0.55
        dx, dy, dz = tx - mx, ty - my, tz - mz
        dist = max(math.hypot(dx, dy), 0.1)

        if wk == "gun":
            spd = max(ws.get("speed", 252.0), 1.0)
            tf = dist / spd
            px, py, pz = tx + tvel[0] * tf - mx, ty + tvel[1] * tf - my, tz + tvel[2] * tf - mz
            return math.atan2(py, px), math.atan2(pz, max(math.hypot(px, py), 0.1))
        if wk == "mortar":
            spd = max(ws.get("speed", 72.0), 1.0)
            g = ws.get("gravity", 22.0)
            tf = dist / (spd * 0.78)
            px, py, pz = tx + tvel[0] * tf - mx, ty + tvel[1] * tf - my, tz + tvel[2] * tf - mz
            r = max(math.hypot(px, py), 0.1)
            v2 = spd * spd
            disc = v2 * v2 - g * (g * r * r + 2.0 * pz * v2)
            pitch = math.atan((v2 - math.sqrt(disc)) / (g * r)) if disc >= 0 else 0.7854
            return math.atan2(py, px), pitch
        return math.atan2(dy, dx), math.atan2(dz, dist)

    def steer(self, pos, vx, vy):
        mag = math.hypot(vx, vy)
        if mag < 1e-4:
            return 0.0, 0.0, False
        ux, uy = vx / mag, vy / mag
        yaw = math.atan2(uy, ux)
        if self.stuck < 4 and not self.map.is_blocked(pos[0] + ux * 5.0, pos[1] + uy * 5.0) and not self.map.is_blocked(pos[0] + ux * 10.0, pos[1] + uy * 10.0):
            return ux, uy, False
        s = self.turn_sign
        for ang in (0.45*s, -0.45*s, 0.9*s, -0.9*s, 1.4*s, -1.4*s, 2.0*s, -2.0*s, 3.14):
            ty = yaw + ang
            tx, tz = math.cos(ty), math.sin(ty)
            if not self.map.is_blocked(pos[0] + tx * 5.0, pos[1] + tz * 5.0) and not self.map.is_blocked(pos[0] + tx * 10.0, pos[1] + tz * 10.0):
                return tx, tz, self.stuck >= 6
        return ux, uy, True

    def step(self):
        act = {"tick": self.tick, "move": [0.0, 0.0], "boost": False, "jump": False, "fire": [False, False], "reload": False}
        try:
            me = self.me
            pos = me.get("pos")
            if not pos:
                return act

            hp_ratio = me.get("hp", 1400) / max(me.get("max_hp", 1400), 1)
            energy = me.get("energy", 100)
            grounded = me.get("grounded", True)
            in_zone = me.get("in_zone", True)
            weapons = list(me.get("weapons", [None, None]))
            while len(weapons) < 2:
                weapons.append(None)

            if self.offer:
                act["pick"] = self.pick_upgrade(self.offer)

            held = [w["kind"] for w in weapons if w]
            empty = [i for i, w in enumerate(weapons) if w is None]

            near_pickup = None
            for p in self.pickups:
                pk = p.get("kind")
                if pk in held:
                    continue
                d = self.d2(pos, p["pos"])
                dz = abs(pos[2] - p["pos"][2])
                if d <= 3.8 and dz <= 3.8:
                    if empty:
                        act["take"] = p["id"]
                        break
                    for idx, w in enumerate(weapons):
                        if w and w["kind"] == "gun":
                            act["take"] = p["id"]
                            act["drop"] = idx
                            break
                elif empty and d < 28.0 and dz <= 4.0:
                    near_pickup = p

            eye = [pos[0], pos[1], pos[2] + me.get("stats", {}).get("eye_height", 8.0)]
            vis_enemies = []
            nearest_any = None
            nearest_any_d = 9999.0

            for e in self.enemies.values():
                d = self.d2(pos, e["pos"])
                if d < nearest_any_d:
                    nearest_any_d = d
                    nearest_any = e
                t_center = [e["pos"][0], e["pos"][1], e["pos"][2] + 5.0]
                if d <= 200.0 and self.map.has_los(eye, t_center):
                    vis_enemies.append((d, e))

            vis_enemies.sort(key=lambda x: x[0])

            zc = self.zone.get("next_center", self.zone.get("center", [0.0, 0.0]))
            zr = self.zone.get("next_radius", self.zone.get("radius", 300.0))
            zdist = self.d2(pos, zc)
            zone_urgent = (not in_zone) or (zdist > zr * 0.85)
            healing_mode = hp_ratio < 0.72 and self.t > 15.0

            anchor_x, anchor_y = zc[0], zc[1]
            if nearest_any and zr > 45.0:
                ax, ay = pos[0] - nearest_any["pos"][0], pos[1] - nearest_any["pos"][1]
                ad = max(math.hypot(ax, ay), 0.1)
                safe_off = min(zr * 0.45, 70.0)
                anchor_x = zc[0] + (ax / ad) * safe_off
                anchor_y = zc[1] + (ay / ad) * safe_off

            gx, gy = anchor_x, anchor_y

            if zone_urgent:
                gx, gy = zc[0], zc[1]
                if energy > 35 and zdist > 18.0:
                    act["boost"] = True
            elif healing_mode and nearest_any and nearest_any_d < 95.0:
                gx, gy = anchor_x, anchor_y
                if energy > 30 and nearest_any_d < 55.0:
                    act["boost"] = True
            elif self.cubes:
                nc = min(self.cubes, key=lambda c: self.d2(pos, c["pos"]))
                if self.d2(pos, nc["pos"]) < 32.0 and (not nearest_any or nearest_any_d > 45.0):
                    gx, gy = nc["pos"][0], nc["pos"][1]
            elif near_pickup and (not nearest_any or nearest_any_d > 45.0):
                gx, gy = near_pickup["pos"][0], near_pickup["pos"][1]
            elif vis_enemies:
                ed, e = vis_enemies[0]
                ex, ey = e["pos"][0] - pos[0], e["pos"][1] - pos[1]
                ux, uy = ex / max(ed, 0.1), ey / max(ed, 0.1)
                px, py = -uy * self.strafe_sign, ux * self.strafe_sign
                ideal = 24.0 if "lightning" in held else 52.0
                if ed < ideal - 8.0:
                    gx, gy = pos[0] - ux * 16.0 + px * 18.0, pos[1] - uy * 16.0 + py * 18.0
                elif ed > ideal + 14.0 and self.alive <= 6 and hp_ratio > 0.75:
                    gx, gy = pos[0] + ux * 14.0 + px * 16.0, pos[1] + uy * 14.0 + py * 16.0
                else:
                    gx, gy = pos[0] + px * 20.0, pos[1] + py * 20.0
            else:
                if self.d2(pos, [anchor_x, anchor_y]) > 20.0:
                    gx, gy = anchor_x, anchor_y
                else:
                    ang = self.t * 0.4
                    gx, gy = anchor_x + math.cos(ang) * 14.0, anchor_y + math.sin(ang) * 14.0

            dx_dodge, dy_dodge = 0.0, 0.0
            for pr in self.projectiles:
                pp, pv = pr["pos"], pr["vel"]
                rx, ry = pos[0] - pp[0], pos[1] - pp[1]
                pd = math.hypot(rx, ry)
                ps = math.hypot(pv[0], pv[1])
                if pd < 30.0 and ps > 1.0:
                    vx, vy = pv[0] / ps, pv[1] / ps
                    if rx * vx + ry * vy > 0:
                        px, py = -vy, vx
                        side = 1.0 if (rx * px + ry * py) >= 0 else -1.0
                        dx_dodge += px * side * 24.0
                        dy_dodge += py * side * 24.0
                        if energy > 25 and pd < 15.0:
                            act["boost"] = True

            sx, sy, need_jump = self.steer(pos, (gx - pos[0]) + dx_dodge, (gy - pos[1]) + dy_dodge)
            act["move"] = [round(sx, 4), round(sy, 4)]
            if need_jump and energy > 20:
                if grounded:
                    act["jump"] = True
                else:
                    act["boost"] = True

            target = None
            th = 9.0
            is_crate = False

            if vis_enemies:
                ed, e = vis_enemies[0]
                if not healing_mode or ed < 80.0:
                    target = e
            elif not healing_mode and (not nearest_any or nearest_any_d > 85.0):
                for n in sorted(self.npcs.values(), key=lambda x: self.d2(pos, x["pos"])):
                    nd = self.d2(pos, n["pos"])
                    if nd < 65.0 and self.map.has_los(eye, [n["pos"][0], n["pos"][1], n["pos"][2] + 2.0]):
                        target, th = n, 3.5
                        break
                if not target and me.get("cubes", 0) < 8:
                    for c in sorted(self.crates, key=lambda x: self.d2(pos, x["pos"])):
                        cd = self.d2(pos, c["pos"])
                        if cd < 32.0 and self.map.has_los(eye, [c["pos"][0], c["pos"][1], c["pos"][2] + 1.5]):
                            target, th, is_crate = c, 2.0, True
                            break

            if target:
                tpos = target["pos"]
                tvel = target.get("vel", [0.0, 0.0, 0.0])
                tdist = self.d2(pos, tpos)
                cur_aim = me.get("aim", [0.0, 0.0])
                aim_y, aim_p = None, None

                for idx in (0, 1):
                    w = weapons[idx]
                    if not w:
                        continue
                    wk = w.get("kind", "gun")
                    ws = w.get("stats", {})
                    cd = w.get("cooldown", 0.0)

                    if wk == "gun":
                        ammo = w.get("ammo", 12)
                        if ammo == 0 or (ammo <= 3 and tdist > 85.0 and w.get("reload", 0) == 0):
                            act["reload"] = True

                    if aim_y is None and cd <= 0.25:
                        aim_y, aim_p = self.aim_at(tpos, tvel, th, wk, ws)

                    yerr = abs((aim_y - cur_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_y is not None else 1.0

                    if wk == "laser":
                        ch = w.get("charge", 0.0)
                        max_c = ws.get("charge", 0.75)
                        min_c = ws.get("min_charge", 0.15)
                        if cd <= 0 and tdist <= ws.get("range", 280.0):
                            if (ch >= max_c * 0.90 or (ch >= min_c and target.get("hp", 999) < 120)) and yerr < 0.09:
                                act["fire"][idx] = False
                            else:
                                act["fire"][idx] = True
                    elif wk == "lightning":
                        if cd <= 0 and tdist <= ws.get("range", 45.0) - 2.0 and energy > 15 and yerr < 0.25:
                            act["fire"][idx] = True
                    elif wk == "mortar":
                        if cd <= 0 and 20.0 <= tdist <= 220.0 and not is_crate and yerr < 0.16:
                            act["fire"][idx] = True
                    else:
                        if cd <= 0 and w.get("reload", 0) <= 0 and tdist <= ws.get("range", 230.0) and yerr < 0.18:
                            act["fire"][idx] = True

                if aim_y is not None:
                    act["aim"] = [round(aim_y, 4), round(aim_p, 4)]
            else:
                for idx in (0, 1):
                    w = weapons[idx]
                    if w and w.get("kind") == "laser" and w.get("charge", 0) > 0:
                        act["fire"][idx] = False

                scan = None
                for ev in self.events:
                    if ev.get("type") == "damaged" and ev.get("bearing") is not None:
                        scan = ev["bearing"]
                        break
                if scan is None and nearest_any:
                    scan = math.atan2(nearest_any["pos"][1] - pos[1], nearest_any["pos"][0] - pos[0])
                if scan is None and self.sounds:
                    ns = [s for s in self.sounds if s.get("dist") in ("near", "mid")]
                    if ns:
                        scan = ns[0].get("bearing")
                if scan is None:
                    scan = (self.t * 1.5) % (2 * math.pi) - math.pi
                act["aim"] = [round(scan, 4), 0.0]

            return act
        except Exception:
            return {"tick": self.tick, "move": [0.0, 0.0], "fire": [False, False]}


def main():
    bot = None
    stdin = sys.stdin.buffer
    while True:
        raw = stdin.readline()
        if not raw:
            break
        batch = [raw]
        while select.select([stdin], [], [], 0.0)[0]:
            nxt = stdin.readline()
            if not nxt:
                break
            batch.append(nxt)

        latest_obs = None
        ended = False
        for line in batch:
            s = line.strip()
            if not s:
                continue
            try:
                msg = json.loads(s)
            except Exception:
                continue
            t = msg.get("type")
            if t == "hello":
                bot = SentinelBot(msg)
                print(json.dumps({"type": "ready", "name": "Sentinel"}), flush=True)
            elif t == "obs":
                latest_obs = msg
            elif t == "end":
                ended = True

        if latest_obs is not None and bot is not None:
            bot.update(latest_obs)
            print(json.dumps(bot.step()), flush=True)

        if ended:
            break


if __name__ == "__main__":
    main()
