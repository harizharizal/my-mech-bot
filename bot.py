import sys
import os
import json
import math
import struct
import select
import heapq


class NavMap:
    def __init__(self, map_info, config):
        self.terrain = None
        self.nx = 0
        self.ny = 0
        self.min_x = 0.0
        self.min_y = 0.0
        self.step = 1.0
        self.prisms = []
        self.tree_buckets = {}
        self.gs = 8.0
        self.gx_min = -80
        self.gy_min = -60
        self.gw = 160
        self.gh = 120
        self.walkable = None
        self.cell_z = None
        self.adj = None

        self._cache_s = -1
        self._cache_g = -1
        self._cache_t = -99.0
        self._cache_path = []

        if not map_info:
            return
        mdir = map_info.get("dir", "/arena")
        wpath = os.path.join(mdir, "v2", "world.json")
        tpath = os.path.join(mdir, "v2", "terrain.i16")

        try:
            if not os.path.exists(wpath):
                return
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
                self.prisms.append({
                    "kind": p.get("kind", "building"),
                    "min_x": min(xs),
                    "max_x": max(xs),
                    "min_y": min(ys),
                    "max_y": max(ys),
                    "base": p.get("base", -100.0),
                    "top": p.get("top", 100.0),
                    "ring": ring,
                    "holes": p.get("holes", []),
                })

            t_obj = w.get("trees", {})
            fields = t_obj.get("fields", [])
            tdata = t_obj.get("data", [])
            if fields and tdata:
                stride = len(fields)
                ix_x = fields.index("x") if "x" in fields else 0
                ix_y = fields.index("y") if "y" in fields else 1
                ix_r = fields.index("crownR") if "crownR" in fields else 4
                for i in range(0, len(tdata) - stride + 1, stride):
                    cr = tdata[i + ix_r]
                    if cr >= 2.2:
                        tx, ty = tdata[i + ix_x], tdata[i + ix_y]
                        bx, by = int(math.floor(tx / 32.0)), int(math.floor(ty / 32.0))
                        self.tree_buckets.setdefault((bx, by), []).append((tx, ty, cr))

            self._build_nav_grid(w.get("bounds", {}), config)
        except Exception:
            self.walkable = None
            self.adj = None

    def ground_z(self, x, y):
        if not self.terrain:
            return 0.0
        fx = (x - self.min_x) / self.step
        fy = (y - self.min_y) / self.step
        ix = max(0, min(self.nx - 2, int(fx)))
        iy = max(0, min(self.ny - 2, int(fy)))
        dx = max(0.0, min(1.0, fx - ix))
        dy = max(0.0, min(1.0, fy - iy))
        i00 = iy * self.nx + ix
        z00 = self.terrain[i00]
        z10 = self.terrain[i00 + 1]
        z01 = self.terrain[i00 + self.nx]
        z11 = self.terrain[i00 + self.nx + 1]
        return (
            (z00 * (1.0 - dx) + z10 * dx) * (1.0 - dy)
            + (z01 * (1.0 - dx) + z11 * dx) * dy
        ) * 0.01

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

    def _build_nav_grid(self, bounds, config):
        gs = self.gs
        x0 = bounds.get("minX", self.min_x)
        y0 = bounds.get("minY", self.min_y)
        x1 = bounds.get("maxX", self.min_x + self.nx * self.step)
        y1 = bounds.get("maxY", self.min_y + self.ny * self.step)

        self.gx_min = int(math.floor(x0 / gs))
        self.gy_min = int(math.floor(y0 / gs))
        self.gw = max(10, int(math.ceil((x1 - x0) / gs)) + 1)
        self.gh = max(10, int(math.ceil((y1 - y0) / gs)) + 1)

        total = self.gw * self.gh
        walk = [False] * total
        cell_z = [0.0] * total

        b_size = 32.0
        buckets = {}
        for p in self.prisms:
            bx0 = int(math.floor((p["min_x"] - 3.0) / b_size))
            bx1 = int(math.ceil((p["max_x"] + 3.0) / b_size))
            by0 = int(math.floor((p["min_y"] - 3.0) / b_size))
            by1 = int(math.ceil((p["max_y"] + 3.0) / b_size))
            for bx in range(bx0, bx1 + 1):
                for by in range(by0, by1 + 1):
                    buckets.setdefault((bx, by), []).append(p)

        offsets = ((0.0, 0.0), (1.8, 0.0), (-1.8, 0.0), (0.0, 1.8), (0.0, -1.8))
        for gy in range(self.gh):
            cy = (self.gy_min + gy + 0.5) * gs
            by = int(math.floor(cy / b_size))
            row = gy * self.gw
            for gx in range(self.gw):
                cx = (self.gx_min + gx + 0.5) * gs
                if cx < x0 + 3.0 or cx > x1 - 3.0 or cy < y0 + 3.0 or cy > y1 - 3.0:
                    continue
                gz = self.ground_z(cx, cy)
                cell_z[row + gx] = gz
                cand = buckets.get((int(math.floor(cx / b_size)), by), ())
                blocked = False
                for p in cand:
                    if gz + 8.5 <= p["base"] + 1.2 or gz >= p["top"]:
                        continue
                    for ox, oy in offsets:
                        px, py = cx + ox, cy + oy
                        if self._point_in_prism(px, py, gz + 2.5, p) or self._point_in_prism(px, py, gz + 7.5, p):
                            blocked = True
                            break
                    if blocked:
                        break
                if not blocked:
                    walk[row + gx] = True

        adj = [[] for _ in range(total)]
        dirs = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        )
        for gy in range(self.gh):
            row = gy * self.gw
            for gx in range(self.gw):
                u = row + gx
                if not walk[u]:
                    continue
                zu = cell_z[u]
                nbrs = adj[u]
                for dx, dy, cost in dirs:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.gw and 0 <= ny < self.gh:
                        v = ny * self.gw + nx
                        if not walk[v]:
                            continue
                        if dx != 0 and dy != 0:
                            if not (walk[row + nx] and walk[ny * self.gw + gx]):
                                continue
                        dz = cell_z[v] - zu
                        if dz <= 8.5 * cost:
                            extra = 1.5 if dz > 1.3 else 0.0
                            nbrs.append((v, cost + extra))

        self.walkable = walk
        self.cell_z = cell_z
        self.adj = adj

    def world_to_cell(self, x, y):
        gx = max(0, min(self.gw - 1, int(math.floor(x / self.gs)) - self.gx_min))
        gy = max(0, min(self.gh - 1, int(math.floor(y / self.gs)) - self.gy_min))
        return gy * self.gw + gx

    def cell_to_world(self, idx):
        gy, gx = divmod(idx, self.gw)
        return (self.gx_min + gx + 0.5) * self.gs, (self.gy_min + gy + 0.5) * self.gs

    def nearest_walkable(self, x, y):
        if not self.walkable:
            return -1
        c = self.world_to_cell(x, y)
        if self.walkable[c]:
            return c
        gy0, gx0 = divmod(c, self.gw)
        for r in range(1, 10):
            for dy in range(-r, r + 1):
                ny = gy0 + dy
                if not (0 <= ny < self.gh):
                    continue
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx = gx0 + dx
                    if 0 <= nx < self.gw:
                        idx = ny * self.gw + nx
                        if self.walkable[idx]:
                            return idx
        return -1

    def line_walkable(self, x0, y0, x1, y1):
        if not self.walkable:
            return True
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(2, int(dist / 4.0))
        for i in range(1, steps + 1):
            t = i / steps
            if not self.walkable[self.world_to_cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)]:
                return False
        return True

    def build_danger_grid(self, threats):
        if not self.walkable or not threats:
            return None
        danger = [0.0] * (self.gw * self.gh)
        gs = self.gs
        gw = self.gw
        gh = self.gh
        for tx, ty, tr, weight in threats:
            cx = int(math.floor(tx / gs)) - self.gx_min
            cy = int(math.floor(ty / gs)) - self.gy_min
            rc = int(math.ceil(tr / gs))
            for gy in range(max(0, cy - rc), min(gh, cy + rc + 1)):
                row = gy * gw
                dy = gy - cy
                for gx in range(max(0, cx - rc), min(gw, cx + rc + 1)):
                    d = math.hypot(gx - cx, dy)
                    if d < tr / gs:
                        danger[row + gx] += (tr / gs - d) * weight
        return danger

    def find_path_dir(self, start_pos, goal_xy, danger_grid=None, now_t=0.0):
        dx = goal_xy[0] - start_pos[0]
        dy = goal_xy[1] - start_pos[1]
        d = math.hypot(dx, dy)
        if d < 1e-4:
            return 0.0, 0.0, False
        if not self.walkable or not self.adj:
            return dx / d, dy / d, False

        if d < 28.0 and not danger_grid and self.line_walkable(start_pos[0], start_pos[1], goal_xy[0], goal_xy[1]):
            ux, uy = dx / d, dy / d
            step_up = (self.ground_z(start_pos[0] + ux * 3.0, start_pos[1] + uy * 3.0) - start_pos[2]) > 1.05
            return ux, uy, step_up

        s = self.nearest_walkable(start_pos[0], start_pos[1])
        g = self.nearest_walkable(goal_xy[0], goal_xy[1])
        if s < 0 or g < 0 or s == g:
            return dx / d, dy / d, False

        # Reuse cached path for 0.4s if start/goal cells match and no active danger grid
        if (
            not danger_grid
            and (now_t - self._cache_t) < 0.40
            and s == self._cache_s
            and g == self._cache_g
            and len(self._cache_path) > 1
        ):
            path = self._cache_path
        else:
            gy_g, gx_g = divmod(g, self.gw)
            open_heap = [(0.0, s)]
            came_from = {s: -1}
            g_score = {s: 0.0}
            adj = self.adj
            gw = self.gw
            best_node = s
            best_h = 1e9
            expansions = 0

            while open_heap and expansions < 480:
                _, cur = heapq.heappop(open_heap)
                if cur == g:
                    best_node = g
                    break
                expansions += 1
                cy, cx = divmod(cur, gw)
                adx, ady = abs(gx_g - cx), abs(gy_g - cy)
                h_cur = max(adx, ady) + 0.414 * min(adx, ady)
                if h_cur < best_h:
                    best_h = h_cur
                    best_node = cur

                gc = g_score[cur]
                for nxt, step_cost in adj[cur]:
                    pen = danger_grid[nxt] if danger_grid else 0.0
                    ng = gc + step_cost + pen
                    if ng < g_score.get(nxt, 1e9):
                        g_score[nxt] = ng
                        came_from[nxt] = cur
                        ny, nx = divmod(nxt, gw)
                        ddx, ddy = abs(gx_g - nx), abs(gy_g - ny)
                        h = max(ddx, ddy) + 0.414 * min(ddx, ddy)
                        heapq.heappush(open_heap, (ng + h * 1.18, nxt))

            path = []
            curr = best_node
            while curr != -1 and curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.reverse()

            self._cache_s = s
            self._cache_g = g
            self._cache_t = now_t
            self._cache_path = path

        if len(path) <= 1:
            return dx / d, dy / d, False

        target_cell = path[1]
        for idx in range(min(4, len(path) - 1), 1, -1):
            wx, wy = self.cell_to_world(path[idx])
            if self.line_walkable(start_pos[0], start_pos[1], wx, wy):
                target_cell = path[idx]
                break

        wx, wy = self.cell_to_world(target_cell)
        wdx, wdy = wx - start_pos[0], wy - start_pos[1]
        wd = math.hypot(wdx, wdy)
        if wd <= 1e-4:
            return dx / d, dy / d, False
        ux, uy = wdx / wd, wdy / wd
        step_up = (self.ground_z(start_pos[0] + ux * 3.0, start_pos[1] + uy * 3.0) - start_pos[2]) > 1.05
        return ux, uy, step_up

    def has_los(self, p1, p2):
        if not self.prisms and not self.terrain:
            return True
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 4.0:
            return True
        steps = min(32, max(4, int(dist / 5.5)))
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
            if self.terrain and z < self.ground_z(x, y) - 0.2:
                return False
            for p in cand:
                if self._point_in_prism(x, y, z, p):
                    return False
        return True

    def mortar_arc_clear(self, muzzle, target, speed, pitch, yaw, g=22.0):
        if not self.prisms:
            return True
        cp = math.cos(pitch)
        vx = speed * cp * math.cos(yaw)
        vy = speed * cp * math.sin(yaw)
        vz = speed * math.sin(pitch)
        horiz = math.hypot(target[0] - muzzle[0], target[1] - muzzle[1])
        t_total = horiz / max(speed * cp, 1.0)
        if t_total <= 0.1 or t_total > 6.5:
            return False
        for i in range(1, 10):
            t = (i / 10.0) * t_total
            x = muzzle[0] + vx * t
            y = muzzle[1] + vy * t
            z = muzzle[2] + vz * t - 0.5 * g * t * t
            if self.terrain and z < self.ground_z(x, y):
                return False
            for p in self.prisms:
                if self._point_in_prism(x, y, z, p):
                    return False
        return True

    def canopy_cover_bonus(self, x, y):
        bx, by = int(math.floor(x / 32.0)), int(math.floor(y / 32.0))
        bonus = 0.0
        for tx, ty, cr in self.tree_buckets.get((bx, by), ()):
            if abs(tx - x) <= cr and abs(ty - y) <= cr:
                if math.hypot(tx - x, ty - y) <= cr:
                    bonus += 18.0
                    if bonus >= 36.0:
                        break
        return bonus


class ValkyrieSovereign:
    UPGRADE_SCORES = {
        "nanites": 100,
        "armor": 96,
        "plating": 92,
        "servos": 88,
        "gyros": 84,
        "jets": 80,
        "reactor": 76,
        "capacitor": 72,
        "sensors": 68,
        "laser_charge": 88,
        "laser_damage": 84,
        "laser_pierce": 70,
        "lightning_efficiency": 87,
        "lightning_range": 85,
        "lightning_chain": 76,
        "mortar_reload": 85,
        "mortar_splash": 81,
        "mortar_velocity": 79,
        "gun_damage": 74,
        "gun_rate": 73,
        "gun_velocity": 71,
        "gun_mag": 66,
    }

    WEAPON_PRIORITY = {"laser": 4, "lightning": 3, "mortar": 2, "gun": 1}
    SOUND_DIST_MAP = {"near": 38.0, "mid": 105.0, "far": 210.0}

    def __init__(self, hello):
        self.cfg = hello.get("config", {})
        self.map = NavMap(hello.get("map", {}), self.cfg)
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
        self.beams = []
        self.events = []
        self.offer = None

        self.last_pos = None
        self.stuck_ticks = 0
        self.strafe_sign = 1.0
        self.last_damage_t = -99.0
        self.last_hit_t = -99.0
        self.cover_spot = None
        self.cover_spot_t = -99.0
        self.sound_threats = []
        self.last_sound_yaw = None
        self.last_sound_t = -99.0

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
        self.beams = obs.get("beams", [])
        self.events = obs.get("events", [])

        pos = self.me.get("pos")

        for ev in self.events:
            et = ev.get("type")
            if et == "damaged":
                self.last_damage_t = self.t
                if ev.get("bearing") is not None and pos:
                    b = ev["bearing"]
                    self.last_sound_yaw = b
                    self.last_sound_t = self.t
                    self.sound_threats.append((pos[0] + math.cos(b) * 45.0, pos[1] + math.sin(b) * 45.0, 55.0, 3.5, self.t + 4.0))
            elif et == "hit":
                self.last_hit_t = self.t

        for bm in self.beams:
            by_id = bm.get("by")
            path = bm.get("path", [])
            if by_id is not None and by_id != self.me.get("id") and path:
                mp = path[0]
                if by_id not in self.enemies:
                    self.enemies[by_id] = {
                        "id": by_id,
                        "kind": "mech",
                        "pos": [mp[0], mp[1], max(0.0, mp[2] - 7.0)],
                        "vel": [0.0, 0.0, 0.0],
                        "aim": [0.0, 0.0],
                        "hp": 1000,
                        "weapons": [bm.get("kind", "laser")],
                        "seen_t": self.t,
                    }

        if pos:
            for snd in self.sounds:
                sk = snd.get("kind", "")
                if sk in ("npc_gun", "crate", "drop"):
                    continue
                b = snd.get("bearing")
                if b is None:
                    continue
                d_est = self.SOUND_DIST_MAP.get(snd.get("dist", "mid"), 105.0)
                sx = pos[0] + math.cos(b) * d_est
                sy = pos[1] + math.sin(b) * d_est
                if sk in ("laser", "laser_charge", "mortar", "explosion", "lightning", "gun"):
                    self.sound_threats.append((sx, sy, 55.0, 2.5, self.t + 3.5))
                if snd.get("dist") in ("near", "mid"):
                    self.last_sound_yaw = b
                    self.last_sound_t = self.t

        self.sound_threats = [st for st in self.sound_threats if st[4] > self.t]
        if len(self.sound_threats) > 10:
            self.sound_threats = self.sound_threats[-10:]

        if pos and self.last_pos:
            if math.hypot(pos[0] - self.last_pos[0], pos[1] - self.last_pos[1]) < 0.25:
                self.stuck_ticks += 1
            else:
                self.stuck_ticks = 0
        if pos:
            self.last_pos = list(pos)

        if self.tick % 20 == 0:
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

        self.enemies = {k: v for k, v in self.enemies.items() if self.t - v.get("seen_t", 0) <= 4.8}
        self.npcs = {k: v for k, v in self.npcs.items() if self.t - v.get("seen_t", 0) <= 3.0}

    @staticmethod
    def d2(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def pick_upgrade(self, offer):
        choices = offer.get("choices", [])
        best_i, best_s = 0, -1
        for i, c in enumerate(choices):
            cid = c.get("id", "").lower()
            s = self.UPGRADE_SCORES.get(cid, 50)
            if s > best_s:
                best_s, best_i = s, i
        return best_i

    def live_zone(self):
        z = self.zone
        c0 = z.get("center", [0.0, 0.0])
        r0 = z.get("radius", 560.0)
        c1 = z.get("next_center", c0)
        r1 = z.get("next_radius", r0)
        ts = z.get("shrink_start", 60.0)
        te = z.get("shrink_end", 110.0)
        if self.t <= ts or te <= ts:
            alpha = 0.0
        elif self.t >= te:
            alpha = 1.0
        else:
            alpha = (self.t - ts) / (te - ts)
        cx = c0[0] + (c1[0] - c0[0]) * alpha
        cy = c0[1] + (c1[1] - c0[1]) * alpha
        cr = r0 + (r1 - r0) * alpha
        return (cx, cy), cr, (c1[0], c1[1]), r1, ts, te

    def aim_at(self, tpos, tvel, z_offset, wk, ws):
        pos = self.me["pos"]
        mh = self.me.get("stats", {}).get("muzzle_height", 7.0) + (2.5 if wk == "mortar" else 0.0)
        mx, my, mz = pos[0], pos[1], pos[2] + mh
        tx, ty, tz = tpos[0], tpos[1], tpos[2] + z_offset
        dx, dy, dz = tx - mx, ty - my, tz - mz
        dist = max(math.hypot(dx, dy), 0.1)

        if wk == "gun":
            spd = max(ws.get("speed", 252.0), 1.0)
            tf = dist / spd
            for _ in range(2):
                px = tx + tvel[0] * tf - mx
                py = ty + tvel[1] * tf - my
                pz = tz + tvel[2] * tf - mz
                tf = math.sqrt(px * px + py * py + pz * pz) / spd
            return math.atan2(py, px), math.atan2(pz, max(math.hypot(px, py), 0.1))

        if wk == "mortar":
            spd = max(ws.get("speed", 72.0), 1.0)
            g = ws.get("gravity", 22.0)
            tf = dist / (spd * 0.82)
            pitch = 0.7854
            px, py = dx, dy
            for _ in range(2):
                px = tx + tvel[0] * tf - mx
                py = ty + tvel[1] * tf - my
                pz = tz + tvel[2] * tf - mz
                r = max(math.hypot(px, py), 0.1)
                v2 = spd * spd
                disc = v2 * v2 - g * (g * r * r + 2.0 * pz * v2)
                if disc >= 0:
                    pitch = math.atan((v2 - math.sqrt(disc)) / (g * r))
                    tf = r / max(spd * math.cos(pitch), 1.0)
                else:
                    pitch = 0.7854
                    break
            return math.atan2(py, px), pitch

        return math.atan2(dy, dx), math.atan2(dz, dist)

    def find_cover_spot(self, pos, zc, zr, threats):
        if self.cover_spot and (self.t - self.cover_spot_t < 2.2) and self.d2(self.cover_spot, zc) < zr * 0.9:
            return self.cover_spot
        if not self.map.walkable or not threats:
            return None

        best_xy = None
        best_score = -1e9
        for ang_deg in range(0, 360, 30):
            rad = math.radians(ang_deg)
            for rad_dist in (18.0, 34.0):
                cx = pos[0] + math.cos(rad) * rad_dist
                cy = pos[1] + math.sin(rad) * rad_dist
                if self.d2((cx, cy), zc) > zr * 0.86:
                    continue
                if not self.map.walkable[self.map.world_to_cell(cx, cy)]:
                    continue
                cz = self.map.ground_z(cx, cy) + 6.5
                hidden = True
                min_td = 999.0
                for e in threats:
                    ep = e["pos"]
                    td = self.d2((cx, cy), ep)
                    if td < min_td:
                        min_td = td
                    if self.map.has_los([ep[0], ep[1], ep[2] + 8.0], [cx, cy, cz]):
                        hidden = False
                        break
                foliage = self.map.canopy_cover_bonus(cx, cy)
                sc = (140.0 if hidden else 0.0) + foliage + min(min_td, 130.0) - self.d2((cx, cy), zc) * 0.25
                if sc > best_score:
                    best_score = sc
                    best_xy = (cx, cy)

        if best_xy:
            self.cover_spot = best_xy
            self.cover_spot_t = self.t
        return best_xy

    def score_target(self, e, pos, held):
        ep = e["pos"]
        ed = self.d2(pos, ep)
        hp = e.get("hp", 1000)
        e_weaps = set(e.get("weapons", []))
        e_aim = e.get("aim", [0.0, 0.0])[0]

        score = 210.0 - ed
        if hp < 340:
            score += 125.0
        elif hp < 650:
            score += 45.0

        if "laser" in held and "mortar" in e_weaps:
            score += 75.0
        if "lightning" in held and "laser" in e_weaps:
            score += 70.0
            if e.get("charging"):
                score += 60.0
        if "mortar" in held and "lightning" in e_weaps:
            score += 60.0

        to_me = math.atan2(pos[1] - ep[1], pos[0] - ep[0])
        ang_diff = abs((to_me - e_aim + math.pi) % (2 * math.pi) - math.pi)
        if ang_diff > math.pi * 0.55:
            score += 50.0

        return score

    def step(self):
        act = {
            "tick": self.tick,
            "move": [0.0, 0.0],
            "boost": False,
            "jump": False,
            "fire": [False, False],
            "reload": False,
        }

        try:
            me = self.me
            pos = me.get("pos")
            if not pos:
                return act

            hp = me.get("hp", 1400)
            max_hp = me.get("max_hp", 1400)
            hp_ratio = hp / max(max_hp, 1)
            energy = me.get("energy", 100)
            grounded = me.get("grounded", True)
            in_zone = me.get("in_zone", True)
            cubes_cnt = me.get("cubes", 0)
            stats = me.get("stats", {})
            walk_spd = max(stats.get("walk_speed", 11.7), 5.0)
            weapons = list(me.get("weapons", [None, None]))
            while len(weapons) < 2:
                weapons.append(None)

            if self.offer:
                act["pick"] = self.pick_upgrade(self.offer)

            held = [w["kind"] for w in weapons if w]
            empty = [i for i, w in enumerate(weapons) if w is None]
            laser_charging = any(w and w.get("kind") == "laser" and w.get("charge", 0.0) > 0.0 for w in weapons)

            # 1. Perception: Muzzle LOS + Indirect Mortar Candidates
            muzzle_h = stats.get("muzzle_height", 7.0)
            muzzle = [pos[0], pos[1], pos[2] + muzzle_h]

            vis_enemies = []
            mortar_enemies = []
            nearest_any = None
            nearest_any_d = 9999.0

            for e in self.enemies.values():
                d = self.d2(pos, e["pos"])
                if d < nearest_any_d:
                    nearest_any_d = d
                    nearest_any = e
                t_center = [e["pos"][0], e["pos"][1], e["pos"][2] + 4.8]
                if d <= 215.0 and self.map.has_los(muzzle, t_center):
                    vis_enemies.append(e)
                elif "mortar" in held and 28.0 <= d <= 195.0:
                    mortar_enemies.append(e)

            # 2. Weapon Loot & Smart Loadout Upgrade
            near_pickup = None
            best_p_prio = 0
            safe_to_swap = (not vis_enemies) or (nearest_any_d > 32.0)
            for p in self.pickups:
                pk = p.get("kind")
                if pk in held:
                    continue
                d = self.d2(pos, p["pos"])
                dz = abs(pos[2] - p["pos"][2])
                p_prio = self.WEAPON_PRIORITY.get(pk, 1)
                if d <= 3.85 and dz <= 3.85 and not laser_charging and safe_to_swap:
                    if empty:
                        act["take"] = p["id"]
                        break
                    worst_idx = min((0, 1), key=lambda i: self.WEAPON_PRIORITY.get(weapons[i]["kind"], 0))
                    if p_prio > self.WEAPON_PRIORITY.get(weapons[worst_idx]["kind"], 0):
                        act["take"] = p["id"]
                        act["drop"] = worst_idx
                        break
                elif empty and d < 36.0 and dz <= 4.5 and p_prio > best_p_prio:
                    best_p_prio = p_prio
                    near_pickup = p

            # 3. Live Zone Pacing & Quiet-Sector Anchor
            cur_zc, cur_zr, next_zc, next_zr, shrink_s, shrink_e = self.live_zone()
            dist_cur_zc = self.d2(pos, cur_zc)
            dist_next_zc = self.d2(pos, next_zc)
            dist_to_next_edge = max(0.0, dist_next_zc - next_zr * 0.85)
            travel_time_needed = dist_to_next_edge / walk_spd
            time_until_closed = max(0.0, shrink_e - self.t)

            zone_urgent = (
                (not in_zone)
                or (dist_cur_zc > cur_zr - 12.0)
                or (dist_to_next_edge > 0.0 and time_until_closed <= travel_time_needed + 7.5)
            )
            healing_mode = (hp_ratio < 0.75) and (self.t > 10.0)

            anchor_x, anchor_y = next_zc[0], next_zc[1]
            if next_zr > 35.0:
                rep_x, rep_y = 0.0, 0.0
                for e in self.enemies.values():
                    dx_e = pos[0] - e["pos"][0]
                    dy_e = pos[1] - e["pos"][1]
                    de = max(math.hypot(dx_e, dy_e), 1.0)
                    if de < 180.0:
                        w = (180.0 - de) / 180.0
                        rep_x += (dx_e / de) * w * 1.5
                        rep_y += (dy_e / de) * w * 1.5
                for sx, sy, _, sw, _ in self.sound_threats:
                    dx_s = pos[0] - sx
                    dy_s = pos[1] - sy
                    ds = max(math.hypot(dx_s, dy_s), 1.0)
                    if ds < 200.0:
                        w = (200.0 - ds) / 200.0
                        rep_x += (dx_s / ds) * w
                        rep_y += (dy_s / ds) * w
                rmag = math.hypot(rep_x, rep_y)
                if rmag > 0.05:
                    off = min(next_zr * 0.50, 72.0)
                    anchor_x = next_zc[0] + (rep_x / rmag) * off
                    anchor_y = next_zc[1] + (rep_y / rmag) * off

            best_drop = None
            if not zone_urgent:
                for dr in self.drops:
                    if dr.get("lands_in", 99) <= 14.0 and (
                        dr.get("holder") in (None, me.get("id")) or self.d2(pos, dr["pos"]) < 14.0
                    ):
                        dd = self.d2(pos, dr["pos"])
                        if dd < 90.0 and (not nearest_any or nearest_any_d > 36.0 or hp_ratio < 0.65):
                            best_drop = dr
                            break

            # 4. Target Selection
            target = None
            target_z_off = 4.8
            is_crate = False
            indirect_mortar = False

            if vis_enemies:
                best_e = max(vis_enemies, key=lambda e: self.score_target(e, pos, held))
                ed = self.d2(pos, best_e["pos"])
                if not healing_mode or ed < 75.0 or best_e.get("hp", 999) < 280 or laser_charging:
                    target = best_e
            elif mortar_enemies and not healing_mode and not zone_urgent:
                target = max(mortar_enemies, key=lambda e: self.score_target(e, pos, held))
                indirect_mortar = True
            elif not healing_mode and cubes_cnt < 10 and (not nearest_any or nearest_any_d > 80.0):
                for n in sorted(self.npcs.values(), key=lambda x: self.d2(pos, x["pos"])):
                    nd = self.d2(pos, n["pos"])
                    if nd < 65.0 and self.map.has_los(muzzle, [n["pos"][0], n["pos"][1], n["pos"][2] + 2.0]):
                        target, target_z_off = n, 1.8
                        break
                if not target:
                    for c in sorted(self.crates, key=lambda x: self.d2(pos, x["pos"])):
                        cd = self.d2(pos, c["pos"])
                        if cd < 35.0 and self.map.has_los(muzzle, [c["pos"][0], c["pos"][1], c["pos"][2] + 1.5]):
                            target, target_z_off, is_crate = c, 1.5, True
                            break

            # 5. Movement & Stealth Pacing
            use_astar = False
            stealth_hold = False
            silent_walk = False
            gx, gy = anchor_x, anchor_y

            if zone_urgent:
                gx, gy = next_zc[0], next_zc[1]
                use_astar = True
                if energy > 25 and dist_to_next_edge > 14.0:
                    act["boost"] = True
            elif healing_mode and (vis_enemies or (nearest_any and nearest_any_d < 95.0)):
                cov = self.find_cover_spot(pos, next_zc, next_zr, list(self.enemies.values()))
                if cov:
                    gx, gy = cov
                    use_astar = True
                    cd_cov = self.d2(pos, cov)
                    if cd_cov < 4.5 and not vis_enemies:
                        stealth_hold = True
                    elif not vis_enemies and nearest_any_d < 45.0:
                        silent_walk = True
                else:
                    gx, gy = anchor_x, anchor_y
                    use_astar = True
                if not stealth_hold and not silent_walk and energy > 30 and vis_enemies:
                    act["boost"] = True
            elif best_drop:
                dpos = best_drop["pos"]
                dd = self.d2(pos, dpos)
                if dd < 6.0 and best_drop.get("lands_in", 0) <= 0.5:
                    ang = self.t * 2.2
                    gx = dpos[0] + math.cos(ang) * 3.5
                    gy = dpos[1] + math.sin(ang) * 3.5
                else:
                    gx, gy = dpos[0], dpos[1]
                    use_astar = True
                    if energy > 45 and dd > 12.0:
                        act["boost"] = True
            elif any(abs(c["pos"][2] - pos[2]) <= 4.5 for c in self.cubes):
                nc = min(
                    (c for c in self.cubes if abs(c["pos"][2] - pos[2]) <= 4.5),
                    key=lambda c: self.d2(pos, c["pos"]),
                )
                if self.d2(pos, nc["pos"]) < 34.0 and (not nearest_any or nearest_any_d > 42.0):
                    gx, gy = nc["pos"][0], nc["pos"][1]
                    use_astar = True
            elif near_pickup and (not nearest_any or nearest_any_d > 42.0):
                gx, gy = near_pickup["pos"][0], near_pickup["pos"][1]
                use_astar = True
            elif vis_enemies and target and target.get("kind") == "mech":
                epos = target["pos"]
                ed = self.d2(pos, epos)
                ux = (epos[0] - pos[0]) / max(ed, 0.1)
                uy = (epos[1] - pos[1]) / max(ed, 0.1)
                px, py = -uy * self.strafe_sign, ux * self.strafe_sign

                ideal = 24.0 if "lightning" in held else (68.0 if "laser" in held else 48.0)
                if ed < ideal - 8.0:
                    gx, gy = pos[0] - ux * 16.0 + px * 18.0, pos[1] - uy * 16.0 + py * 18.0
                elif ed > ideal + 14.0 and (self.alive <= 6 or target.get("hp", 999) < 340):
                    gx, gy = pos[0] + ux * 15.0 + px * 15.0, pos[1] + uy * 15.0 + py * 15.0
                else:
                    gx, gy = pos[0] + px * 20.0, pos[1] + py * 20.0
            else:
                if self.d2(pos, (anchor_x, anchor_y)) > 16.0:
                    gx, gy = anchor_x, anchor_y
                    use_astar = True
                    if nearest_any and nearest_any_d < 45.0:
                        silent_walk = True
                else:
                    stealth_hold = True

            # Projectile & Enemy Laser-Charge Dodge Override
            dx_dodge, dy_dodge = 0.0, 0.0
            for pr in self.projectiles:
                pp, pv = pr["pos"], pr["vel"]
                rx, ry = pos[0] - pp[0], pos[1] - pp[1]
                pd = math.hypot(rx, ry)
                ps = math.hypot(pv[0], pv[1])
                if pd < 32.0 and ps > 1.0:
                    vx, vy = pv[0] / ps, pv[1] / ps
                    if rx * vx + ry * vy > 0:
                        px, py = -vy, vx
                        side = 1.0 if (rx * px + ry * py) >= 0 else -1.0
                        dx_dodge += px * side * 28.0
                        dy_dodge += py * side * 28.0
                        stealth_hold = False
                        silent_walk = False
                        if energy > 20 and pd < 18.0:
                            act["boost"] = True

            for e in vis_enemies:
                if e.get("charging"):
                    ep = e["pos"]
                    e_yaw = e.get("aim", [0.0, 0.0])[0]
                    to_us = math.atan2(pos[1] - ep[1], pos[0] - ep[0])
                    if abs((to_us - e_yaw + math.pi) % (2 * math.pi) - math.pi) < 0.34:
                        rx, ry = pos[0] - ep[0], pos[1] - ep[1]
                        rd = max(math.hypot(rx, ry), 0.1)
                        px, py = -ry / rd, rx / rd
                        dx_dodge += px * self.strafe_sign * 34.0
                        dy_dodge += py * self.strafe_sign * 34.0
                        stealth_hold = False
                        silent_walk = False
                        if energy > 20:
                            act["boost"] = True

            if stealth_hold:
                act["move"] = [0.0, 0.0]
            else:
                step_up_jump = False
                if use_astar:
                    d_grid = None
                    if healing_mode or zone_urgent:
                        path_threats = [(e["pos"][0], e["pos"][1], 45.0, 3.0) for e in self.enemies.values()]
                        for sx, sy, sr, sw, _ in self.sound_threats:
                            path_threats.append((sx, sy, sr, sw * 0.7))
                        d_grid = self.map.build_danger_grid(path_threats)
                    sx, sy, step_up_jump = self.map.find_path_dir(pos, (gx, gy), d_grid, self.t)
                    rx, ry = sx * 20.0 + dx_dodge, sy * 20.0 + dy_dodge
                    rm = math.hypot(rx, ry)
                    mx, my = (rx / rm, ry / rm) if rm > 1e-4 else (sx, sy)
                else:
                    rx = (gx - pos[0]) + dx_dodge
                    ry = (gy - pos[1]) + dy_dodge
                    rm = math.hypot(rx, ry)
                    mx, my = (rx / rm, ry / rm) if rm > 1e-4 else (0.0, 0.0)
                    if self.map.walkable and not self.map.walkable[self.map.world_to_cell(pos[0] + mx * 6.0, pos[1] + my * 6.0)]:
                        mx, my = -mx, -my

                if silent_walk and not act["boost"]:
                    scale = min(0.40, 4.6 / walk_spd)
                    mx *= scale
                    my *= scale

                act["move"] = [round(mx, 4), round(my, 4)]

                if ((step_up_jump and energy > 25) or (self.stuck_ticks >= 4 and energy > 20)):
                    if grounded:
                        act["jump"] = True
                    else:
                        act["boost"] = True

            # 6. Per-Weapon Independent Aiming & Firing
            if target:
                tpos = target["pos"]
                tvel = target.get("vel", [0.0, 0.0, 0.0])
                tdist = self.d2(pos, tpos)
                cur_aim = me.get("aim", [0.0, 0.0])
                t_weaps = set(target.get("weapons", []))

                slot_aims = [None, None]
                slot_ready = [False, False]

                for idx in (0, 1):
                    w = weapons[idx]
                    if not w:
                        continue
                    wk = w.get("kind", "gun")
                    ws = w.get("stats", {})
                    cd = w.get("cooldown", 0.0)
                    ch = w.get("charge", 0.0)

                    if wk == "gun":
                        ammo = w.get("ammo", 12)
                        if ammo == 0 or (ammo <= 3 and tdist > 85.0 and w.get("reload", 0) == 0):
                            act["reload"] = True

                    if indirect_mortar and wk != "mortar":
                        continue

                    z_off = 0.8 if wk == "mortar" else target_z_off
                    ay, ap = self.aim_at(tpos, tvel, z_off, wk, ws)
                    slot_aims[idx] = (ay, ap)

                    if wk == "laser":
                        lightning_threat = ("lightning" in t_weaps and tdist < 52.0 and ch == 0.0)
                        if ch > 0.0:
                            slot_ready[idx] = True
                        else:
                            slot_ready[idx] = (
                                cd <= 0.2
                                and not indirect_mortar
                                and not lightning_threat
                                and tdist <= ws.get("range", 280.0)
                                and not zone_urgent
                                and not healing_mode
                            )
                    elif wk == "lightning":
                        slot_ready[idx] = (
                            cd <= 0.15
                            and not indirect_mortar
                            and tdist <= ws.get("range", 45.0) - 1.5
                            and energy > 12
                        )
                    elif wk == "mortar":
                        can_mortar = cd <= 0.2 and 18.0 <= tdist <= 220.0 and not is_crate
                        if can_mortar and indirect_mortar:
                            can_mortar = self.map.mortar_arc_clear(
                                [pos[0], pos[1], pos[2] + muzzle_h + 2.5],
                                tpos,
                                ws.get("speed", 72.0),
                                ap,
                                ay,
                                ws.get("gravity", 22.0),
                            )
                        slot_ready[idx] = can_mortar
                    else:
                        slot_ready[idx] = (
                            cd <= 0.2
                            and not indirect_mortar
                            and w.get("reload", 0) <= 0
                            and tdist <= ws.get("range", 230.0)
                        )

                chosen_slot = None
                best_p = -1
                for idx in (0, 1):
                    if slot_ready[idx] and weapons[idx]:
                        p = self.WEAPON_PRIORITY.get(weapons[idx].get("kind", "gun"), 0)
                        if weapons[idx].get("kind") == "laser" and weapons[idx].get("charge", 0) > 0:
                            p += 5
                        if p > best_p:
                            best_p = p
                            chosen_slot = idx
                if chosen_slot is None:
                    for idx in (0, 1):
                        if slot_aims[idx] is not None:
                            chosen_slot = idx
                            break

                if chosen_slot is not None:
                    des_y, des_p = slot_aims[chosen_slot]
                    act["aim"] = [round(des_y, 4), round(des_p, 4)]

                    for idx in (0, 1):
                        if not slot_ready[idx] or slot_aims[idx] is None:
                            continue
                        sy, sp = slot_aims[idx]
                        yerr = abs((sy - cur_aim[0] + math.pi) % (2 * math.pi) - math.pi)
                        perr = abs(sp - cur_aim[1])
                        wk = weapons[idx].get("kind", "gun")
                        ws = weapons[idx].get("stats", {})

                        if wk == "laser":
                            ch = weapons[idx].get("charge", 0.0)
                            max_c = ws.get("charge", 0.75)
                            min_c = ws.get("min_charge", 0.15)
                            abort_release = (zone_urgent or healing_mode) and ch >= min_c and yerr < 0.14
                            ready_release = (
                                abort_release
                                or (ch >= max_c * 0.88 and yerr < 0.11 and perr < 0.11)
                                or (ch >= max_c * 0.96 and yerr < 0.20)
                                or (ch >= min_c and target.get("hp", 999) < 120 and yerr < 0.11 and perr < 0.11)
                            )
                            act["fire"][idx] = not ready_release
                        elif wk == "lightning":
                            if yerr < 0.24 and perr < 0.30:
                                act["fire"][idx] = True
                        elif wk == "mortar":
                            if yerr < 0.14 and perr < 0.10:
                                act["fire"][idx] = True
                        else:
                            if yerr < 0.16 and perr < 0.14:
                                act["fire"][idx] = True
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
                if scan is None and self.last_sound_yaw is not None and (self.t - self.last_sound_t) < 2.5:
                    scan = self.last_sound_yaw
                if scan is None:
                    scan = (self.t * 1.45) % (2 * math.pi) - math.pi
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
                bot = ValkyrieSovereign(msg)
                print(json.dumps({"type": "ready", "name": "ValkyrieSovereign"}), flush=True)
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
