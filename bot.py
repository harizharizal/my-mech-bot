import sys
import json
import math
import os
import struct
import select

class FastMap:
    def __init__(self, map_info):
        self.terrain = None
        self.nx = 0
        self.ny = 0
        self.min_x = 0.0
        self.min_y = 0.0
        self.step = 1.0
        self.grid_size = 8.0
        self.blocked_cells = set()

        if not map_info:
            return

        map_dir = map_info.get('dir', '/arena')
        world_path = os.path.join(map_dir, 'v2', 'world.json')
        terrain_path = os.path.join(map_dir, 'v2', 'terrain.i16')

        try:
            if os.path.exists(terrain_path):
                with open(terrain_path, 'rb') as f:
                    raw = f.read()
                    if os.path.exists(world_path):
                        with open(world_path, 'r') as wf:
                            wdata = json.load(wf)
                            t_info = wdata.get('terrain', {})
                            self.nx = t_info.get('nx', 1070)
                            self.ny = t_info.get('ny', 724)
                            self.min_x = t_info.get('minX', 0.0)
                            self.min_y = t_info.get('minY', 0.0)
                            self.step = t_info.get('step', 1.0)
                            count = self.nx * self.ny
                            if len(raw) >= count * 2:
                                self.terrain = struct.unpack(f'<{count}h', raw[:count*2])
                            self._bake_buildings(wdata.get('prisms', []))
        except Exception:
            pass

    def get_ground_z(self, x, y):
        if not self.terrain:
            return 0.0
        ix = max(0, min(self.nx - 1, int((x - self.min_x) / self.step)))
        iy = max(0, min(self.ny - 1, int((y - self.min_y) / self.step)))
        return self.terrain[iy * self.nx + ix] * 0.01

    def _point_in_poly(self, x, y, ring):
        inside = False
        n = len(ring) // 2
        j = n - 1
        for i in range(n):
            xi, yi = ring[2*i], ring[2*i + 1]
            xj, yj = ring[2*j], ring[2*j + 1]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi):
                inside = not inside
            j = i
        return inside

    def _bake_buildings(self, prisms):
        gs = self.grid_size
        for p in prisms:
            if p.get('kind') not in ('building', 'stadium'):
                continue
            ring = p.get('ring', [])
            if len(ring) < 6:
                continue
            holes = p.get('holes', [])
            base = p.get('base', -100.0)
            top = p.get('top', 100.0)
            xs = ring[0::2]
            ys = ring[1::2]
            gx0 = int(math.floor((min(xs) - 2.5) / gs))
            gx1 = int(math.ceil((max(xs) + 2.5) / gs))
            gy0 = int(math.floor((min(ys) - 2.5) / gs))
            gy1 = int(math.ceil((max(ys) + 2.5) / gs))
            for gx in range(gx0, gx1 + 1):
                cx = (gx + 0.5) * gs
                for gy in range(gy0, gy1 + 1):
                    cy = (gy + 0.5) * gs
                    gz = self.get_ground_z(cx, cy)
                    if gz + 8.5 > base + 1.2 and gz < top:
                        if self._point_in_poly(cx, cy, ring):
                            in_hole = False
                            for h in holes:
                                if len(h) >= 6 and self._point_in_poly(cx, cy, h):
                                    in_hole = True
                                    break
                            if not in_hole:
                                self.blocked_cells.add((gx, gy))

    def is_blocked(self, x, y):
        if not self.blocked_cells:
            return False
        gx = int(math.floor(x / self.grid_size))
        gy = int(math.floor(y / self.grid_size))
        return (gx, gy) in self.blocked_cells


class WorldState:
    def __init__(self, hello_msg):
        self.config = hello_msg.get('config', {})
        self.map = FastMap(hello_msg.get('map', {}))
        self.tick = 0
        self.time = 0.0
        self.alive = 16
        self.me = {}
        self.zone = {}
        self.enemies = {}
        self.npcs = {}
        self.pickups = {}
        self.cubes = {}
        self.crates = {}
        self.drops = []
        self.projectiles = []
        self.sounds = []
        self.events = []
        self.offer = None
        self.last_pos = None
        self.stuck_ticks = 0
        self.strafe_dir = 1.0
        self.avoid_dir = 1.0

    def update(self, obs):
        self.tick = obs.get('tick', 0)
        self.time = obs.get('t', 0.0)
        self.alive = obs.get('alive', 16)
        self.me = obs.get('self', {})
        self.zone = obs.get('zone', {})
        self.offer = obs.get('offer')
        self.drops = obs.get('drops', [])
        self.projectiles = obs.get('projectiles', [])
        self.sounds = obs.get('sounds', [])
        self.events = obs.get('events', [])

        pos = self.me.get('pos')
        if pos and self.last_pos:
            moved = math.hypot(pos[0] - self.last_pos[0], pos[1] - self.last_pos[1])
            if moved < 0.25:
                self.stuck_ticks += 1
                if self.stuck_ticks == 8:
                    self.avoid_dir *= -1.0
            else:
                self.stuck_ticks = 0
        if pos:
            self.last_pos = list(pos)

        if self.tick % 30 == 0:
            self.strafe_dir *= -1.0

        for m in obs.get('mechs', []):
            mid = m.get('id')
            if mid is None:
                continue
            m['last_seen'] = self.time
            if m.get('kind') == 'mech':
                self.enemies[mid] = m
            elif m.get('kind') == 'npc':
                self.npcs[mid] = m

        self.enemies = {k: v for k, v in self.enemies.items() if self.time - v.get('last_seen', 0) <= 4.5}
        self.npcs = {k: v for k, v in self.npcs.items() if self.time - v.get('last_seen', 0) <= 3.0}
        self.pickups = {p['id']: p for p in obs.get('pickups', []) if 'id' in p}
        self.cubes = {c['id']: c for c in obs.get('cubes', []) if 'id' in c}
        self.crates = {c['id']: c for c in obs.get('crates', []) if 'id' in c}


class ApexBrain:
    def __init__(self, world):
        self.world = world

    def dist2d(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def pick_best_upgrade(self, offer):
        choices = offer.get('choices', [])
        if not choices:
            return 0
        priority = ['armor', 'hp', 'regen', 'speed', 'boost', 'energy', 'damage', 'rate', 'sensors']
        best_idx = 0
        best_score = -1
        for i, c in enumerate(choices):
            cid = c.get('id', '').lower()
            score = 1
            for rank, key in enumerate(reversed(priority)):
                if key in cid:
                    score = rank + 10
                    break
            if score > best_score:
                best_score = score
                best_idx = i
        return best_idx

    def compute_aim(self, target_pos, target_vel, target_height, w_kind, w_stats):
        me = self.world.me
        my_pos = me.get('pos', [0, 0, 0])
        muzzle_h = me.get('stats', {}).get('muzzle_height', 7.0)
        if w_kind == 'mortar':
            muzzle_h += 2.5

        mx, my, mz = my_pos[0], my_pos[1], my_pos[2] + muzzle_h
        tx, ty, tz = target_pos[0], target_pos[1], target_pos[2] + (target_height * 0.55)
        vx, vy, vz = target_vel[0], target_vel[1], target_vel[2]

        dx, dy, dz = tx - mx, ty - my, tz - mz
        horiz_dist = max(math.hypot(dx, dy), 0.1)

        if w_kind == 'gun':
            speed = w_stats.get('speed', 252.0)
            t_flight = horiz_dist / max(speed, 1.0)
            px = tx + vx * t_flight - mx
            py = ty + vy * t_flight - my
            pz = tz + vz * t_flight - mz
            return math.atan2(py, px), math.atan2(pz, max(math.hypot(px, py), 0.1))

        elif w_kind == 'mortar':
            speed = w_stats.get('speed', 72.0)
            grav = w_stats.get('gravity', 22.0)
            t_flight = horiz_dist / max(speed * 0.75, 1.0)
            px = tx + vx * t_flight - mx
            py = ty + vy * t_flight - my
            pz = tz + vz * t_flight - mz
            r = max(math.hypot(px, py), 0.1)
            v2 = speed * speed
            disc = v2 * v2 - grav * (grav * r * r + 2.0 * pz * v2)
            yaw = math.atan2(py, px)
            if disc >= 0:
                pitch = math.atan((v2 - math.sqrt(disc)) / (grav * r))
            else:
                pitch = 0.7854
            return yaw, pitch

        else:
            return math.atan2(dy, dx), math.atan2(dz, horiz_dist)

    def steer_around_obstacles(self, my_pos, desired_vx, desired_vy):
        mag = math.hypot(desired_vx, desired_vy)
        if mag < 0.001:
            return 0.0, 0.0, False
        ux, uy = desired_vx / mag, desired_vy / mag
        base_yaw = math.atan2(uy, ux)

        blocked = (
            self.world.map.is_blocked(my_pos[0] + ux * 5.0, my_pos[1] + uy * 5.0) or
            self.world.map.is_blocked(my_pos[0] + ux * 10.0, my_pos[1] + uy * 10.0)
        )
        if not blocked and self.world.stuck_ticks < 4:
            return ux, uy, False

        d = self.world.avoid_dir
        angles = [0.45*d, -0.45*d, 0.9*d, -0.9*d, 1.35*d, -1.35*d, 1.8*d, -1.8*d, 2.4*d, -2.4*d, 3.14]
        for ang in angles:
            tyaw = base_yaw + ang
            tx, ty = math.cos(tyaw), math.sin(tyaw)
            if (not self.world.map.is_blocked(my_pos[0] + tx * 5.0, my_pos[1] + ty * 5.0) and
                not self.world.map.is_blocked(my_pos[0] + tx * 10.0, my_pos[1] + ty * 10.0)):
                return tx, ty, (self.world.stuck_ticks >= 6)

        return ux, uy, True

    def decide_action(self):
        action = {
            "tick": self.world.tick,
            "move": [0.0, 0.0],
            "boost": False,
            "jump": False,
            "fire": [False, False],
            "reload": False
        }

        try:
            me = self.world.me
            if not me or 'pos' not in me:
                return action

            my_pos = me['pos']
            hp = me.get('hp', 1400)
            max_hp = me.get('max_hp', 1400)
            hp_ratio = hp / max(max_hp, 1)
            energy = me.get('energy', 100)
            grounded = me.get('grounded', True)
            in_zone = me.get('in_zone', True)
            weapons = me.get('weapons', [None, None])
            while len(weapons) < 2:
                weapons.append(None)

            if self.world.offer:
                action['pick'] = self.pick_best_upgrade(self.world.offer)

            current_kinds = [w['kind'] for w in weapons if w is not None]
            empty_slots = [i for i, w in enumerate(weapons) if w is None]

            # Weapon pickup & smart swap
            best_pickup = None
            for p in self.world.pickups.values():
                pk = p.get('kind')
                if pk in current_kinds:
                    continue
                d = self.dist2d(my_pos, p.get('pos', [0, 0, 0]))
                dz = abs(my_pos[2] - p.get('pos', [0, 0, 0])[2])
                if d <= 3.9 and dz <= 3.9:
                    if empty_slots:
                        action['take'] = p['id']
                        break
                    else:
                        for idx, w in enumerate(weapons):
                            if w and w['kind'] == 'gun':
                                action['take'] = p['id']
                                action['drop'] = idx
                                break
                elif empty_slots and d < 32.0 and dz <= 4.0:
                    best_pickup = p

            # Zone awareness
            zone = self.world.zone
            z_center = zone.get('next_center', zone.get('center', [0.0, 0.0]))
            z_radius = zone.get('next_radius', zone.get('radius', 300.0))
            dist_to_zcenter = self.dist2d(my_pos, z_center)
            urgency_zone = (not in_zone) or (dist_to_zcenter > z_radius * 0.85)

            nearest_enemy = None
            nearest_enemy_dist = 9999.0
            for e in self.world.enemies.values():
                d = self.dist2d(my_pos, e.get('pos', [0, 0, 0]))
                if d < nearest_enemy_dist:
                    nearest_enemy_dist = d
                    nearest_enemy = e

            # Navigation state machine (Survival-first placement logic)
            goal_x, goal_y = z_center[0], z_center[1]

            if urgency_zone:
                goal_x, goal_y = z_center[0], z_center[1]
                if energy > 35 and dist_to_zcenter > 18.0:
                    action['boost'] = True
            elif self.world.cubes:
                nc = min(self.world.cubes.values(), key=lambda c: self.dist2d(my_pos, c.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, nc['pos']) < 35.0:
                    goal_x, goal_y = nc['pos'][0], nc['pos'][1]
            elif best_pickup:
                goal_x, goal_y = best_pickup['pos'][0], best_pickup['pos'][1]
            elif nearest_enemy:
                epos = nearest_enemy['pos']
                ex, ey = epos[0] - my_pos[0], epos[1] - my_pos[1]
                ed = max(nearest_enemy_dist, 0.1)
                ux, uy = ex / ed, ey / ed
                px, py = -uy * self.world.strafe_dir, ux * self.world.strafe_dir

                ideal_dist = 24.0 if 'lightning' in current_kinds else 50.0

                if hp_ratio < 0.45 or (self.alive > 5 and hp_ratio < 0.65):
                    # Disengage toward safe zone center while strafing
                    zx, zy = z_center[0] - my_pos[0], z_center[1] - my_pos[1]
                    zd = max(math.hypot(zx, zy), 0.1)
                    goal_x = my_pos[0] + (zx / zd) * 24.0 + px * 14.0
                    goal_y = my_pos[1] + (zy / zd) * 24.0 + py * 14.0
                    if energy > 30:
                        action['boost'] = True
                elif ed < ideal_dist - 8.0:
                    goal_x = my_pos[0] - ux * 16.0 + px * 18.0
                    goal_y = my_pos[1] - uy * 16.0 + py * 18.0
                elif ed > ideal_dist + 12.0:
                    goal_x = my_pos[0] + ux * 16.0 + px * 16.0
                    goal_y = my_pos[1] + uy * 16.0 + py * 16.0
                else:
                    goal_x = my_pos[0] + px * 20.0
                    goal_y = my_pos[1] + py * 20.0
            else:
                if dist_to_zcenter > 25.0:
                    goal_x, goal_y = z_center[0], z_center[1]
                else:
                    ang = self.world.time * 0.45
                    goal_x = z_center[0] + math.cos(ang) * 18.0
                    goal_y = z_center[1] + math.sin(ang) * 18.0

            # Projectile Dodge
            dodge_x, dodge_y = 0.0, 0.0
            for proj in self.world.projectiles:
                ppos = proj.get('pos', [0, 0, 0])
                pvel = proj.get('vel', [0, 0, 0])
                rx, ry = my_pos[0] - ppos[0], my_pos[1] - ppos[1]
                pdist = math.hypot(rx, ry)
                if pdist < 32.0:
                    pspd = math.hypot(pvel[0], pvel[1])
                    if pspd > 1.0:
                        vx, vy = pvel[0] / pspd, pvel[1] / pspd
                        if rx * vx + ry * vy > 0:
                            perp_x, perp_y = -vy, vx
                            side = 1.0 if (rx * perp_x + ry * perp_y) >= 0 else -1.0
                            dodge_x += perp_x * side * 24.0
                            dodge_y += perp_y * side * 24.0
                            if energy > 25 and pdist < 16.0:
                                action['boost'] = True

            raw_dx = (goal_x - my_pos[0]) + dodge_x
            raw_dy = (goal_y - my_pos[1]) + dodge_y
            sx, sy, need_jump = self.steer_around_obstacles(my_pos, raw_dx, raw_dy)
            action['move'] = [round(sx, 4), round(sy, 4)]

            if need_jump and energy > 20:
                if grounded:
                    action['jump'] = True
                else:
                    action['boost'] = True

            # Target Selection
            combat_target = None
            target_height = 9.0
            is_crate = False

            if nearest_enemy and nearest_enemy_dist < 190.0:
                combat_target = nearest_enemy
            elif self.world.npcs:
                n_npc = min(self.world.npcs.values(), key=lambda n: self.dist2d(my_pos, n.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, n_npc['pos']) < 70.0:
                    combat_target = n_npc
                    target_height = 3.5
            elif self.world.crates and not nearest_enemy:
                n_crate = min(self.world.crates.values(), key=lambda c: self.dist2d(my_pos, c.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, n_crate['pos']) < 30.0:
                    combat_target = n_crate
                    target_height = 2.0
                    is_crate = True

            if combat_target:
                tpos = combat_target['pos']
                tvel = combat_target.get('vel', [0.0, 0.0, 0.0])
                tdist = self.dist2d(my_pos, tpos)
                actual_aim = me.get('aim', [0.0, 0.0])
                aim_yaw, aim_pitch = None, None

                for idx in (0, 1):
                    w = weapons[idx]
                    if not w:
                        continue
                    wk = w.get('kind', 'gun')
                    ws = w.get('stats', {})
                    cd = w.get('cooldown', 0.0)

                    if wk == 'gun':
                        ammo = w.get('ammo', 12)
                        if ammo == 0 or (ammo <= 3 and tdist > 90.0 and w.get('reload', 0) == 0):
                            action['reload'] = True

                    if aim_yaw is None and cd <= 0.25:
                        aim_yaw, aim_pitch = self.compute_aim(tpos, tvel, target_height, wk, ws)

                    if wk == 'laser':
                        charge = w.get('charge', 0.0)
                        max_c = ws.get('charge', 0.75)
                        min_c = ws.get('min_charge', 0.15)
                        if cd <= 0 and tdist <= ws.get('range', 280.0):
                            yerr = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if (charge >= max_c * 0.90 or (charge >= min_c and combat_target.get('hp', 999) < 120)) and yerr < 0.09:
                                action['fire'][idx] = False
                            else:
                                action['fire'][idx] = True
                        else:
                            action['fire'][idx] = False
                    elif wk == 'lightning':
                        if cd <= 0 and tdist <= ws.get('range', 45.0) - 2.0 and energy > 15:
                            yerr = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yerr < 0.25:
                                action['fire'][idx] = True
                    elif wk == 'mortar':
                        if cd <= 0 and 18.0 <= tdist <= 225.0 and not is_crate:
                            yerr = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yerr < 0.16:
                                action['fire'][idx] = True
                    else:
                        if cd <= 0 and w.get('reload', 0) <= 0 and tdist <= ws.get('range', 230.0):
                            yerr = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yerr < 0.18:
                                action['fire'][idx] = True

                if aim_yaw is not None:
                    action['aim'] = [round(aim_yaw, 4), round(aim_pitch, 4)]
            else:
                scan_yaw = None
                for ev in self.world.events:
                    if ev.get('type') == 'damaged' and ev.get('bearing') is not None:
                        scan_yaw = ev['bearing']
                        break
                if scan_yaw is None and self.world.sounds:
                    ns = [s for s in self.world.sounds if s.get('dist') in ('near', 'mid')]
                    if ns:
                        scan_yaw = ns[0].get('bearing')
                if scan_yaw is None:
                    scan_yaw = (self.world.time * 1.4) % (2 * math.pi) - math.pi
                action['aim'] = [round(scan_yaw, 4), 0.0]

            return action

        except Exception:
            return {"tick": self.world.tick, "move": [0.0, 0.0], "fire": [False, False]}


def run_bot():
    world = None
    brain = None

    while True:
        line = sys.stdin.readline()
        if not line:
            break

        # Drain any queued observations on stdin so we NEVER lag behind the 50ms tick budget
        lines = [line]
        while select.select([sys.stdin], [], [], 0.0)[0]:
            extra = sys.stdin.readline()
            if not extra:
                break
            lines.append(extra)

        latest_obs = None
        ended = False

        for raw in lines:
            raw = raw.strip()
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            mtype = msg.get('type')
            if mtype == 'hello':
                world = WorldState(msg)
                brain = ApexBrain(world)
                print(json.dumps({"type": "ready", "name": "ApexPrime"}), flush=True)
            elif mtype == 'obs':
                latest_obs = msg
            elif mtype == 'end':
                ended = True

        if latest_obs and world and brain:
            world.update(latest_obs)
            action = brain.decide_action()
            print(json.dumps(action), flush=True)

        if ended:
            break


if __name__ == '__main__':
    run_bot()



if __name__ == '__main__':
    run_bot()
