import sys
import json
import math
import os
import struct

class WorldMap:
    def __init__(self, map_info):
        self.terrain = None
        self.nx = 0
        self.ny = 0
        self.min_x = 0.0
        self.min_y = 0.0
        self.step = 1.0
        self.buildings = []
        
        if not map_info:
            return
            
        map_dir = map_info.get('dir', '/arena')
        world_path = os.path.join(map_dir, 'v2', 'world.json')
        terrain_path = os.path.join(map_dir, 'v2', 'terrain.i16')
        
        try:
            if os.path.exists(world_path):
                with open(world_path, 'r') as f:
                    wdata = json.load(f)
                    t_info = wdata.get('terrain', {})
                    self.nx = t_info.get('nx', 1070)
                    self.ny = t_info.get('ny', 724)
                    self.min_x = t_info.get('minX', 0.0)
                    self.min_y = t_info.get('minY', 0.0)
                    self.step = t_info.get('step', 1.0)
                    
                    for p in wdata.get('prisms', []):
                        if p.get('kind') in ('building', 'stadium'):
                            ring = p.get('ring', [])
                            if len(ring) >= 6:
                                xs = ring[0::2]
                                ys = ring[1::2]
                                self.buildings.append({
                                    'min_x': min(xs) - 2.6,
                                    'max_x': max(xs) + 2.6,
                                    'min_y': min(ys) - 2.6,
                                    'max_y': max(ys) + 2.6,
                                    'base': p.get('base', -100),
                                    'top': p.get('top', 100)
                                })
        except Exception:
            pass

        try:
            if os.path.exists(terrain_path) and self.nx > 0 and self.ny > 0:
                with open(terrain_path, 'rb') as f:
                    raw = f.read()
                    count = self.nx * self.ny
                    if len(raw) >= count * 2:
                        self.terrain = struct.unpack(f'<{count}h', raw[:count*2])
        except Exception:
            pass

    def get_ground_z(self, x, y):
        if not self.terrain:
            return 0.0
        fx = (x - self.min_x) / self.step
        fy = (y - self.min_y) / self.step
        ix = max(0, min(self.nx - 2, int(fx)))
        iy = max(0, min(self.ny - 2, int(fy)))
        dx = max(0.0, min(1.0, fx - ix))
        dy = max(0.0, min(1.0, fy - iy))
        
        idx00 = iy * self.nx + ix
        idx10 = iy * self.nx + (ix + 1)
        idx01 = (iy + 1) * self.nx + ix
        idx11 = (iy + 1) * self.nx + (ix + 1)
        
        z00 = self.terrain[idx00] * 0.01
        z10 = self.terrain[idx10] * 0.01
        z01 = self.terrain[idx01] * 0.01
        z11 = self.terrain[idx11] * 0.01
        
        return (z00 * (1 - dx) * (1 - dy) +
                z10 * dx * (1 - dy) +
                z01 * (1 - dx) * dy +
                z11 * dx * dy)

    def is_blocked(self, x, y, z):
        feet_z = z
        head_z = z + 8.5
        for b in self.buildings:
            if b['min_x'] <= x <= b['max_x'] and b['min_y'] <= y <= b['max_y']:
                if head_z > b['base'] + 1.2 and feet_z < b['top']:
                    return True
        return False


class WorldState:
    def __init__(self, hello_msg):
        self.config = hello_msg.get('config', {})
        self.map = WorldMap(hello_msg.get('map', {}))
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
            else:
                self.stuck_ticks = 0
        if pos:
            self.last_pos = list(pos)

        if self.tick % 35 == 0:
            self.strafe_dir *= -1.0

        for m in obs.get('mechs', []):
            kind = m.get('kind')
            mid = m.get('id')
            if mid is None:
                continue
            m['last_seen'] = self.time
            if kind == 'mech':
                self.enemies[mid] = m
            elif kind == 'npc':
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

    def dist3d(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

    def pick_best_upgrade(self, offer):
        choices = offer.get('choices', [])
        if not choices:
            return 0
        priority = [
            'armor', 'hp', 'regen', 'speed', 'boost', 'energy',
            'gun_damage', 'gun_rate', 'laser_damage', 'mortar_damage',
            'lightning_damage', 'sensors'
        ]
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

    def evaluate_weapon_desirability(self, kind, current_kinds, enemy_kinds):
        if kind in current_kinds:
            return -100
        score = 50
        # Counter the weapon triangle
        if 'mortar' in enemy_kinds and kind == 'laser':
            score += 40
        if 'lightning' in enemy_kinds and kind == 'mortar':
            score += 40
        if 'laser' in enemy_kinds and kind == 'lightning':
            score += 40
        if kind == 'laser':
            score += 20
        elif kind == 'lightning':
            score += 18
        elif kind == 'mortar':
            score += 15
        return score

    def compute_aim(self, target_pos, target_vel, target_height, w_kind, w_stats):
        me = self.world.me
        my_pos = me.get('pos', [0, 0, 0])
        my_stats = me.get('stats', {})
        muzzle_h = my_stats.get('muzzle_height', 7.0)
        if w_kind == 'mortar':
            muzzle_h += 2.5

        mx, my, mz = my_pos[0], my_pos[1], my_pos[2] + muzzle_h
        tx, ty, tz = target_pos[0], target_pos[1], target_pos[2] + (target_height * 0.55)
        vx, vy, vz = target_vel[0], target_vel[1], target_vel[2]

        dx = tx - mx
        dy = ty - my
        dz = tz - mz
        horiz_dist = max(math.hypot(dx, dy), 0.1)

        if w_kind == 'gun':
            speed = w_stats.get('speed', 252.0)
            # 2-step iterative lead solver
            t_flight = horiz_dist / max(speed, 1.0)
            for _ in range(2):
                px = tx + vx * t_flight - mx
                py = ty + vy * t_flight - my
                pz = tz + vz * t_flight - mz
                d3 = max(math.sqrt(px*px + py*py + pz*pz), 0.1)
                t_flight = d3 / max(speed, 1.0)
            yaw = math.atan2(py, px)
            pitch = math.atan2(pz, max(math.hypot(px, py), 0.1))
            return yaw, pitch

        elif w_kind == 'mortar':
            speed = w_stats.get('speed', 72.0)
            grav = w_stats.get('gravity', 22.0)
            t_flight = horiz_dist / max(speed * 0.75, 1.0)
            px = tx + vx * t_flight - mx
            py = ty + vy * t_flight - my
            pz = tz + vz * t_flight - mz
            r = max(math.hypot(px, py), 0.1)
            v2 = speed * speed
            v4 = v2 * v2
            disc = v4 - grav * (grav * r * r + 2.0 * pz * v2)
            yaw = math.atan2(py, px)
            if disc >= 0:
                # Low-arc direct trajectory for faster impact
                pitch = math.atan((v2 - math.sqrt(disc)) / (grav * r))
            else:
                pitch = 0.7854
            return yaw, pitch

        else:
            # Laser and Lightning are instant hitscan/arc
            yaw = math.atan2(dy, dx)
            pitch = math.atan2(dz, horiz_dist)
            return yaw, pitch

    def steer_around_obstacles(self, my_pos, desired_vx, desired_vy):
        mag = math.hypot(desired_vx, desired_vy)
        if mag < 0.001:
            return 0.0, 0.0, False
        ux, uy = desired_vx / mag, desired_vy / mag
        base_yaw = math.atan2(uy, ux)
        
        # Check probe points ahead
        probe_dists = [4.5, 9.0]
        blocked_ahead = False
        for pd in probe_dists:
            if self.world.map.is_blocked(my_pos[0] + ux * pd, my_pos[1] + uy * pd, my_pos[2]):
                blocked_ahead = True
                break
                
        if not blocked_ahead and self.world.stuck_ticks < 4:
            return ux, uy, False

        # Fan search for clear steering angle
        angles = [0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4, 1.8, -1.8, 2.3, -2.3, 3.14]
        for ang in angles:
            test_yaw = base_yaw + ang
            tx, ty = math.cos(test_yaw), math.sin(test_yaw)
            clear = True
            for pd in probe_dists:
                if self.world.map.is_blocked(my_pos[0] + tx * pd, my_pos[1] + ty * pd, my_pos[2]):
                    clear = False
                    break
            if clear:
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

            # 1. Upgrade selection
            if self.world.offer:
                action['pick'] = self.pick_best_upgrade(self.world.offer)

            # Collect known enemy weapon types
            enemy_kinds = set()
            for e in self.world.enemies.values():
                for wk in e.get('weapons', []):
                    enemy_kinds.add(wk)

            # 2. Weapon Pickup / Swap Management
            current_kinds = [w['kind'] for w in weapons if w is not None]
            empty_slots = [i for i, w in enumerate(weapons) if w is None]
            
            best_pickup = None
            best_pickup_score = 0
            for p in self.world.pickups.values():
                p_kind = p.get('kind')
                d = self.dist2d(my_pos, p.get('pos', [0, 0, 0]))
                dz = abs(my_pos[2] - p.get('pos', [0, 0, 0])[2])
                if dz > 5.0:
                    continue
                desire = self.evaluate_weapon_desirability(p_kind, current_kinds, enemy_kinds)
                if desire > 0 and d < 45.0:
                    sc = desire - d
                    if sc > best_pickup_score:
                        best_pickup_score = sc
                        best_pickup = p

            if best_pickup:
                p_dist = self.dist2d(my_pos, best_pickup['pos'])
                p_dz = abs(my_pos[2] - best_pickup['pos'][2])
                if p_dist <= 3.9 and p_dz <= 3.9:
                    if empty_slots:
                        action['take'] = best_pickup['id']
                    else:
                        # Replace gun if we have two slots full and found a better counter weapon
                        for idx, w in enumerate(weapons):
                            if w and w['kind'] == 'gun':
                                action['take'] = best_pickup['id']
                                action['drop'] = idx
                                break

            # 3. Movement Goal Calculation
            zone = self.world.zone
            z_center = zone.get('next_center', zone.get('center', [0.0, 0.0]))
            z_radius = zone.get('next_radius', zone.get('radius', 300.0))
            dist_to_zcenter = self.dist2d(my_pos, z_center)

            goal_x, goal_y = z_center[0], z_center[1]
            urgency_zone = (not in_zone) or (dist_to_zcenter > z_radius * 0.85)

            nearest_enemy = None
            nearest_enemy_dist = 9999.0
            for e in self.world.enemies.values():
                d = self.dist2d(my_pos, e.get('pos', [0, 0, 0]))
                if d < nearest_enemy_dist:
                    nearest_enemy_dist = d
                    nearest_enemy = e

            # Check safe supply drops
            active_drop = None
            if not urgency_zone and (hp_ratio < 0.7 or me.get('cubes', 0) < 6):
                for dr in self.world.drops:
                    if dr.get('lands_in', 99) < 8.0:
                        dd = self.dist2d(my_pos, dr.get('pos', [0, 0, 0]))
                        if dd < 75.0 and (not nearest_enemy or nearest_enemy_dist > 35.0):
                            active_drop = dr
                            break

            # Priority navigation state machine
            if urgency_zone:
                goal_x, goal_y = z_center[0], z_center[1]
                if energy > 35 and dist_to_zcenter > 20.0:
                    action['boost'] = True
            elif active_drop:
                goal_x, goal_y = active_drop['pos'][0], active_drop['pos'][1]
                if energy > 50 and self.dist2d(my_pos, active_drop['pos']) > 12.0:
                    action['boost'] = True
            elif self.world.cubes:
                # Grab nearby power cubes
                nearest_cube = min(self.world.cubes.values(), key=lambda c: self.dist2d(my_pos, c.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, nearest_cube['pos']) < 35.0:
                    goal_x, goal_y = nearest_cube['pos'][0], nearest_cube['pos'][1]
            elif best_pickup and empty_slots and self.dist2d(my_pos, best_pickup['pos']) < 30.0:
                goal_x, goal_y = best_pickup['pos'][0], best_pickup['pos'][1]
            elif nearest_enemy:
                epos = nearest_enemy['pos']
                ex, ey = epos[0] - my_pos[0], epos[1] - my_pos[1]
                ed = max(nearest_enemy_dist, 0.1)
                ux, uy = ex / ed, ey / ed
                px, py = -uy * self.world.strafe_dir, ux * self.world.strafe_dir

                # Optimal engagement range depends on loadout and health
                ideal_dist = 32.0
                if 'lightning' in current_kinds:
                    ideal_dist = 22.0
                elif 'laser' in current_kinds or 'mortar' in current_kinds:
                    ideal_dist = 55.0

                if hp_ratio < 0.35 or self.alive > 6 and hp_ratio < 0.55:
                    # Tactical retreat toward zone center while strafing
                    zx, zy = z_center[0] - my_pos[0], z_center[1] - my_pos[1]
                    zd = max(math.hypot(zx, zy), 0.1)
                    goal_x = my_pos[0] + (zx / zd) * 20.0 + px * 12.0
                    goal_y = my_pos[1] + (zy / zd) * 20.0 + py * 12.0
                    if energy > 30:
                        action['boost'] = True
                elif ed < ideal_dist - 8.0:
                    # Kite backwards + strafe
                    goal_x = my_pos[0] - ux * 15.0 + px * 18.0
                    goal_y = my_pos[1] - uy * 15.0 + py * 18.0
                elif ed > ideal_dist + 12.0:
                    # Close distance + strafe
                    goal_x = my_pos[0] + ux * 18.0 + px * 14.0
                    goal_y = my_pos[1] + uy * 18.0 + py * 14.0
                else:
                    # Pure circle strafe at optimal range
                    goal_x = my_pos[0] + px * 20.0
                    goal_y = my_pos[1] + py * 20.0
            else:
                # Patrol slowly near zone center to conserve energy and avoid unnecessary noise
                if dist_to_zcenter > 25.0:
                    goal_x, goal_y = z_center[0], z_center[1]
                else:
                    ang = self.world.time * 0.4
                    goal_x = z_center[0] + math.cos(ang) * 18.0
                    goal_y = z_center[1] + math.sin(ang) * 18.0

            # Projectile Dodge Override
            dodge_x, dodge_y = 0.0, 0.0
            for proj in self.world.projectiles:
                ppos = proj.get('pos', [0, 0, 0])
                pvel = proj.get('vel', [0, 0, 0])
                rx, ry = my_pos[0] - ppos[0], my_pos[1] - ppos[1]
                pdist = math.hypot(rx, ry)
                if pdist < 35.0:
                    pspd = math.hypot(pvel[0], pvel[1])
                    if pspd > 1.0:
                        vx, vy = pvel[0] / pspd, pvel[1] / pspd
                        # Check if projectile is heading toward us
                        dot = rx * vx + ry * vy
                        if dot > 0:
                            perp_x, perp_y = -vy, vx
                            side = 1.0 if (rx * perp_x + ry * perp_y) >= 0 else -1.0
                            dodge_x += perp_x * side * 25.0
                            dodge_y += perp_y * side * 25.0
                            if energy > 25 and pdist < 18.0:
                                action['boost'] = True

            raw_dx = (goal_x - my_pos[0]) + dodge_x
            raw_dy = (goal_y - my_pos[1]) + dodge_y
            steer_x, steer_y, need_jump = self.steer_around_obstacles(my_pos, raw_dx, raw_dy)
            action['move'] = [round(steer_x, 4), round(steer_y, 4)]

            if need_jump and energy > 20:
                if grounded:
                    action['jump'] = True
                else:
                    action['boost'] = True

            # 4. Target Selection (Enemies > NPCs > Crates)
            combat_target = None
            target_height = 9.0
            is_crate = False

            if nearest_enemy and nearest_enemy_dist < 190.0:
                combat_target = nearest_enemy
                target_height = 9.0
            elif self.world.npcs:
                nearest_npc = min(self.world.npcs.values(), key=lambda n: self.dist2d(my_pos, n.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, nearest_npc['pos']) < 75.0:
                    combat_target = nearest_npc
                    target_height = 3.5
            elif self.world.crates and not nearest_enemy:
                nearest_crate = min(self.world.crates.values(), key=lambda c: self.dist2d(my_pos, c.get('pos', [0, 0, 0])))
                if self.dist2d(my_pos, nearest_crate['pos']) < 35.0:
                    combat_target = nearest_crate
                    target_height = 2.0
                    is_crate = True

            # 5. Aiming & Firing Logic (Including Laser Charge/Release & Rear Armor Protection)
            if combat_target:
                tpos = combat_target['pos']
                tvel = combat_target.get('vel', [0.0, 0.0, 0.0])
                tdist = self.dist2d(my_pos, tpos)
                actual_aim = me.get('aim', [0.0, 0.0])

                aim_yaw, aim_pitch = None, None

                for slot_idx in (0, 1):
                    w = weapons[slot_idx]
                    if not w:
                        continue
                    w_kind = w.get('kind', 'gun')
                    w_stats = w.get('stats', {})
                    cd = w.get('cooldown', 0.0)

                    # Auto-reload gun during downtime
                    if w_kind == 'gun':
                        ammo = w.get('ammo', 12)
                        rel = w.get('reload', 0.0)
                        if ammo == 0 or (ammo <= 3 and tdist > 90.0 and rel == 0):
                            action['reload'] = True

                    # Compute optimal aim for primary active weapon
                    if aim_yaw is None and cd <= 0.25:
                        aim_yaw, aim_pitch = self.compute_aim(tpos, tvel, target_height, w_kind, w_stats)

                    # Firing conditions per weapon type
                    if w_kind == 'laser':
                        charge = w.get('charge', 0.0)
                        max_charge = w_stats.get('charge', 0.75)
                        min_charge = w_stats.get('min_charge', 0.15)
                        w_range = w_stats.get('range', 280.0)
                        if cd <= 0 and tdist <= w_range:
                            yaw_err = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if charge >= max_charge * 0.92 and yaw_err < 0.08:
                                # Release trigger to fire full-power beam!
                                action['fire'][slot_idx] = False
                            elif charge >= min_charge and combat_target.get('hp', 999) < 120 and yaw_err < 0.08:
                                # Finish low HP target immediately
                                action['fire'][slot_idx] = False
                            else:
                                # Hold trigger to build charge
                                action['fire'][slot_idx] = True
                        else:
                            action['fire'][slot_idx] = False

                    elif w_kind == 'lightning':
                        w_range = w_stats.get('range', 45.0)
                        if cd <= 0 and tdist <= w_range - 2.0 and energy > 15:
                            yaw_err = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yaw_err < 0.25:
                                action['fire'][slot_idx] = True

                    elif w_kind == 'mortar':
                        w_range = 225.0
                        # Never fire mortar if target is too close (prevents self-splash & self-concussion)
                        if cd <= 0 and 18.0 <= tdist <= w_range and not is_crate:
                            yaw_err = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yaw_err < 0.15:
                                action['fire'][slot_idx] = True

                    else:  # gun
                        w_range = w_stats.get('range', 230.0)
                        if cd <= 0 and w.get('reload', 0) <= 0 and tdist <= w_range:
                            yaw_err = abs((aim_yaw - actual_aim[0] + math.pi) % (2 * math.pi) - math.pi) if aim_yaw is not None else 1.0
                            if yaw_err < 0.18:
                                action['fire'][slot_idx] = True

                if aim_yaw is not None:
                    action['aim'] = [round(aim_yaw, 4), round(aim_pitch, 4)]

            else:
                # No visible target: look toward recent damage or sound bearing to protect rear armor (1.5x mult!)
                scan_yaw = None
                for ev in self.world.events:
                    if ev.get('type') == 'damaged' and ev.get('bearing') is not None:
                        scan_yaw = ev['bearing']
                        break
                if scan_yaw is None and self.world.sounds:
                    # Face nearest sound source
                    near_sounds = [s for s in self.world.sounds if s.get('dist') in ('near', 'mid')]
                    if near_sounds:
                        scan_yaw = near_sounds[0].get('bearing')
                if scan_yaw is None:
                    # Sweep torso slowly to maintain 360-degree vision coverage
                    scan_yaw = (self.world.time * 1.4) % (2 * math.pi) - math.pi

                action['aim'] = [round(scan_yaw, 4), 0.0]

            return action

        except Exception:
            return {"tick": self.world.tick, "move": [0.0, 0.0], "fire": [False, False]}


def run_bot():
    world = None
    brain = None

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        msg_type = msg.get('type')

        if msg_type == 'hello':
            world = WorldState(msg)
            brain = ApexBrain(world)
            print(json.dumps({"type": "ready", "name": "ApexOmega"}), flush=True)

        elif msg_type == 'obs':
            if world and brain:
                world.update(msg)
                action = brain.decide_action()
                print(json.dumps(action), flush=True)

        elif msg_type == 'end':
            break


if __name__ == '__main__':
    run_bot()
