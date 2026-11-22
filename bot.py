import sys
import json
import math

class WorldState:
    def __init__(self, config):
        self.config = config
        self.tick = 0
        self.time = 0.0
        self.me = None
        self.zone = None
        self.enemies = {}
        self.pickups = {}
        self.offer = None
        
    def update(self, obs):
        self.tick = obs['tick']
        self.time = obs['t']
        self.me = obs['self']
        self.zone = obs['zone']
        self.offer = obs.get('offer')
        
        # 5-second enemy memory processing
        for m in obs.get('mechs', []):
            if m['kind'] == 'mech':
                self.enemies[m['id']] = m
                self.enemies[m['id']]['last_seen'] = self.time
                
        # Purge stale memory
        stale = [eid for eid, e in self.enemies.items() if self.time - e.get('last_seen', 0) > 5.0]
        for eid in stale: 
            del self.enemies[eid]
            
        self.pickups = {p['id']: p for p in obs.get('pickups', [])}

class TacticalBrain:
    def __init__(self, world):
        self.world = world

    def distance(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def calculate_aim(self, target, weapon_kind):
        my_pos = self.world.me['pos']
        t_pos = target['pos']
        t_vel = target.get('vel', [0, 0, 0])
        
        dx = t_pos[0] - my_pos[0]
        dy = t_pos[1] - my_pos[1]
        dist = math.hypot(dx, dy)
        
        if weapon_kind == 'gun':
            speed = self.world.config['weapons']['gun']['speed']
            time_of_flight = dist / max(speed, 1)
            # Lead the target
            lead_x = t_pos[0] + (t_vel[0] * time_of_flight) - my_pos[0]
            lead_y = t_pos[1] + (t_vel[1] * time_of_flight) - my_pos[1]
            yaw = math.atan2(lead_y, lead_x)
            return [yaw, 0.0]
            
        elif weapon_kind == 'mortar':
            speed = self.world.config['weapons']['mortar']['speed']
            gravity = self.world.config['weapons']['mortar']['gravity']
            # Calculate ballistic trajectory pitch
            try:
                val = (gravity * dist) / (speed * speed)
                val = max(-1.0, min(1.0, val))
                pitch = 0.5 * math.asin(val)
            except ValueError:
                pitch = 0.785  # Default 45 degrees
            yaw = math.atan2(dy, dx)
            return [yaw, pitch]
            
        else: # laser or lightning (hitscan/instant)
            yaw = math.atan2(dy, dx)
            return [yaw, 0.0]

    def decide_action(self):
        action = {"tick": self.world.tick, "move": [0.0, 0.0], "fire": [False, False]}
        my_pos = self.world.me['pos']
        
        # 1. Zone Survival (Highest Priority)
        next_center = self.world.zone['next_center']
        z_dx = next_center[0] - my_pos[0]
        z_dy = next_center[1] - my_pos[1]
        z_dist = math.hypot(z_dx, z_dy)
        
        if z_dist > 2.0:
            action['move'] = [z_dx/z_dist, z_dy/z_dist]
            
        # 2. Advanced Looting
        empty_slots = [i for i, w in enumerate(self.world.me['weapons']) if w is None]
        if empty_slots and self.world.pickups:
            best_pickup = min(self.world.pickups.values(), key=lambda p: self.distance(my_pos, p['pos']))
            dist_to_loot = self.distance(my_pos, best_pickup['pos'])
            
            if dist_to_loot < 4.0:
                action['take'] = best_pickup['id']
            else:
                # Override move to go get the loot if we need it
                ldx = best_pickup['pos'][0] - my_pos[0]
                ldy = best_pickup['pos'][1] - my_pos[1]
                action['move'] = [ldx/dist_to_loot, ldy/dist_to_loot]

        # 3. Weapon-Specific Combat
        if self.world.enemies:
            # Target closest enemy
            target = min(self.world.enemies.values(), key=lambda e: self.distance(my_pos, e['pos']))
            
            w0 = self.world.me['weapons'][0]
            w1 = self.world.me['weapons'][1]
            
            aim = None
            if w0 and w0.get('cooldown', 0) <= 0:
                aim = self.calculate_aim(target, w0['kind'])
                action['fire'][0] = True
                
            if w1 and w1.get('cooldown', 0) <= 0:
                # Prioritize aiming for the secondary weapon if firing both
                if aim is None: 
                    aim = self.calculate_aim(target, w1['kind'])
                action['fire'][1] = True
                
            if aim:
                action['aim'] = aim
                
        # 4. Instant Upgrades
        if self.world.offer:
            action['pick'] = 0
            
        return action

def run_bot():
    world = None
    brain = None
    
    for line in sys.stdin:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
            
        msg_type = msg.get('type')
        
        if msg_type == 'hello':
            world = WorldState(msg['config'])
            brain = TacticalBrain(world)
            # Answer ready within 20s
            print(json.dumps({"type": "ready", "name": "ApexPredator"}), flush=True)
            
        elif msg_type == 'obs':
            if world:
                world.update(msg)
                action = brain.decide_action()
                print(json.dumps(action), flush=True)
                
        elif msg_type == 'end':
            break

if __name__ == '__main__':
    run_bot()
