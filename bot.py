
import sys
import json
import math


BOT_NAME = "MechSurvivor AI"


def distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = (a[2] if len(a) > 2 else 0) - (
        b[2] if len(b) > 2 else 0
    )
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def normalize(x, y):
    length = math.hypot(x, y)

    if length < 0.001:
        return [0.0, 0.0]

    return [x / length, y / length]


def angle_difference(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def choose_target(obs):
    me = obs["self"]
    my_pos = me["pos"]

    enemies = []

    for enemy in obs.get("mechs", []):
        if enemy.get("id") == me["id"]:
            continue

        if enemy.get("kind") not in ("mech", "npc"):
            continue

        pos = enemy.get("pos")
        if not pos:
            continue

        d = distance(my_pos, pos)

        # Prefer nearby enemies, but also consider weak targets.
        hp = enemy.get("hp", 500)
        score = d + hp * 0.025

        enemies.append((score, enemy))

    if not enemies:
        return None

    enemies.sort(key=lambda item: item[0])

    return enemies[0][1]


def calculate_aim(me, target, weapon):
    my_pos = me["pos"]
    target_pos = target["pos"]

    dx = target_pos[0] - my_pos[0]
    dy = target_pos[1] - my_pos[1]

    dz = target_pos[2] + 4.0 - (my_pos[2] + 7.0)

    d = math.sqrt(dx * dx + dy * dy)

    # Estimate projectile travel time.
    speed = 252.0

    if weapon and weapon.get("stats"):
        speed = weapon["stats"].get("speed", speed)

    if weapon and weapon.get("kind") != "gun":
        if weapon.get("kind") != "mortar":
            speed = 252.0

    travel_time = min(d / max(speed, 1.0), 1.5)

    velocity = target.get("vel", [0, 0, 0])

    # Predict where the enemy will be.
    dx += velocity[0] * travel_time
    dy += velocity[1] * travel_time
    dz += velocity[2] * travel_time

    yaw = math.atan2(dy, dx)
    horizontal = math.hypot(dx, dy)
    pitch = math.atan2(dz, max(horizontal, 0.001))

    return [yaw, pitch], d


def safe_zone_movement(obs):
    me = obs["self"]
    pos = me["pos"]

    zone = obs.get("zone", {})

    center = zone.get("center", [0, 0])
    radius = zone.get("radius", 500)

    next_center = zone.get("next_center", center)
    next_radius = zone.get("next_radius", radius)

    t = obs.get("t", 0)

    start = zone.get("shrink_start", 999999)
    end = zone.get("shrink_end", 999999)

    # Predict the zone during its shrinking phase.
    if end > start and start <= t <= end:
        alpha = (t - start) / (end - start)

        cx = center[0] + (next_center[0] - center[0]) * alpha
        cy = center[1] + (next_center[1] - center[1]) * alpha
        radius = radius + (next_radius - radius) * alpha

    elif t > end:
        cx, cy = next_center
        radius = next_radius

    else:
        cx, cy = center

    dx = cx - pos[0]
    dy = cy - pos[1]

    d = math.hypot(dx, dy)

    if d > radius * 0.75:
        return normalize(dx, dy)

    # Slight movement towards the centre.
    if d > 10:
        return normalize(dx, dy)

    return [0.0, 0.0]


def choose_upgrade(obs):
    offer = obs.get("offer")

    if not offer:
        return None

    choices = offer.get("choices", [])

    priorities = {
        "armor": 10,
        "max_hp": 9,
        "health": 9,
        "gun_damage": 8,
        "gun_rate": 8,
        "damage": 8,
        "energy": 6,
        "sensors": 5,
    }

    best_index = 0
    best_score = -1

    for i, choice in enumerate(choices):
        upgrade_id = choice.get("id", "")
        score = priorities.get(upgrade_id, 1)

        if score > best_score:
            best_score = score
            best_index = i

    return best_index


def make_action(obs):
    me = obs["self"]

    # Default movement: stay within the safe zone.
    move = safe_zone_movement(obs)

    target = choose_target(obs)

    aim = me.get("aim", [0.0, 0.0])

    fire = [False, False]
    reload = False

    weapons = me.get("weapons", [None, None])

    if target:
        # Prefer a usable weapon for aiming.
        weapon = weapons[0]

        if weapon is None and len(weapons) > 1:
            weapon = weapons[1]

        aim, target_distance = calculate_aim(
            me, target, weapon
        )

        for slot, current_weapon in enumerate(weapons):
            if current_weapon is None:
                continue

            kind = current_weapon.get("kind")
            cooldown = current_weapon.get("cooldown", 0)

            if cooldown > 0:
                continue

            if kind == "gun":
                ammo = current_weapon.get("ammo", 0)
                reloading = current_weapon.get("reload", 0)

                if ammo <= 0:
                    reload = reloading <= 0
                else:
                    fire[slot] = True

            elif kind == "lightning":
                if target_distance <= 42:
                    fire[slot] = True

            elif kind == "laser":
                charge = current_weapon.get("charge", 0)

                if charge >= 0.65:
                    fire[slot] = False
                else:
                    fire[slot] = True

            elif kind == "mortar":
                if target_distance < 180:
                    fire[slot] = True

        # Keep some distance from nearby enemies.
        if target_distance < 35:
            dx = me["pos"][0] - target["pos"][0]
            dy = me["pos"][1] - target["pos"][1]
            move = normalize(dx, dy)

    # Face the direction of incoming attacks.
    for event in obs.get("events", []):
        if event.get("type") == "damaged":
            bearing = event.get("bearing")

            if bearing is not None:
                aim[0] = bearing

    # Upgrade selection.
    pick = choose_upgrade(obs)

    action = {
        "tick": obs["tick"],
        "move": move,
        "boost": me.get("energy", 0) > 25,
        "jump": False,
        "aim": aim,
        "fire": fire,
        "reload": reload,
    }

    if pick is not None:
        action["pick"] = pick

    return action


def main():
    for line in sys.stdin:
        try:
            message = json.loads(line)

            msg_type = message.get("type")

            if msg_type == "hello":
                response = {
                    "type": "ready",
                    "name": BOT_NAME
                }

            elif msg_type == "obs":
                response = make_action(message)

            elif msg_type == "end":
                break

            else:
                continue

            print(json.dumps(response, separators=(",", ":")),
                  flush=True)

        except Exception as error:
            # Never allow an unexpected observation to crash the bot.
            print(
                "Bot error: " + str(error),
                file=sys.stderr,
                flush=True
            )


if __name__ == "__main__":
    main()
