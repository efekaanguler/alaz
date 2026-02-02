#!/usr/bin/env python3

import argparse
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import carla
import yaml


# ----------------------------
# YAML helpers
# ----------------------------
def read_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a dict: {path}")
    return data


def resolve_path(base_file: str, rel: str) -> str:
    base_dir = os.path.dirname(os.path.abspath(base_file))
    return os.path.abspath(os.path.join(base_dir, rel))


def get_by_dotted_path(root: Dict[str, Any], dotted: str) -> Dict[str, Any]:
    """
    Example:
      dotted = "presets.rgb_720p_20hz_fov90"
      root   = contents of camera_rgb.yaml
    """
    cur: Any = root
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise KeyError(f"Preset path not found: {dotted} (missing '{part}')")
        cur = cur[part]
    if not isinstance(cur, dict):
        raise ValueError(f"Dotted path must resolve to dict: {dotted}")
    return cur


# ----------------------------
# CARLA helpers
# ----------------------------
def connect(host: str, port: int, timeout: float) -> carla.Client:
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def apply_world_settings(world: carla.World, sync: bool, fixed_dt: float,
                         max_substeps: int = 10, max_substep_dt: float = 0.01) -> None:
    settings = world.get_settings()
    settings.synchronous_mode = sync
    settings.fixed_delta_seconds = fixed_dt if sync else None
    settings.max_substeps = max_substeps
    settings.max_substep_delta_time = max_substep_dt
    world.apply_settings(settings)


def to_transform(d: Dict[str, Any]) -> carla.Transform:
    loc = carla.Location(
        x=float(d.get("x", 0.0)),
        y=float(d.get("y", 0.0)),
        z=float(d.get("z", 0.0)),
    )
    rot = carla.Rotation(
        roll=float(d.get("roll", 0.0)),
        pitch=float(d.get("pitch", 0.0)),
        yaw=float(d.get("yaw", 0.0)),
    )
    return carla.Transform(loc, rot)


def destroy_ego_and_children(world: carla.World, ego_role_name: str) -> int:
    """
    Destroy ego vehicle(s) with matching role_name and all attached children sensors.
    Returns destroyed actor count.
    """
    destroyed = 0
    actors = world.get_actors()
    vehicles = actors.filter("vehicle.*")

    targets: List[carla.Actor] = []
    for v in vehicles:
        if v.attributes.get("role_name") == ego_role_name:
            targets.append(v)
            # attached sensors
            for child in v.get_children():
                targets.append(child)

    # destroy children first, then vehicle
    for a in reversed(targets):
        try:
            a.destroy()
            destroyed += 1
        except Exception:
            pass

    return destroyed


def pick_spawn_point(world: carla.World, fixed_index: int, attempt: int) -> carla.Transform:
    sps = world.get_map().get_spawn_points()
    if not sps:
        raise RuntimeError("No spawn points on this map.")
    idx = fixed_index + attempt
    if idx >= len(sps):
        idx = len(sps) - 1
    if idx < 0:
        idx = 0
    return sps[idx]


# ----------------------------
# Sensor profile loading
# ----------------------------
def load_sensor_profile(profile_file: str) -> Dict[str, Any]:
    """
    Loads:
      - sensor_profile.yaml
      - imported preset yamls
    Expands physical_sensors into fully resolved {type, attributes, attach, role_name}.
    """
    profile = read_yaml(profile_file)

    # Load imports
    imports = profile.get("imports", {})
    loaded: Dict[str, Dict[str, Any]] = {}
    for alias, rel in imports.items():
        full = resolve_path(profile_file, rel)
        loaded[alias] = read_yaml(full)

    expanded_physical: List[Dict[str, Any]] = []
    for s in profile.get("physical_sensors", []):
        if "from_preset" not in s:
            raise ValueError("physical_sensors entry missing from_preset")

        fp = str(s["from_preset"])
        # expected form: "<alias>.<dotted path inside imported yaml>"
        alias, rest = fp.split(".", 1)
        if alias not in loaded:
            raise KeyError(f"Unknown import alias '{alias}' in from_preset='{fp}'")

        preset = get_by_dotted_path(loaded[alias], rest)  # dict containing {type, attributes}
        if "type" not in preset:
            raise ValueError(f"Preset missing 'type': {fp}")

        merged = {
            "id": s.get("id", ""),
            "role_name": s["role_name"],
            "type": preset["type"],
            "attributes": dict(preset.get("attributes", {})),
            "attach": dict(s.get("attach", {})),
            "expected_ros_topics": dict(s.get("expected_ros_topics", {})),
        }
        expanded_physical.append(merged)

    return {
        "profile_name": profile.get("profile_name", "unknown"),
        "physical_sensors": expanded_physical,
        "virtual_sensors": profile.get("virtual_sensors", []),
        "raw": profile,
    }


# ----------------------------
# Main
# ----------------------------
def main():
    ap = argparse.ArgumentParser(description="Load CARLA scenario from YAML and spawn ego + sensors.")
    ap.add_argument("--config", default="../scenario/default.yaml", help="Path to scenario default.yaml")
    ap.add_argument("--host", default=os.environ.get("CARLA_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    ap.add_argument("--timeout", type=float, default=float(os.environ.get("CARLA_TIMEOUT", "10")))
    ap.add_argument("--cleanup", action="store_true", help="Destroy existing ego_vehicle actors before spawn")
    ap.add_argument("--dry-run", action="store_true", help="Only print actions, do not spawn")
    args = ap.parse_args()

    cfg_file = os.path.abspath(args.config)
    cfg = read_yaml(cfg_file)

    client = connect(args.host, args.port, args.timeout)

    # 1) Load map
    map_name = cfg["world"]["map"]
    print(f"[scenario_loader] Loading map: {map_name}")
    world = client.load_world(map_name)

    # 2) Apply world settings
    wcfg = cfg["world"]
    phys = wcfg.get("physics", {})
    sync = bool(wcfg.get("synchronous_mode", True))
    fixed_dt = float(wcfg.get("fixed_delta_seconds", 0.05))
    max_substeps = int(phys.get("max_substeps", 10))
    max_substep_dt = float(phys.get("max_substep_delta_time", 0.01))

    print(f"[scenario_loader] World settings: sync={sync}, fixed_dt={fixed_dt}")
    if not args.dry_run:
        apply_world_settings(world, sync, fixed_dt, max_substeps, max_substep_dt)

    # 3) Ego vehicle config
    ecfg = cfg["ego_vehicle"]
    ego_blueprint = ecfg["blueprint"]
    ego_role = ecfg.get("role_name", "ego_vehicle")
    autopilot = bool(ecfg.get("autopilot", False))

    spawn_cfg = ecfg.get("spawn", {})
    fixed_index = int(spawn_cfg.get("fixed_index", 0))
    retry_cfg = spawn_cfg.get("retry_policy", {})
    retry_enabled = bool(retry_cfg.get("enabled", True))
    max_retries = int(retry_cfg.get("max_retries", 20))

    # cleanup (optional)
    if args.cleanup and not args.dry_run:
        destroyed = destroy_ego_and_children(world, ego_role)
        if destroyed:
            print(f"[scenario_loader] Cleanup: destroyed {destroyed} actors for role_name={ego_role}")

    # spawn ego
    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.find(ego_blueprint)
    ego_bp.set_attribute("role_name", ego_role)

    vehicle: Optional[carla.Actor] = None
    attempts = max_retries if retry_enabled else 1

    print(f"[scenario_loader] Spawning ego vehicle: {ego_blueprint} role_name={ego_role} at fixed_index={fixed_index}")
    if args.dry_run:
        print("[scenario_loader] dry-run: skipping actual spawn")
        return

    for attempt in range(attempts):
        sp = pick_spawn_point(world, fixed_index=fixed_index, attempt=attempt)
        vehicle = world.try_spawn_actor(ego_bp, sp)
        if vehicle is not None:
            break

    if vehicle is None:
        raise RuntimeError("Failed to spawn ego vehicle (all retries failed).")

    print(f"[scenario_loader] Ego spawned: id={vehicle.id} type={vehicle.type_id}")

    # autopilot?
    if autopilot:
        vehicle.set_autopilot(True)
        print("[scenario_loader] Autopilot: enabled")
    else:
        print("[scenario_loader] Autopilot: disabled (expect ROS2 Ackermann control)")

    # 4) Sensors
    sensors_block = cfg.get("sensors", {})
    profile_rel = sensors_block.get("profile_file")
    if not profile_rel:
        print("[scenario_loader] No sensors.profile_file set; skipping sensor spawn.")
    else:
        profile_file = resolve_path(cfg_file, profile_rel) if not os.path.isabs(profile_rel) else profile_rel
        print(f"[scenario_loader] Loading sensor profile: {profile_file}")
        prof = load_sensor_profile(profile_file)

        print(f"[scenario_loader] Sensor profile: {prof['profile_name']}")
        print(f"[scenario_loader] Spawning {len(prof['physical_sensors'])} physical sensors...")

        for s in prof["physical_sensors"]:
            sbp = bp_lib.find(s["type"])
            sbp.set_attribute("role_name", s["role_name"])
            for k, v in s.get("attributes", {}).items():
                sbp.set_attribute(str(k), str(v))

            tf = to_transform(s["attach"])
            actor = world.spawn_actor(sbp, tf, attach_to=vehicle)
            print(f"[scenario_loader] Sensor spawned: id={actor.id} role={s['role_name']} type={s['type']}")

        # virtual sensors (no spawn)
        vs = prof.get("virtual_sensors", [])
        if vs:
            print("[scenario_loader] Virtual sensors (no CARLA actor spawned):")
            for item in vs:
                print("  -", item.get("id", "<unknown>"), item.get("from_virtual", ""))

    # 5) Tick a bit so bridge can discover actors
    if world.get_settings().synchronous_mode:
        for _ in range(10):
            world.tick()
    else:
        time.sleep(1.0)

    print("[scenario_loader] Done. Now start carla_ros_bridge; topics should appear.")


if __name__ == "__main__":
    main()