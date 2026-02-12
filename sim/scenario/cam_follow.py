#!/usr/bin/env python3

import argparse
import os
import time
import math

import carla


def main():
    ap = argparse.ArgumentParser(
        description="Follow ego vehicle with CARLA spectator camera"
    )
    ap.add_argument(
        "--host", 
        default=os.environ.get("CARLA_HOST", "localhost"),
        help="CARLA server host"
    )
    ap.add_argument(
        "--port", 
        type=int, 
        default=int(os.environ.get("CARLA_PORT", "2000")),
        help="CARLA server port"
    )
    ap.add_argument(
        "--timeout", 
        type=float, 
        default=float(os.environ.get("CARLA_TIMEOUT", "10.0")),
        help="Connection timeout"
    )
    ap.add_argument(
        "--role-name", 
        default="ego_vehicle",
        help="Role name of ego vehicle to follow"
    )
    ap.add_argument(
        "--distance", 
        type=float, 
        default=8.0,
        help="Camera distance behind vehicle (meters)"
    )
    ap.add_argument(
        "--height", 
        type=float, 
        default=4.0,
        help="Camera height above vehicle (meters)"
    )
    ap.add_argument(
        "--update-rate", 
        type=float, 
        default=0.0,
        help="Update rate in seconds (0 = every tick for sync mode)"
    )
    args = ap.parse_args()

    # Connect to CARLA
    print(f"[cam_follow] Connecting to CARLA at {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    
    world = client.get_world()
    spectator = world.get_spectator()
    
    print(f"[cam_follow] Looking for ego vehicle with role_name='{args.role_name}'")
    
    # Check if world is in synchronous mode
    settings = world.get_settings()
    is_sync = settings.synchronous_mode
    print(f"[cam_follow] World synchronous_mode: {is_sync}")
    
    try:
        while True:
            # Find ego vehicle
            ego_vehicle = None
            actors = world.get_actors().filter("vehicle.*")
            
            for actor in actors:
                if actor.attributes.get("role_name") == args.role_name:
                    ego_vehicle = actor
                    break
            
            if ego_vehicle is None:
                print(f"[cam_follow] Ego vehicle '{args.role_name}' not found, waiting...")
                time.sleep(1.0)
                continue
            
            # Get vehicle transform
            vehicle_transform = ego_vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            vehicle_rotation = vehicle_transform.rotation
            
            # Calculate camera position behind and above the vehicle
            # Convert yaw to radians
            yaw_rad = math.radians(vehicle_rotation.yaw)
            
            # Position camera behind the vehicle
            camera_location = carla.Location(
                x=vehicle_location.x - args.distance * math.cos(yaw_rad),
                y=vehicle_location.y - args.distance * math.sin(yaw_rad),
                z=vehicle_location.z + args.height
            )
            
            # Point camera at vehicle
            camera_rotation = carla.Rotation(
                pitch=-15.0,  # Look down slightly
                yaw=vehicle_rotation.yaw,
                roll=0.0
            )
            
            # Update spectator
            spectator.set_transform(
                carla.Transform(camera_location, camera_rotation)
            )
            
            # Wait before next update
            if is_sync:
                # In synchronous mode, tick the world
                world.tick()
                if args.update_rate > 0:
                    time.sleep(args.update_rate)
            else:
                # In async mode, just sleep
                sleep_time = args.update_rate if args.update_rate > 0 else 0.05
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n[cam_follow] Stopped by user (Ctrl+C)")
    except Exception as e:
        print(f"[cam_follow] Error: {e}")
        raise
    finally:
        print("[cam_follow] Exiting...")


if __name__ == "__main__":
    main()
