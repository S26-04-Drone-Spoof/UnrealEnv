import asyncio
import math
import sys

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.lidar_utils import LidarDisplay
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay

async def main():
    client = ProjectAirSimClient()
    image_display = ImageDisplay()

    lidar_subwin = image_display.get_subwin_info(2)
    lidar_display = LidarDisplay(x=lidar_subwin["x"], y=lidar_subwin["y"] + 30)
    
    # Track last print time for intensity stats
    last_intensity_print = [0]  # Use list to allow modification in lambda
    latest_lidar = [None]  # Store latest lidar data

    try:
        client.connect()
        world = World(client, "scene_lidar_drone.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "Drone1")

        chase_cam_window = "chaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(drone.sensors["Chase"]["scene_camera"], lambda _, chase: image_display.receive(chase, chase_cam_window))

        # Subscribe to Depth camera
        depth_name = "Depth-Image"
        image_display.add_image(depth_name, subwin_idx=1)
        client.subscribe(
            drone.sensors["DownCamera"]["depth_camera"],
            lambda _, depth: image_display.receive(depth, depth_name),
        )

        image_display.start()

        # Subscribe to lidar sensor
        def lidar_callback(_, lidar):
            import time
            latest_lidar[0] = lidar
            current_time = time.time()
            
            # Print intensity stats every 5 seconds
            if current_time - last_intensity_print[0] >= 1.0:
                intensities = lidar['intensity_cloud']
                if intensities:
                    min_int = min(intensities)
                    max_int = max(intensities)
                    avg_int = sum(intensities) / len(intensities)
                    projectairsim_log().info(
                        f"Intensity Stats - Min: {min_int:.3f}, Max: {max_int:.3f}, Avg: {avg_int:.3f}"
                    )
                last_intensity_print[0] = current_time
            
            lidar_display.receive(lidar)
        
        client.subscribe(drone.sensors["lidar1"]["lidar"], lidar_callback)
        lidar_display.start()

        drone.enable_api_control()
        drone.arm()

        # ==================== COMPREHENSIVE FLIGHT DEMONSTRATION ====================
        print("\n=== Starting Comprehensive Flight Demo ===")
        print("This demo showcases drone capabilities and all sensors\n")
        
        # 1. TAKEOFF
        projectairsim_log().info("taking_async: starting takeoff")
        print("1. Taking off...")
        takeoff_task = (await drone.takeoff_async())
        await takeoff_task
        projectairsim_log().info("takeoff_async: completed take off")
        await asyncio.sleep(1)
        
        # 2. MOVE NORTH (Forward)
        print("2. Moving North (forward) at 3 m/s for 4 seconds...")
        move_north = await drone.move_by_velocity_async(3.0, 0.0, 0.0, 4.0)
        await move_north
        projectairsim_log().info("Completed northward movement")
        await asyncio.sleep(0.5)
        
        # 3. MOVE EAST (Right)
        print("3. Moving East (right) at 3 m/s for 4 seconds...")
        move_east = await drone.move_by_velocity_async(0.0, 3.0, 0.0, 4.0)
        await move_east
        projectairsim_log().info("Completed eastward movement")
        await asyncio.sleep(0.5)
        
        # 4. MOVE SOUTH (Backward)
        print("4. Moving South (backward) at 3 m/s for 4 seconds...")
        move_south = await drone.move_by_velocity_async(-3.0, 0.0, 0.0, 4.0)
        await move_south
        projectairsim_log().info("Completed southward movement")
        await asyncio.sleep(0.5)
        
        # 5. MOVE WEST (Left)
        print("5. Moving West (left) at 3 m/s for 4 seconds...")
        move_west = await drone.move_by_velocity_async(0.0, -3.0, 0.0, 4.0)
        await move_west
        projectairsim_log().info("Completed westward movement")
        await asyncio.sleep(0.5)
        
        # 6. DIAGONAL MOVEMENT - Northeast
        print("6. Moving Northeast diagonally at 2 m/s each axis for 3 seconds...")
        move_ne = await drone.move_by_velocity_async(2.0, 2.0, 0.0, 3.0)
        await move_ne
        projectairsim_log().info("Completed diagonal northeast movement")
        await asyncio.sleep(0.5)
        
        # 7. ASCEND while moving
        print("7. Ascending while moving North (3D movement)...")
        move_up = await drone.move_by_velocity_async(2.0, 0.0, -2.0, 3.0)
        projectairsim_log().info("move_by_velocity_async: moving up and north")
        await move_up
        projectairsim_log().info("Completed 3D movement")
        await asyncio.sleep(0.5)
        
        # 8. HOVER at current position
        print("8. Hovering in place for 2 seconds...")
        await drone.hover_async()
        projectairsim_log().info("hover_async: hovering")
        await asyncio.sleep(2.0)
        projectairsim_log().info("hover_async: completed hovering")
        
        # 9. CIRCULAR PATH using waypoints
        print("9. Flying circular path using waypoints...")
        pose = drone.get_ground_truth_pose()
        center_x, center_y, center_z = pose["translation"]["x"], pose["translation"]["y"], pose["translation"]["z"]
        radius = 8.0
        num_points = 12
        circle_path = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            circle_path.append([x, y, center_z])
        
        circle_task = await drone.move_on_path_async(circle_path, velocity=4.0, timeout_sec=60)
        await circle_task
        projectairsim_log().info("Completed circular path")
        await asyncio.sleep(0.5)
        
        # 10. ROTATE in place (yaw rotation)
        print("10. Rotating 360 degrees in place (yaw rotation)...")
        rotate_task = await drone.rotate_by_yaw_rate_async(yaw_rate=math.radians(45), duration=8.0)
        await rotate_task
        projectairsim_log().info("Completed yaw rotation")
        await asyncio.sleep(0.5)
        
        # 11. RETURN TO START using position control
        print("11. Returning to start position using waypoint navigation...")
        return_task = await drone.move_to_position_async(0.0, 0.0, -5.0, velocity=4.0, timeout_sec=15)
        await return_task
        projectairsim_log().info("Returned to start area")
        await asyncio.sleep(0.5)
        
        # 12. DESCEND slowly
        print("12. Descending slowly...")
        move_down = await drone.move_by_velocity_async(0.0, 0.0, 1.5, 3.0)
        projectairsim_log().info("move_by_velocity_async: moving down")
        await move_down
        projectairsim_log().info("Completed descent")
        
        # 13. FINAL HOVER
        print("13. Final hover before landing...")
        await drone.hover_async()
        await asyncio.sleep(2.0)
        
        # 14. LAND
        projectairsim_log().info("landing_async: starting landing")
        print("14. Landing...")
        land_task = (await drone.land_async())
        await land_task
        projectairsim_log().info("landing_async: completed landing")
        
        print("\n=== Flight Demo Complete ===")
        print("All sensors (Chase Camera, Depth Camera, Lidar) were active throughout flight\n")

        drone.disarm()
        drone.disable_api_control()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    
    finally:
        client.disconnect()
        image_display.stop()
        lidar_display.stop()

if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function
