import asyncio

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

        projectairsim_log().info("taking_async: starting takeoff")
        takeoff_task = (await drone.takeoff_async())
        await takeoff_task
        projectairsim_log().info("takeoff_async: completed take off")

        move_up = await drone.move_by_velocity_async(0.0, 0.0, -3.0, 3.0)
        projectairsim_log().info("move_by_velocity_async: moving up for 3 seconds")
        await move_up
        projectairsim_log().info("move_by_velocity_async: completed moving up")

        await drone.hover_async()
        projectairsim_log().info("hover_async: hovering for 2 seconds")
        await asyncio.sleep(3.0)
        projectairsim_log().info("hover_async: completed hovering")

        move_down = await drone.move_by_velocity_async(0.0, 0.0, 3.0, 3.0)
        projectairsim_log().info("move_by_velocity_async: moving down for 3 seconds")
        await move_down
        projectairsim_log().info("move_by_velocity_async: completed moving down")

        projectairsim_log().info("landing_async: starting landing")
        land_task = (await drone.land_async())
        await land_task
        projectairsim_log().info("landing_async: completed landing")

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
