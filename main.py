import sys
import os
import time

# Add the 'src' directory to the Python path to allow importing modules from it.
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

try:
    from hand_control_midas import DynamixelController
except ImportError as e:
    print(f"Failed to import DynamixelController: {e}")
    print("Please ensure 'src/hand_control_midas.py' exists and is correctly named.")
    sys.exit(1)

def main():
    """
    Main function to initialize the hand controller, execute a sequence of motions,
    and then properly close the connection.
    """
    dxl_controller = None
    try:
        # Initialize the controller. Set verbose=False to hide detailed setup logs.
        print("Initializing the Dynamixel controller...")
        dxl_controller = DynamixelController(verbose=True)
        print("Controller initialized successfully.")

        # --- Define and execute a sequence of motions ---
        motion_sequence = ["home", "grasp_tool_motion_0"]
        
        for motion in motion_sequence:
            try:
                print(f"Executing motion: '{motion}'...")
                # The 'execute_motion' method will handle loading and applying the motion.
                dxl_controller.execute_motion(motion, verbose=True)
                print(f"Motion '{motion}' completed. Waiting for 2 seconds...")
                time.sleep(2)
            except ValueError as e:
                print(f"Error executing motion '{motion}': {e}")
                print("Skipping to next motion.")
                continue

        print("Motion sequence finished.")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Ensure the connection is always closed, even if errors occur.
        if dxl_controller:
            print("Closing the controller connection.")
            dxl_controller.close()
            print("Connection closed.")

if __name__ == "__main__":
    main()
