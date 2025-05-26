import time
import datetime
import os
import sys
import numpy as np
import cv2
import tifffile
from ctypes import *
from thorlabs_tsi_sdk.tl_camera import TLCameraSDK, OPERATION_MODE

def initialize_motor(serial_str):
    if sys.version_info < (3, 8):
        os.chdir(r"C:\Program Files\Thorlabs\Kinesis")
    else:
        os.add_dll_directory(r"C:\Program Files\Thorlabs\Kinesis")

    lib = cdll.LoadLibrary("Thorlabs.MotionControl.KCube.DCServo.dll")
    serial_num = c_char_p(serial_str.encode())

    if lib.TLI_BuildDeviceList() == 0:
        lib.CC_Open(serial_num)
        lib.CC_StartPolling(serial_num, c_int(200))
        time.sleep(1)

        # Setup motor params (behövs även för device units)
        STEPS_PER_REV = c_double(1919.64186)
        gbox_ratio = c_double(1.0)
        pitch = c_double(1.0)
        lib.CC_SetMotorParamsExt(serial_num, STEPS_PER_REV, gbox_ratio, pitch)

        return lib, serial_num
    else:
        raise Exception("Could not build device list / open motor")

def move_motor_to_device_position(lib, serial_num, position_device_units):
    pos = c_int(position_device_units)
    lib.CC_SetMoveAbsolutePosition(serial_num, pos)
    time.sleep(0.25)
    lib.CC_MoveAbsolute(serial_num)
    time.sleep(1)

def shutdown_motor(lib, serial_num):
    lib.CC_Close(serial_num)

def wait_until_motor_stops(lib, serial_num, timeout_sec=10):
    status_bits = c_ulong()
    start_time = time.time()

    while True:
        lib.CC_RequestStatusBits(serial_num)
        lib.CC_GetStatusBits(serial_num, byref(status_bits))
        if (status_bits.value & 0x0020) == 0:
            break
        if time.time() - start_time > timeout_sec:
            print("Motor is moving...")
            break
        time.sleep(0.1)

def get_current_device_position(lib, serial_num):
    pos_dev = c_int()
    lib.CC_GetPosition(serial_num, byref(pos_dev))
    return pos_dev.value

# CAMERA SETTINGS
def capture_image_from_camera():
    from windows_setup import configure_path
    if configure_path:
        configure_path()

    with TLCameraSDK() as sdk:
        available_cameras = sdk.discover_available_cameras()
        if len(available_cameras) < 1:
            print("no cameras detected")
            return

        with sdk.open_camera(available_cameras[0]) as camera:
            camera.exposure_time_us = 50000
            camera.frames_per_trigger_zero_for_unlimited = 0
            camera.image_poll_timeout_ms = 1000

            camera.arm(2)
            camera.issue_software_trigger()

            frame = camera.get_pending_frame_or_null()
            if frame is not None:
                print(f"frame #{frame.frame_count} received!")
                image_buffer_copy = np.copy(frame.image_buffer)
                shaped = image_buffer_copy.reshape(camera.image_height_pixels, camera.image_width_pixels)
                image = np.stack((shaped,) * 3, axis=-1)

                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"image_at_position_{timestamp}.tiff"
                #tifffile.imwrite(filename, image, photometric='rgb', compression=None)
                tifffile.imwrite(filename, shaped.astype(np.uint16))
                print(f"Saved image as {filename}")
            else:
                print("Failed to capture image")
            camera.disarm()

# MAIN PROGRAM
def main():
    sample_stage_sn = "0" # Enter sample stage serial number
    camera_stage_sn = "0" # Enter camera stage serial number

    # Initialize motors
    lib_sample, serial_sample = initialize_motor(sample_stage_sn)
    lib_camera, serial_camera = initialize_motor(camera_stage_sn)
    
    wait_until_motor_stops(lib_sample, serial_sample)
    wait_until_motor_stops(lib_camera, serial_camera)

    # Initial positions (DEVICE UNITS)
    current_sample_pos = 0  # Enter initial position for sample stage
    current_camera_pos = 0  # Enter initial position for camera stage

    # Saving initial positions
    initial_sample_pos = current_sample_pos
    initial_camera_pos = current_camera_pos

    print(f"Initial position (sample): {current_sample_pos} DU")
    print(f"Initial position (camera): {current_camera_pos} DU")
    time.sleep(1)

    num_steps = 0 #Enter number of steps to cover the sample
    step_sample = 0  #Enter the step size for sample stage 
    step_camera = 0 # Enter the step size for the camera stage

    for i in range(num_steps):
        sample_position = current_sample_pos + i * step_sample
        camera_position = current_camera_pos + i * step_camera
        print(f"Step {i+1}: Sample to: {sample_position} DU, Camera to: {camera_position} DU")
        
        move_motor_to_device_position(lib_sample, serial_sample, sample_position)
        wait_until_motor_stops(lib_sample, serial_sample)
        
        move_motor_to_device_position(lib_camera, serial_camera, camera_position)
        wait_until_motor_stops(lib_camera, serial_camera)
        
        capture_image_from_camera()
        time.sleep(1.5)

    # Move back to initial position
    move_motor_to_device_position(lib_sample, serial_sample, initial_sample_pos)
    wait_until_motor_stops(lib_sample, serial_sample)

    move_motor_to_device_position(lib_camera, serial_camera, initial_camera_pos)
    wait_until_motor_stops(lib_camera, serial_camera)

    # Shut down motors
    shutdown_motor(lib_sample, serial_sample)
    shutdown_motor(lib_camera, serial_camera)

    print("Program completed.")

if __name__ == "__main__":
    main()