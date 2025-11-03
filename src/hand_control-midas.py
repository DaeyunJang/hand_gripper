#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import os
import json
import time
import threading
import sys
import ctypes

from dynamixel_sdk import *

# from dynamixel_sdk import (
#     PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
#     COMM_SUCCESS, DXL_LOBYTE, DXL_LOWORD, DXL_HIBYTE, DXL_HIWORD
# )

project_name = "midas"
name = f"../config/{project_name}/config.json"
# Read definitions from .json
config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"../config/{project_name}/config.json")
with open(config_path, "r") as config_file:
    config = json.load(config_file)

control_table_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/control_table.json")
with open(control_table_path, "r") as config_file:
    control_table = json.load(config_file)

motion_table_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"../config/{project_name}/motion_table.json")
with open(motion_table_path, "r") as config_file:
    motion_table = json.load(config_file)

class DynamixelController:
    from typing import Dict, Any, Union
    def __init__(self):
        """
        author: DY
        note
        load configuration from json file.
        """
        self.lock = threading.RLock()

        self.portHandler = PortHandler(config["communication"]["port"])
        self.packetHandler = PacketHandler(config["communication"]["protocol_version"])
        self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, control_table["ADDR_GOAL_POSITION"], control_table["LEN_GOAL_POSITION"])
        self.groupSyncWriteVelocity = GroupSyncWrite(self.portHandler, self.packetHandler, control_table["ADDR_GOAL_VELOCITY"], control_table["LEN_GOAL_VELOCITY"])
        self.groupSyncWriteCurrent = GroupSyncWrite(self.portHandler, self.packetHandler, control_table["ADDR_GOAL_CURRENT"], control_table["LEN_GOAL_CURRENT"])
        self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, control_table["ADDR_PRESENT_POSITION"], control_table["LEN_PRESENT_POSITION"])
        self.groupSyncReadVelocity = GroupSyncRead(self.portHandler, self.packetHandler, control_table["ADDR_PRESENT_VELOCITY"], control_table["LEN_PRESENT_VELOCITY"])
        self.groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, control_table["ADDR_PRESENT_CURRENT"], control_table["LEN_PRESENT_CURRENT"])
        self.operation_mode = config["motor_config"]["operating_mode"]

        self.motor_ids = []
        for i in range(1, config["hand"]["num_of_fingers"] + 1):
            finger_key = f"finger_{i}"
            if finger_key in config["hand"]:
                self.motor_ids.extend(config["hand"][finger_key]["ids"])

        self.state_data = {motor_id: {"position": 0, "velocity": 0, "current": 0} for motor_id in self.motor_ids}

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            # getch()
            quit()
            
        # Set port initial baudrate for connection
        if self.portHandler.setBaudRate(config["communication"]["baudrate"]):
            print(f"Succeeded to change the baudrate to {config['communication']['baudrate']} for initial connection")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            # getch()
            quit()

        # Change baudrate of motors to target baudrate
        self.set_baudrate()

        # set initial states
        print(f"============================")
        print(f"Set operation mode")
        for motor_id in self.motor_ids:
            self.set_op_mode(motor_id=motor_id, mode=self.operation_mode)
            
        print(f"==========================================")
        print(f"Set motor details from config.json")
        for motor_id in self.motor_ids:
            self.set_motor_details(motor_id)
        
        print(f"==========================================")
        print(f"Logs...")
        for motor_id in self.motor_ids:
            self.log_motor_status(motor_id)

        # Regist Parameter
        print(f"==========================================")
        print(f"Registry parameters")
        for motor_id in self.motor_ids:
            dxl_addparam_result = self.groupSyncReadPosition.addParam(motor_id)
            if dxl_addparam_result == True:
                print("[ID:%03d] groupSyncReadPosition addparam succeed" % motor_id)
            else:
                print("[ID:%03d] groupSyncReadPosition addparam failed" % motor_id)
                quit()
            dxl_addparam_result = self.groupSyncReadVelocity.addParam(motor_id)
            if dxl_addparam_result == True:
                print("[ID:%03d] groupSyncReadVelocity addparam succeed" % motor_id)
            else:
                print("[ID:%03d] groupSyncReadVelocity addparam failed" % motor_id)
                quit()
            dxl_addparam_result = self.groupSyncReadCurrent.addParam(motor_id)
            if dxl_addparam_result == True:
                print("[ID:%03d] groupSyncReadCurrent addparam succeed" % motor_id)
            else:
                print("[ID:%03d] groupSyncReadCurrent addparam failed" % motor_id)
                quit()

        print(f"==========================================")
        self.get_states()
        
        # print(f"Starting with 'torque off'")
        # self.disable_torque_all()
        print(f"Starting with 'torque on'")
        self.enable_torque_all()

        print(f"Threading start")
        # flag for threading
        self.running = True
        # self.read_state_thread = threading.Thread(target=self.read_states_loop, daemon=True)
        # self.read_state_thread.start()

    def log_motor_status(self, motor_id: int):
        with self.lock:
            
            # Read Operating Mode
            op_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_OPERATING_MODE"])
            if dxl_comm_result == COMM_SUCCESS:
                pass
            else:
                print(f"  Failed to read Operating Mode: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

            # Read Drive Mode
            drive_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_DRIVE_MODE"])
            if dxl_comm_result == COMM_SUCCESS:
                pass
            else:
                print(f"  Failed to read Drive Mode: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

            # Read Homing Offset
            homing_offset_unsigned, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, control_table["ADDR_HOMING_OFFSET"])
            if dxl_comm_result == COMM_SUCCESS:
                homing_offset_signed = ctypes.c_int32(homing_offset_unsigned).value
                pass
            else:
                print(f"  Failed to read Homing Offset: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

            # Read Return Delay Time
            return_delay_time, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_RETURN_DELAY"])
            if dxl_comm_result != COMM_SUCCESS:
                print(f"  Failed to read Return Delay Time: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

            # Read Current Limit
            current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, control_table["ADDR_CURRENT_LIMIT"])
            if dxl_comm_result == COMM_SUCCESS:
                pass
            else:
                print(f"  Failed to read Current Limit: {self.packetHandler.getTxRxResult(dxl_comm_result)}")


            print(f"Motor ID: {motor_id:<4} | Operating Mode: {op_mode:2d} | Drive Mode: {drive_mode:2d} | Homing Offset: {homing_offset_signed:+6d} | Return Delay Time: {return_delay_time:4d} ({return_delay_time*2}us) | Current Limit: {current_limit:+5d} |")
            # print(f"==========================================")


    def set_motor_details(self, motor_id: int):
        motor_id_str = str(motor_id)
        if motor_id_str in config["motor_details"]:
            details = config["motor_details"][motor_id_str]

            # Set Drive Mode
            if "drive_mode" in details:
                drive_mode_str = details["drive_mode"]
                drive_mode = int(drive_mode_str, 16)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_DRIVE_MODE"], drive_mode)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} has been set drive mode to {drive_mode_str}")
                    pass

            # Set Return Delay Time
            if "return_delay_time" in details:
                return_delay_time = details["return_delay_time"]
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_RETURN_DELAY"], return_delay_time)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} has been set return delay time to {return_delay_time} ({return_delay_time*2}us)")
                    pass
            
            # Set Current Limit
            current_limit = details["current_limit"]
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, control_table["ADDR_CURRENT_LIMIT"], current_limit)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Dynamixel#{motor_id} has been set current limit to {current_limit}")
                pass

            # Set Homing Offset
            homing_offset = details["homing_offset"]
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, control_table["ADDR_HOMING_OFFSET"], homing_offset)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print(f"motor_id: {motor_id} / homing_offset: {homing_offset}")
            elif dxl_error != 0:
                print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Dynamixel#{motor_id} has been set homing offset to {homing_offset}")
                pass

            # Set Max Position
            if "max_position" in details:
                max_position = details["max_position"]
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, control_table["ADDR_MAX_POSITION"], max_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} has been set max position to {max_position}")
                    pass

            # Set Min Position
            if "min_position" in details:
                min_position = details["min_position"]
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, control_table["ADDR_MIN_POSITION"], min_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} has been set min position to {min_position}")
                    pass

    def set_baudrate(self):
        """
        Sets the baud rate for all motors and the port handler based on the motor_details in the config file.
        """
        with self.lock:
            # Assume the target baudrate is the same for all motors.
            # Get the target from the first motor in the list.
            first_motor_id_str = str(self.motor_ids[0])
            if first_motor_id_str not in config["motor_details"] or "target_baudrate" not in config["motor_details"][first_motor_id_str]:
                print("Warning: 'target_baudrate' not found in motor_details for the first motor. Skipping baudrate change.")
                return

            target_baudrate = config["motor_details"][first_motor_id_str]["target_baudrate"]
            initial_baudrate = config["communication"]["baudrate"]

            # If the target is the same as the initial, no need to change.
            if target_baudrate == initial_baudrate:
                print(f"Baudrate is already set to {target_baudrate}. Skipping change.")
                return

            baudrate_map = control_table["BAUDRATE_MAP"]
            
            # JSON keys are strings, so we need to look up with a string
            target_baudrate_str = str(target_baudrate)

            if target_baudrate_str not in baudrate_map:
                print(f"Error: Baudrate {target_baudrate} is not supported in control_table.json.")
                quit()

            baudrate_code = baudrate_map[target_baudrate_str]
            
            print(f"==========================================")
            print(f"Attempting to change baudrate of all motors to {target_baudrate} (Code: {baudrate_code})")

            # It's safer to disable torque before changing critical settings
            self.disable_torque_all()
            time.sleep(0.1)

            # Change baudrate for each motor
            for motor_id in self.motor_ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_BAUD_RATE"], baudrate_code)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to change baudrate for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"Error on motor {motor_id} while changing baudrate: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    print(f"Successfully sent baudrate change command to motor #{motor_id}.")
            
            time.sleep(0.5) # Give motors time to process the baudrate change

            # Change the baudrate of the port handler to match the new motor baudrate
            if self.portHandler.setBaudRate(target_baudrate):
                print(f"Succeeded to change the port handler baudrate to {target_baudrate}")
            else:
                print(f"Failed to change the port handler baudrate to {target_baudrate}")
                print("Press any key to terminate...")
                # getch()
                quit()

    def read_states_loop(self):
        # while self.running:
        self.get_states()

    def enable_torque(self, motor_id: int):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_TORQUE_ENABLE"], config["motor_config"]["torque_enable"])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has successfully enable." % motor_id)

    def enable_torque_all(self):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, control_table["BROADCAST_ID"], control_table["ADDR_TORQUE_ENABLE"], config["motor_config"]["torque_enable"])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#ALL has successfully enable.")

    def disable_torque(self, motor_id: int):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_TORQUE_ENABLE"], 1)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has successfully disable." % motor_id)
            
    def disable_torque_all(self):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, control_table["BROADCAST_ID"], control_table["ADDR_TORQUE_ENABLE"], 0)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#ALL has successfully disable.")

    def set_goal_positions(self, positions: list[int], limit_clamping: bool=False):
        with self.lock:
            for motor_id, position in zip(self.motor_ids, positions):
                # print(f"[set_goal_position] motor id: {motor_id} / position: {position} / limit: {limit_clamping}")
                if limit_clamping:
                    motor_id_str = str(motor_id)
                    if motor_id_str in config["motor_details"]:
                        max_limit = config["motor_details"][motor_id_str]["max_position"]
                        min_limit = config["motor_details"][motor_id_str]["min_position"]
                        position = max(min_limit, min(position, max_limit))

                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(position)), DXL_HIBYTE(DXL_LOWORD(position)),
                                       DXL_LOBYTE(DXL_HIWORD(position)), DXL_HIBYTE(DXL_HIWORD(position))]
                dxl_addparam_result = self.groupSyncWritePosition.addParam(motor_id, param_goal_position)
                if dxl_addparam_result != True:
                    print(f"[set_goal_position] #{motor_id} fail")
                    print("[ID:%03d] groupSyncWrite addparam failed" % motor_id)
                    quit()
        
    def set_goal_velocity(self, velocities: list[int]):
        with self.lock:
            for motor_id, velocity in zip(self.motor_ids, velocities):
                param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(velocity)), DXL_HIBYTE(DXL_LOWORD(velocity)),
                                       DXL_LOBYTE(DXL_HIWORD(velocity)), DXL_HIBYTE(DXL_HIWORD(velocity))]
                dxl_addparam_result = self.groupSyncWriteVelocity.addParam(motor_id, param_goal_velocity)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % motor_id)
                    quit()

    def set_goal_current(self, currents: list[int]):
        with self.lock:
            for motor_id, current in zip(self.motor_ids, currents):
                param_goal_current = [DXL_LOBYTE(DXL_LOWORD(current)), DXL_HIBYTE(DXL_LOWORD(current))]
                dxl_addparam_result = self.groupSyncWriteCurrent.addParam(motor_id, param_goal_current)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % motor_id)
                    quit()

    def apply_motion(self, same_eta=True):
        with self.lock:
            dxl_comm_result = self.groupSyncWriteCurrent.txPacket()
            # if dxl_comm_result != COMM_SUCCESS:
            #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            dxl_comm_result = self.groupSyncWriteVelocity.txPacket()
            # if dxl_comm_result != COMM_SUCCESS:
            #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            dxl_comm_result = self.groupSyncWritePosition.txPacket()
            # if dxl_comm_result != COMM_SUCCESS:
            #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            self.groupSyncWritePosition.clearParam()
            self.groupSyncWriteVelocity.clearParam()
            self.groupSyncWriteCurrent.clearParam()

    def set_op_mode(self, motor_id: int, mode: int):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id,
                                                                           control_table["ADDR_OPERATING_MODE"],
                                                                           mode)
            if dxl_comm_result != COMM_SUCCESS:
                print("------------------------------------------")
                print("set_op_mode() failed")
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print("------------------------------------------")
            elif dxl_error != 0:
                print("------------------------------------------")
                print("set_op_mode() failed")
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                print("------------------------------------------")
            else:
                print("Dynamixel#%d has successfully changed its mode -> %d." % (motor_id, mode))

    def get_states(self, visual_flag=True):
        with self.lock:
            self.groupSyncReadPosition.txRxPacket()
            self.groupSyncReadVelocity.txRxPacket()
            self.groupSyncReadCurrent.txRxPacket()

            # Check if groupsyncread data of Dynamixel#1 is available
            for motor_id in self.motor_ids:
                if not self.groupSyncReadPosition.isAvailable(motor_id, control_table["ADDR_PRESENT_POSITION"],
                                                      control_table["LEN_PRESENT_POSITION"]):
                    print(f"[ID:{motor_id}] groupSyncRead getdata failed")
                    continue
                    # quit()
                if not self.groupSyncReadVelocity.isAvailable(motor_id, control_table["ADDR_PRESENT_VELOCITY"],
                                                              control_table["LEN_PRESENT_VELOCITY"]):
                    print(f"[ID:{motor_id}] groupSyncReadVelocity getdata failed")
                    continue
                    # quit()
                if not self.groupSyncReadCurrent.isAvailable(motor_id, control_table["ADDR_PRESENT_CURRENT"],
                                                             control_table["LEN_PRESENT_CURRENT"]):
                    print(f"[ID:{motor_id}] groupSyncReadCurrent getdata failed")
                    continue
                    # quit()

            self.state_data = {motor_id: {
                "present_position": self.groupSyncReadPosition.getData(motor_id, control_table["ADDR_PRESENT_POSITION"], control_table["LEN_PRESENT_POSITION"]),
                "present_velocity": self.groupSyncReadVelocity.getData(motor_id, control_table["ADDR_PRESENT_VELOCITY"], control_table["LEN_PRESENT_VELOCITY"]),
                "present_current": self.groupSyncReadCurrent.getData(motor_id, control_table["ADDR_PRESENT_CURRENT"], control_table["LEN_PRESENT_CURRENT"])
            } for motor_id in self.motor_ids}

            # self.groupSyncReadPosition.clearParam()
            # self.groupSyncReadVelocity.clearParam()
            # self.groupSyncReadCurrent.clearParam()

            if visual_flag:
                print("====================================================================")
                for motor_id in self.motor_ids:
                    pos = ctypes.c_int32(self.state_data[motor_id]["present_position"]).value
                    vel = ctypes.c_int32(self.state_data[motor_id]["present_velocity"]).value
                    cur = ctypes.c_int16(self.state_data[motor_id]["present_current"]).value
                    print(f"[read_states_loop] [#{motor_id}] present - pos:{pos},    vel:{vel},    cur:{cur}")
                print("====================================================================")
                
                print("\"new_motion\": [")
                for i, motor_id in enumerate(self.motor_ids):
                    pos = self.state_data[motor_id]["present_position"]

                    # Determine Velocity and Current based on motor ID pattern
                    last_digit = motor_id % 10
                    vel = 100
                    cur = 100 # Default
                    if last_digit == 0:
                        cur = 200
                    elif last_digit == 1:
                        cur = 200
                    elif last_digit == 2:
                        cur = 200

                    # Format the output string
                    line_end = "," if i < len(self.motor_ids) - 1 else ""
                    print(f'      {{"ID": {motor_id}, "Position": {pos}, "Velocity": {vel}, "Current": {cur}}}{line_end}')
                print("    ],")
                
                time.sleep(1)

    def close(self):
        with self.lock:
            # self.disable_torque()
            self.running = False
            self.disable_torque_all()
            print(f"torque off all !")
            self.portHandler.closePort()
            print(f"DXL port closed")
            self.read_state_thread.join()
            print(f"read_state_thread closed")

def load_motion_positions(data: dict, motion_name: str) -> list[int]:
    motion_data = data
    if motion_name not in motion_data["motion"]:
        raise ValueError(f"Motion '{motion_name}' not found in motion table.")

    motion_list = motion_data["motion"][motion_name]

    # ID 기준으로 정렬하고 Position 리스트 추출
    sorted_motors = sorted(motion_list, key=lambda m: m["ID"])
    positions = [motor["Position"] for motor in sorted_motors]

    return positions

def load_motion_current(data: dict, motion_name: str) -> list[int]:
    motion_data = data
    if motion_name not in motion_data["motion"]:
        raise ValueError(f"Motion '{motion_name}' not found in motion table.")

    motion_list = motion_data["motion"][motion_name]

    # ID 기준으로 정렬하고 Position 리스트 추출
    sorted_motors = sorted(motion_list, key=lambda m: m["ID"])
    currents = [motor["Current"] for motor in sorted_motors]

    return currents

if __name__ == "__main__":
    import time, keyboard, signal

    motor_position_list_for_motion = []
    motor_current_list_for_motion = []
    dxl = DynamixelController()

    # dxl.read_states_loop(visual_flag=True)

    try:
        while True:
            cmd = input("Enter command (torque_on, torque_off, get_states, exit): ").strip()
            print(f"input cmd: {cmd}")
            if cmd == "torqueOnAll":
                dxl.enable_torque_all()
                print(f"torque on All!")
            elif cmd == "torqueOffAll":
                dxl.disable_torque_all()
                print(f"torque off All!")
            elif cmd == "states":
                dxl.read_states_loop()
            elif cmd == "loop_motion_full":
                motion_sequence = [
                    "left_hand_open",
                    "left_pinch_index",
                    "left_pinch_middle",
                    "left_pinch_ring",
                    "left_pinch_middle",
                    "left_pinch_index",
                    "left_hand_open",
                    "left_hand_close"
                ]
                for i in range(5):
                    for motion_name in motion_sequence:
                        print(f"Executing motion: {motion_name}")
                        try:
                            motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name=motion_name)
                            motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name=motion_name)
                            
                            dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                            dxl.set_goal_current(motor_current_list_for_motion)
                            dxl.apply_motion()
                            
                            print(f"'{motion_name}' motion applied. Waiting for 2 seconds...")
                            time.sleep(1)
                        except ValueError as e:
                            print(f"Error executing motion '{motion_name}': {e}")
                            print("Aborting loop motion.")
                            break
                    print("Loop motion finished.")
            
            elif cmd == "loop_motion_palm":
                motion_sequence = [
                    "left_hand_open",
                    "left_hand_close"
                ]
                for i in range(5):
                    for motion_name in motion_sequence:
                        print(f"Executing motion: {motion_name}")
                        try:
                            motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name=motion_name)
                            motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name=motion_name)
                            dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                            dxl.set_goal_current(motor_current_list_for_motion)
                            dxl.apply_motion()
                            
                            print(f"'{motion_name}' motion applied. Waiting for 0.5 seconds...")
                            time.sleep(0.5)
                        except ValueError as e:
                            print(f"Error executing motion '{motion_name}': {e}")
                            print("Aborting loop motion.")
                            break
                print("Loop motion finished.")
            
            elif cmd == "loop_motion_pinch":
                try:
                    motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name="left_hand_open")
                    motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name="left_hand_open")
                    dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                    dxl.set_goal_current(motor_current_list_for_motion)
                    dxl.apply_motion()
                    time.sleep(0.5)
                except ValueError as e:
                    print("Aborting loop motion.")
                    break
                motion_sequence = [
                    "left_pinch_index",
                    "left_pinch_middle",
                    "left_pinch_ring",
                    "left_pinch_middle",
                    "left_pinch_index",
                ]
                for i in range(5):
                    for motion_name in motion_sequence:
                        print(f"Executing motion: {motion_name}")
                        try:
                            motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name=motion_name)
                            motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name=motion_name)
                            dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                            dxl.set_goal_current(motor_current_list_for_motion)
                            dxl.apply_motion()
                            
                            print(f"'{motion_name}' motion applied. Waiting for 2 seconds...")
                            time.sleep(0.5)
                        except ValueError as e:
                            print(f"Error executing motion '{motion_name}': {e}")
                            print("Aborting loop motion.")
                            break
                print("Loop motion finished.")
                
            elif cmd == "exit":
                break
            else:   # cmd = motion name
                try:
                    motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name=cmd)
                    motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name=cmd)
                    # 위치로 이동
                    dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                    dxl.set_goal_current(motor_current_list_for_motion)
                    dxl.apply_motion()
                    print("Moved to position:", motor_position_list_for_motion)
                except ValueError as e:
                    print(f"Error: {e}")
                    print("Please enter a valid command.")

    except Exception as e:
        print("Exception occurred:", e)
    finally:
        dxl.close()