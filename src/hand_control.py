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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Read definitions from .json
config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/config.json")
with open(config_path, "r") as config_file:
    config = json.load(config_file)

control_table_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/control_table.json")
with open(control_table_path, "r") as config_file:
    control_table = json.load(config_file)

motion_table_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/motion_table.json")
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
        self.lock = threading.Lock()

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
            
        # Set port baudrate
        if self.portHandler.setBaudRate(config["communication"]["baudrate"]):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            # getch()
            quit()

        for motor_id in self.motor_ids:
            self.set_op_mode(motor_id=motor_id, mode=self.operation_mode)

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

        # flag for threading
        self.running = True
        self.enable_torque_all()
        self.read_state_thread = threading.Thread(target=self.read_states_loop, daemon=True)
        self.read_state_thread.start()

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
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, control_table["ADDR_TORQUE_ENABLE"], config["motor_config"]["torque_disable"])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has successfully disable." % motor_id)
            
    def disable_torque_all(self):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, control_table["BROADCAST_ID"], control_table["ADDR_TORQUE_ENABLE"], config["motor_config"]["torque_disable"])
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
                    """
                    author DY
                    TODO
                    max & min 은 control table에서 직접 드라이버르부터 읽어온 값을 쓸 수 있도록 수정 필요
                    """
                    max_limit = config["max_position_limit"]
                    min_limit = config["min_position_limit"]
                    position = max(max_limit, min(position, min_limit))

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
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            dxl_comm_result = self.groupSyncWriteVelocity.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            dxl_comm_result = self.groupSyncWritePosition.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

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
                print("Dynamixel#%d has successfully changed its mode." % motor_id)

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
                    quit()
                if not self.groupSyncReadVelocity.isAvailable(motor_id, control_table["ADDR_PRESENT_VELOCITY"],
                                                              control_table["LEN_PRESENT_VELOCITY"]):
                    print(f"[ID:{motor_id}] groupSyncReadVelocity getdata failed")
                    quit()
                if not self.groupSyncReadCurrent.isAvailable(motor_id, control_table["ADDR_PRESENT_CURRENT"],
                                                             control_table["LEN_PRESENT_CURRENT"]):
                    print(f"[ID:{motor_id}] groupSyncReadCurrent getdata failed")
                    quit()

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
                    pos = self.state_data[motor_id]["present_position"]
                    vel = self.state_data[motor_id]["present_velocity"]
                    cur = self.state_data[motor_id]["present_current"]
                    print(f"[read_states_loop] [#{motor_id}] present - pos:{pos},    vel:{vel},    cur:{cur}")
                print("====================================================================")
                time.sleep(2)

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
            elif cmd == "exit":
                break
            else:   # cmd = motion name
                motor_position_list_for_motion = load_motion_positions(data=motion_table, motion_name=cmd)
                motor_current_list_for_motion = load_motion_current(data=motion_table, motion_name=cmd)
                # 위치로 이동
                dxl.set_goal_positions(motor_position_list_for_motion, limit_clamping=False)
                dxl.set_goal_current(motor_current_list_for_motion)
                dxl.apply_motion()
                print("Moved to position:", motor_position_list_for_motion)

    except Exception as e:
        print("Exception occurred:", e)
    finally:
        dxl.close()