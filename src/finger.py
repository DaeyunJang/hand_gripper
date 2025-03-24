#!/usr/bin/env python
# -*- coding: utf-8 -*-

from typing import List, Tuple
import numpy as np

class Finger:
    def __init__(self, finger_name: str, joint_ids: List[int], link_lengths: np.ndarray) -> None:
        """
        손가락 기구학 및 역기구학 관리 클래스
        :param finger_name: 손가락 이름 (예: 'thumb', 'index', 'middle' ...)
        :param joint_ids: 손가락의 각 관절에 해당하는 모터 ID 리스트 (예: [11, 12, 13, 14, 15])
        :param link_lengths: 손가락의 각 마디 길이 리스트 (NumPy 배열, 단위: cm)
        """
        self.finger_name: str = finger_name
        self.joint_ids: List[int] = joint_ids  # 각 관절을 제어하는 모터 ID
        self.link_lengths: np.ndarray = link_lengths  # 손가락 마디 길이 (NumPy 배열)
        self.joint_angles: np.ndarray = np.zeros_like(link_lengths)  # 초기 관절 각도 (라디안)

    def forward_kinematics(self) -> np.ndarray:
        """
        TODO
        DY
        2025.03.20.
        기구학은 matlab으로 작성했는데, 이거 반영해야 함.
        """
        theta_sum = np.cumsum(self.joint_angles)  # 각 관절 각도의 누적 합
        x = np.sum(self.link_lengths * np.cos(theta_sum))
        y = np.sum(self.link_lengths * np.sin(theta_sum))

        return np.array([x, y], dtype=np.float64)

    def inverse_kinematics(self, target_x: float, target_y: float) -> np.ndarray:
        """
        TODO
        DY
        2025.03.20.
        역기구학은 사실 기하학적으로는 못구했고, 수치적으로 접근했으나 해가 발산해서 
        나중에 해결하고 돌아오겠다.
        """
        if len(self.link_lengths) != 2:
            raise ValueError("현재 2개 링크 역기구학만 지원됩니다.")

        l1, l2 = self.link_lengths
        d = np.sqrt(target_x ** 2 + target_y ** 2)

        if d > (l1 + l2):
            raise ValueError("목표 위치가 손가락의 최대 길이를 초과합니다!")

        cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

        theta1 = np.arctan2(target_y, target_x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))

        self.joint_angles = np.array([theta1, theta2], dtype=np.float64)
        return self.joint_angles

    def set_joint_angles(self, angles: np.ndarray) -> None:
        """
        현재 손가락의 관절 각도를 업데이트
        :param angles: 새로운 관절 각도 배열 (라디안)
        """
        if angles.shape != self.joint_angles.shape:
            raise ValueError("각 관절에 대한 정확한 개수의 각도를 입력해야 합니다.")
        self.joint_angles = angles

    def get_joint_ids(self) -> List[int]:
        """손가락 관절을 제어하는 모터 ID 리스트 반환"""
        return self.joint_ids

    def get_joint_angles(self) -> np.ndarray:
        """현재 관절 각도 반환"""
        return self.joint_angles
