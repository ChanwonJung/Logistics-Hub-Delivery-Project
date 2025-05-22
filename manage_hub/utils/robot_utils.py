"""로봇 제어 관련 유틸리티 함수"""

import time

from ..config.constants import (
    INITIAL_POSE
)
from ..config.robot_config import ROBOT_CONFIG

def setup_robot() -> bool:
    """로봇 초기 설정"""
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp
        )   
        # 툴 및 TCP 설정
        set_tool(ROBOT_CONFIG['tool'])
        set_tcp(ROBOT_CONFIG['tcp'])
        
    except Exception as e:
        print(f"로봇 설정 중 에러: {e}")
        return False
              
    return True

def get_object_width() -> float:
    """물체 너비를 측정"""
    try:
        from DSR_ROBOT2 import get_digital_input
        
        # 디지털 입력으로 물체 너비 측정
        width = 0.0
        for i in range(8):  # 8개의 디지털 입력 사용
            if get_digital_input(i):
                width += 0.1  # 각 센서 간격 0.1m
                
        return width
        
    except Exception as e:
        print(f"물체 너비 측정 중 에러: {e}")
        return 0.0

def safe_shutdown():
    """안전한 종료 처리"""
    try:
        from DSR_ROBOT2 import movej
        
        print("안전 종료 중...")
        
        # 초기 위치로 이동
        movej(INITIAL_POSE, 
              vel=ROBOT_CONFIG['default_velocity'],
              acc=ROBOT_CONFIG['default_acceleration'])
              
        time.sleep(1)  # 이동 완료 대기
        
    except Exception as e:
        print(f"안전 종료 중 에러: {e}")
    finally:
        print("로봇을 안전하게 종료했습니다.")
    