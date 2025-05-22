"""그리퍼 제어 클래스"""

from ..config.constants import ON, OFF

class Gripper:
    def __init__(self):
        self.is_gripped = False      # 잡혀있는지 여부
    
    def grip(self) -> bool:
        self.release()
        try:
            from DSR_ROBOT2 import set_digital_output, wait
            
            # 그리퍼 제어 로직
            set_digital_output(1, ON)   # 그리퍼 ON
            set_digital_output(2, OFF)  # 그리퍼 OFF
            wait(1)                     # 동작 대기
            self.is_gripped = True      # 상태 업데이트
            return True
            
        except Exception as e:
            print(f"그리퍼 동작 중 에러: {e}")
            return False
    
    def release(self) -> bool:
        try:
            from DSR_ROBOT2 import set_digital_output, wait
            
            # 그리퍼 해제 로직
            set_digital_output(2, ON)   # 그리퍼 ON
            set_digital_output(1, OFF)  # 그리퍼 OFF
            wait(1)                     # 동작 대기
            self.is_gripped = False     # 상태 업데이트
            return True
            
        except Exception as e:
            print(f"그리퍼 해제 중 에러: {e}")
            return False