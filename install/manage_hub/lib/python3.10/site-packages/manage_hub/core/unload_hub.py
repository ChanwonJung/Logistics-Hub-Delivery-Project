"""출하 관련 클래스"""


from ..config.constants import (
    OUTPUT_POSITION,
    UNLOAD_AREAS
)
from ..config.robot_config import MOTION_CONFIG, COMPLIANCE_STIFFNESS, FORCE_CONTROL_PARAMS
from .gripper import Gripper

class UnloadHub:
    """출하 허브 클래스"""
    def __init__(self, hub_type):
        self.hub_type = hub_type
        self.stack_height = 0
        self.blocks = {}
        self._initialize_blocks()

    def _initialize_blocks(self):
        """박스 객체 초기화"""
        if self.hub_type == '2x3':
            self.blocks = {
                0: UNLOAD_AREAS['2x3']['0'],
                1: UNLOAD_AREAS['2x3']['1'],
                2: UNLOAD_AREAS['2x3']['2'],
                3: UNLOAD_AREAS['2x3']['3']
            }
        else:  # 2x2
            self.blocks = {
                0: UNLOAD_AREAS['2x2']['0'],
                1: UNLOAD_AREAS['2x2']['1'],
                2: UNLOAD_AREAS['2x2']['2'],
                3: UNLOAD_AREAS['2x2']['3']
            }

    def detect_object(self):
        """외력 및 순응 제어를 사용하여 물체를 감지"""
        from DSR_ROBOT2 import (
            task_compliance_ctrl, set_desired_force,
            check_force_condition, DR_AXIS_Z, DR_FC_MOD_REL,
            release_force, release_compliance_ctrl, movel
        )
        print(f"{self.hub_type} 블록의 물체를 감지합니다.")
        
        # 순응 제어 시작
        ret = task_compliance_ctrl(stx=COMPLIANCE_STIFFNESS['force'])
        if ret == 0:
            print("순응 제어 준비 완료")
        else:
            print("순응 제어 준비 실패!")
            return 0
            
        # 외력 설정
        ret = set_desired_force(fd=FORCE_CONTROL_PARAMS['insert'], dir=FORCE_CONTROL_PARAMS['direction'], mod=DR_FC_MOD_REL)
        if ret == 0:
            print("외력 준비 완료")
        else:
            print("외력 준비 실패!")
            return 0
            
        # 물체 감지
        detected_object = 0
        force_condition = check_force_condition(DR_AXIS_Z, max=40)
        print("외력을 가하며 물체를 감지합니다.")
        
        while force_condition > -1:
            detected_object += 1
            movel([0, 0, -10, 0, 0, 0], 
                  vel=MOTION_CONFIG['slow_velocity'],
                  acc=MOTION_CONFIG['slow_acceleration'],
                  mod=1)
            force_condition = check_force_condition(DR_AXIS_Z, max=30)
        
        # 순응 제어 해제
        if release_force() == 0:
            print("외력을 해제합니다.")
        if release_compliance_ctrl() == 0:
            print("순응 제어를 해제합니다.")
            
        return detected_object

    def unstack(self):
        """박스 출고 동작"""
        from DSR_ROBOT2 import (
            movel, wait, trans, DR_BASE
        )
        # 현재 위치에서 출고 위치로 이동
        movel(OUTPUT_POSITION, 
              vel=MOTION_CONFIG['default_velocity'],
              acc=MOTION_CONFIG['default_acceleration'])
        wait(0.5)
        down_pos = trans(OUTPUT_POSITION, [0, 0, -117, 0, 0, 0], ref=DR_BASE)
        movel(down_pos, 
              vel=MOTION_CONFIG['default_velocity']//2,
              acc=MOTION_CONFIG['default_acceleration']//2)
        wait(0.5)
        gripper = Gripper()
        gripper.release()

    def find_objects(self):
        """각 위치에서 물체를 찾는 메서드"""
        from DSR_ROBOT2 import (
            movel, wait
        )
        print(f"{self.hub_type} 허브에서 물체를 찾습니다.")
        
        # 각 블록의 위치를 순회하며 물체 확인
        for block in self.blocks.items():
            print(f"{block.pos_id} 위치에서 물체를 확인합니다.")
            
            # 해당 위치로 이동
            movel(block.position, 
                  vel=MOTION_CONFIG['slow_velocity'],
                  acc=MOTION_CONFIG['slow_acceleration'])
            wait(0.5)
            
            # detect_object 메서드를 사용하여 물체 존재 여부 확인
            object = self.detect_object()
            if object > 0:
                print(f"{block.pos_id} 위치에서 물체를 발견했습니다.")
                return True
            else:
                print(f"{block.pos_id} 위치에 물체가 없습니다.")
        
        print(f"{self.hub_type} 허브에서 물체를 찾지 못했습니다.")
        return False 

    def unload(self):
        """출하 처리 메인 함수"""
        print(f"{self.hub_type} 허브 출하를 시작합니다.")
        
        # 물체 찾기
        if not self.find_objects():
            print(f"{self.hub_type} 허브에 출하할 물체가 없습니다.")
            return False
            
        # 물체 출고
        if not self.unstack():
            print(f"{self.hub_type} 허브 물체 출고에 실패했습니다.")
            return False
            
        print(f"{self.hub_type} 허브 출하가 완료되었습니다.")
        return True 
    
class UnloadHub2x3(UnloadHub):
    """2x3 허브 클래스"""
    def __init__(self):
        super().__init__('2x3')
        
class UnloadHub2x2(UnloadHub):
    """2x2 허브 클래스"""
    def __init__(self):
        super().__init__('2x2')