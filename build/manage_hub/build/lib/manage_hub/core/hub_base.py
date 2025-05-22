"""기본 허브 클래스"""
import time

from ..config.constants import (
    HUB_AREAS,
    OVERFLOW_POSITION,
    MOVE_DISTANCE,
    Z_OFFSET,
    INITIAL_POSE,
    MOVE_DISTANCE_X,
    VELOCITY,
    ACC
)
from ..core.gripper import Gripper
from ..config.robot_config import FORCE_CONTROL_PARAMS, COMPLIANCE_STIFFNESS, MOTION_CONFIG
from ..utils.motion_utils import compliance_place_with_force
from DR_common2 import posx

class HubBase:
    # 허브 타입별 stack_count를 저장하는 딕셔너리
    stack_count = {
        '2x3': 0,
        '2x2': 0,
        'overflow': 0
    }

    over_coord = OVERFLOW_POSITION
    over_cnt = 0
    
    def __init__(self):
        self.hub_coord = None      # 허브 좌표
        self.hub_type = None       # 허브 타입
        self.stack_height = 0      # 쌓인 물체 높이

    def calculate_next_position(self, current_pos) -> list:
        """다음 허브 위치를 계산"""
        try:
            from DSR_ROBOT2 import trans, DR_BASE
            
            if self.stack_height >= 3:  # 3층까지 쌓았으면
                # 해당 허브 타입의 stack_count 증가
                HubBase.stack_count[self.hub_type] += 1
                
                if HubBase.stack_count[self.hub_type] % 2 == 1:   # 홀수 번째 3층 (1,3,5...)
                    # y축만 이동
                    new_pos = trans(current_pos, [0, MOVE_DISTANCE, 0, 0, 0, 0], ref=DR_BASE)
                else:   # 짝수 번째 3층 (2,4,6...)
                    # x축과 y축 모두 이동
                    new_pos = trans(current_pos, [MOVE_DISTANCE_X, -MOVE_DISTANCE, 0, 0, 0, 0], ref=DR_BASE)
                
                self.stack_height = 0  # 높이 초기화
            else:
                # z축으로만 이동
                new_pos = trans(current_pos, [0, 0, -Z_OFFSET, 0, 0, 0], ref=DR_BASE)
                self.stack_height += 1
            
            return new_pos  # 다음 위치 반환
            
        except Exception as e:
            print(f"다음 위치 계산 중 에러: {e}")
            return current_pos

    def is_all_areas_full(self) -> bool:
        """모든 작업 영역이 가득 찼는지 확인"""
        try:
            from DSR_ROBOT2 import movej
            # 2x3 허브 작업 영역 체크
            is_2x3_full = (self.hub_coord[0] >= HUB_AREAS['2x3']['end'][0] and 
                          self.hub_coord[1] >= HUB_AREAS['2x3']['end'][1])
            
            # 2x2 허브 작업 영역 체크
            is_2x2_full = (self.hub_coord[0] >= HUB_AREAS['2x2']['end'][0] and 
                          self.hub_coord[1] >= HUB_AREAS['2x2']['end'][1])
            
            # 오버플로우 영역 체크 (4번 적재하면 full)
            is_overflow_full = HubBase.over_cnt >= 4
            
            # 모든 영역이 가득 찼는지 확인
            if is_2x3_full and is_2x2_full and is_overflow_full:
                print("모든 작업 영역이 가득 찼습니다. 작업을 종료합니다.")
                movej(INITIAL_POSE)  # 초기 좌표로 이동
                return True
            return False
            
        except Exception as e:
            print(f"작업 영역 체크 중 에러: {e}")
            return False

    def move_hub(self) -> str:
        """허브로 물체 이동
        Returns:
            str: 작업 상태
                'success': 정상적으로 물체 이동 완료
                'area_full': 작업 영역 가득 참
                'overflow': 오버플로우 영역 가득 참
                'error': 에러 발생
        """
        try:
            from DSR_ROBOT2 import movel, movej
            
            print(f"{self.hub_type} 허브 작업 시작...")
            
            # 1. 허브 위치로 이동
            movel(self.hub_coord, vel=MOTION_CONFIG['default_velocity'],
                  acc=MOTION_CONFIG['default_acceleration'])
            
            # 2. 외력 감지로 물체 적재
            if not compliance_place_with_force():
                print("허브에 물체 배치 실패")
                
                movej(INITIAL_POSE)  # 초기 좌표로 이동
                return 'error'
            
            # 3. 적재 후 다시 원위치로 이동
            movel(self.hub_coord, vel=MOTION_CONFIG['default_velocity'],
                  acc=MOTION_CONFIG['default_acceleration'])
                                 
            # 4. 다음 위치 계산
            next_position = self.calculate_next_position(self.hub_coord)
            
            # 5. 작업 영역을 벗어나는지 확인 
            if next_position[0] > HUB_AREAS[self.hub_type]['end'][0] or next_position[1] > HUB_AREAS[self.hub_type]['end'][1]:
                print("작업 영역 초과, 오버플로우 영역으로 이동")
                return self.move_to_overflow()
            elif HubBase.over_cnt >= 3:
                print("오버플로우 영역이 가득찼습니다. 오버플로우 영역을 정리해주세요.")
                return 'overflow'
            # 오버플로우 영역 청소를 해도 확인하는 로직 부재(필요하면 추가)

            # 6. 새로운 위치로 업데이트
            self.hub_coord = next_position
            print(f"허브 내에 최근 적재한 물체의 층 수: {self.stack_height})")
            
            return 'success'
            
        except Exception as e:
            print(f"물체 이동 중 에러: {e}")
            movej(INITIAL_POSE)  # 초기 좌표로 이동
            return 'error'

    def move_to_overflow(self) -> bool:
        """작업 영역이 가득 찬 경우 오버플로우 영역으로 이동"""
        try:
            from DSR_ROBOT2 import wait, movel, trans, DR_BASE
            
            wait(1)
            # 1. 오버플로우 위치로 이동
            movel(HubBase.over_coord, vel=MOTION_CONFIG['default_velocity'],
                  acc=MOTION_CONFIG['default_acceleration'])
            
            # 2. 순응제어와 힘제어로 물체 놓기
            if not compliance_place_with_force():
                print("오버플로우 영역에 물체 배치 실패")
                return False
            
            # 3. 다음 오버플로우 영역 내의 위치 계산
            if HubBase.stack_count["overflow"] % 2 == 1:   # 홀수 번째 위치 (1,3,5...)
                # y축만 이동
                next_pos = trans(HubBase.over_coord, [0, MOVE_DISTANCE, 0, 0, 0, 0], ref=DR_BASE)
                HubBase.over_cnt += 1
            else:   # 짝수 번째 위치 (2,4,6...)
                # x축과 y축 모두 이동
                next_pos = trans(HubBase.over_coord, [MOVE_DISTANCE_X, -MOVE_DISTANCE, 0, 0, 0, 0], ref=DR_BASE)
                HubBase.over_cnt += 1
            # 4. 새로운 위치로 업데이트
            self.hub_coord = next_pos
            HubBase.stack_count["overflow"] += 1
            print(f"오버플로우 영역에 물체 적재 완료, 허브에 공간이 생기면 다시 물체를 적재해주세요")
            
            return True
            
        except Exception as e:
            print(f"오버플로우 영역 이동 중 에러: {e}")
            return False

# 2x3 허브 클래스
class Hub2x3(HubBase): 
    hub_type = '2x3'  # 허브 타입

    def __init__(self, hub_coord: posx):
        super().__init__()
        self.hub_coord = hub_coord

# 2x2 허브 클래스
class Hub2x2(Hub2x3):
    hub_type = '2x2'  # 허브 타입

    def __init__(self, hub_coord: posx):
        super().__init__(hub_coord)

# 불량품 허브 클래스
class HubDefect(HubBase):
    """불량품 허브 클래스"""
    def __init__(self, hub_coord: posx):
        super().__init__()
        self.hub_coord = hub_coord
    
    def trash_twist(self):
        from DSR_ROBOT2 import move_periodic, DR_TOOL
        move_periodic([0,0,0,0,0,15], 2.0, 0.5, 8, DR_TOOL)

    def trash_wiggle_xy():
        from DSR_ROBOT2 import move_spiral, DR_AXIS_Z
        move_spiral(rev=5.0,rmax=10.0,lmax=10.0,vel=(VELOCITY+20), acc= (ACC-+20), time=10.0, axis=DR_AXIS_Z,ref=DR_TOOL)

    # TODO 수정 사항 적용 필요
    # 불량품 처리 로직은 오버라이딩으로 정의 
    def move_hub(self) -> bool:
        """불량품 허브로 물체 이동"""
        try:
            from DSR_ROBOT2 import movej,wait, movel, mwait, task_compliance_ctrl, set_desired_force, release_force, release_compliance_ctrl, check_force_condition, get_current_posx, DR_AXIS_Z, DR_FC_MOD_REL
            
            print(f"{self.hub_type} 허브 작업 시작...")
            wait(1)
            # 1. 허브 위치로 이동
            movel(self.hub_coord, vel=MOTION_CONFIG['default_velocity'],
                  acc=MOTION_CONFIG['default_acceleration'])
            time.sleep(1)
            
            # 2. 불량품 투하
            gripper = Gripper()
            gripper.release()
            print("해당 상품을 불량품으로 인식하여 적재하였습니다.")
            
            # 3. 불량품 영역 내 불량품이 다 찼는지 확인
            time.sleep(1)
            print("불량품 영역 내 불량품이 다 찼는지 확인중입니다 ...")
            gripper.grip()
            mwait(0.3)

            # 순응 제어 활성화
            print("순응 제어를 시작합니다.")
            ret = task_compliance_ctrl(COMPLIANCE_STIFFNESS['force'])
            if ret == 0:
                print("순응 제어 준비 완료")
            else:
                print("순응 제어 준비 실패!!")
            time.sleep(1)

            # 힘 제어 활성화
            print("외력 동작을 준비합니다. ")
            ret = set_desired_force(fd=FORCE_CONTROL_PARAMS['insert'], dir=FORCE_CONTROL_PARAMS['direction'], mod=DR_FC_MOD_REL)
            if ret == 0:
                mwait(0.5)
                print("외력 준비 완료")
            else:
                mwait(0.5)
                print("외력 준비 실패!")

            # 4. 허브 내 불량품 영역의 높이 확인
            time.sleep(1)
            print("현재 허브 안 불량품이 쌓여져 있는 높이를 측정중입니다 ...")
            force_condition = check_force_condition(DR_AXIS_Z, max=20)
            while (force_condition > -1): 
                force_condition = check_force_condition(DR_AXIS_Z, max=3)
                
            print("현재 좌표의 위치는 ", get_current_posx())
            current_trash_z = get_current_posx()[0][2]
            print(f"현재 감지된 가장 높은 z 위치는: {current_trash_z:.1f}")

            # 5. 측정 완료 → 힘/순응 제어 해제
            if release_force() == 0:
                print("현재 허브 안 불량품 영역의 높이를 확인하였습니다. 외력를 해제합니다.")
            time.sleep(0.1)
            if release_compliance_ctrl() == 0:
                print("현재 허브 안 불량품 영역의 높이를 확인하였습니다. 순응 제어를 해제합니다.")
            time.sleep(0.1)

            release_force()
            release_compliance_ctrl()

            # 6. 불량품 영역이 가득 찼는지 판별
            if current_trash_z >= 75:
                print("현재 허브 안 불량품 영역에 불량품이 가득 찼습니다. 불량품 영역을 청소해주세요.")
                wait(3)
                movel(self.hub_coord, vel=MOTION_CONFIG['default_velocity'],
                  acc=MOTION_CONFIG['default_acceleration'])
            else:
                print("현재 불량품이 쌓여져 있는 높이는 약 ", current_trash_z, "입니다.")
                gripper.grip()
                time.sleep(1)
                print("쌓여져 있는 불량품의 높이를 균일하게 맞추겠습니다.")
                self.trash_wiggle_xy()
                wait(1.0)
                self.trash_twist()     
            return True
            
        except Exception as e:
            print(f"물체 이동 중 에러: {e}")
            movej(INITIAL_POSE)  # 초기 좌표로 이동
            return False

    