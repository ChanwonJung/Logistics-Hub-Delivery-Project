"""모션 제어 관련 유틸리티 함수"""

import sys
import time
from ..core.gripper import Gripper
from ..config.constants import OFF

from ..config.robot_config import (
    FORCE_CONTROL_PARAMS,
    COMPLIANCE_STIFFNESS,
    FORCE_CONTROL_PARAMS
)

def compliance_place_with_force() -> bool:
    """외력 감지로 물체를 배치"""
    try:
        from DSR_ROBOT2 import (
            task_compliance_ctrl,
            set_desired_force,
            release_compliance_ctrl,
            release_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            check_force_condition,
            mwait,
            set_digital_output
        )
        set_digital_output(1, OFF)  # optional
        # 1. 순응제어 설정
        time.sleep(1)
        print("순응 제어를 시작합니다")
        ret = task_compliance_ctrl(
            COMPLIANCE_STIFFNESS['force']
        )
        if ret == 0:
             print("순응 제어 준비 완료")
        else:
             print("순응 제어 준비 실패!!")
        time.sleep(1)
        print("허브에 안전히 물품을 내리는 중입니다.")
        
        # 2. 힘제어 설정
        ret = set_desired_force(
            FORCE_CONTROL_PARAMS['insert'],
            FORCE_CONTROL_PARAMS['direction'],
            mod=DR_FC_MOD_REL
        )
        if ret == 0:
             print("외력 준비 완료")
        else:
             print("외력 준비 실패!")
        time.sleep(0.5)

        force_condition = check_force_condition(DR_AXIS_Z, max=40)
        print("외력을 가하기 시작합니다. 현재 상태는 ", force_condition)
        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=30)

        print("해당 상품이 안전히 입고 되었는지 로봇팔의 힘 안정성 체크 중입니다...")
        timeout = time.time() + 2.0
        while time.time() < timeout:
            if check_force_condition(DR_AXIS_Z, max=3) == -1:  # 3N 이하로 힘이 거의 없음
                print("힘이 안정되었습니다. 상품이 정상적으로 입고되었습니다.")
                break
            print("아직 힘이 안정화되지 않았습니다.")
            time.sleep(0.1)
        
        # 3. 삽입 완료 → 힘/순응 제어 해제
        if release_force() == 0:
            print("외력를 해제합니다.")
        time.sleep(0.1)
        if release_compliance_ctrl() == 0:
            print("순응 제어를 해제합니다.")
        time.sleep(0.1)

        # 4. 물체 적재
        print("해당 상품을 허브 안으로 정상적으로 입고하였습니다.")
        gripper = Gripper()
        gripper.release()
        mwait(0.5)
        
        return True
        
    except Exception as e:
        print(f"물체 배치 중 에러: {e}")
        # 에러 발생 시 제어 모드 초기화
        release_force()
        release_compliance_ctrl()
        gripper.release()
        return False