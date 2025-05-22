"""Input Hub 적재 제어 노드"""

import rclpy
import DR_init

from ..config.constants import (
    ROBOT_ID,
    ROBOT_MODEL,
    INITIAL_POSE,
    OBJECT_DIV,
    HUB_START_POSITIONS,
    CONTAINER_POSITIONS,
    Z_OFFSET
)
from ..config.robot_config import MOTION_CONFIG
from ..core.hub_base import Hub2x3, Hub2x2, HubDefect
from ..core.gripper import Gripper
from ..utils.robot_utils import setup_robot, get_object_width, safe_shutdown

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    try:
        rclpy.init(args=args)
        node = rclpy.create_node("input_hub_node", namespace=ROBOT_ID)
        DR_init.__dsr__node = node

        from DSR_ROBOT2 import (
            movej,
            wait,
            movel,
            trans,
            DR_BASE
        )
        
        # 로봇 설정
        if not setup_robot():
            print("로봇 설정에 실패했습니다. 프로그램을 종료합니다.")
            return
            
        # 적재 모드
        print("적재 모드를 시작합니다.")
        container_index = 0
        gripper = Gripper()
        
        while rclpy.ok():
            try:
                print("초기 자세로 이동합니다.")
                movej(INITIAL_POSE, 
                      vel=MOTION_CONFIG['default_velocity'],
                      acc=MOTION_CONFIG['default_acceleration'])
                wait(0.5)
                
                # 1. 컨테이너로 이동
                movel(CONTAINER_POSITIONS[container_index], 
                      vel=MOTION_CONFIG['default_velocity'],
                      acc=MOTION_CONFIG['default_acceleration'])
                
                # 2. z축 하강
                current_pos = trans(CONTAINER_POSITIONS[container_index], [0, 0, Z_OFFSET, 0, 0, 0], ref=DR_BASE)
                movel(current_pos, vel=MOTION_CONFIG['default_velocity'],
                      acc=MOTION_CONFIG['default_acceleration'])
                
                # 3. 물체 잡기
                if not gripper.grip():
                    print("물체를 잡지 못했습니다.")
                    continue
                
                # 4. 물체 너비 측정
                object = get_object_width()
                
                # 5. 허브 인스턴스 생성
                if object == OBJECT_DIV['2x3']:
                    hub = Hub2x3(HUB_START_POSITIONS['2x3'])
                elif object == OBJECT_DIV['2x2']:
                    hub = Hub2x2(HUB_START_POSITIONS['2x2'])
                elif object == OBJECT_DIV['defect']:
                    hub = HubDefect(HUB_START_POSITIONS['defect'])
                else:
                    print("작업할 물체가 없으므로 초기 위치로 이동합니다")
                    gripper.release()
                    movej(INITIAL_POSE, 
                          vel=MOTION_CONFIG['default_velocity'],
                          acc=MOTION_CONFIG['default_acceleration'])  # 초기 위치로 이동
                    break  # while문 종료
                
                # 6. 물체 이동
                result = hub.move_hub()
                if result == 'error':
                    print("허브로 물체 이동 실패")
                    continue
                elif result == 'area_full':
                    print("작업 영역이 가득 찼습니다. 작업을 종료합니다.")
                    movej(INITIAL_POSE, 
                          vel=MOTION_CONFIG['default_velocity'],
                          acc=MOTION_CONFIG['default_acceleration'])  # 초기 좌표로 이동
                    break  # while문 종료
                elif result == 'overflow':
                    # 물체를 원래 위치(컨테이너)에 다시 놓기
                    movel(CONTAINER_POSITIONS[container_index], 
                      vel=MOTION_CONFIG['default_velocity'],
                      acc=MOTION_CONFIG['default_acceleration'])
                    rollback_con = trans(CONTAINER_POSITIONS[container_index], [0, 0, Z_OFFSET, 0, 0, 0], ref=DR_BASE)
                    movel(rollback_con, vel=MOTION_CONFIG['default_velocity']//2,
                          acc=MOTION_CONFIG['default_acceleration']//2)
                    gripper.release()  # 물체 놓기
                    movej(INITIAL_POSE, 
                          vel=MOTION_CONFIG['default_velocity'],
                          acc=MOTION_CONFIG['default_acceleration'])  # 초기 좌표로 이동
                    continue
                elif result == 'success':
                    # 7. 컨테이너 인덱스 업데이트
                    container_index = (container_index + 1) % 2
                
                # 작업 완료 메시지
                print(f"{hub.hub_type} 허브로 이동이 완료되었습니다.")
                
            except Exception as e:
                print(f"작업 중 에러 발생: {e}")
                if "작업 종료" in str(e):
                    break
                continue
            
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
        safe_shutdown()  # 안전 종료 처리
    except Exception as e:
        print(f"Error occurred: {e}")
        safe_shutdown()  # 안전 종료 처리
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main() 