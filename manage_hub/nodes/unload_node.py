"""Unload Hub 하역 제어 노드"""

import rclpy
import DR_init

from ..config.constants import (
    ROBOT_ID,
    ROBOT_MODEL,
    INITIAL_POSE
)
from ..core.unload_hub import UnloadHub2x3, UnloadHub2x2
from ..utils.robot_utils import setup_robot, safe_shutdown

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("unload_hub_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            wait,
            set_tool,
            set_tcp
        )
        
        # 로봇 설정
        if not setup_robot():
            print("로봇 설정에 실패했습니다. 프로그램을 종료합니다.")
            return
            
        # 하역 모드
        print("하역 모드를 시작합니다.")
        big_hub = UnloadHub2x3()
        small_hub = UnloadHub2x2()
        
        while rclpy.ok():
            try:
                print("초기 자세로 이동합니다.")
                movej(INITIAL_POSE)
                wait(0.5)
                
                # 사용자 입력 받기
                command = input("하역할 허브를 선택하세요 (1: 2x3, 2: 2x2, q: 종료): ")
                
                if command == 'q':
                    print("프로그램을 종료합니다.")
                    break
                elif command == '1':
                    print("2x3 허브 하역을 시작합니다.")
                    big_hub.unload()
                elif command == '2':
                    print("2x2 허브 하역을 시작합니다.")
                    small_hub.unload()
                else:
                    print("잘못된 명령입니다. 다시 시도해주세요.")
                    continue
                
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
        rclpy.shutdown()

if __name__ == '__main__':
    main() 