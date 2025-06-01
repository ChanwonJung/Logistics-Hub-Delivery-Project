import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("input_hub_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl, check_force_condition, task_compliance_ctrl,
            set_desired_force, set_digital_output, get_digital_input, get_digital_output,
            set_tool, set_tcp, movej, wait, mwait, movel, release_force,move_spiral,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, move_periodic, DR_TOOL, movec, get_current_posj,
            get_current_posx,check_motion
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().error(f"Import 오류: {e}")
        return

    def trash_twist_1():
        move_periodic([8,8,0,-3,5,7], 2.0, 0.5, 3, DR_TOOL)
        return
    
    def trash_twist_2():
        move_periodic([-8,-8,0,3,-5,-7], 2.0, 0.5, 3, DR_TOOL)
        return 
    
    # def push_and_twist_repeatedly():
    #     for i in range(3): 
    #         # 약간 더 누르기
    #         move_periodic([0, 0, 3.0, 0, 0, 0], 1.0, 0.5, 1, DR_TOOL)  # Z축으로 아주 살짝 푸시
    #def trash_wiggle_xy():
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    # ── 좌표값 완전 고정 ──
    initial_pose       = posj(0,0, 90,0, 90, 0)
    con_rectangle_up   = posx([281, 275, 130, 51.86, -180, 51.93])
    con_rectangle_down = posx([281, 275,  27.5, 51.86, -180, 51.93])

    con_square_up      = posx([379.02, 270.80, 120.05, 108.41, 179.94, 108.81])
    con_square_down    = posx([379.02, 270.80, 28.29, 108.41, 179.94, 108.81])

    hub_rectangle_up   = posx([305,  28,  130, 51.86, -180, 51.93])
    hub_rectangle_down = posx([299.38, 28, 6.5, 51.86, -180, 51.93])
    hub_rectangle_down_2 = posx([299.38, 28, 26.5, 51.86, -180, 51.93])
    hub_rectangle_down_3 = posx([299.38, 28, 47.5, 51.86, -180, 51.93])

    hub_square_up = posx([292, -101.5, 130, 168.15, 179.3, 168.47])
    hub_square_down_1 = posx([291.11, -101.5, 12.70, 168.15, 179.3, 168.47])
    hub_square_down_2 = posx([291.11, -101.5, 32.05, 168.15, 179.3, 168.47])
    hub_square_down_3 = posx([291.11, -101.5, 50.80, 168.15, 179.3, 168.47]) 

    out_rectangle_up = posx([602.59, 1.55, 130.00, 179.85, -180, 90.24])
    out_rectangle_down = posx([602.59, 1.55, 13, 179.85, -180, 90.24])

    hub_trash_up = posx([467.1,-118.36, 120, 109.81, -179.36, 110.45])

    def perform_force_insert_rectangle(hub_rectangle_up):
        # 허브 위치로 이동
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        set_digital_output(1, OFF)  # optional
        time.sleep(1)
        print("순응 제어를 시작합니다.")
        ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
        if ret == 0:
             print("순응 제어 준비 완료")
        else:
             print("순응 제어 준비 실패!!")
        time.sleep(1)

        print("허브에 안전히 물품을 내리는 중입니다.")
        ret = set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
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

        # 삽입 완료 → 힘/순응 제어 해제
        if release_force() == 0:
            print("외력를 해제합니다.")
        time.sleep(0.1)
        if release_compliance_ctrl() == 0:
            print("순응 제어를 해제합니다.")
        time.sleep(0.1)

        print("해당 상품을 허브 안으로 정상적으로 입고하였습니다.")
        release()
        mwait(1)
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
    
    def perform_force_insert_square(hub_square_up):
         # 허브 위치로 이동~순응제어~릴리스 등 기존 흐름
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        set_digital_output(1, OFF)  # optional
        time.sleep(1)
        print("순응 제어를 시작합니다")
        ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
        if ret == 0:
             print("순응 제어 준비 완료")
        else:
             print("순응 제어 준비 실패!!")
        time.sleep(1)

        print("허브에 안전히 물품을 내리는 중입니다.")
        ret = set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
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

        # 삽입 완료 → 힘/순응 제어 해제
        if release_force() == 0:
            print("외력를 해제합니다.")
        time.sleep(0.1)
        if release_compliance_ctrl() == 0:
            print("순응 제어를 해제합니다.")
        time.sleep(0.1)

        release()
        mwait(1.0)
        print("해당 상품을 허브 안으로 정상적으로 입고하였습니다.")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)


    def big_block():
        #2x3 블록 
        #====1층=====
        print("====1층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        time.sleep(1.5)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up)
        
        #==2층 쌓기== 
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        time.sleep(1)

        print("====2층====")
        print("큰 상품(2x3 블록)을 입고시키려 이동중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        time.sleep(1.5)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up)
  
        #==3층 쌓기==
        release_force()
        release_compliance_ctrl()
        movej(initial_pose, vel=30, acc=30)
        print("초기좌표로 이동합니다.")
        time.sleep(1)

        print("====3층====")
        print("큰 상품(2x3 블록)을 입고시키려 이동중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        time.sleep(1.5)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up)

        #==허브 안 3층 2x3 블록 출고==
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down_3, vel=VELOCITY, acc=ACC)
        time.sleep(1.0)
        grip()

        # 돌려서 빼기
        print("해당 상품을 꺼내는 중입니다.")
        time.sleep(1.5)
        movel([300.25,27.58,46.94,7.42,173.94,8.17], vel=3, acc=3)
        time.sleep(1.0)
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        time.sleep(1.0)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        time.sleep(1.0)
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        time.sleep(1.0)

        #허브 안 2층 2x3 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down_2, vel=VELOCITY, acc=ACC)
        time.sleep(1.0)
        grip()

        # 돌려서 빼기
        print("해당 상품을 꺼내는 중입니다.")
        time.sleep(1.5)
        movel([299.86,28.84,27.89,4.83,174.24,4.74], vel=3, acc=3)
        time.sleep(1.0)
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        time.sleep(1.0)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        mwait(0.5)

        #허브 안 1층 2x3 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        time.sleep(1.0)
        movel(hub_rectangle_up, vel=5, acc=5)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)        
        release()
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        return 
        
    def small_block():
        #2x2 블록 
        #====1층=====
        print("====1층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_square(hub_square_up)
        
        #==2층 쌓기== 
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        wait(0.5)

        print("====2층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        wait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)

        perform_force_insert_square(hub_square_up)

        #==3층 쌓기==
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        wait(0.5)
        print("====3층====")
        print("2x2 물체를 잡습니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        wait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)

        perform_force_insert_square(hub_square_up)
        movej(initial_pose, vel=30, acc=30)
        wait(2.0)

        #2x2 3층부터 분리 후 출고
        #허브 안 3층 2x2 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(hub_square_down_3, vel=VELOCITY, acc=ACC)
        wait(1.0)
        grip()
        time.sleep(1)
        # 돌려서 빼기
        print("해당 물품을 꺼내는 중입니다. 잠시 기다려주세요.")
        mwait(0.5)
        movel([291.38, -101.37, 50.60, 162.05, -161.29, 164.48], vel=3, acc=3)
        mwait(1.0)
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        wait(2.0)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        wait(1.0)
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        mwait(0.5)

        #허브 안 2층 2x2 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(hub_square_down_2, vel=VELOCITY, acc=ACC)
        wait(1.0)
        grip()
        wait(2.0)

        # 돌려서 빼기
        print("해당 물품을 꺼내는 중입니다. 잠시 기다려주세요.")
        movel([291.38, -101.37, 31.96, 162.05, -161.29, 164.48], vel=3, acc=3)
        mwait(2.0)
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        mwait(0.5)
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        mwait(0.5)

        #허브 안 1층 2x2 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(hub_square_down_1, vel=VELOCITY, acc=ACC)
        grip()
        wait(2.0)
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        wait(1.0)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(1.0)
        movel(out_rectangle_down, vel=15, acc=15)        
        release()
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        wait(0.5)

        return 

    def trash(): #불량품 함수
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        wait(1)
        movel(hub_trash_up, vel=VELOCITY, acc=ACC)
        time.sleep(1)
        release()
        print("해당 상품을 불량품으로 인식하여 적재하였습니다.")
        time.sleep(1)
        print("불량품 영역 내 불량품이 다 찼는지 확인중입니다 ...")
        grip()
        mwait(0.3)
        release_force()
        release_compliance_ctrl()
        # 순응 제어
        print("순응 제어를 시작합니다.")
        ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
        if ret == 0:
             print("순응 제어 준비 완료")
        else:
             print("순응 제어 준비 실패!!")
        time.sleep(1)
        wait(2.0)
        print("외력 동작을 준비합니다. ")
        ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("ret는 ", ret)
        if ret == 0:
             print("외력 준비 완료")
        else:
             print("외력 준비 실패!")
        print("현재 허브 안 불량품이 쌓여져 있는 높이를 측정중입니다 ...")
        force_condition = check_force_condition(DR_AXIS_Z, max=10)

        while (force_condition > -1): 
            force_condition = check_force_condition(DR_AXIS_Z, max=5)
            
        print("현재 좌표의 위치는 ", get_current_posx())
        current_trash_z = get_current_posx()[0][2]
        print(f"현재 감지된 가장 높은 z 위치는: {current_trash_z:.1f}")

        # 측정 완료 → 힘/순응 제어 해제
        if release_force() == 0:
            print("현재 허브 안 불량품 영역의 높이를 확인하였습니다. 외력를 해제합니다.")
        time.sleep(0.1)

        if release_compliance_ctrl() == 0:
            print("현재 허브 안 불량품 영역의 높이를 확인하였습니다. 순응 제어를 해제합니다.")
        time.sleep(0.1)

        if current_trash_z >= 70:
            print("현재 허브 안 불량품 영역에 불량품이 가득 찼습니다. 불량품 영역을 청소해주세요.")
            wait(3)
            movel(hub_trash_up, vel=VELOCITY, acc=ACC)
            return 
        else:
            print("현재 불량품이 쌓여져 있는 높이는 약 ", current_trash_z, "입니다.")
            grip()
            time.sleep(1)
            print("쌓여져 있는 불량품의 높이를 균일하게 맞추겠습니다.잠시 기다려주세요")
            time.sleep(1)
            trash_twist_1()
            wait(1.0)
            trash_twist_2()

            time.sleep(1)
            print("불균형하게 쌓여있는 불량품의 높이를 균일하게 맞췄습니다. 초기 위치로 돌아갑니다.")
            movel(hub_trash_up, vel=VELOCITY, acc=ACC)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    #초기 위치

    while rclpy.ok():
        print("===물류 공정을 시작합니다===")
        print("초기 좌표로 이동합니다.")
        movej(initial_pose, vel=30, acc=30)
        mwait(1.0)
        release()
        print("공장에서 배송온 물품을 허브 안으로 입고하기 위한 작업을 시작합니다.")
        print("안전에 유의하세요.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        time.sleep(0.5)
        print("물체를 잡은 그리퍼의 1번 포트 입력값: ", get_digital_input(1))
        time.sleep(0.5)
        print("물체를 잡은 그리퍼의 2번 포트 입력값: ", get_digital_input(2))
        time.sleep(0.5)
        print("물체를 잡은 그리퍼의 3번 포트 입력값: ", get_digital_input(3))
        time.sleep(0.5)

        if get_digital_input(1) == 1 and get_digital_input(2) == 0 and get_digital_input(3) == 0:
            print("큰 상품(2x3 블록)을 인식하였습니다. 해당 허브 영역으로 이동을 시작합니다.")
            big_block()

        elif get_digital_input(1) == 1 and get_digital_input(2) == 1 and get_digital_input(3) == 0:
            print("작은 상품(2x2) 블록을 잡았습니다. 해당 허브 영역으로 이동을 시작합니다.")
            small_block()

        elif get_digital_input(1) == 1 and get_digital_input(2) == 1 and get_digital_input(3) == 0:
            print("불량품으로 인식됩니다. 해당 허브 영역으로 이동을 시작합니다.")
            trash()
        
        else:
            print("입고된 상품이 존재하지 않습니다. 적재를 종료합니다.")
            break
            

    rclpy.shutdown()

if __name__ == "__main__":
    main()
