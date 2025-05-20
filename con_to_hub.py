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
            DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, move_periodic, DR_TOOL, movec, get_current_posj
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Import 오류: {e}")
        return

    # def twist_and_insert():
    #     move_periodic([0,0,0,0,0,8], 2.0, 0.5, 8, DR_TOOL)
    
    # def push_and_twist_repeatedly():
    #     for i in range(3): 
    #         # 약간 더 누르기
    #         move_periodic([0, 0, 3.0, 0, 0, 0], 1.0, 0.5, 1, DR_TOOL)  # Z축으로 아주 살짝 푸시
    # def wiggle_xy():
    #     move_spiral(rev=5.0,rmax=10.0,lmax=10.0,vel=(VELOCITY+20), acc= (ACC-+20), time=10.0, axis=DR_AXIS_Z,ref=DR_TOOL)


    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    # ── 좌표값 완전 고정 ──
    initial_pose       = posj(0,0, 90,0, 90, 0)
    con_rectangle_up   = posx([281, 275, 103.22, 51.86, -180, 51.93])
    con_rectangle_down = posx([281, 275,  27.5, 51.86, -180, 51.93])

    con_square_up      = posx([379.02, 270.80, 120.05, 108.41, 179.94, 108.81])
    con_square_down    = posx([379.02, 270.80, 28.29, 108.41, 179.94, 108.81])

    hub_rectangle_up   = posx([305,  28,  88.00, 51.86, -180, 51.93])
    hub_rectangle_down = posx([299.38, 28, 6.5, 51.86, -180, 51.93])
    hub_rectangle_down_2 = posx([299.38, 28, 26.5, 51.86, -180, 51.93])
    hub_rectangle_down_3 = posx([299.38, 28, 47.5, 51.86, -180, 51.93])

    hub_square_up = posx([294, -100.5, 88.00, 168.15, 179.3, 168.47])
    hub_square_down = posx([291.11, -100.5, 12.70, 168.15, 179.3, 168.47])
    hub_square_down_2 = posx([291.11, -100.5, 32.05, 168.15, 179.3, 168.47])
    hub_square_down_3 = posx([291.11, -100.5, 50.80, 168.15, 179.3, 168.47]) 

    out_rectangle_up = posx([602.59, 1.55, 88.00, 179.85, -180, 90.24])
    out_rectangle_down = posx([602.59, 1.55, 10.36, 179.85, -180, 90.24])

    def perform_force_insert_rectangle(hub_rectangle_up, hub_rectangle_down):
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
    
    def perform_force_insert_square(hub_square_up, hub_square_down):
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
        global hub_rectangle_down
        #====1층=====
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        print("====1층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        mwait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up, hub_rectangle_down)
        
        #==2층 쌓기== 
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)

        print("====2층====")
        print("큰 상품(2x3 블록)을 입고시키려 이동중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        mwait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up, hub_rectangle_down)
  
        #==3층 쌓기==
        release_force()
        release_compliance_ctrl()
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        print("초기좌표로 이동합니다.")
        wait(0.5)

        print("====3층====")
        print("큰 상품(2x3 블록)을 입고시키려 이동중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        mwait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_rectangle(hub_rectangle_up, hub_rectangle_down)

        #==허브 안 3층 2x3 블록 출고==
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down_3, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        grip()

        # 돌려서 빼기
        print("해당 상품을 꺼내는 중입니다.")
        movel([300.25,27.58,46.94,7.42,173.94,8.17], vel=3, acc=3)
        mwait(0.5)
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        mwait(0.5)

        #허브 안 2층 2x3 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down_2, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        grip()

        # 돌려서 빼기
        print("해당 상품을 꺼내는 중입니다.")
        movel([299.86,28.84,27.89,4.83,174.24,4.74], vel=3, acc=3)
        mwait(0.5)
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        mwait(0.5)

        #허브 안 1층 2x3 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(hub_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        mwait(0.5)
        hub_rectangle_down = posj(get_current_posj())
        print("hub_rectangle_down의 posj 좌표는는", hub_rectangle_down)
        hub_rectangle_up1_c = posx(300.21,28.58,8.5,4.64,171.04,4.57)
        hub_rectangle_up2_c = posx([305, 28, 88.00, 51.86, -180, 51.93])
        movec(hub_rectangle_up1_c, hub_rectangle_up2_c, vel=10, acc=10)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)        
        release()
        print("출고 준비가 완료되었습니다. 초기 좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        return 
        
    def small_block():
        global hub_square_down
        #2x2 블록 
        #====1층=====
        print("====1층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        perform_force_insert_square(hub_square_up, hub_square_down)
        
        #==2층 쌓기== 
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)

        print("====2층====")
        print("해당 상품을 허브로 입고시키는 중입니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        wait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)

        perform_force_insert_square(hub_square_up, hub_square_down)

        #==3층 쌓기==
        release_force()
        release_compliance_ctrl()
        print("초기좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)
        print("====3층====")
        print("2x2 물체를 잡습니다.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        wait(1.0)
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)

        perform_force_insert_square(hub_square_up, hub_square_down)
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(2.0)

        #2x2 3층부터 분리 후 출고
        #허브 안 3층 2x2 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(hub_square_down_3, vel=VELOCITY, acc=ACC)
        wait(1.0)
        grip()
        wait(2.0)
        # 돌려서 빼기
        print("해당 물품을 꺼내는 중입니다. 잠시 기다려주세요.")
        movel([291, -104.89, 44.98, 158.64, -173.96, 159.54], vel=2.5, acc=2.5)
        wait(2.0)
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        wait(2.0)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        wait(1.0)
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
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
        movel([291, -104.89, 22, 158.64, -173.96, 159.54], vel=2.5, acc=2.5)
        mwait(2.0)
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(0.5)
        movel(out_rectangle_down, vel=15, acc=15)
        release()
        mwait(0.5)
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        mwait(0.5)

        #허브 안 1층 2x2 블록 출고
        print("출고를 시작합니다. 잠시 기다려주세요 ...")
        movel(hub_square_up, vel=VELOCITY, acc=ACC)
        movel(hub_square_down, vel=VELOCITY, acc=ACC)
        grip()
        wait(2.0)
        hub_square_down = posj(get_current_posj())
        print("hub_square_down의 posj 좌표는", hub_square_down)
        print("해당 물품을 꺼내는 중입니다. 잠시 기다려주세요.")
        hub_square_up1_c = posx(280.81, -102.11, 11.59, 159.86, -168.90, 160.48)
        hub_square_up2_c = posx([291.11, -100.5, 88, 168.15, 179.3, 168.47])
        wait(1.0)
        movec(hub_square_up1_c, hub_square_up2_c, vel=10, acc=10)
        movel(out_rectangle_up, vel=VELOCITY, acc=ACC)
        mwait(1.0)
        movel(out_rectangle_down, vel=15, acc=15)        
        release()
        print("출고 준비가 완료되었습니다. 초기 위치로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)

        return 

    def trash(): #불량품 함수
        pass

    def max_height():
        # 2x3 블록 혹은 2x2 블록 3층 이후 또 쌓으려고 할 때 예외처리 다음 영역으로 넘어감
        # get_current_posx()[2]
        pass

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    #초기 위치

    while rclpy.ok():
        release()
        release_force()
        release_compliance_ctrl()
        print("===물류 공정을 시작합니다===")
        print("초기 좌표로 이동합니다.")
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)
        print("공장에서 배송온 물품을 허브 안으로 입고하기 위한 작업을 시작합니다.")
        print("안전에 유의하세요.")
        movel(con_rectangle_up, vel=VELOCITY, acc=ACC)
        movel(con_rectangle_down, vel=VELOCITY, acc=ACC)
        grip()
        time.sleep(1)
        print("물체를 잡은 그리퍼의 1번 포트 입력값: ", get_digital_input(1))
        print("물체를 잡은 그리퍼의 2번 포트 입력값: ", get_digital_input(2))
        print("물체를 잡은 그리퍼의 3번 포트 입력값: ", get_digital_input(3))

        if get_digital_input(1) == 1 and get_digital_input(2) == 0 and get_digital_input(3) == 0:
            print("큰 상품(2x3 블록)을 인식하였습니다. 해당 허브 영역으로 이동을 시작합니다.")
            big_block()

        elif get_digital_input(1) == 1 and get_digital_input(2) == 1 and get_digital_input(3) == 0:
            print("작은 상품(2x2) 블록을 잡았습니다. 해당 허브 영역으로 이동을 시작합니다.")
            small_block()

        else:
            trash()
            

    rclpy.shutdown()

if __name__ == "__main__":
    main()
