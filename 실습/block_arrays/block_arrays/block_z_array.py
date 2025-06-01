import rclpy
import DR_init
import time


# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 45, 45

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("block_arrays_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_robot_state

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl, check_force_condition, task_compliance_ctrl,
            set_desired_force, set_digital_output, get_digital_input, get_digital_output,
            set_tool, set_tcp, movej, wait, mwait, movel, release_force,move_spiral,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, move_periodic, DR_TOOL, movec, get_current_posx
        )

        from DR_common2 import posx, posj
    
    except ImportError as e:
        node.get_logger().error(f"Import 오류: {e}")
        return
    
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(1)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1)   
     
    # ── 좌표값 완전 고정 ──
    initial_pose = posj(0,0, 90,0, 90, 0)

    #왼쪽 판 좌표
    a_1_up = posx([248.99, 298.52, 110.00, 4.59, -179.99, 5.36])
    a_1_down = posx([248.99, 298.52, 36.1, 4.59, -179.99, 5.36])
    a_2_up = posx([248.99, 248.28, 110.00, 4.59, -179.99, 5.36])
    a_2_down = posx([248.99, 248.28, 36.1, 4.59, -179.99, 5.36])
    a_3_up = posx([248.99, 198.04, 110.00, 4.59, -179.99, 5.36])
    a_3_down = posx([248.99, 198.04, 36.1, 4.59, -179.99, 5.36])


    b_1_up = posx([305.01, 298.52, 110.00, 4.59, -179.99, 5.36])
    b_1_down = posx([305.01, 298.52, 36.1, 4.59, -179.99, 5.36])
    b_2_up = posx([305.01, 248.28, 110.00, 4.59, -179.99, 5.36])
    b_2_down = posx([305.01, 248.28, 36.1, 4.59, -179.99, 5.36])
    b_3_up = posx([305.01, 198.04, 110.00, 4.59, -179.99, 5.36])
    b_3_down = posx([305.01, 198.04, 36.1, 4.59, -179.99, 5.36])

    c_1_up = posx([356.99, 298.52, 110.00, 4.59, -179.99, 5.36])
    c_1_down = posx([356.99, 298.52, 36.1, 4.59, -179.99, 5.36])
    c_2_up = posx([356.99, 248.28, 110.00, 4.59, -179.99, 5.36])
    c_2_down = posx([356.99, 248.28, 36.1, 4.59, -179.99, 5.36])
    c_3_up = posx([356.99, 198.04, 110.00, 4.59, -179.99, 5.36])
    c_3_down = posx([356.99, 198.04, 36.1, 4.59, -179.99, 5.36])

    #옮길려고 하는 판(오른쪽) 좌표
    d_1_up = posx([404.57, 298.52, 110.00, 46.87, 179.72, 47.93])
    d_1_down = posx()
    d_2_up = posx([404.57, 248.28, 110.00, 46.87, 179.72, 47.93])
    d_2_down = posx()
    d_3_up = posx([404.57, 198.04, 110.00, 46.87, 179.72, 47.93])
    d_3_down = posx()

    e_1_up = posx([452.59, 298.52, 110.00, 46.87, 179.72, 47.93])
    e_1_down = posx()
    e_2_up = posx([452.59, 248.28, 110.00, 46.87, 179.72, 47.93])
    e_2_down = posx()
    e_3_up = posx([452.59, 198.04, 110.00, 46.87, 179.72, 47.93])
    e_3_down = posx()

    f_1_up = posx([504.61, 298.52, 110.00, 46.87, 179.72, 47.93])
    f_1_down = posx()
    f_2_up = posx([504.61, 248.28, 110.00, 46.87, 179.72, 47.93])
    f_2_down = posx()
    f_3_up = posx([504.61, 198.04, 110.00, 46.87, 179.72, 47.93])
    f_3_down = posx()

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        left_positions = [a_1_up, a_2_up, a_3_up,
            b_1_up, b_2_up, b_3_up,
            c_1_up, c_2_up, c_3_up]
        
        left_positions_down = [a_1_down, a_2_down, a_3_down,
            b_1_down, b_2_down, b_3_down,
            c_1_down, c_2_down, c_3_down]
        
        right_positions = [d_1_up, d_2_up, d_3_up,
            e_1_up, e_2_up, e_3_up,
            f_1_up, f_2_up, f_3_up]
        
        #측정된 z값 높이 list
        measured_z_data = []

        grip()
        release_force()
        release_compliance_ctrl()
        print("초기 좌표로 이동합니다")
        movel([252.99, 298.52, 200.00, 4.59, -179.99, 5.36], vel=30, acc=30)
        movej(initial_pose, vel=VELOCITY, acc=ACC)
        wait(0.5)
        print("각 좌표별 외력 감지를 시작합니다.")

        # 3x3 판 안에 9개 좌표를 그리퍼가 닫은 상태에서
        # 한번씩 콕 찍는다 (외력 감지할때까지) 반복문 돌때까지 
        # 이때 좌표는 list로 만들어서 반복문 돌고
        for i in range(len(left_positions)):
            grip()
            movel(left_positions[i], vel = 30, acc = 30)
            mwait(0.3)

            print("순응 제어를 시작합니다")
            ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
            if ret == 0:
                print("Compliance_ctrl Set")
            else:
                print("Compliance_ctrl failed!!")
            time.sleep(1)
            print("외력이 발생할때까지 내립니다.")
            ret = set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            if ret == 0:
                print("set_desired_force Set")
            else:
                print("set_desired_force Set Failed!!!!!")
            
            time.sleep(1)
            print("외력을 확인 중입니다 ...")
            force_condition = check_force_condition(DR_AXIS_Z, max=20)
            while (force_condition > -1): 
                force_condition = check_force_condition(DR_AXIS_Z, max=5)
            
            print("현재 좌표의 pose: ", get_current_posx())
            current_z = get_current_posx()[0][2]
            print(f"[{i+1}] 감지된 Z좌표: {current_z:.1f}")
            measured_z_data.append((i, current_z))

            if release_force() == 0:
                print("release force")
            time.sleep(0.1)

            if release_compliance_ctrl() == 0:
                print("release compliance ctrl")
            time.sleep(0.1)
        release_force()
        release_compliance_ctrl()

        measured_z_data.sort(key=lambda x: x[1])  # x[1]은 z값
        print("정렬된 물체의 z값:", measured_z_data)

        movel([508.61, 198.04, 100.00, 46.87, 179.72, 47.93], vel = 30, acc = 30)
        mwait(0.5)
        movej(initial_pose, vel=VELOCITY, acc=ACC)

        for i in range(len(measured_z_data)):
            idx = measured_z_data[i][0]  # 정렬된 블록의 인덱스만 사용
            print(f"[{i+1}] 블록 {(idx+1)} → 위치 {(i+1)}")

            mwait(1)
            movej(initial_pose, vel=15, acc=15)
            #왼쪽에서 잡기
            movel(left_positions[idx], vel=VELOCITY, acc=ACC)
            release()
            print("왼쪽 블럭 잡으러 가는중 ..")
            movel(left_positions_down[idx], vel=VELOCITY, acc=ACC)
            grip()
            mwait(0.5)
            movel(left_positions[idx], vel=VELOCITY, acc=ACC)

            # 오른쪽 target 위치로 이동
            print("오른쪽 target 좌표로 이동 중 ..")
            movel(right_positions[i], VELOCITY, ACC)
            mwait(0.5)

            print("오른쪽 target에서의 순응 제어를 시작합니다")
            ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
            if ret == 0:
                print("Compliance_ctrl Set")
            else:
                print("Compliance_ctrl failed!!")
            time.sleep(1)
            print("외력이 발생할때까지 내립니다. 삽입중 ...")
            ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            if ret == 0:
                print("set_desired_force Set")
            else:
                print("set_desired_force Set Failed!!!!!")
            
            time.sleep(1)

            force_condition = check_force_condition(DR_AXIS_Z, max=25)
            while (force_condition > -1): 
                force_condition = check_force_condition(DR_AXIS_Z, max=15)

            if release_force() == 0:
                print("release force")
            time.sleep(1)

            if release_compliance_ctrl() == 0:
                print("release compliance ctrl")
            time.sleep(1)

            release()
            movel(right_positions[i], VELOCITY, ACC)
        
        print("삽입 완료")
        release_force()
        release_compliance_ctrl()
        mwait(0.5)

        print("삽입 끝. 초기 좌표로 이동합니다")
        movej(initial_pose, vel=15, acc=15)

   
        # 콕 찍었을 때 외력 감지된 그 순간에 get_current_posx[2] return
        # 즉, z축의 높이를 인식해서 정렬하려고 한다.
        # 9개 값 다 비교해서 작은거부터
        # 오른쪽에 있는 판에다가 오름차순으로 정렬

        rclpy.shutdown()

if __name__ == "__main__":
    main()
