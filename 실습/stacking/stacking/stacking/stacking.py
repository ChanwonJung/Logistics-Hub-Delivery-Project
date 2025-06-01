import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 20, 20
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("rokey_stacking", namespace=ROBOT_ID)
DR_init.__dsr__node = node

ON, OFF = 1, 0
HOME_READY = [0, 0, 90, 0, 90, 0]

import time

try:
    from DSR_ROBOT2 import (
        get_digital_input,
        set_digital_output,
        get_current_posx,
        trans,
        set_tool,
        set_tcp,
        movej,
        movel,
        wait,
        mwait,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        check_force_condition,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        posx
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(1)
    #wait_digital_input(2)

def grip():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1)
    #wait_digital_input(1)


class Box:
    def __init__(self, id, pos_id, position):
        self.id = id
        self.pos_id = pos_id
        self.position = position
        self.target_offset = 100
        self.stacked = False

    def set_pos_id(self, pos_id):
        self.pos_id = pos_id

    def set_box_id(self, id):
        self.id = id
    
    def set_position(self, pos_list):
        self.position = pos_list
    
    def info(self):
        return f"id : {self.id}\nposition : {self.pos_id} -> {self.position}\n=====\n"

    def __move_to_pos(self, target_pos, action=None):
        init_pos = get_current_posx()[0]
        print("init_pos:", init_pos)
        mwait()
        time.sleep(0.1)

        ready_pos = list(init_pos)
        ready_pos[2] = 350
        print("ready_pos:", ready_pos)

        print("moving to ready_pos")
        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        mwait()
        time.sleep(0.1)

        print("moving to target_pos")
        movel(target_pos, vel=VELOCITY, acc=ACC, mod=0)
        print("target_pos:", target_pos)
        mwait()
        time.sleep(0.1)

        if action == 'grip':
            grip()
        elif action == 'release':
            release()
        mwait()

        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        movej(HOME_READY, vel=VELOCITY, acc=ACC)
        return target_pos


    def stack(self):
        print(f"[stack] 호출됨 - Box {self.id}")
        if self.stacked:
            print(f"Box {self.id} is already stacked!")
            return
        if not self.position:
            print(f"[stack] position이 설정되지 않음!")
            return
        self.__move_to_pos(self.position, action='release')
        print(f"[stack] 이동 완료 → stacked 상태 True로 설정")
        self.stacked = True


    def unstack(self):
        if not self.stacked:
            print(f"Box {self.id} is already unstacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        self.__move_to_pos(self.position, action='grip  ')
        self.stacked = False

def to_grip():
    movel([0,-70, 70, 0, 0, 0], vel = VELOCITY, acc=ACC, mod=1)
    print("초기 좌표로 이동합니다. ")
    movej(HOME_READY, vel=VELOCITY, acc=ACC)
    grip()
    print("밑으로 내리겠습니다.")
    movel([0,0,-30,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    mwait(1)
    print("순응 제어를 시작합니다")
    ret = task_compliance_ctrl(stx=[400,400,400,100,100,100])
    if ret == 0:
        print("Compliance_ctrl Set")
    else:
        print("Compliance_ctrl failed!!")
    time.sleep(1)
    print("외력이 발생할때까지 내립니다.")
    #release_force()
    time.sleep(1)
    ret = set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    if ret == 0:
        print("set_desired_force Set")
    else:
        print("set_desired_force Set Failed!!!!!")
    
    time.sleep(1)
    force_condition = check_force_condition(DR_AXIS_Z, max=15)
    print("force_condition Start : ", force_condition)
    while (force_condition > -1): 
        force_condition = check_force_condition(DR_AXIS_Z, max=5)
    if release_force() == 0:
        print("release force")
    time.sleep(0.1)
    if release_compliance_ctrl() == 0:
        print("release compliance ctrl")
    time.sleep(0.1)

    movel([0,0,20,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    release()
    movel([0,0,-45,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    grip()
    movej(HOME_READY, vel=VELOCITY, acc=ACC)


def main():
    box_dict = {}
    Cup_1 = Box(id=1, pos_id='Up', position= posx(338.98, 467.61, 341.46, 55.14, 174.38, -123.76))
    Lego_1 = Box(id=2,pos_id='Down',position = posx(332.94, 519.58, 74.86, 92.05, 116.48, 88.60))
    box_dict[Cup_1.id] = Cup_1
    box_dict[Lego_1.id] = Lego_1

    while rclpy.ok():
        print("""
        ====== 명령어 입력 ======
        grip    : 물체를 집고 이동
        unstack : 물체를 원래 위치로 되돌림
        exit    : 종료
        =========================
        """)
        cmd = input("명령어를 입력하세요: ").strip().lower()

        if cmd == 'grip':
            try:
                id = int(input("이동할 박스 ID: "))
                pos_id = input("새로운 위치 ID: ")
                if id not in box_dict:
                    print("존재하지 않는 ID입니다.")
                    continue
                
                box = box_dict[id]
                box.set_pos_id(pos_id)

                release()
                to_grip()  # 외력 기반으로 물체 집기
                box.stack()  # 등록된 position으로 이동 후 release()

            except Exception as e:
                print("오류 발생:", e)

        elif cmd == 'unstack':
            try:
                id = int(input("되돌릴 박스 ID: "))
                if id in box_dict:
                    box_dict[id].unstack()
                else:
                    print("존재하지 않는 ID입니다.")
            except:
                print("잘못된 입력입니다.")

        elif cmd == 'exit':
            print("프로그램을 종료합니다.")
            break

        else:
            print("지원하지 않는 명령입니다.")

    rclpy.shutdown()

    	# Your Code Here 
        

if __name__ == "__main__":
    main()
