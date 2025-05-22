import rclpy
import DR_init

try:
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        set_tool,
        set_tcp,
        movej,
        wait,
        mwait,
        movel,
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")


# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(1)
        #wait_digital_input(2)

def grip():
    release()
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1)
    #wait_digital_input(1)

def trans(coordinates, axis, value):
    if not isinstance(coordinates, list) or len(coordinates) != 6:
        raise ValueError("coordinates는 [x, y, z, a, b, c] 형식의 리스트여야 합니다.")
    
    if axis.lower() not in ['x', 'y', 'z', 'a', 'b', 'c']:
        raise ValueError("axis는 'x', 'y', 'z', 'a', 'b', 'c' 중 하나여야 합니다.")
    
    new_coordinates = coordinates.copy()
    axis_index = {'x': 0, 'y': 1, 'z': 2, 'a': 3, 'b': 4, 'c': 5}
    
    new_coordinates[axis_index[axis.lower()]] += value
    
    return new_coordinates

class InputHub_2x3:
    def __init__(self, con_coord:list, width:float, hub_coord:list):
        self.con_coord = con_coord
        self.width = width
        self.hub_coord = hub_coord

    def move_con(self):
        release()
        movel(self.con_coord, vel=VELOCITY, acc=ACC)
        movel(trans(self.con_coord, 'z', -80), vel=VELOCITY, acc=ACC)
        grip()
        movel(self.con_coord, vel=VELOCITY, acc=ACC)

    def move_hub(self):
        movel(self.hub_coord, vel=VELOCITY, acc=ACC)
        movel(trans(self.hub_coord, 'z', -80), vel=VELOCITY, acc=ACC)
        release()
        movel(self.hub_coord, vel=VELOCITY, acc=ACC)

class InputHub_2x2(InputHub_2x3):
    def __init__(self, con_coord:list, width:float, hub_coord:list):
        super().__init__(con_coord, width, hub_coord)

    def move_con(self):
        super().move_con()

    def move_hub(self):
        super().move_hub()

class InputHub_2x1(InputHub_2x3):
    def __init__(self, con_coord:list, width:float, hub_coord:list):
        super().__init__(con_coord, width, hub_coord)

    def move_con(self):
        super().move_con()

    def move_hub(self):
        super().move_hub()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("input_hub_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    
    # def wait_digital_input(sig_num):
    #     while not get_digital_input(sig_num):
    #         wait(0.5)
    #         print("Wait for digital input")
    #         pass    

    #JReady = [0, 0, 90, 0, 90, 0]
    initial_pose = posx([367.6, 2.88, 194.88, 74.52, -179.98, 75.06])

    con_rectangle_up = posx([226.06,212.33,103.22,57.85,-179.8,58.05])
    con_rectangle_down = posx([226.06,212.33,20.25,57.85,-179.8,58.05])

    con_square_up = posx([326.63,220.4,103.22,57.85,-179.8,58.05])
    con_square_down = posx([326.63,220.4,20.25,57.85,-179.8,58.05])

    hub_rectangle_up = posx([288.48,37.92,88,57.85,-178.92,58.05])
    hub_rectangle_down = posx([288.48,37.92,7.89,57.85,-178.92,58.05])

    hub_square_up = posx([288.48,-50.58,88,57.85,-178.92,58.05])
    hub_square_down = posx([288.48,-50.58, 7.89, 57.85,-178.92,58.05])
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    input_hub_2x3 = InputHub_2x3(con_rectangle_up, 48, hub_rectangle_up)
    input_hub_2x2 = InputHub_2x2(con_square_up, 32, hub_square_up)

    while rclpy.ok():

        print("초기 좌표로 이동합니다.")
        movel(initial_pose, vel=VELOCITY, acc=ACC)
        
        print("2x3 물체를 잡습니다.")
        input_hub_2x3.move_con()
        input_hub_2x3.move_hub()

        print("2x2 물체를 잡습니다.")
        input_hub_2x2.move_con()
        input_hub_2x2.move_hub()

        #print("movel-1")
        #movel(pos1, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos1, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos2, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos3, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()
if __name__ == "__main__":
    main()


