# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    import os
    import cv2
    import json
    import numpy as np

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    source_path = "/home/rokey4090/ros2_ws/src/doosan-robot2/dsr_rokey/rokey/rokey/basic/data"
    os.makedirs(source_path, exist_ok=True)

    cap = cv2.VideoCapture(4)

    write_data = {}
    write_data['poses'] = []
    write_data['file_name'] = []
    while True:


        ret, frame = cap.read()
        cv2.imshow("camera", frame)

        if not ret:
            print("ERROR")
            break

        cv2.imshow("camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            pos = get_current_posx()[0]
            file_name = f"{pos[0]}_{pos[1]}_{pos[2]}.jpg"

            cv2.imwrite(f'{source_path}/{file_name}', frame)
            print("current position1 : ", pos)
            write_data["file_name"].append(file_name)
            write_data["poses"].append(pos)
            print(f"save img to {source_path}/{file_name}")
            with open(f"{source_path}/calibrate_data.json", "w") as json_file:
                json.dump(write_data, json_file, indent=4)


    rclpy.shutdown()


if __name__ == "main":
    main()