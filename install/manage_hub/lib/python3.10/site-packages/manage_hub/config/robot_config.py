"""로봇 관련 설정"""

# 힘제어/순응제어 관련 상수
COMPLIANCE_STIFFNESS = {
    'force': [400,400,400,100,100,100]    # 강성 설정
}

# 힘제어/순응제어 관련 상수
FORCE_CONTROL_PARAMS = {
    'insert': [0, 0, -30, 0, 0, 0],  # Z축 방향 10N
    'direction': [0, 0, 1, 0, 0, 0]   # Z축 방향
}

# 로봇 설정
ROBOT_CONFIG = {
    'tool': "Tool Weight_2FG",
    'tcp': "2FG_TCP"
}

# 모션 설정
MOTION_CONFIG = {
    'default_velocity': 60,      # mm/s, 기본 속도
    'default_acceleration': 60   # mm/s^2, 기본 가속도
} 