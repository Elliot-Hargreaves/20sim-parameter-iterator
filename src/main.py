import controllab

PID_KP_NAME = 'TrainPassengerAndControl.LoopControl.PID.kp'
PID_TAUD_NAME = 'TrainPassengerAndControl.LoopControl.PID.tauD'
PID_TAUI_NAME = 'TrainPassengerAndControl.LoopControl.PID.tauI'

SIM_DURATION_SECS = 350
SIM_TICKS = 35005
SIM_SECONDS_PER_TICK = SIM_TICKS / SIM_DURATION_SECS

SETPOINT_MAX = 7.0
SETPOINT_MIN = 0.0

JUMP_SIZE = 100
BISECTION_COUNT = 10

SETPOINT_LOG_NAME = "SpeedSetpoint\\output"
TRAIN_VELOCITY_LOG_NAME = "TrainPassengerAndControl\\vTrain"
TRAIN_POSITION_LOG_NAME = "TrainPassengerAndControl\\Train\\pTrain"
PASSENGER_OFFSET_LOG_NAME = "TrainPassengerAndControl\\Passenger\\positionBody\\output"

best_kp, best_tauI, best_tauD = None, None, None
max_vel_overshoot, min_vel_overshoot = None, None


class ResultClass(object):
    max_speed = None
    min_speed = None
    max_distance = None
    max_rollback = None
    max_passenger_displacement = None
    min_passenger_displacement = None
    __slots__ = [
        "max_speed",
        "min_speed",
        "max_distance",
        "max_rollback",
        "max_passenger_displacement",
        "min_passenger_displacement"
    ]

    def __init__(self):
        pass


def get_log_values(logs, value_name):
    for log in logs:
        if log.get("name") == value_name:
            return log.get("values")


def run_sim(_sim, proportion_gain, taui, taud):
    _sim.clear_all_runs()
    print(_sim.set_parameters(PID_KP_NAME, proportion_gain))
    _sim.set_variables(PID_TAUI_NAME, taui)
    _sim.set_variables(PID_TAUD_NAME, taud)
    _sim.run()
    _res = _sim.get_log_values()
    return {
        "max_speed": (
            max(get_log_values(_res, TRAIN_VELOCITY_LOG_NAME)),
            get_log_values(_res, TRAIN_VELOCITY_LOG_NAME).index(
                max(get_log_values(_res, TRAIN_VELOCITY_LOG_NAME))) / SIM_SECONDS_PER_TICK
        ),
        "min_speed": min(get_log_values(_res, TRAIN_VELOCITY_LOG_NAME)),
        "max_distance": max(get_log_values(_res, TRAIN_POSITION_LOG_NAME)),
        "min_distance": min(get_log_values(_res, TRAIN_POSITION_LOG_NAME)),
        "max_passenger_offset": max(get_log_values(_res, PASSENGER_OFFSET_LOG_NAME)),
        "min_passenger_offset": min(get_log_values(_res, PASSENGER_OFFSET_LOG_NAME))
    }


if __name__ == "__main__":
    sim = controllab.XXSim()
    sim.connect()
    sim.set_scriptmode(True)
    sim.set_log_variables([
        SETPOINT_LOG_NAME,
        TRAIN_VELOCITY_LOG_NAME,
        TRAIN_POSITION_LOG_NAME,
        PASSENGER_OFFSET_LOG_NAME
    ])
    kp = 1
    interval_jump_size = JUMP_SIZE
    running = True
    for i in range(BISECTION_COUNT):
        kp += 5
        res = run_sim(sim, kp, 1, 1)
        print(res)
        if max_vel_overshoot is None or (
                res.get("max_speed")[0] - SETPOINT_MAX < max_vel_overshoot and res.get("max_speed")[
                    0] - SETPOINT_MAX > 0):
            best_kp = kp
            max_vel_overshoot = res.get("max_speed")[0] - SETPOINT_MAX
    sim.set_scriptmode(False)
    sim.disconnect()
