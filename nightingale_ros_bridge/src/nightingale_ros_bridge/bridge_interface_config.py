# different states that robot can be in
class RobotStatus:
    # statuses of robot sent from mission planner to tablet UI
    IDLE_HOME = 0
    DRIVING = 1
    BEDSIDE_IDLE = 2
    BEDSIDE_DELIVER = 3
    ITEM_STOCK_REACHED = 4
    EXTENDING_ARM = 5
    ARM_EXTENDED = 6
    RETRACTING_ARM = 7
    ARM_RETRACTED = 8

class BridgeConfig:
    USER_INPUT_TOPIC = "nightingale_ui/user_input"
    ROBOT_STATUS_TOPIC = "nightingale_ui/robot_status"
    ESTOP_TOPIC = "nightingale_ui/estop_status"
    UPDATE_UI_SERVICE = "nightingale_ui/update_ui_state"

