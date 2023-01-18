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

class UserInputs:
    # User inputs sent from tablet UI to mission planner
    NO_INPUT = -2
    NO_ROS_ACTION = -1
    STOCK_ITEMS = 9
    DELIVER_ITEMS = 10 
    RETURN_HOME = 11
    START_EXTEND_ARM = 12
    ITEMS_TAKEN = 13
    START_RETRACT_ARM = 14
    ESTOP = 15
    ESTOP_CANCEL = 16
    WD_TIMEOUT = 17



