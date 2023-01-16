# TODO: need to potentially make cfg a thread safe class
class ScreenConfig:
    CANCEL_BUTTON_FONTSIZE = 40

    ESTOP_XPOS = 0.925
    ESTOP_YPOS = 0.9
    ESTOP_XHINT = 0.15
    ESTOP_YHINT = 0.2

    INCR_BUTTON_HEIGHT = 0.05
    INCR_BUTTON_WIDTH = 0.1
    INCR_BUTTON_XPOS = 0.75
    INCR_BUTTON_FONTSIZE = 50

    SCREEN_X_CENTER = 0.5
    SCREEN_Y_CENTER = 0.5

    LONG_RECT_WIDTH = 0.4
    LONG_RECT_HEIGHT = 0.12

    SHORT_RECT_WIDTH = 0.2
    SHORT_RECT_HEIGHT = 0.1


    # paramters to store values
    current_action = "Waiting"
    pending_action = "Waiting"
    last_screen = "facescreen"

    payload = ""
    # User inputs sent from tablet UI to mission planner
    NO_ROS_ACTION = -1
    DELIVER_REQUEST = 8
    DELIVER_ITEMS = 9
    RETURN_HOME = 10
    START_EXTEND_ARM = 11
    ITEMS_TAKEN = 12
    START_RETRACT_ARM = 13
    ESTOP = 14


