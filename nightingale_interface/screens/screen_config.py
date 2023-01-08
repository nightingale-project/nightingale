class ScreenConfig:
    CANCEL_BUTTON_FONTSIZE = 40
    ESTOP_XPOS = 0.925
    ESTOP_YPOS = 0.9
    ESTOP_XHINT = 0.15
    ESTOP_YHINT = 0.15
    INCR_BUTTON_HEIGHT = 0.05
    INCR_BUTTON_WIDTH = 0.1
    INCR_BUTTON_XPOS = 0.75
    INCR_BUTTON_FONTSIZE = 50

    CURRENT_ACTION = "Waiting"
    PENDING_ACTION = "Waiting"
    LAST_SCREEN = "facescreen"

    # action enums for user buttons
    NO_ROS_ACTION = 0
    ESTOP_CANCEL = 1
    STOCK = 2
    DELIVER = 3
    GO_HOME = 4
    ITEM_DELIVERED = 5
    EXTEND_ARM = 6
    RETRACT_ARM = 7
