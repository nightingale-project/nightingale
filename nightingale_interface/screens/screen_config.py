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

    VIDEO_PLAYER_XPOS = 0.35
    VIDEO_PLAYER_YPOS = SCREEN_Y_CENTER 
    VIDEO_PLAYER_HEIGHT = 0.7
    VIDEO_PLAYER_WIDTH = 0.7

    # screen names
    HUB_SCREEN_NAME = 'hubscreen'
    FACE_SCREEN_NAME =  'facescreen'
    ADMIN_SCREEN_NAME = 'adminscreen'
    CONFIRMATION_SCREEN_NAME = 'confirmationscreen'
    ESTOP_SCREEN_NAME = 'estoppedscreen'
    ITEM_SELECT_SCREEN_NAME = 'itemselectscreen'
    ITEM_FILL_SCREEN_NAME = 'itemfillscreen'
    EXTEND_ARM_SCREEN_NAME = 'extendarmscreen'
    WAIT_ITEM_GET_SCREEN_NAME = 'waititemgetscreen'
    RETRACT_ARM_SCREEN_NAME = 'retractarmscreen'
    NURSE_ALERT_SCREEN_NAME = 'nursealertscreen'
    VIDEO_CALL_SCREEN_NAME = 'videocallscreen'
    WATCHDOG_TIMEOUT_SCREEN_NAME = 'watchdogscreen'

    # screen id nums
    ITEM_SELECT_SCREEN_ID = 9
    ITEM_FILL_SCREEN_ID = 10

    WATCHDOG_TIMER_SECONDS = 5 


