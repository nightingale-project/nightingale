from kivy.uix.screenmanager import Screen
from kivy.uix.screenmanager import SlideTransition
from kivy.uix.button import Button

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class NurseAlertScreen:
    def nurse_alert_cancel(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "right"
        self.screen_stack.append(button_data.parent.manager.current)
        self.pending_action = UserInputs.NO_ROS_ACTION

        # Also need to send alert or message to topic to nurse station

        button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def nurse_alert_build(self):
        screen = Screen(name=cfg.NURSE_ALERT_SCREEN_NAME)

        # estop button
        screen.add_widget(
            Button(
                background_normal="images/stop.png",
                size_hint_x=cfg.ESTOP_XHINT,
                size_hint_y=cfg.ESTOP_YHINT,
                pos_hint={"center_x": cfg.ESTOP_XPOS, "center_y": cfg.ESTOP_YPOS},
                on_release=self.estop,
            )
        )

        # For MDlabel the label must be created before font size can be changed
        alert_label = MDLabel(
            text="A nurse has been alerted",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": cfg.SCREEN_Y_CENTER},
        )
        alert_label.font_size = "100sp"

        screen.add_widget(alert_label)
        screen.add_widget(
            MDRectangleFlatButton(
                text="Cancel Alert",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                size_hint=(cfg.LONG_RECT_WIDTH_CONFIRM, cfg.LONG_RECT_HEIGHT_CONFIRM),
                pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.2},
                on_release=self.nurse_alert_cancel,
            )
        )

        return screen
