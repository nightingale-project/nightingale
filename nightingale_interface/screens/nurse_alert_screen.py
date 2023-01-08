from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition
from screens.screen_config import ScreenConfig as cfg


class NurseAlertScreen:
    nurse_alert_name = "nursealertscreen"

    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.nurse_alert_name
        button_data.parent.manager.current = "estoppedscreen"

    def nurse_alert_cancel(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "right"
        cfg.LAST_SCREEN = self.nurse_alert_name
        cfg.PENDING_ACTION = cfg.NO_ROS_ACTION
        button_data.parent.manager.current = "confirmationscreen"

    def nurse_alert_build(self):
        screen = Screen(name=self.nurse_alert_name)

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
            pos_hint={"center_x": 0.5, "center_y": 0.6},
        )
        alert_label.font_size = "100sp"

        screen.add_widget(alert_label)
        screen.add_widget(
            MDRectangleFlatButton(
                text="Cancel Alert",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.1},
                size_hint=(0.2, 0.1),
                on_release=self.nurse_alert_cancel,
            )
        )

        return screen
