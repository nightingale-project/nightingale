from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition
from screens.screen_config import ScreenConfig as cfg


class NurseAlertScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "estoppedscreen"

    def nurse_alert_cancel(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "right"
        button_data.parent.manager.current = "homescreen"

    def nurse_alert_build(self):
        screen = Screen(name="nursealertscreen")
        # screen.add_widget(
        #     Image(
        #         source='images/background.jpg',
        #         allow_stretch=True,
        #         keep_ratio=False
        #     )
        # )
        screen.add_widget(
            Image(
                source="images/stop.png",
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.15,
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
                font_size=cfg.CANCEL_BUTTON_FONT_SIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.1},
                size_hint=(0.2, 0.1),
                on_release=self.nurse_alert_cancel,
            )
        )

        return screen
