from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition

from screens.screen_config import ScreenConfig as cfg


class AdminScreen:
    admin_name = 'adminscreen'

    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.admin_name 
        button_data.parent.manager.current = "estoppedscreen"

    def to_home(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.admin_name
        button_data.parent.manager.current = "homescreen"

    def shutdown(self, button_data):
        # shutdown system, might not be useful since HW button exists
        pass

    def release_bin(self, button_data):
        # publish message to ros topic to release
        pass

    def grasp_bin(self, button_data):
        # publish message to ros topic to grip
        pass

    def admin_build(self):
        screen = Screen(name=self.admin_name)

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

        screen.add_widget(
            MDRectangleFlatButton(
                text="Exit",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.9},
                size_hint=(0.2, 0.1),
                on_release=self.to_home,
            )
        )

        # functionality
        screen.add_widget(
            MDRectangleFlatButton(
                text="System Shutdown",
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.35},
                size_hint=(0.4, 0.12),
                on_release=self.shutdown,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="Release bin",
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.release_bin,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="Grasp bin",
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.65},
                size_hint=(0.4, 0.12),
                on_release=self.grasp_bin,
            )
        )

        return screen
