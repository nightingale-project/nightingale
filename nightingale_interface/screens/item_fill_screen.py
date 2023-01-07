from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class ItemFillScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN='itemfillscreen'
        button_data.parent.manager.current = "estoppedscreen"

    def fill_done(self, button_data):
        # publish to ROS topic to let robot go away
        cfg.LAST_SCREEN='itemfillscreen'
        cfg.PENDING_ACTION = cfg.DELIVER
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "facescreen"

    def fill_cancel(self, button_data):
        # cancel and publish topic to go to home
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN='itemfillscreen'
        cfg.PENDING_ACTION = NO_ROS_ACTION
        button_data.parent.manager.current = "confirmationscreen"
 
    def item_fill_build(self):
        screen = Screen(name="itemfillscreen")

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

        # Cancel and send request buttons
        screen.add_widget(
            MDRectangleFlatButton(
                text="Cancel",
                font_style="H4",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                size_hint=(0.4, 0.12),
                on_release=self.cancel_request,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="Done",
                font_style="H4",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                size_hint=(0.4, 0.12),
                on_release=self.send_request,
            )
        )

        return screen
