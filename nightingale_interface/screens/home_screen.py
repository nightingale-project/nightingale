from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class HomeScreen:
    home_name = "homescreen"

    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.last_screen = button_data.parent.manager.current
        button_data.parent.manager.current = "estoppedscreen"

    def to_nurse_alert(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        cfg.last_screen = button_data.parent.manager.current
        button_data.parent.manager.current = "nursealertscreen"
        #cfg.LAST_SCREEN = self.screen_name

    def to_video_call(self, button_data):
        # publish to topic to tell robot not to move
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        cfg.last_screen = button_data.parent.manager.current
        button_data.parent.manager.current = "videocallscreen"
        # open web browser to specific size of scree

    def to_item_select(self, button_data):
        # publish to topic to tell robot not to move
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        cfg.last_screen = button_data.parent.manager.current
        button_data.parent.manager.current = "itemselectscreen"

    def dismiss(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.transition.direction = "left"
        cfg.last_screen = button_data.parent.manager.current
        cfg.pending_action = cfg.GO_HOME
        button_data.parent.manager.current = "confirmationscreen"

    def to_admin_control(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.last_screen = button_data.parent.manager.current
        button_data.parent.manager.current = "adminscreen"

    def home_build(self):
        screen = Screen(name=self.home_name)

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
                text="Request Items",
                font_style="H4",
                # pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.35},
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER,
                },
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.to_item_select,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="Dismiss",
                font_style="H4",
                pos_hint={"center_x": 0.125, "center_y": 0.1},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.dismiss,
            )
        )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Admin Control",
        #        font_size=cfg.CANCEL_BUTTON_FONTSIZE,
        #        pos_hint={"center_x": 0.125, "center_y": 0.9},
        #        size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
        #        on_release=self.to_admin_control,
        #    )
        # )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Chat with Nurse",
        #        font_style="H4",
        #        pos_hint={
        #            "center_x": cfg.SCREEN_X_CENTER,
        #            "center_y": cfg.SCREEN_Y_CENTER,
        #        },
        #        size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
        #        on_release=self.to_video_call,
        #    )
        # )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Urgent Assistance",
        #        font_style="H4",
        #        pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.65},
        #        size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
        #        on_release=self.to_nurse_alert,
        #    )
        # )
        return screen
