from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.uix.screenmanager import SlideTransition, NoTransition

from kivymd.uix.button import MDRectangleFlatButton
from screens.screen_config import ScreenConfig as cfg


class VideoCallScreen:
    def exit_call(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.HUB_SCREEN_NAME

    def video_call_build(self):
        screen = Screen(name=cfg.VIDEO_CALL_SCREEN_NAME)

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

        # exit call screen
        screen.add_widget(
            MDRectangleFlatButton(
                text="Exit",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.9},
                size_hint=(0.2, 0.1),
                on_release=self.exit_call,
            )
        )

        return screen
