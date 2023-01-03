from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition


class VideoCallScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "estoppedscreen"

    def exit_call(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "homescreen"

    def video_call_build(self):
        screen = Screen(name="videocallscreen")

        # estop button
        screen.add_widget(
            Image(
                source="images/stop.png",
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.15,
                pos_hint={"center_x": 0.85, "center_y": 0.85},
                on_release=self.estop,
            )
        )

        # exit call screen
        screen.add_widget(
            MDRectangleFlatButton(
                text="Exit",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.exit_call,
            )
        )

        return screen
