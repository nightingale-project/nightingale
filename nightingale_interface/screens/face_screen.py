from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivy.uix.button import Button
from kivy.uix.label import Label

from kivy.uix.screenmanager import NoTransition
from screens.screen_config import ScreenConfig as cfg


class FaceScreen:
    def to_homescreen(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = (
            "homescreen"  # Should just be the "Home" screen
        )

    def face_build(self):
        screen = Screen(name="facescreen")

        current_action_label = Label(
            text="CURRENT_ACTION",
            # font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.9},
        )
        current_action_label.font_size = "50sp"

        screen.add_widget(current_action_label)

        # remove press color animation
        screen.add_widget(
            Button(
                background_normal="images/Face.png",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size_hint=(0.5, 0.5),
                on_release=self.to_homescreen,
            )
        )

        return screen
