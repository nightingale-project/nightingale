from kivy.uix.screenmanager import Screen

# from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivy.uix.button import Button
from kivy.uix.label import Label

from kivy.uix.screenmanager import NoTransition
from screens.screen_config import ScreenConfig as cfg


class FaceScreen:

    def to_homescreen(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.HUB_SCREEN_NAME 

    def face_build(self):
        screen = Screen(name=cfg.FACE_SCREEN_NAME)

        # remove press color animation
        screen.add_widget(
            Button(
                background_normal="images/Face.png",
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER,
                },
                size_hint=(0.7, 1),
                on_release=self.to_homescreen,
            )
        )

        return screen
