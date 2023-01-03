from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivy.uix.button import Button

from kivy.uix.screenmanager import NoTransition

class FaceScreen:
    def to_homescreen(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'homescreen' # Should just be the "Home" screen

    def face_build(self):
        screen = Screen(name='facescreen')

        screen.add_widget(
            MDLabel(
                text='CURRENT_ACTION',
                font_style="H4",
                halign='center',
                pos_hint={"center_x": 0.4, "center_y": 0.85}
            )
        )
        screen.add_widget(
            Button(
                background_normal="images/Face.png",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size=(50,50),
                on_release=self.to_homescreen
            )
        )


        return screen








