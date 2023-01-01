from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import Button

from kivy.uix.screenmanager import NoTransition

class FaceScreen:
    def to_homescreen(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'screen1' # Should just be the "Home" screen

    def facescreen_estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estopped' # Should just be the "Home" screen

    def facescreen_build(self):
        screen = Screen(name='facescreen')

        screen.add_widget(
            Image(
                source='images/stop.png',
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.15,
                pos_hint={"center_x": 0.85, "center_y": 0.85}
            )
        )
        screen.add_widget(
            Button(
                background_normal="images/Face.png",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size=(100,100),
                on_release=self.to_homescreen
            )
        )


        return screen








