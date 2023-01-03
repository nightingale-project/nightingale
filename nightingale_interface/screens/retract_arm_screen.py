from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.videoplayer import VideoPlayer

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition

class RetractArmScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'

    def to_home(self):
        # goes to next screen to tell user to take items
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'home'

    def retract_arm_build(self):
        screen = Screen(name='retractarmscreen')

        # estop button
        screen.add_widget(
            Image(
                source='images/stop.png',
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.15,
                pos_hint={"center_x": 0.85, "center_y": 0.85},
                on_release=self.estop
            )
        )

        # Video player of robot moving
        #screen.add_widget(
        #    VideoPlayer(
        #        source='retract_arm.mkv',
        #        state='play',
        #        size_hint_x=0.15,
        #        pos_hint={"center_x": 0.3, "center_y": 0.5},
        #)
         

        return screen






