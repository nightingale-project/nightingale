from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.videoplayer import VideoPlayer

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition

class ExtendArmScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'

    def to_item_get_screen(self):
        # goes to next screen to tell user to take items
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'waititemgetscreen'

    def retract_arm(self, button_data):
        # publishes message to stop to retract arm
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'retractarmscreen'

    def extend_arm_build(self):
        screen = Screen(name='extendarmscreen')

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

        # retract arm
        screen.add_widget(
            MDRectangleFlatButton(
                text='Cancel',
                font_style="H4",
                pos_hint={"center_x": 0.85, "center_y": 0.15},
                size_hint=(0.4, 0.12),
                on_release=self.retract_arm
            )
        )

        # Video player of robot moving
        #screen.add_widget(
        #    VideoPlayer(
        #        source='extend_arm.mkv',
        #        state='play',
        #        size_hint_x=0.15,
        #        pos_hint={"center_x": 0.3, "center_y": 0.5},
        #)
         

        return screen






