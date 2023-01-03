from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.label import Label

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import NoTransition

class EStoppedScreen:
    def exit_estop(self, button_data):
        #returns to previous screen that came before estop pressed
        button_data.parent.manager.transition = NoTransition()
        # Insert functionality to hold the last screen name
        # for simplicity now just go back to face screen and restart interaction
        button_data.parent.manager.current = "facescreen"  
        

    def force_stop(self, button_data):
        #kill robot in emergency if pressed
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "SHUTDOWN"  

    def estopped_build(self):
        screen = Screen(name='estoppedscreen')

        # ADD IMAGE FOR STOPPED STATE
        screen.add_widget(
            Label(
                text="STOPPED",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size=(30,30),
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Restart',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size_hint=(0.5, 0.1),
                on_release=self.exit_estop
            )
        )



        return screen








 




