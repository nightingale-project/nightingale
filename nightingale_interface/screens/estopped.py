from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton, Button

from kivy.uix.screenmanager import NoTransition

class EStoppedScreen:
    def back(self, button_data):
        #returns to previous screen that came before estop pressed
        button_data.parent.manager.transition = NoTransition()
        # Insert functionality to hold the last screen name
        button_data.parent.manager.current = "LAST_SCREEN"  
        

    def force_stop(self, button_data):
        #kill robot in emergency if pressed
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "SHUTDOWN"  

    def estopped_build(self):
        screen = Screen(name='facescreen')

        # ADD IMAGE FOR STOPPED STATE
        screen.add_widget(
            Button(
                background_normal="images/stop.png",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size=(100,100),
                on_release=self.to_homescreen
            )
        )
        screen.add_widget(
            MDRectangelFlatButton(
                background_normal="images/stop.png",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size=(100,100),
                on_release=self.to_homescreen
            )
        )


        return screen








 




