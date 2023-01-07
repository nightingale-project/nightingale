from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.label import Label

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import NoTransition
from screens.screen_config import ScreenConfig as cfg


class EStoppedScreen:
    screen_name = 'estoppedscreen'

    def exit_estop(self, button_data):
        # returns to previous screen that came before estop pressed
        button_data.parent.manager.transition = NoTransition()
        #cfg.LAST_SCREEN = self.screen_name 
        cfg.LAST_SCREEN = 'estoppedscreen' 
        button_data.parent.manager.current = 'confirmationscreen' 

    def force_stop(self, button_data):
        # kill robot in emergency if pressed
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "SHUTDOWN"
        cfg.LAST_SCREEN = self.screen_name 

    def estopped_build(self):
        screen = Screen(name='estoppedscreen')

        # ADD IMAGE FOR STOPPED STATE
        stopped_label = MDLabel(
            text="EMERGENCY STOP ACTIVATED",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.5},
        )
        stopped_label.font_size = "100sp"

        screen.add_widget(stopped_label)
        screen.add_widget(
            MDRectangleFlatButton(
                text="Restart",
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.2},
                size_hint=(0.5, 0.1),
                on_release=self.exit_estop,
            )
        )

        return screen
