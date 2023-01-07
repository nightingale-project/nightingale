from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class ConfirmationScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = 'confirmationscreen'
        button_data.parent.manager.current = "estoppedscreen"

    def press(self, button_data):
        if button_data.id == 'yes':
            # do yes and return home
            button_data.parent.manager.transition = NoTransition()
            cfg.LAST_SCREEN = 'confirmationscreen'
            button_data.parent.manager.current = "homescreen"
        elif button_data.id == 'no':
            #do nothing and return to previous screen
            button_data.parent.manager.transition = NoTransition()
            button_data.parent.manager.current = cfg.LAST_SCREEN 

    def confirmation_build(self):
        screen = Screen(name="confirmationscreen")

        # estop button
        screen.add_widget(
            Button(
                background_normal="images/stop.png",
                size_hint_x=cfg.ESTOP_XHINT,
                size_hint_y=cfg.ESTOP_YHINT,
                pos_hint={"center_x": cfg.ESTOP_XPOS, "center_y": cfg.ESTOP_YPOS},
                on_release=self.estop,
            )
        )

        # add label asking to confirm 
        confirmation_label = MDLabel(
            text="Confirm selection",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.7},
        )
        confirmation_label.font_size = "100sp"
        screen.add_widget(confirmation_label)

        screen.add_widget(
            MDRectangleFlatButton(
                text="Yes",
                id="yes",
                font_style="H4",
                pos_hint={"center_x": 0.25, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.press
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="No",
                id="no",
                font_style="H4",
                pos_hint={"center_x": 0.75, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.press,
            )
        )


        return screen

