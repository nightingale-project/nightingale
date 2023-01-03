from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition


class NurseAlertScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'

    def nurse_alert_cancel(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = 'right'
        button_data.parent.manager.current = 'homescreen'

    def nurse_alert_build(self):
        screen = Screen(name='nursealertscreen')
        # screen.add_widget(
        #     Image(
        #         source='images/background.jpg',
        #         allow_stretch=True,
        #         keep_ratio=False
        #     )
        # )
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
        screen.add_widget(
            MDLabel(
                text='A nurse has been alerted',
                font_style="H4",
                halign='center',
                pos_hint={"center_x": 0.5, "center_y": 0.6}
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Cancel Alert',
                pos_hint={"center_x": 0.2, "center_y": 0.2},
                size_hint=(0.7, 0.12),
                on_release=self.nurse_alert_cancel
            )
        )

        return screen
