from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition


class Screen1:
    def screen1_next(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = 'left'
        button_data.parent.manager.current = 'screen2'


    def screen1_back(self, button_data):
        pass

    def screen1_build(self):
        screen = Screen(name='screen1')

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
            MDRectangleFlatButton(
                text='Admin Control',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.screen1_next
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text='Request Items',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.35},
                size_hint=(0.4, 0.12),
                on_release=self.screen1_back
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Chat with Nurse',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.screen1_next
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text='Urgent Assistance',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.65},
                size_hint=(0.4, 0.12),
                on_release=self.screen1_next
            )
        )


        return screen
