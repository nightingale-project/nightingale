from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition


class Screen2:
    def screen2_next(self, button_data):
        pass

    def screen2_back(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = 'right'
        button_data.parent.manager.current = 'screen1'

    def screen2_build(self):
        screen = Screen(name='screen2')
        # screen.add_widget(
        #     Image(
        #         source='images/background.jpg',
        #         allow_stretch=True,
        #         keep_ratio=False
        #     )
        # )
        screen.add_widget(
            MDLabel(
                text='Sample Text 2',
                font_style="H4",
                halign='center',
                pos_hint={"center_x": 0.5, "center_y": 0.6}
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Back',
                pos_hint={"center_x": 0.2, "center_y": 0.2},
                on_release=self.screen2_back
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Next',
                pos_hint={"center_x": 0.8, "center_y": 0.2},
                on_release=self.screen2_next
            )
        )
        return screen
