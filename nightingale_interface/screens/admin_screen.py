from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition


class AdminScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'

    def exit_home(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'homescreen'

    def shutdown(self, button_data):
        # shutdown system, might not be useful since HW button exists
        pass

    def release_bin(self, button_data):
        # publish message to ros topic to release
        pass

    def grasp_bin(self, button_data):
        # publish message to ros topic to grip
        pass

    def admin_build(self):
        screen = Screen(name='adminscreen')

        screen.add_widget(
            MDRectangleFlatButton(
                text='Exit',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.exit_home
            )
        )

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

        # functionality
        screen.add_widget(
            MDRectangleFlatButton(
                text='System Shutdown',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.35},
                size_hint=(0.4, 0.12),
                on_release=self.shutdown
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Release bin',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.release_bin
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Grasp bin',
                font_style="H4",
                pos_hint={"center_x": 0.5, "center_y": 0.65},
                size_hint=(0.4, 0.12),
                on_release=self.grasp_bin
            )
        )


        return screen


