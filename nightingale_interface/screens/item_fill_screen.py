from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition


class ItemFillScreen:
     def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'

    def fill_done(self, button_data):
        # publish to ROS topic to let robot go away 
        pass

    def fill_cancel(self, button_data):
        # cancel and publish topic to go to home
        pass

     def item_fill_build(self):
        screen = Screen(name='itemfillscreen')


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


        # Cancel and send request buttons
        screen.add_widget(
            MDRectangleFlatButton(
                text='Cancel',
                font_style="H4",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                size_hint=(0.4, 0.12),
                on_release=self.cancel_request
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='Done',
                font_style="H4",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                size_hint=(0.4, 0.12),
                on_release=self.send_request
            )
        )
 
        return screen
