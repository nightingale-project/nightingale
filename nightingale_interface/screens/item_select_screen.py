from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from kivy.uix.screenmanager import SlideTransition, NoTransition

class ItemSelectScreen:
    # way to have a single item to add or subtract? Use button data?
    def add_item1(self, button_data):
        # water bottle
        pass

    def add_item2(self, button_data):
        # ice cup
        pass

    def add_item3(self, button_data):
        # blankets
        pass

    def send_request(self, button_data):
        # send to ROS topic and return to homescren
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = 'left'
        button_data.parent.manager.current = 'homescreen'


    def cancel_request(self, button_data):
        # return to homescreen
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = 'left'
        button_data.parent.manager.current = 'homescreen'


    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = 'estoppedscreen'


    def item_select_build(self):
        screen = Screen(name='itemselectscreen')

        # water bottle
        screen.add_widget(
            Image(
                source='images/stop.png',
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.15,
                pos_hint={"center_x": 0.15, "center_y": 0.15},
                on_release=self.estop
            )
        )

        # ice cup
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

        # blanket
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

        # add and subtract buttons
        screen.add_widget(
            MDRectangleFlatButton(
                text='+',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='-',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='+',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='-',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='+',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text='-',
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                on_release=self.home_next
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
                text='Send',
                font_style="H4",
                pos_hint={"center_x": 0.15, "center_y": 0.85},
                size_hint=(0.4, 0.12),
                on_release=self.send_request
            )
        )
 
        return screen




