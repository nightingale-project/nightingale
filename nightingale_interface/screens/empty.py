from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image


# default screen
def build(name_in='empty'):
    screen = Screen(name=name_in)
    # screen.add_widget(
    #     Image(
    #         source='images/background.jpg',
    #         allow_stretch=True,
    #         keep_ratio=False
    #     )
    # )
    return screen
