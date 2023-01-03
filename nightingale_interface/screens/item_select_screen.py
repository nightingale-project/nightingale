from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg

from kivy.app import App



class ItemSelectScreen:
    # below are the idx for the labels in the children array
    water_count_label_idx = 2
    ice_count_label_idx = 1
    blanket_count_label_idx = 0

    # screen id from the screen manager
    item_select_screen_id = 9

    # screenname
    item_select_name = "itemselectscreen"

    def add_item(self, button_data):
        if button_data.id == "water" and self.water_count < 3:
            self.water_count += 1
        if button_data.id == "ice" and self.ice_count < 3:
            self.ice_count += 1
        if button_data.id == "blanket" and self.blanket_count < 3:
            self.blanket_count += 1

    def remove_item(self, button_data):
        if button_data.id == "water" and self.water_count > 0:
            self.water_count -= 1
        if button_data.id == "ice" and self.ice_count > 0:
            self.ice_count -= 1
        if button_data.id == "blanket" and self.blanket_count > 0:
            self.blanket_count -= 1

    # functions to update text on count change
    def on_water_count(self, *args):
        self.root.screens[self.item_select_screen_id].children[
            self.water_count_label_idx
        ].text = str(self.water_count)
        self.root.screens[self.item_fill_screen_id].children[
            self.water_count_label_idx
        ].text = str(self.water_count)

    def on_ice_count(self, *args):
        self.root.screens[self.item_select_screen_id].children[
            self.ice_count_label_idx
        ].text = str(self.ice_count)
        self.root.screens[self.item_fill_screen_id].children[
            self.ice_count_label_idx
        ].text = str(self.ice_count)

    def on_blanket_count(self, *args):
        self.root.screens[self.item_select_screen_id].children[
            self.blanket_count_label_idx
        ].text = str(self.blanket_count)
        self.root.screens[self.item_fill_screen_id].children[
            self.blanket_count_label_idx
        ].text = str(self.blanket_count)

    def send_request(self, button_data):
        # send to ROS topic and return to homescren
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "right"
        cfg.last_screen = button_data.parent.manager.current
        cfg.pending_action = cfg.STOCK
        button_data.parent.manager.current = "confirmationscreen"

    def cancel_request(self, button_data):
        # return to homescreen
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "right"
        cfg.last_screen = button_data.parent.manager.current
        cfg.pending_action = cfg.NO_ROS_ACTION
        button_data.parent.manager.current = "confirmationscreen"

    def item_select_build(self):
        screen = Screen(name=self.item_select_name)

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

        # Cancel and send request buttons
        screen.add_widget(
            MDRectangleFlatButton(
                text="Cancel",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.9},
                size_hint=(0.2, 0.1),
                on_release=self.cancel_request,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="Send",
                font_style="H4",
                pos_hint={"center_x": 0.9, "center_y": 0.1},
                size_hint=(0.2, 0.1),
                on_release=self.send_request,
            )
        )

        # water bottle
        screen.add_widget(
            Image(
                source="images/waterbottle.png",
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.05,
                pos_hint={"center_x": 0.2, "center_y": 0.7},
            )
        )

        # ice cup
        screen.add_widget(
            Image(
                source="images/icecube.png",
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.1,
                pos_hint={"center_x": 0.2, "center_y": 0.5},
            )
        )

        # blanket
        screen.add_widget(
            Image(
                source="images/blanket.png",
                allow_stretch=True,
                keep_ratio=True,
                size_hint_x=0.1,
                pos_hint={"center_x": 0.2, "center_y": 0.3},
            )
        )

        # Add text labels for items
        water_label = MDLabel(
            text="Water",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.7},
        )
        water_label.font_size = "70sp"

        ice_label = MDLabel(
            text="Ice",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.5},
        )
        ice_label.font_size = "70sp"

        blanket_label = MDLabel(
            text="Blanket",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.3},
        )
        blanket_label.font_size = "70sp"

        screen.add_widget(water_label)
        screen.add_widget(ice_label)
        screen.add_widget(blanket_label)

        # add and subtract buttons
        screen.add_widget(
            MDRectangleFlatButton(
                text="+",
                id="water",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.75},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.add_item,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="-",
                id="water",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.65},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.remove_item,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="+",
                id="ice",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.55},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.add_item,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="-",
                id="ice",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.45},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.remove_item,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="+",
                id="blanket",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.35},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.add_item,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="-",
                id="blanket",
                font_size=cfg.INCR_BUTTON_FONTSIZE,
                pos_hint={"center_x": cfg.INCR_BUTTON_XPOS, "center_y": 0.25},
                size_hint=(cfg.INCR_BUTTON_WIDTH, cfg.INCR_BUTTON_HEIGHT),
                on_release=self.remove_item,
            )
        )

        # counters for each item placed at end so indexes in children array are easy to find
        water_counter_label = MDLabel(
            text="0",
            id="water_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": 0.7},
        )
        water_counter_label.font_size = "70sp"

        screen.add_widget(water_counter_label)

        ice_counter_label = MDLabel(
            text="0",
            id="ice_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": cfg.SCREEN_Y_CENTER},
        )
        ice_counter_label.font_size = "70sp"

        screen.add_widget(ice_counter_label)

        blanket_counter_label = MDLabel(
            text="0",
            id="blanket_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": 0.3},
        )
        blanket_counter_label.font_size = "70sp"

        screen.add_widget(blanket_counter_label)

        return screen
