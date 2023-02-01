from kivy.uix.screenmanager import Screen
from kivy.uix.screenmanager import SlideTransition, NoTransition
from kivy.uix.button import Button

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton

from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class ItemFillScreen:
    def fill_done(self, button_data):
        # publish to ROS topic to let robot go away
        self.screen_stack.append(button_data.parent.manager.current)
        self.pending_action = UserInputs.DELIVER_ITEMS
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def fill_cancel(self, button_data):
        # cancel and publish topic to go to home
        button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        self.pending_action = UserInputs.DISMISS
        button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def item_fill_build(self):
        screen = Screen(name=cfg.ITEM_FILL_SCREEN_NAME)

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
                font_style="H4",
                pos_hint={"center_x": 0.25, "center_y": 0.15},
                size_hint=(0.4, 0.12),
                on_release=self.fill_cancel,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="Done",
                font_style="H4",
                pos_hint={"center_x": 0.75, "center_y": 0.15},
                size_hint=(0.4, 0.12),
                on_release=self.fill_done,
            )
        )

        # Add text labels for items
        water_label = MDLabel(
            text="Water",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.7},
        )
        water_label.font_size = "70sp"

        ice_label = MDLabel(
            text="Ice",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.5},
        )
        ice_label.font_size = "70sp"

        blanket_label = MDLabel(
            text="Blanket",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.5, "center_y": 0.3},
        )
        blanket_label.font_size = "70sp"

        screen.add_widget(water_label)
        screen.add_widget(ice_label)
        screen.add_widget(blanket_label)

        # counters for each item placed at end so indexes in children array are easy to find
        water_counter_label_fill = MDLabel(
            text=str(self.water_count),
            id="water_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": 0.7},
        )
        water_counter_label_fill.font_size = "70sp"

        screen.add_widget(water_counter_label_fill)

        ice_counter_label_fill = MDLabel(
            text=str(self.ice_count),
            id="ice_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": 0.5},
        )
        ice_counter_label_fill.font_size = "70sp"

        screen.add_widget(ice_counter_label_fill)

        blanket_counter_label_fill = MDLabel(
            text=str(self.blanket_count),
            id="blanket_count",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": 0.9, "center_y": 0.3},
        )
        blanket_counter_label_fill.font_size = "70sp"

        screen.add_widget(blanket_counter_label_fill)

        return screen
