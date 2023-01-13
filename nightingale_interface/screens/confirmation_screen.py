from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class ConfirmationScreen:
    confirmation_name = "confirmationscreen"

    def reset_counts(self):
        # only reset if starting new selection
        if cfg.last_screen == "itemfillscreen":
            self.water_count = 0
            self.ice_count = 0
            self.blanket_count = 0

    def reset_counts(self):
        # only reset if starting new selection
        if cfg.LAST_SCREEN == "itemfillscreen":
            self.water_count = 0
            self.ice_count = 0
            self.blanket_count = 0

    def call_ros_action(self, action: int, args: dict = {}) -> bool:
        """
        override expected with ros functionality

        :param action: Enum for which action to be called
        :param args: optional additional arguments
        :return: True if successful
        """
        print("call_ros_action override not implemented")
        return False

    def confirmation_press_yes(self, button_data):
        # do yes and return home
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.confirmation_name

        # state machine to do things based on the executed action
        # if given a 'yes' confirmation
        cfg.CURRENT_ACTION = cfg.PENDING_ACTION
        cfg.PENDING_ACTION = ""
        print(f"CUR ACTION {cfg.CURRENT_ACTION}")

        # reset counters regardless of cancel or send
        self.reset_counts()

        if (cfg.CURRENT_ACTION == cfg.NO_ROS_ACTION) or (cfg.CURRENT_ACTION == cfg.ESTOP_CANCEL):
            # cancel and wait for other inputs. No ROS funcs
            button_data.parent.manager.current = "homescreen"
            return True

        button_data.parent.manager.current = "facescreen"
        if self.call_ros_action(int(cfg.CURRENT_ACTION)):
            return True

        print("Unknown error occurred while sending ros action command")
        return False

    def confirmation_press_no(self, button_data):
        # do nothing and return to previous screen
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = cfg.LAST_SCREEN

    def confirmation_build(self):
        screen = Screen(name=self.confirmation_name)

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
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.7},
        )
        confirmation_label.font_size = "100sp"
        screen.add_widget(confirmation_label)

        screen.add_widget(
            MDRectangleFlatButton(
                text="Yes",
                id="yes",
                font_style="H4",
                pos_hint={"center_x": 0.25, "center_y": cfg.SCREEN_Y_CENTER},
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.press,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="No",
                id="no",
                font_style="H4",
                pos_hint={"center_x": 0.75, "center_y": cfg.SCREEN_Y_CENTER},
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.press,
            )
        )

        return screen
