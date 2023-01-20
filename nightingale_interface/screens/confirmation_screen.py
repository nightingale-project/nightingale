from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button
from kivy.uix.popup import Popup

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import UserInputs


class ConfirmationScreen:

    def reset_counts(self):
        # only reset if starting new selection
        if self.screen_stack.pop() == cfg.ITEM_FILL_SCREEN_NAME:
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
        self.reset_wd()

        # state machine to do things based on the executed action
        # if given a 'yes' confirmation
        self.current_action = self.pending_action
        self.pending_action = ""

        # reset counters regardless of cancel or send
        self.reset_counts()

        if self.current_action == UserInputs.NO_ROS_ACTION:
            # cancel and wait for other inputs. No ROS funcs
            button_data.parent.manager.current = cfg.HUB_SCREEN_NAME
            return True

        if self.current_action == UserInputs.ESTOP_CANCEL:
            # stop ros estop 
            self.call_ros_action(int(self.current_action))
            # temporarily have homescreen as the screen after estop pressed. likely have to impement queue
            screen_before_estop = cfg.HUB_SCREEN_NAME
            while self.screen_stack[-1] in [cfg.ESTOP_SCREEN_NAME, cfg.CONFIRMATION_SCREEN_NAME]:
                self.screen_stack.pop()
            button_data.parent.manager.current = self.screen_stack.pop()
            return True
        
        if self.current_action == UserInputs.START_RETRACT_ARM:
            self.call_ros_action(int(self.current_action))
            # should already be on the screen
            #button_data.parent.manager.current = cfg.RETRACT_ARM_SCREEN_NAME 
            return True
 

        button_data.parent.manager.current = cfg.FACE_SCREEN_NAME 
        if self.call_ros_action(int(cfg.current_action)):
            return True

        print("Unknown error occurred while sending ros action command")
        return False

    def confirmation_press_no(self, button_data):
        # do nothing and return to previous screen
        button_data.parent.manager.transition = NoTransition()
        self.reset_wd()
        button_data.parent.manager.current = self.screen_stack.pop() 


    def confirmation_build(self):
        screen = Screen(name=cfg.CONFIRMATION_SCREEN_NAME)

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
                on_release=self.confirmation_press_yes,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="No",
                id="no",
                font_style="H4",
                pos_hint={"center_x": 0.75, "center_y": cfg.SCREEN_Y_CENTER},
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.confirmation_press_no,
            )
        )

        return screen
