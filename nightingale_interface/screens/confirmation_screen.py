from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class ConfirmationScreen:
    confirmation_name = "confirmationscreen"

    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.confirmation_name
        button_data.parent.manager.current = "estoppedscreen"

    def reset_counts(self):
        # only reset if starting new selection
        if cfg.LAST_SCREEN == "itemfillscreen":
            self.water_count = 0
            self.ice_count = 0
            self.blanket_count = 0

    def press(self, button_data):
        if button_data.id == "yes":
            # do yes and return home
            button_data.parent.manager.transition = NoTransition()
            cfg.LAST_SCREEN = self.confirmation_name

            def execute_action():
                # state machine to do things based on the executed action
                # if given a 'yes' confirmation
                cfg.CURRENT_ACTION = cfg.PENDING_ACTION
                cfg.PENDING_ACTION = ""
                print(f"CUR ACTION {cfg.CURRENT_ACTION}")

                # reset counters regardless of cancel or send
                self.reset_counts()

                if cfg.CURRENT_ACTION == cfg.NO_ROS_ACTION:
                    # cancel and wait for other inputs. No ROS funcs

                    button_data.parent.manager.current = "homescreen"
                elif cfg.CURRENT_ACTION == cfg.ESTOP_CANCEL:
                    # estop cancel
                    # send ROS message to resume
                    button_data.parent.manager.current = "homescreen"
                elif cfg.CURRENT_ACTION == cfg.STOCK:
                    # get items
                    # send ros message to move to stock room
                    button_data.parent.manager.current = "facescreen"
                elif cfg.CURRENT_ACTION == cfg.DELIVER:
                    # deliver items
                    # send ros message to move to patient
                    button_data.parent.manager.current = "facescreen"
                elif cfg.CURRENT_ACTION == cfg.GO_HOME:
                    # deliver items
                    # send ros message to move to patient
                    button_data.parent.manager.current = "facescreen"
                elif cfg.CURRENT_ACTION == cfg.EXTEND_ARM:
                    # start arm movement with ROs and go back to screen
                    button_data.parent.manager.current = "extendarmscreen"
                elif cfg.CURRENT_ACTION == cfg.RETRACT_ARM:
                    # start arm movement with ROs and go to retract arm screen
                    button_data.parent.manager.current = "retractarmscreen"

            execute_action()

        elif button_data.id == "no":
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
            pos_hint={"center_x": 0.5, "center_y": 0.7},
        )
        confirmation_label.font_size = "100sp"
        screen.add_widget(confirmation_label)

        screen.add_widget(
            MDRectangleFlatButton(
                text="Yes",
                id="yes",
                font_style="H4",
                pos_hint={"center_x": 0.25, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.press,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="No",
                id="no",
                font_style="H4",
                pos_hint={"center_x": 0.75, "center_y": 0.5},
                size_hint=(0.4, 0.12),
                on_release=self.press,
            )
        )

        return screen
