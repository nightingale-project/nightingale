#!/usr/bin/env python3
from enum import Enum


class TaskCodes(Enum):
    # task codes for mission planner
    ERROR = -1
    SUCCESS = 0
    STOCK_ITEMS = 1
    DELIVER_ITEMS = 2
    DISMISS = 3
    WD_TIMEOUT = 9


class Task:
    # common code for tasks
    is_task = True
