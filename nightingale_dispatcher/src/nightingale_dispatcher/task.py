#!/usr/bin/env python3
from enum import Enum


class Task(Enum):
    ERROR = -1
    SUCCESS = 0
    STOCK_ITEMS = 1
    DELIVER_ITEMS = 2
    DISMISS = 3
    WD_TIMEOUT = 9
