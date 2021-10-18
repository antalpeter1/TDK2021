#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import main

import time
from itertools import count
from multiprocessing import Process
import timeout_decorator

from tkinter import *
from PIL import ImageTk, Image
from tkinter import messagebox

# root = Tk()
# root.title('Some sort of title')
# root.iconbitmap()

# response = messagebox.showinfo("This is my Popup!", "Hello World!")

max_time_allowed = 0.1
@timeout_decorator.timeout(max_time_allowed, use_signals=True)
def run_main():
    res = main.main()
    return res

max_trials = 2
trial_count = 0
while trial_count < max_trials:
    try:
        res = run_main()
        print('Simulation was an utter success!')
        print('Costs:')
        print(res)
        trial_count = max_trials
        input("Press Enter to continue...")
        
    except:
        print('Ooops, I have time-outed :/')
        trial_count += 1
        
        break
        
        @timeout_decorator.timeout(max_time_allowed, use_signals=True)
        def run_main():
            res = main.main()
            return res
        
    
