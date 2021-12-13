#!/usr/bin/env python3
import tkinter as tk
import evdev

window = tk.Tk()
#window.attributes('-fullscreen', True)
#window.title("Test")

width = window.winfo_screenwidth()
height = window.winfo_screenheight()
window.geometry("%dx%d" %(width, height))
window.title("Test")

label = tk.Label(window, text="Python rocks")
label.pack() 
window.mainloop()