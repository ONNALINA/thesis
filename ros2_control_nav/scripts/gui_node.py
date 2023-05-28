#!/usr/bin/python3
import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String,Bool

import tkinter as tk
import customtkinter

class GUI_NODE(Node):
    def __init__(self,master=None):
        super().__init__('gui_node')
        self.root = tk.Tk()
        self.root.geometry("440x720")

        # create a frame to hold the checkboxes
        self.checkbox_frame = tk.Frame(self.root)
        self.checkbox_frame.place(relx=0.5, rely=0.48, anchor=tk.CENTER)

        self.checkbox_vars = []  # create a list to hold the variables for the checkboxes
        self.checkbox_names = []  # create a list to hold the names of the checkboxes
       
        for i in range(15):
            var = tk.BooleanVar()
            self.checkbox_vars.append(var)
            self.checkbox_names.append(f"box {i+1}")
            checkbox = tk.Checkbutton(self.checkbox_frame, text=f"box {i+1}", variable=var)
            checkbox.pack(side=tk.TOP, pady=5)

        # create a label for the button section
        self.label = tk.Label(self.root, text="Please select 1-3 boxes")
        self.label.place(relx=0.5, rely=0.08, anchor=tk.CENTER)
        # Use CTkButton instead of tkinter Button
        self.button = customtkinter.CTkButton(master=self.root, text="Submit", command=self.submit_callback)
        self.button.place(relx=0.5, rely=0.9, anchor=tk.CENTER)
        self.pub = self.create_publisher(String, '/order_list', 10)

    def submit_callback(self):
        msg = String()
        selected_checkboxes = []  # create an empty list to hold the selected checkboxes
        
        for i, var in enumerate(self.checkbox_vars):
            if var.get() == True:
                selected_checkboxes.append(i+1)  # add the index (1-based) of the selected checkbox to the list

        if len(selected_checkboxes) < 1:
            self.label.config(text="Please select at least one box")
            print("Please select at least one box")

        elif len(selected_checkboxes) > 3:
            self.label.config(text="Please select no more than three boxes")
            print("Please select no more than three boxes")

        else:
            self.label.config(text=f"Selected boxes: {selected_checkboxes}")
            msg.data = str(selected_checkboxes)
            self.pub.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data) 

    def run(self):
        self.root.mainloop()      

def main(args=None):
    rclpy.init(args=args)
    gui_node = GUI_NODE()
    gui_node.run()
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()