#!pip install customtkinter
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import tkinter
import tkinter.messagebox
import customtkinter 
import numpy as np
import subprocess
import codecs
from tkinter import filedialog,ttk
customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

command=['cd','~/autosail_ws']

class App(customtkinter.CTk,Node):
    def __init__(self):
        super().__init__()
        # configure window
        self.title("Autosail Simulator Wizard")
        self.geometry(f"{1200}x{780}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2, 3), weight=1)

        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Autosail", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"], command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"], command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))
        self.scaling_optionemenu.set('100%')
        # create main tabview
        self.tabview = customtkinter.CTkTabview(self)
        self.tabview.grid(row=0, column=1,rowspan=3, padx=(20, 0), pady=(20, 0), sticky="nsew",columnspan=3)
        self.tabview.add("Simulation")
        self.tabview.tab("Simulation").grid_columnconfigure(0, weight=0) # Genera la prioridad de visualizacion de la columna 0 como superior
        self.tabview.tab("Simulation").grid_columnconfigure((1,2,3), weight=1)  # Genera la prioridad de visualizacion de las columnas 1,2 y 3
        self.tabview.tab("Simulation").grid_rowconfigure((0,1,2,3),weight=1)

        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------
        #                                                                                         TAB 1
        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------
        # Tab 1 Frames description

        self.sim_info_frame = customtkinter.CTkFrame(self.tabview.tab("Simulation"), corner_radius=0)
        self.sim_info_frame.grid(row=0, column=0, sticky="nsew",columnspan=4)
        self.sim_info_frame.grid_columnconfigure((0,1,2),weight=1)
        self.sim_info_frame.grid_rowconfigure((0,1,2,3),weight=1)
        self.custom_info_frame =customtkinter.CTkFrame(self.tabview.tab("Simulation"),corner_radius=0)
        self.custom_info_frame.grid(row= 1, column=0, padx=20, pady=10,sticky="nsew",columnspan=4)
        self.custom_info_frame.grid_columnconfigure((0,1,2),weight=1)
        self.custom_info_frame.grid_rowconfigure((0,1,2),weight=1)
        self.main_command_frame =customtkinter.CTkFrame(self.tabview.tab("Simulation"), corner_radius=0)
        self.main_command_frame.grid(row= 2, column=0, padx=20, pady=10,sticky="nsew",columnspan=4)
        self.main_command_frame.grid_columnconfigure((0,1,2,3),weight=1)
        self.main_command_frame.grid_rowconfigure((0,1,2,3),weight=1)
        # ---------------******---------------------
        # Tab1: Frame 1 - Basic Sim Info
        # ---------------******---------------------
        self.world_name_select_label = customtkinter.CTkLabel(self.sim_info_frame, text="World name: ")
        self.world_name_select_label.grid(row=0, column=0)
        self.world_name_select_entry = customtkinter.CTkComboBox(self.sim_info_frame,values=["None", "sydney_regatta","fortress","test_world", "test_robot_wolrd","nbpark"])
        self.world_name_select_entry.grid(row=0, column=1,columnspan=2)
        self.robot_name_select_label = customtkinter.CTkLabel(self.sim_info_frame, text="Robot name: ")
        self.robot_name_select_label.grid(row=1, column=0)
        self.robot_name_select_entry = customtkinter.CTkComboBox(self.sim_info_frame,values=["None","sailboat", "wamv", "storm"])
        self.robot_name_select_entry.grid(row=1, column=1,columnspan=2)

        self.sim_type_label = customtkinter.CTkLabel(self.sim_info_frame, text="Simulation type: ")
        self.sim_type_label.grid(row=2, column=0)
        self.sim_type_entry = customtkinter.CTkComboBox(self.sim_info_frame,values=["full", "sim", "bridge"])
        self.sim_type_entry.grid(row=2, column=1)

        self.headless_var = tkinter.BooleanVar(value=False)
        self.headless_check = customtkinter.CTkCheckBox(self.sim_info_frame, text= "Headless simulation: ", variable=self.headless_var, onvalue=True, offvalue=False)
        self.headless_check.grid(row=3, column=0)
        for widget in self.sim_info_frame.winfo_children():
            widget.grid_configure(padx=10, pady=5)
        # ---------------******---------------------
        # End - Basic Sim Info
        # ---------------******---------------------
        # ---------------******---------------------
        # Tab1: Frame 2 - Custom Info
        # ---------------******---------------------
        self.custom_enable_var = tkinter.BooleanVar(value=False)
        self.custom_enable_check = customtkinter.CTkCheckBox(self.custom_info_frame, text= "Use custom files? ", variable=self.custom_enable_var, onvalue=True, offvalue=False)
        self.custom_enable_check.grid(row=0, column=0)
        self.custom_config_model_label = customtkinter.CTkLabel(self.custom_info_frame, text="Configuration file: ")
        self.custom_config_model_label.grid(row=1, column=0)
        self.custom_config_model_entry = customtkinter.CTkEntry(self.custom_info_frame)
        self.custom_config_model_entry.grid(row=1, column=1)
        self.config_browse_button = customtkinter.CTkButton(self.custom_info_frame, text="...", command= self.browseCONFIG)
        self.config_browse_button.grid(row=1, column=2, sticky="news", padx=20, pady=10)
        self.custom_URDF_label = customtkinter.CTkLabel(self.custom_info_frame, text="URDF: ")
        self.custom_URDF_label.grid(row=2, column=0)
        self.custom_URDF_entry = customtkinter.CTkEntry(self.custom_info_frame)
        self.custom_URDF_entry.grid(row=2, column=1)
        self.URDF_browse_button = customtkinter.CTkButton(self.custom_info_frame, text="...", command= self.browseURDF)
        self.URDF_browse_button.grid(row=2, column=2, sticky="news", padx=20, pady=10)

        for widget in self.custom_info_frame.winfo_children():
            widget.grid_configure(padx=10, pady=5)
        # ---------------******---------------------
        # End - Custom Info
        # ---------------******---------------------
        # ---------------******---------------------
        # Tab1: Frame 3 - Controller
        # ---------------******---------------------
        self.controller_label=customtkinter.CTkLabel(self.main_command_frame,text='Controller select')
        self.controller_label.grid(row=0,column=1,sticky="news",padx=20,pady=10)
        self.controller_name=customtkinter.CTkLabel(self.main_command_frame,text='Node: ')
        self.controller_name.grid(row=1,column=0,sticky="news",padx=20,pady=10)
        self.controller_name_entry= customtkinter.CTkEntry(self.main_command_frame)
        self.controller_name_entry.grid(row=1,column=1)
        self.package_name=customtkinter.CTkLabel(self.main_command_frame,text='Package: ')
        self.package_name.grid(row=2,column=0,sticky="news",padx=20,pady=10)
        self.package_name_entry= customtkinter.CTkEntry(self.main_command_frame)
        self.package_name_entry.grid(row=2,column=1)
        # create radiobutton frame
        self.radiobutton_frame = customtkinter.CTkFrame(self.main_command_frame)
        self.radiobutton_frame.grid(row=1, column=3, padx=(20, 20), pady=(20, 0), sticky="nsew")
        self.radio_var = tkinter.StringVar(value='Ros')
        self.label_radio_group = customtkinter.CTkLabel(master=self.radiobutton_frame, text="Sensor Source:")
        self.label_radio_group.grid(row=0, column=2, columnspan=1, padx=10, pady=10, sticky="")
        self.radio_button_1 = customtkinter.CTkRadioButton(master=self.radiobutton_frame,text='Gazebo', variable=self.radio_var, value='Gazebo')
        self.radio_button_1.grid(row=1, column=2, pady=10, padx=20, sticky="n")
        self.radio_button_2 = customtkinter.CTkRadioButton(master=self.radiobutton_frame,text='Ros', variable=self.radio_var, value='Ros')
        self.radio_button_2.grid(row=2, column=2, pady=10, padx=20, sticky="n")
        self.radio_button_3 = customtkinter.CTkRadioButton(master=self.radiobutton_frame,text='uController', variable=self.radio_var, value='uController')
        self.radio_button_3.grid(row=4, column=2, pady=10, padx=20, sticky="n")
        self.radio_button_4 = customtkinter.CTkRadioButton(master=self.radiobutton_frame,text='Hardware in the loop', variable=self.radio_var, value='HITL')
        self.radio_button_4.grid(row=4, column=2, pady=10, padx=20, sticky="n")
       
        # ---------------******---------------------
        # End - Controller
        # ---------------******---------------------
        self.button = customtkinter.CTkButton(self.tabview.tab('Simulation'), text="Start simulation!", command= self.enter_data)
        self.button.grid(row=3, column=2, sticky="news", padx=20, pady=10)
        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------
        #                                                                                      END TAB 3
        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------
    
    # Appearance Functions
    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)
    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

    ### Simulation Configuration Functions
    def enter_data(self):
        # User info
        self.world_name = self.world_name_select_entry.get()
        self.robot_name = self.robot_name_select_entry.get()
        global command
        command =['ros2','launch','autosail_gz','spawn.launch.py']
        if ((self.world_name!="None" and self.robot_name!="None") and (self.world_name!="" and self.robot_name!="")):
            self.sim_mode = self.sim_type_entry.get()
            self.headless = str(self.headless_var.get())
            self.robot_urdf = self.custom_URDF_entry.get()
            self.config_file = self.custom_config_model_entry.get()
            self.controller=self.controller_name_entry.get()
            self.controller_package=self.package_name_entry.get()
            command.append(f'world:={self.world_name}')
            command.append(f'robot:={self.robot_name}')
            if self.sim_mode!='':
                command.append(f'sim_mode:={self.sim_mode}')
            command.append(f'headless:={self.headless}')
            if self.controller!='':
                command.append(f'controller:={self.controller}')
            if self.controller_package!='':    
                command.append(f'controller_pkg:={self.controller_package}')
            custom_EN=self.custom_enable_var.get()
            if custom_EN:
                if self.config_file!='':
                    command.append(f'config_file:={self.config_file}')
                if self.robot_urdf!='':
                    command.append(f'urdf:={self.robot_urdf}')
            self.sensor_s=str(self.radio_var.get())
            command.append(f'sensor_source:={self.sensor_s}')
            self.quit()
        else:
            tkinter.messagebox.showwarning(title="Error", message="World name and robot name are required")
    def browseURDF(self):
        filename = filedialog.askopenfilename(initialdir = "~/", title = "Select a URDF model", filetypes = (("Text files", "*.urdf*"), ("all files", "*.*")))
        if filename!='':
            self.custom_URDF_entry.delete(0,tkinter.END)
            self.custom_URDF_entry.insert(0,str(filename))
    def browseCONFIG(self):
        filename = filedialog.askopenfilename(initialdir = "~/", title = "Select configuration a file", filetypes = (("Text files", "*.txt*"), ("all files", "*.*")))
        if filename!='':
            self.custom_config_model_entry.delete(0,tkinter.END)
            self.custom_config_model_entry.insert(0,str(filename))
def main(args=None):
    app = App()
    app.mainloop()
    # time.sleep(5)
    app.destroy()
    print('Launching process: ', ' '.join(command))
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stderr = process.communicate()[1]
    err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
    for line in err_output.splitlines():
        if line.find('undefined local') > 0:
            raise RuntimeError(line)
# If you want it to run from the terminal
if __name__ == '__main__':
    main()