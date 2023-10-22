#!pip install customtkinter
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import tkinter
import tkinter.messagebox
import customtkinter
from .spinbox19 import FloatSpinbox 
import tkintermapview
import requests
import qutip as qt
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import subprocess
import codecs
from tkinter import filedialog,ttk
import pandas as pd
customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

 

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        # configure window
        self.title("Autosail Simulator Control")
        self.geometry(f"{1200}x{780}")
        self.world_name='sydney_reggatta'
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
        self.tabview.add("Control")
        self.tabview.add("Map")
        self.tabview.tab("Control").grid_columnconfigure(0, weight=0) # Genera la prioridad de visualizacion de la columna 0 como superior
        self.tabview.tab("Control").grid_columnconfigure((1,2,3), weight=1)  # Genera la prioridad de visualizacion de las columnas 1,2 y 3
        self.tabview.tab("Map").grid_columnconfigure(0, weight=0) # Genera la prioridad de visualizacion de la columna 0 como superior
        self.tabview.tab("Map").grid_columnconfigure((1,2), weight=1)  # Genera la prioridad de visualizacion de las columnas 1,2 y 3

        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------
        #                                                                                         TAB 1
        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------


        # Tab 1 Frames description
        self.Frame2_Main = customtkinter.CTkFrame(self.tabview.tab("Control"), fg_color="transparent")# Manual Control
        self.Frame2_Main.grid(row=0, column=0, columnspan=2, sticky="nsew")
        self.Frame2_Main.grid_columnconfigure((0,1,2),weight=1)
        self.Frame2_Main.grid_rowconfigure((0,1,2,3,4,5),weight=1)
        self.Frame4_Main = customtkinter.CTkFrame(self.tabview.tab("Control"), fg_color="transparent")# Waves Control
        self.Frame4_Main.grid(row=0, column=2, columnspan=2, sticky="nsew")
        self.Frame4_Main.grid_columnconfigure((0,1,2,3,4),weight=1)
        self.Frame4_Main.grid_rowconfigure((0,1,2,3,4),weight=1)
        self.Frame5_Main = customtkinter.CTkFrame(self.tabview.tab("Control"), fg_color="transparent")# Wind Control
        self.Frame5_Main.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.Frame5_Main.grid_columnconfigure((0,1,2),weight=1)
        self.Frame5_Main.grid_rowconfigure((0,1,2,3),weight=1)
        self.Frame3_Main = customtkinter.CTkFrame(self.tabview.tab("Control"), fg_color="transparent")# Vector Display
        self.Frame3_Main.grid(row=1, column=2, columnspan=3, sticky="nsew")
        self.Frame3_Main.grid_columnconfigure((0,1,2),weight=1)
        self.Frame3_Main.grid_rowconfigure((0,1,2),weight=1)

        # ---------------******---------------------
        # Tab1: Frame 2 - Manual Control
        # ---------------******---------------------
        self.Label_Tittle_F2 = customtkinter.CTkLabel(self.Frame2_Main, text="Manual Control", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.Label_Tittle_F2.grid(row=0, column=1, sticky="nsew")

        self.Label1_F2 = customtkinter.CTkLabel(self.Frame2_Main,text="Rudder:")
        self.Label1_F2.grid(row=1, column=1, sticky="nsew")
        self.Slider1_F2 = customtkinter.CTkSlider(self.Frame2_Main, from_= -90, to=90, number_of_steps=181, command=self.slider_event_Helm) 
        self.Slider1_F2.grid(row=1, column=2, sticky="nsew", padx=(10, 10), pady=(5, 5))

        self.Label2_F2 = customtkinter.CTkLabel(self.Frame2_Main,text="Main sail:")
        self.Label2_F2.grid(row=2, column=1, sticky="nsew")
        self.Slider2_F2 = customtkinter.CTkSlider(self.Frame2_Main, from_= -90, to=90, number_of_steps=181, command=self.slider_event_MainSail) 
        self.Slider2_F2.grid(row=2, column=2, sticky="nsew", padx=(10, 10), pady=(5, 5))

        self.Label3_F2 = customtkinter.CTkLabel(self.Frame2_Main,text="Secondary sail:")
        self.Label3_F2.grid(row=3, column=1, sticky="nsew")
        self.Slider3_F2 = customtkinter.CTkSlider(self.Frame2_Main, from_= -90, to=90, number_of_steps=181, command=self.slider_event_Gennaker) 
        self.Slider3_F2.grid(row=3, column=2, sticky="nsew", padx=(10, 10), pady=(5, 5))

        self.Button1_F4 = customtkinter.CTkButton(self.Frame2_Main, text="Send command",command=self.button_event_ManualControl)
        self.Button1_F4.grid(row=5, column=2, sticky="nsew", padx=(10, 10), pady=(5, 10))

        self.manual_enable_var = tkinter.BooleanVar(value=False)
        self.manual_enable_check = customtkinter.CTkCheckBox(self.Frame2_Main, text= "Enable manual control", variable=self.manual_enable_var, onvalue=True, offvalue=False)
        self.manual_enable_check.grid(row=5, column=0)
        # ---------------******---------------------
        # End - Manual Control
        # ---------------******---------------------
        # ---------------******---------------------
        # Tab1: Frame 4 - Wave Control
        # ---------------******---------------------
        self.Label_Tittle_F4 = customtkinter.CTkLabel(self.Frame4_Main, text="Wave Control", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.Label_Tittle_F4.grid(row=0, column=3, sticky="nsew")
        self.entry1_F4 = customtkinter.CTkEntry(self.Frame4_Main, placeholder_text="Magnitude")
        self.entry1_F4.grid(row=1, column=2, padx=(10, 10), pady=(20, 20), sticky="nsew")
        self.entry2_F4 = customtkinter.CTkEntry(self.Frame4_Main, placeholder_text="Frequency")
        self.entry2_F4.grid(row=1, column=3, padx=(10, 10), pady=(20, 20), sticky="nsew")
        self.entry3_F4 = customtkinter.CTkEntry(self.Frame4_Main, placeholder_text="Direction")
        self.entry3_F4.grid(row=1, column=4, padx=(10, 10), pady=(20, 20), sticky="nsew")
        self.Button1_F4 = customtkinter.CTkButton(self.Frame4_Main, text="Send parameters",command=self.button_event_WaveControl)
        self.Button1_F4.grid(row=4, column=3, sticky="nsew", padx=(10, 10), pady=(10, 10))

        # ---------------******---------------------
        # End - Wave Control
        # ---------------******---------------------
        # ---------------******---------------------
        # Tab1: Frame 5 - Wind Control
        # ---------------******---------------------
        # Direccion
        self.Label_Tittle_F3 = customtkinter.CTkLabel(self.Frame5_Main, text="Wind Control", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.Label_Tittle_F3.grid(row=0, column=1, sticky="nsew")
        self.Label_Wangle_F3 = customtkinter.CTkLabel(self.Frame5_Main, text="Wind angle:")
        self.Label_Wangle_F3.grid(row=1, column=0, sticky="nsew")
        self.Spinbox1_F3 = FloatSpinbox(self.Frame5_Main, width=150, step_size=1, from_=-90, to=90)
        self.Spinbox1_F3.grid(row=2, column=0, sticky="nsew", padx=(5, 5), pady=(0, 5))
        #Command
        self.Button1_F3 = customtkinter.CTkButton(self.Frame5_Main, text="Set Wind",command=self.button_event)
        self.Button1_F3.grid(row=3, column=1, sticky="nsew", padx=(10, 10), pady=(0, 5))
        # Magnitud
        self.Label_WMag_F3 = customtkinter.CTkLabel(self.Frame5_Main, text="Magnitude")
        self.Label_WMag_F3.grid(row=1, column=2, sticky="nsew")
        self.Spinbox2_F3 = FloatSpinbox(self.Frame5_Main, width=150, step_size=1, from_=0, to=60)
        self.Spinbox2_F3.grid(row=2, column=2, sticky="nsew", padx=(5, 5), pady=(0, 5))
        # ---------------******---------------------
        # End - Wind Control
        # ---------------******---------------------
         # ---------------******---------------------
        # Tab1: Frame 3 - Display
        # ---------------******--------------------
        self.Label_Help_F3 = customtkinter.CTkLabel(self.Frame3_Main, text="Position", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.Label_Help_F3.grid(row=0, column=0, columnspan=3,rowspan=3, sticky="nsew")
         # ---------------******---------------------
        # End - Wind Control
        # ---------------******---------------------

        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------
        #                                                                                      END TAB 2
        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------


        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------
        #                                                                                         TAB 2
        # -------------------------------------------------------------------------------------**********---------------------------------------------------------------------------------------

        # ---------------******---------------------
        # Tab 2 - Map
        # ---------------******---------------------
        # create map widget
        # ip, latitud, longitud = self.obtener_lat_lon()
        # print(f"La dirección IP del equipo es: {ip}")
        # print(f"latitud: {latitud}")
        # print(f"longitud: {longitud}")
        latitud = 6.260374289703808
        longitud = -75.57112045599285
        self.map_widget = tkintermapview.TkinterMapView(self.tabview.tab("Map"), width=720, height=640, corner_radius=2)
        # self.map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER, relheight=1, relwidth=1)
        self.map_widget.grid(row=0, column=0, sticky="nsew",rowspan=8,columnspan=2)
        self.map_widget.set_position(latitud, longitud, marker=False,text="Ubicacion")  
        self.map_widget.set_zoom(15)
        self.map_widget.add_right_click_menu_command(label="Add stage",  command=self.add_stage, pass_coords=True)
        self.trajectory_path = self.map_widget.set_path([(latitud, longitud),(6.57112045599285,-75.260374289703808)])
        self.position_marker=self.map_widget.set_marker(latitud, longitud, text="Sailboat")
        # ---------------******---------------------
        # Tab 2 - Trajectory
        # ---------------******---------------------
        # Create a trajectory table
        columnas = ('stage','coordinates', 'reached')
        self.trajectory = ttk.Treeview(self.tabview.tab("Map"),columns=columnas,show='headings')
        self.trajectory.heading('stage', text='Stages')
        self.trajectory.heading('coordinates', text='Coordinates')
        self.trajectory.heading('reached', text='State')
        self.trajectory.column('stage',minwidth=70,width=85,anchor='center')
        self.trajectory.column('coordinates', minwidth=150,width=300,anchor='center')
        self.trajectory.column('reached', minwidth=70,width=100,anchor='center')
        self.trajectory.grid(row=0,column=2)
        ladox=ttk.Scrollbar(self.tabview.tab("Map"),orient='horizontal',command=self.trajectory.xview)
        ladox.grid(row=1,column=2,sticky="ew")
        ladoy=ttk.Scrollbar(self.tabview.tab("Map"),orient='vertical',command=self.trajectory.yview)
        ladoy.grid(row=0,column=3,sticky="ns")
        self.trajectory.configure(xscrollcommand=ladox.set,yscrollcommand=ladoy.set)
        self.last_index=0
        self.trajectory.insert('',index=0,values=(0,'(6.57112045599285, -75.260374289703808)','not reached'))
        self.trajectory.bind("<Double-1>",self.delete_stage)

        #Trajectory points setting
        self.trajectory_buttons_frame = customtkinter.CTkFrame(self.tabview.tab("Map"), fg_color="transparent")
        self.trajectory_buttons_frame.grid(row=1, column=2, columnspan=2, sticky="nsew")
        self.trajectory_buttons_frame.grid_columnconfigure((0,1),weight=1)
        self.latitude_entry = customtkinter.CTkEntry(self.trajectory_buttons_frame, placeholder_text="latitude")
        self.latitude_entry.grid(row=0, column=0, padx=(5, 5), pady=(10, 10), sticky="ns")
        self.longitude_entry = customtkinter.CTkEntry(self.trajectory_buttons_frame, placeholder_text="longitud")
        self.longitude_entry.grid(row=1, column=0, padx=(5, 5), pady=(10, 10), sticky="ns")
        self.stage_entry = customtkinter.CTkEntry(self.trajectory_buttons_frame, placeholder_text="stage")
        self.stage_entry.grid(row=2, column=0, padx=(5, 5), pady=(10, 10), sticky="ns")


        self.map_button_1 = customtkinter.CTkButton(self.trajectory_buttons_frame,text="Add Point", command=self.add_stage)
        self.map_button_1.grid(row=3, column=0, padx=20, pady=10, sticky="ns")
        self.map_button_2 = customtkinter.CTkButton(self.trajectory_buttons_frame,text="Load Trajectory", command=self.Load_trajectory)
        self.map_button_2.grid(row=4, column=0, padx=20, pady=10, sticky="ns")
        self.map_button_3 = customtkinter.CTkButton(self.trajectory_buttons_frame,text="Save Trajectory", command=self.save_trajectory)
        self.map_button_3.grid(row=5, column=0, padx=20, pady=10, sticky="ns")
        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------
        #                                                                                      END TAB 2
        # -----------------------------------------------------------------------------------************---------------------------------------------------------------------------------------
    def Load_trajectory(self):
        """If the file selected is valid this will load the file into the Treeview"""
        file_path = filedialog.askopenfilename(initialdir = "~/", title = "Select a model", filetypes = (("Text files", "*.urdf*"), ("all files", "*.*")))
        try:
            excel_filename = r"{}".format(file_path)
            if excel_filename[-4:] == ".csv":
                df = pd.read_csv(excel_filename)
            else:
                df = pd.read_excel(excel_filename)
        except ValueError:
            tkinter.messagebox.showerror("Information", "The file you have chosen is invalid")
            return None
        except FileNotFoundError:
            tkinter.messagebox.showerror("Information", f"No such file as {file_path}")
            return None
        self.trajectory["column"] = list(df.columns)
        self.trajectory["show"] = "headings"
        for column in self.trajectory["columns"]:
            self.trajectory.heading(column, text=column) # let the column heading = column name

        df_rows = df.to_numpy().tolist() # turns the dataframe into a list of lists
        for row in df_rows:
            self.trajectory.insert("", "end", values=row) # inserts each list into the treeview. For parameters see https://docs.python.org/3/library/tkinter.ttk.html#tkinter.ttk.Treeview.insert
            stage_to_add=row[1]
            res = tuple(map(float, stage_to_add[1:-1].split(', ')))
            self.trajectory_path.add_position(res[0],res[1])
            # path_1.set_position_list(new_position_list)

        return None
    def delete_stage(self,event):
        x=tkinter.messagebox.askquestion('Eliminate Stage','Want to delete')
        if x=='yes':
            for item in self.trajectory.selection():
                stage_to_delete=self.trajectory.item(item)['values'][1]
                res = tuple(map(float, stage_to_delete[1:-1].split(', ')))
                self.trajectory_path.remove_position(res[0],res[1])
                self.trajectory.delete(item)
            self.update_trajectory(res)
    def save_trajectory(self):
        pass
    def update_trajectory(self,coords):
        print("Coordinates updated:", coords)
        print("Trajectory updated")
        self.map_widget.set_position(coords[0],coords[1])  # Paris, France
        self.map_widget.set_zoom(15)
        return None
    def add_stage(self,coords,s_index=None):
        if s_index==None:
            self.last_index+=1
            s_index=self.last_index
        try:
            self.trajectory_path.add_position(coords[0], coords[1])
            self.trajectory.insert('',index=tkinter.END,values=(s_index,str(coords),'Not reached'))
            self.update_trajectory(coords)
        except:
            tkinter.messagebox.showwarning(title='Stage Error',message='The new stage in f{coords} could not be established')
        # # set a path
        # # methods
        # path_1.remove_position(position)
        # path_1.delete()
    # def open_input_dialog_event(self):
    #     dialog = customtkinter.CTkInputDialog(text="Type in a number:", title="CTkInputDialog")
    #     print("CTkInputDialog:", dialog.get_input())
    
    # Appearance Functions
    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)
    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)
    def obtener_lat_lon(self):
        # Get your IP and coordinates related
        ip_response = requests.get('https://api.ipify.org?format=json')
        ip = ip_response.json()['ip']
        location_response = requests.get(f'http://ip-api.com/json/{ip}')
        location_data = location_response.json()
        latitud = location_data['lat']
        longitud = location_data['lon']
        return ip, latitud, longitud

    def gz_wind_action(self,x, y, z, Mag):
        # Normalize vector
        norm = np.sqrt(x**2 + y**2 + z**2)
        x /= norm
        y /= norm
        z /= norm
        wind_command=['gz','topic','-t',f'/world/{self.world_name}/wind','-m','gz.msgs.Wind','-p']
        wind_vectorstr= "'" +'linear_velocity:{'+f'x:{x*Mag},'+f'y:{y*Mag},'+f'z:{z*Mag}'+'}, enable_wind:true'+"'"
        wind_command.append(wind_vectorstr)
        print(wind_command)
        process = subprocess.Popen(wind_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)
    def wind_display(self,mag,boatx=None,boaty=None):
        # Obtener los valores de X y Y
        angle = int(self.Spinbox1_F3.get())*np.pi/180 #This meassure was in degrees
        x=-np.sin(angle)
        y=np.cos(angle)
        # Limpiar el frame antes de graficar nuevamente
        for widget in self.Frame3_Main.winfo_children():
            widget.destroy()

        # Crear la figura de Matplotlib
        fig = plt.Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot(111)
        fig.set_facecolor('lightgray')  # Cambiar el color de fondo de la figura
        # Agregar la flecha desde el origen hasta el punto (X, Y)
        ax.annotate('', xy=(x, y), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='red'))
        ax.plot(x, y, 'ro')
        if ((boatx!=None) and (boaty!=None)):
            ax.annotate('', xy=(boatx, boaty), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='blue'))
            ax.plot(boatx, boaty, 'bo')

        ax.set_xlim(-max(abs(x)+1, abs(y)+1), max(abs(x)+1, abs(y)+1))  # Ajustar los límites del eje x
        ax.set_ylim(-max(abs(x)+1, abs(y)+1), max(abs(x)+1, abs(y)+1))  # Ajustar los límites del eje y
        ax.set_xticks([])  # Ocultar las escalas del eje x
        ax.set_yticks([])  # Ocultar las escalas del eje y
        
        ax.text(0.5, 1.05, 'N', transform=ax.transAxes, ha='center', va='top',weight='bold', color='red')
        ax.text(0.5, -0.05, 'S', transform=ax.transAxes, ha='center', va='bottom',weight='bold', color='red')
        ax.text(1.04, 0.5, 'O', transform=ax.transAxes, ha='right', va='center',weight='bold', color='red') # Oriente - Oeste
        ax.text(-0.03, 0.5, 'E', transform=ax.transAxes, ha='left', va='center',weight='bold', color='red') # Occidente - Este

        LabelCanvas_FG = customtkinter.CTkLabel(self.Frame3_Main, text="Grafica:", font=customtkinter.CTkFont(size=20, weight="bold"))
        LabelCanvas_FG.grid(row=0, column=0, sticky="nsew")
        
        # Crear el lienzo de la figura y agregarlo al frame
        canvas = FigureCanvasTkAgg(fig, master=self.Frame3_Main)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        # Send GAZEBO command
        self.gz_wind_action(x,y,0,mag)
    ### Manual Control - Slider Events
    def slider_event_Helm(self,value):
        self.Label1_F2.configure(text=f"Rudder: {str(value)}")
        # print(f'Helm: {int(Helm)}')
        self.button_event_ManualControl()
    def slider_event_MainSail(self,value):
        self.Label2_F2.configure(text=f"Main sail: {str(value)}")
        # print(f'MainSail: {int(MainSail)}')
        self.button_event_ManualControl()
    def slider_event_Gennaker(self,value):
        self.Label3_F2.configure(text=f"Secondary sail: {str(value)}")
        # print(f'Gennaker: {int(Gennaker)}')
        self.button_event_ManualControl()
    def button_event_ManualControl(self):
        Helm = self.Slider1_F2.get()
        MainSail = self.Slider2_F2.get()
        Gennaker = self.Slider3_F2.get()
        # ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
        manual_rudder_command=['ros2','topic','pub','autosail/rudder_ctrl','std_msgs/msg/Double','"{'+f'data:{Helm*np.pi/180}'+'}"']
        print(' '.join(manual_rudder_command))
        # process = subprocess.Popen(wind_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # stderr = process.communicate()[1]
        # err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        # for line in err_output.splitlines():
        #     if line.find('undefined local') > 0:
        #         raise RuntimeError(line)
        # print(f'Rudder: {Helm}    Main sail: {MainSail} Secondary sail: {Gennaker}')

    ### Wind Control - Set Wind Direction
    def button_event(self):
        Mag = float(self.Spinbox2_F3.get())
        if (Mag==0):
            tkinter.messagebox.showwarning(title='Graph Error', message=' The wind vector could not be displayed with magnitude 0')
        else:
            self.wind_display(mag=Mag)

    ## Wave Control - Set Wave Direction
    def button_event_WaveControl(self):
        if self.entry1_F4.get() != "": 
            print(f'Magnitude = {self.entry1_F4.get()}')
        else:
            print("Magnitude = 0")

        if self.entry2_F4.get() != "": 
            print(f'Magnitude = {self.entry2_F4.get()}')
        else:
            print("Frequency = 0")

        if self.entry3_F4.get() != "": 
            print(f'Magnitude = {self.entry3_F4.get()}')
        else:
            print("Direction = 0")
    def update_from_ros(self,sensors):
        ''' This function brings a sensor dictionary to update all graphics in the GUI so the user know whats going on in the simulation, while the 
            manual mode is not enabled.
        '''
        if not(self.manual_enable_var.get()):
            # self.position_marker.set_position(sensors['latitude'],sensors['longitude']) #Update marker
            # self.wind_display(mag=1,boatx=np.cos(sensors['imuYaw']),boaty=np.sin(sensors['imuYaw'])) #Update graph
            # self.Label1_F2.configure(text=f"Rudder: {sensors['rudder']}")
            # self.Slider1_F2.set(int(sensors['rudder']))
            # self.Label2_F2.configure(text=f"Main sail: {sensors['main_sail']}")
            # self.Slider2_F2.set(int(sensors['main_sail']))
            # self.Label3_F2.configure(text=f"Secondary sail: {sensors['second_sail']}")
            # self.Slider3_F2.set(int(sensors['second_sail']))
            print("Si se hace la actualizacion cada segundo")
        else:
            pass