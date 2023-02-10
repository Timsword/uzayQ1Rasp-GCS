from tkinter import *
from tkinter import ttk
import tkintermapview
import time
from pymavlink import mavutil
import serial
from dronekit import VehicleMode, LocationGlobalRelative, Command

# parameters
speed = 0
altitude = 0
latitude = 0
longitude = 0
coordList = []
uzunlukSabiti = 0.00001 # 1.1 metre


def retrieve_telemetry(iha):
    # Read the next message from the drone
    msg = iha.recv_msg()

    # Check if the message is a GLOBAL_POSITION_INT message
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        # Extract the speed, altitude, latitude, and longitude
        speed = msg.vx * 0.0194384
        altitude = msg.alt / 1000.0
        latitude = msg.lat * 1e-7
        longitude = msg.lon * 1e-7

        # Print the values
        #print(f"Speed: {speed} m/s, Altitude: {altitude} m, Latitude: {latitude}, Longitude: {longitude}")


def initializeGui(iha):

    def koordinataGit(coords):
        # pymavlink ile gitme emri verilecek
        #new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        # komut.clear()
        # komut.add(
        #     Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
        #             0, 0, 0, 0, coords[0], coords[1], 10))
        # komut.upload()
        # iha.mode = VehicleMode("AUTO")
        # Send a status text message with severity info
        # text = "commandGCS:deneme".encode('utf-8')
        # iha.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        #
        # # Wait a bit to give Pixhawk time to process the message
        # time.sleep(1)
        # while True:
        #     msg = iha.recv_msg()
        #     print(msg)
        print("2")

    def koordinatiListeyeEkle(coords):
        # # adres listeye eklenecek
        # new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        # coordList.append(coords)
        # komut.add(
        #     Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
        #             0, 0, 0, 0, coords[0], coords[1], 10))
        print("2")
    def alaniTara(coords):
        # # alan tarama kodunu çağır
        # new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        # # Send a status text message with severity info
        # text = f'commandGCS:alaniTara({coords[0]}, {coords[1]})'.encode('utf-8')
        # iha.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        print("2")

    def koordinatListesiniAktiflestir():
        # komut.upload()
        # iha.mode = VehicleMode("AUTO")
        # time.sleep(1)
        # komut.clear()
        print("2")

    #komut = iha.commands

    root = Tk()
    root.title('KTUUZAYQ1 Yer İstasyonu')
    #root.geometry("1200x900")
    root.state('zoomed')

    # komut.clear()
    # time.sleep(1)

    # harita
    map_widget = tkintermapview.TkinterMapView(root, width=350, height=200, corner_radius=0)
    map_widget.grid(row=0,column=0, padx=20, pady=20)

    # set current widget position and zoom
    map_widget.set_position(latitude, longitude)
    map_widget.set_zoom(15)

    position = map_widget.set_marker(latitude, longitude, text="KAYRA")

    map_widget.add_right_click_menu_command(label="Git",
                                          command=koordinataGit,
                                          pass_coords=True)
    map_widget.add_right_click_menu_command(label="Adresi komut listesine ekle",
                                          command=koordinatiListeyeEkle,
                                          pass_coords=True)
    map_widget.add_right_click_menu_command(label="Alanı tara",
                                          command=alaniTara,
                                          pass_coords=True)


    # # set current widget position by address
    # marker_1 = map_widget.set_address("colosseo, rome, italy", marker=True)
    #
    # marker_1.set_text("Colosseo in Rome")  # set new text
    # # marker_1.set_position(48.860381, 2.338594)  # change position
    # # marker_1.delete()

    # parametreler
    parameters = ttk.Notebook(root)
    parameters.grid(row=0,column=1)

    parameters_1 = Frame(parameters, width= 200, height=180)
    parameters_2 = Frame(parameters, width=200, height=180)

    parameters.add(parameters_1, text="Parametreler")
    parameters.add(parameters_2, text="Sekme")

    parametersTable = ttk.Treeview(parameters_1,columns=(1,2), selectmode="browse",height="8")
    parametersTable['show'] = 'headings'

    parametersTable.pack()
    parametersTable.heading(1, text="Parameter name")
    parametersTable.heading(2, text="Value")
    parametersTable.column(1, width=130)
    parametersTable.column(2, width=70,anchor=CENTER)


    parametersTable.insert('','end',values=("Altitude", altitude))
    parametersTable.insert('', 'end', values=("Speed", speed))
    parametersTable.insert('', 'end', values=("Enlem", latitude))
    parametersTable.insert('', 'end', values=("Boylam", longitude))

    # controller layout
    controller = ttk.Notebook(root)
    controller.grid(row=1, column=1)

    controller_1 = Frame(controller, width=200, height=300)
    controller_2 = Frame(controller, width=200, height=300)
    # tabs
    controller.add(controller_1, text="Genel kontrol")
    controller.add(controller_2, text="Tarama kontrol")

    #controller_1


    #tv.heading(3, text="Parameter")

    # altitude = Text(parameters_1, height = 10, width=50)
    # x=12
    # altitude.insert(INSERT, f'Yükseklik: {x}')
    # altitude.pack()

    #root.after(100, retrieve_telemetry(iha))
    root.mainloop()