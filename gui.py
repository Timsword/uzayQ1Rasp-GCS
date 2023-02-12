from tkinter import *
from tkinter import ttk
import tkintermapview
import time
from pymavlink import mavutil
import serial
from dronekit import VehicleMode, LocationGlobalRelative, Command
import math
from PIL import Image, ImageTk

# parameters
global speed
global altitude
global latitude
global longitude
coordList = []
uzunlukSabiti = 0.00001  # 1.1 metre

yerHizi = 1
yawRate = 15


# Print the values
# print(f"Speed: {speed} m/s, Altitude: {altitude} m, Latitude: {latitude}, Longitude: {longitude}")
def sendStatusTextWithDroneKit(iha, command):
    textToSend = "commandGCS:" + command
    #iha.message_factory.statustext_send(severity=1, text=textToSend)
    iha.message_factory.statustext_send(severity=1, text=(textToSend).encode('utf-8'))
    iha.flush()

# def receiveStatusText():
#     from dronekit import connect, VehicleMode, LocationGlobalRelative
#     import time
#
#     # Connect to Pixhawk using the default connection string
#     vehicle = connect('/dev/serial0', baud=921600, wait_ready=True)
#
#     # Listen for `STATUSTEXT` messages
#     @vehicle.on_message('STATUSTEXT')
#     def handle_statustext_message(self, name, message):
#         text = message.text
#         if text.startswith("commandGCS:"):
#             # Process the message
#             print("Received command:", text)
#             # You can add your own logic here to handle the received message
#
#     # Wait for messages
#     while True:
#         time.sleep(1)

def initializeGui(iha):
    def retrieve_telemetry(iha):
        # ####### mavlink ##########
        # msg = iha.recv_msg()
        #
        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     speed = msg.vx * 0.0194384
        #     altitude = msg.alt / 1000.0
        #     latitude = msg.lat * 1e-7
        #     longitude = msg.lon * 1e-7

        if iha.armed:
            armButton.config(image=on)
            is_on = True
        elif not iha.armed:
            armButton.config(image=off)
            is_on = False
        else:
            is_on = False


        speed = iha.groundspeed
        altitude = iha.location.global_relative_frame.alt
        latitude = iha.location.global_relative_frame.lat
        longitude = iha.location.global_relative_frame.lon
        #print(f"Speed: {speed} m/s, Altitude: {altitude} m, Latitude: {latitude}, Longitude: {longitude}")
        parametersTable.set("Altitude", 2, altitude)
        parametersTable.set("Speed", 2, speed)
        parametersTable.set("Enlem", 2, latitude)
        parametersTable.set("Boylam", 2, longitude)

        map_widget.set_position(latitude, longitude)
        map_widget.set_zoom(15)
        #position.delete()
        position = map_widget.set_marker(latitude, longitude, text="KAYRA")

        root.after(500, lambda: retrieve_telemetry(iha))

    def koordinataGit(coords):
        # pymavlink ile gitme emri verilecek
        new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        komut.clear()
        komut.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, 0, 0, 0, coords[0], coords[1], 10))
        komut.upload()
        iha.mode = VehicleMode("AUTO")

    def koordinatiListeyeEkle(coords):
        # adres listeye eklenecek
        new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        coordList.append(coords)
        komut.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, 0, 0, 0, coords[0], coords[1], 10))

    def alaniTara(coords):
        # alan tarama kodunu çağır
        new_marker = map_widget.set_marker(coords[0], coords[1], text="new marker")
        # Send a status text message with severity info
        text = f'commandGCS:alanTaraKare({coords[0]}, {coords[1]})'.encode('utf-8')
        # iha.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

    def koordinatListesiniAktiflestir():
        komut.upload()
        iha.mode = VehicleMode("AUTO")
        time.sleep(1)
        komut.clear()

    def hizBelirle(iha, vx, vy, vz, yaw):
        msg = iha.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # -- BITMASK -> Consider only the velocities
            0, 0, 0,  # -- POSITION
            vx, vy, vz,  # -- VELOCITY
            0, 0, 0,  # -- ACCELERATIONS
            0, math.radians(yaw))
        iha.send_mavlink(msg)
        iha.flush()

    def koordinataGitController(lat_, lon_, alt_):
        coords = LocationGlobalRelative(lat_, lon_, alt_)
        iha.mode = VehicleMode("GUIDED")
        time.sleep(1)
        iha.simple_goto(coords)

    def key(event):
        if event.char == event.keysym:  # -- standard keys
            if event.keysym == 'r':
                print("r pressed >> Set the vehicle to RTL")
                iha.mode = VehicleMode("RTL")
            elif event.keysym == 'w':  # ileri
                hizBelirle(iha, yerHizi, 0, 0, 0)
            elif event.keysym == 's':  # geri
                hizBelirle(iha, -yerHizi, 0, 0, 0)
            elif event.keysym == 'a':  # sola
                hizBelirle(iha, 0, -yerHizi, 0, 0)
            elif event.keysym == 'd':  # sağa
                hizBelirle(iha, 0, yerHizi, 0, 0)
            elif event.keysym == 'q':  # sola dön
                hizBelirle(iha, 0, 0, 0, -yawRate)
            elif event.keysym == 'e':  # sağa dön
                hizBelirle(iha, 0, 0, 0, yawRate)
        else:  # -- non standard keys
            if event.keysym == 'Up':  # yüksel
                hizBelirle(iha, 0, 0, yerHizi, 0)
            elif event.keysym == 'Down':  # alçal
                hizBelirle(iha, 0, 0, -yerHizi, 0)

    def armSwitch():
        if iha.armed:
            iha.armed = False
            while iha.armed:
                i = 0
                if i == 10:
                    informationTable.insert('', 'end', values=("armSwitch", "10 saniyenin ardından arm kapatılamadı.", "Sorun"))
                    break
                time.sleep(1)

            # armButton.config(image=off)
            # is_on = False
        elif not iha.armed:
            iha.mode = VehicleMode("GUIDED")
            iha.armed = True
            while not iha.armed:
                i = 0
                if i == 10:
                    informationTable.insert('', 'end', values=("armSwitch", "10 saniyenin ardından arm yapılamadı.", "Sorun"))
                    break
                time.sleep(1)
            # armButton.config(image=on)
            # is_on = True
        else:
            # is_on = False
            print("else")

    def kalkisaGec():
        if iha.armed:
            iha.simple_takeoff(float(takeOffAltitude.get()))
            while True:
                # Break and return from function just below target altitude.
                if iha.location.global_relative_frame.alt >= float(takeOffAltitude.get()) * 0.90:
                    break
                time.sleep(1)
        else:
            informationTable.insert('', 'end', values=("kalkisaGec", "İha arm değil. Kalkışa geçilemedi.", "Sorun"))
            return

    def durdur():
        sendStatusTextWithDroneKit(iha, "BRAKE")
        time.sleep(1)

    def inisYap():
        sendStatusTextWithDroneKit(iha, "LAND")
        time.sleep(1)

    def eveDon():
        sendStatusTextWithDroneKit(iha, "RTL")
        time.sleep(1)

    def alanTara(uzunluk, latitude, longitude):
        sendStatusTextWithDroneKit(iha, "alanTaraKare(," + str(uzunluk) + ",," + str(latitude) + ",," + str(longitude) + ",)")

    root = Tk()
    root.title('KTUUZAYQ1 Yer İstasyonu')
    #  root.geometry("1200x900")
    root.state('zoomed')
    #  icon = PhotoImage(file="assets/icon.png")
    root.iconbitmap(r'assets/favicon.ico')
    #  root.iconphoto(False,icon)

    komut = iha.commands

    # genel değişkenler
    on = Image.open("assets/on.png")
    on = on.resize((50, 30), Image.ANTIALIAS)
    on = ImageTk.PhotoImage(on)

    off = Image.open("assets/off.png")
    off = off.resize((50, 30), Image.ANTIALIAS)
    off = ImageTk.PhotoImage(off)

    wImg = Image.open("assets/keyboard/white/w.png")
    wImg = wImg.resize((70, 70), Image.ANTIALIAS)
    wImg = ImageTk.PhotoImage(wImg)
    aImg = Image.open("assets/keyboard/white/a.png")
    aImg = aImg.resize((70, 70), Image.ANTIALIAS)
    aImg = ImageTk.PhotoImage(aImg)
    sImg = Image.open("assets/keyboard/white/s.png")
    sImg = sImg.resize((70, 70), Image.ANTIALIAS)
    sImg = ImageTk.PhotoImage(sImg)
    dImg = Image.open("assets/keyboard/white/d.png")
    dImg = dImg.resize((70, 70), Image.ANTIALIAS)
    dImg = ImageTk.PhotoImage(dImg)
    qImg = Image.open("assets/keyboard/white/q.png")
    qImg = qImg.resize((70, 70), Image.ANTIALIAS)
    qImg = ImageTk.PhotoImage(qImg)
    eImg = Image.open("assets/keyboard/white/e.png")
    eImg = eImg.resize((70, 70), Image.ANTIALIAS)
    eImg = ImageTk.PhotoImage(eImg)
    upImg = Image.open("assets/keyboard/white/up.png")
    upImg = upImg.resize((70, 70), Image.ANTIALIAS)
    upImg = ImageTk.PhotoImage(upImg)
    downImg = Image.open("assets/keyboard/white/down.png")
    downImg = downImg.resize((70, 70), Image.ANTIALIAS)
    downImg = ImageTk.PhotoImage(downImg)

    is_on = iha.armed

    altitude = 0.0
    latitude = 0.0
    longitude = 0.0

    ################################# HARİTA #############################
    map_widget = tkintermapview.TkinterMapView(root, width=350, height=200, corner_radius=0)
    map_widget.grid(row=0, column=0, padx=20, pady=20)

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


    ################################# PARAMETRELER ##############################
    parameters = ttk.Notebook(root)
    parameters.grid(row=0, column=1)

    parameters_1 = Frame(parameters, width=200, height=180)
    parameters_2 = Frame(parameters, width=200, height=180)

    parameters.add(parameters_1, text="Parametreler")
    parameters.add(parameters_2, text="Sekme")

    parametersTable = ttk.Treeview(parameters_1, columns=(1, 2), selectmode="browse", height="8")
    parametersTable['show'] = 'headings'

    parametersTable.pack()
    parametersTable.heading(1, text="Parameter name")
    parametersTable.heading(2, text="Value")
    parametersTable.column(1, width=130)
    parametersTable.column(2, width=70, anchor=CENTER)

    parametersTable.insert('', 'end', iid="Altitude", values=("İrtifa", 0))
    parametersTable.insert('', 'end', iid="Speed", values=("Hız", 0))
    parametersTable.insert('', 'end', iid="Enlem", values=("Enlem", 0))
    parametersTable.insert('', 'end', iid="Boylam", values=("Boylam", 0))
    ############################## KONTROL ####################################
    controller = ttk.Notebook(root)
    controller.grid(row=1, column=1)

    controller_1 = Frame(controller, width=200, height=300)
    controller_2 = Frame(controller, width=200, height=300)
    # tabs
    controller.add(controller_1, text="Genel kontrol")
    controller.add(controller_2, text="Tarama kontrol")

    ### genel kontroller
    armButton = Button(controller_1, image=off, bd=0, command=armSwitch)
    armButton.grid(row=0, column=0, padx=5, pady=5)

    takeOffAltitude = (Entry(controller_1, width=5))
    takeOffAltitude.grid(row=0, column=1, padx=5, pady=5)
    takeOffButton = Button(controller_1, height=1, width=10, text="Kalkışa geç", command=kalkisaGec)
    takeOffButton.grid(row=0, column=2, padx=5, pady=5)

    eveDonButton = Button(controller_1, height=1, width=30, text="Eve dön", command=eveDon)
    eveDonButton.grid(row=1, column=0, padx=5, pady=5, columnspan=3)

    durdurButton = Button(controller_1, height=1, width=30, text="Durdur", command=durdur)
    durdurButton.grid(row=1, column=0, padx=5, pady=5, columnspan=3)

    inisYapButton = Button(controller_1, height=1, width=30, text="İniş yap", command=inisYap())
    inisYapButton.grid(row=1, column=0, padx=5, pady=5, columnspan=3)

    latitude_entry = Entry(controller_1, width=15)
    latitude_entry.insert(0, latitude)
    latitude_entry.grid(row=2, column=0, padx=5, pady=5)
    longitude_entry = Entry(controller_1, width=15)
    longitude_entry.insert(0, longitude)
    longitude_entry.grid(row=2, column=1, padx=5, pady=5)
    altitude_entry = Entry(controller_1, width=15)
    altitude_entry.insert(0, altitude)
    altitude_entry.grid(row=2, column=2, padx=5, pady=5)
    goToButton = Button(controller_1, width=30, text="Koordinata git",
                        command=lambda: koordinataGitController(float(latitude_entry.get()), float(longitude_entry.get()), float(altitude_entry.get())))
    goToButton.grid(row=3, column=0, padx=5, pady=5, columnspan=3)

    ### alan tarama kontrolleri
    latitude_entry2 = Entry(controller_2, width=15)
    latitude_entry2.insert(0, latitude)
    latitude_entry2.grid(row=0, column=0, padx=5, pady=5)
    longitude_entry2 = Entry(controller_2, width=15)
    longitude_entry2.insert(0, longitude)
    longitude_entry2.grid(row=0, column=1, padx=5, pady=5)
    uzunluk_entry = Entry(controller_2, width=15)
    uzunluk_entry.insert(0, "Uzunluk")
    uzunluk_entry.grid(row=0, column=2, padx=5, pady=5)
    alanTaraButton = Button(controller_2, width=30, text="Alanı tara",
                        command=lambda: alanTara(float(latitude_entry.get()), float(longitude_entry.get()), float(uzunluk_entry.get())))
    alanTaraButton.grid(row=1, column=0, padx=5, pady=5, columnspan=3)

    ####################### klavye kontrol ##########################3
    keyboardControl = Frame(root)
    keyboardControl.grid(row=1, column=0, padx=5, pady=5)
    wButton = Button(keyboardControl, image=wImg, bd=0)
    wButton.grid(row=0, column=1, padx=5, pady=5)
    aButton = Button(keyboardControl, image=aImg, bd=0)
    aButton.grid(row=1, column=0, padx=5, pady=5)
    sButton = Button(keyboardControl, image=sImg, bd=0)
    sButton.grid(row=1, column=1, padx=5, pady=5)
    dButton = Button(keyboardControl, image=dImg, bd=0)
    dButton.grid(row=1, column=2, padx=5, pady=5)
    qButton = Button(keyboardControl, image=qImg, bd=0)
    qButton.grid(row=0, column=0, padx=5, pady=5)
    eButton = Button(keyboardControl, image=eImg, bd=0)
    eButton.grid(row=0, column=2, padx=5, pady=5)
    upButton = Button(keyboardControl, image=upImg, bd=0)
    upButton.grid(row=0, column=3, padx=5, pady=5)
    downButton = Button(keyboardControl, image=downImg, bd=0)
    downButton.grid(row=1, column=3, padx=5, pady=5)

    ################################# LOGO ###############################
    logoFrame = Frame(root, width=200, height=200)
    logoFrame.grid(row=0, column=3)

    logoImg = ImageTk.PhotoImage(Image.open("assets/logo.jpg"))
    logoLabel = Label(logoFrame, image=logoImg)
    logoLabel.grid(row=0, grid=0)

    ################################# INFORMATION ###############################
    information = ttk.Notebook(root)
    information.grid(row=0, column=3)#, rowspan=2

    information_1 = Frame(information, width=330, height=600)
    information.add(information_1, text="Bilgi")

    informationTable = ttk.Treeview(information_1, columns=(1, 2,3), selectmode="browse", height="8")
    informationTable['show'] = 'headings'

    informationTable.pack()
    informationTable.heading(1, text="Kaynak")
    informationTable.heading(2, text="Mesaj")
    informationTable.heading(3, text="Tür")
    informationTable.column(1, width=50)
    informationTable.column(2, width=200)
    informationTable.column(3, width=50)

    informationTable.insert('', 'end', values=("Root", "Bağlantı sağlandı", "Bilgi"))


    # tv.heading(3, text="Parameter")

    # altitude = Text(parameters_1, height = 10, width=50)
    # x=12
    # altitude.insert(INSERT, f'Yükseklik: {x}')
    # altitude.pack()

    root.bind_all('<Key>', key)
    # root.after(100, retrieve_telemetry(iha))
    root.after(100, lambda: retrieve_telemetry(iha))
    root.mainloop()
