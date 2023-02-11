import gui
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative

def ihayaBaglan():
    # ihaya bağlan
    iha = mavutil.mavlink_connection('COM9', baud=57600)  # com port değiştirilebilir
    print("Bağlantı stringi oluşturuldu.")
    # cevap bekle
    iha.wait_heartbeat()
    print("İletişim sağlandı.")
    return iha
def ihayaBaglanWithDronekit():
    # Connect to Pixhawk using the default connection string
    iha = connect('COM9', baud=57600, wait_ready=True)
    print("Bağlantı sağlandı.")
    return iha

# ihayaBaglan()
# ihayaBaglanWithDronekit()
gui.initializeGui(ihayaBaglanWithDronekit())