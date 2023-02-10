import gui
from pymavlink import mavutil

def ihayaBaglan():
    # ihaya bağlan
    iha = mavutil.mavlink_connection('COM9', baud=57600)  # com port değiştirilebilir
    print(2)
    # cevap bekle
    iha.wait_heartbeat()
    print(3)
    return iha
#ihayaBaglan()
gui.initializeGui(2)