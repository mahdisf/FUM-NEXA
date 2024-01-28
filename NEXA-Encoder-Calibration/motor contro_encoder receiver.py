import canopen
import time
import csv
from math import sin
import can


def motor():

    Pos_vel_file = open("/home/pi/Desktop/Motor trajectory/motordata_col.csv", 'w')
    Pos_vel_filew = csv.writer(Pos_vel_file)
    Pos_vel_filew.writerow(['time','actualPosition', 'actualvel'])
    print("save started")

    # Start with creating a network representing one CAN bus
    network = canopen.Network()

    # Add some nodes with corresponding Object Dictionaries
    node1 = canopen.RemoteNode(0x1b, '/home/pi/Desktop/Motor trajectory/ASDA_A2_1042sub980_C.eds')
    network.add_node(node1)

    network.connect(bustype='socketcan', channel='can1')

    def init_drive(node, network):
        node.nmt.state = 'PRE-OPERATIONAL'

        node.sdo['Modes of operation'].raw = 1

        node.sdo['Controlword'].raw = 128  # Clear_Error
        node.sdo['Controlword'].raw = 0  # Switch_ON
        node.sdo['Controlword'].raw = 6
        node.sdo['Controlword'].raw = 7  # Init & Run
        node.sdo['Controlword'].raw = 15
        # node.sdo['Controlword'].raw=63#
        # node.sdo['Modes of operation'].raw=1
        # Read PDO configuration from node
        node.rpdo.read()
        node.tpdo.read()

        # Re-map TPDO[1]
        node.tpdo[1].clear()
        # node.tpdo[1].add_variable('Torque actual value')
        node.tpdo[1].add_variable('Velocity actual value')
        node.tpdo[1].add_variable('Position actual value')
        node.tpdo[1].enabled = True
        # Save new PDO configuration to node
        node.tpdo[1].save()

        node.rpdo[1].clear()
        node.rpdo[1].add_variable('Target Position')
        # node.rpdo[1].add_variable('Target velocity')
        node.rpdo[1].add_variable('Profile velocity')

        node.rpdo[1].enabled = True

        # node.rpdo[2].enabled = True

        node.rpdo.save()

        node.rpdo[1].start(0.005)
        # node.rpdo[2].start(0.005)

    init_drive(node1, network)

    print("all nodes are initiated")

    node1.nmt.state = 'OPERATIONAL'

    node1.rpdo[1]['Profile velocity'].raw = 100000
    node1.rpdo[1]['Target Position'].raw = 0
    node1.sdo['Controlword'].raw = 63
    time.sleep(2)
    t = 0
    try:
        while t < 50 :
            a =(-1)**t
            if a > 0:
                position = -1*720 * (1/(360 / (1280000))) #sin(t/50)
            else:
                position = 720 * (1/(360 / (1280000)))
            node1.rpdo[1]['Target Position'].raw = position
            ACtualp = node1.tpdo[1]["Position actual value"].raw
            p = ACtualp * (360 / (1280000))
            node1.sdo['Controlword'].raw = 15
            node1.sdo['Controlword'].raw = 63
            # node1.sdo['Controlword'].raw = 31
            node1.rpdo[1]['Profile velocity'].raw = 510000 + t * 51000
            v = node1.tpdo[1]['Velocity actual value'].raw
            # vel.append(v/10)
            motortime = time.time()
            Pos_vel_filew.writerow([motortime,p, v/100])
            
            print("t ", t, v/100)
            if ACtualp < int(position) + 2 and ACtualp > int(position) - 2:
                t = t + 1
            
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("STOP!")
        raise

    finally:
        node1.rpdo[1]['Profile velocity'].raw = 0
        node1.sdo['Controlword'].raw = 0
        network.disconnect()


def encoder():
    t = 0
    filencode = open('encoderdata_final.csv', 'w')
    filencodew = csv.writer(filencode)

    # can setup
    bus = can.Bus(channel='can1', interface='socketcan', is_extended_id=False)
    message = bus.recv(1.0)  # get data, Timeout in seconds.

    while 1:
        if message is None:
            print('Timeout occurred, no message.')
        else:
            data0 = 0.087890625*(message.data[1]*256+message.data[0])
            #print("deg=" + str(data0))
            filencodew.writerow([data0])
        t += 1


if __name__ == "__main__":
    
    print("motor stared!")
    motor()


    print("Done!")
