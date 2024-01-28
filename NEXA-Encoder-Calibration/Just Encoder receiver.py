import time
import csv
import can

def encoder():‎
‎    t = 0‎
‎    filencode = open('encoderdata_final.csv', 'w')‎
‎    filencodew = csv.writer(filencode)‎
‎    filencodew.writerow(['time','encoderPosition'])‎

‎    # can setup‎
‎    bus = can.Bus(channel='can0', interface='socketcan', is_extended_id=False)‎

‎    while 1:‎
‎        message = bus.recv(1.0)  # get data, Timeout in seconds.‎

‎        if message is None:‎
‎            print('Timeout occurred, no message.')‎
‎        else:‎
‎            data0 = 0.087890625*(message.data[1]*256+message.data[0])‎
‎            # print("deg=" + str(data0))‎
‎            encodertime = time.time()‎
‎            filencodew.writerow([encodertime,data0])‎
‎        t += 1‎

if __name__ == "__main__":‎

‎    print("encoder stared!")‎
‎    encoder()‎
‎    ‎
‎    print("Done!")‎
