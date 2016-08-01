"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
"""
from __future__ import print_function
import power_meter as pm
import datetime
from Queue import Empty
from sys import argv

def main(serial_port, output_fh, duration):

    meter = pm.PowerMeter(serial_port)
    start = datetime.datetime.now()
    meter.open()
    header = ','.join([pm.PowerData.csv_header(), 'wattSeconds'])
    print(header)
    while (datetime.datetime.now() - start < duration):
        try:
            power = meter.next(False)
        except Empty :
            power = None
        if power:
            line = ','.join([power.csv(),str(meter._calib_watt_seconds)])
            print(power)
            print(line, file=output_fh)

if __name__ == '__main__':

    if len(argv) >= 2:
        file_name = argv[1]
    else:
        file_name = 'power_measurment.csv'
    fh = open(file_name, 'w')

    if len(argv) >= 3:
        port = argv[2]
    else:
        port = '/dev/tty.usbserial-A1057F91'

    if len(argv) >= 4:
        duration = datetime.timedelta(minutes=argv[2])
    else:
        duration = datetime.timedelta(minutes=5)


    main(port, fh, duration)