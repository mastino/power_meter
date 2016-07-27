"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
"""
from __future__ import print_function
import power_meter as pm
import datetime
from Queue import Empty

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
    fh = open('battery_charge.csv', 'w')
    duration = datetime.timedelta(minutes=10)

    main('/dev/tty.usbserial-A1057F91', fh, duration)