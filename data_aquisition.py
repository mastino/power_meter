"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
"""
from __future__ import print_function
import power_meter as pm
import ina219
import datetime
from Queue import Empty
from sys import argv

def main(pc_output_fh, line_output_fh, interval, duration):

    pc_monitor = pm.INA219_Monitor(interval, 0x40, 1, ina219.INA219_CALIB_32V_1A)
    line_monitor = pm.INA219_Monitor(interval, 0x41, 1, ina219.INA219_CALIB_32V_1A)
    pc_meter = pm.PowerMeter(pc_monitor)
    line_meter = pm.PowerMeter(line_monitor)

    start = datetime.datetime.now()
    pc_meter.open()
    line_meter.open()


    header = ','.join([pm.PowerData.csv_header(), 'wattSeconds'])
    print(header, file=line_output_fh)
    print(header, file=pc_output_fh)

    # output once per second to stdout
    start_time = datetime.datetime.now()
    mark_time = start_time
    check_time = start_time
    output_delay = datetime.timedelta(seconds=1)

    while check_time - start < duration:
        try:
            pc_power = pc_meter.next(False)
        except Empty :
            pc_power = None
        try:
            line_power = line_meter.next(False)
        except Empty:
            line_power = None
        if pc_power:
            line = ','.join([pc_power.csv(),str(pc_meter._calib_watt_seconds)])
            print(line, file=pc_output_fh)
        if line_power:
            line = ','.join([line_power.csv(),str(line_meter._calib_watt_seconds)])
            print(line, file=line_output_fh)
        if check_time - mark_time >= output_delay:
            print('pc  ', pc_meter._last)
            print('line', line_meter._last)
            mark_time = check_time
        check_time = datetime.datetime.now()

    pc_meter.close()
    line_meter.close()

if __name__ == '__main__':

    # filename input
    if len(argv) >= 2:
        file_name = argv[1]
    else:
        file_name = 'power_measurment'

    pc_fh = open(file_name+'_pc.csv', 'w')
    line_fh = open(file_name+'_line.csv', 'w')

    # interval input
    if len(argv) >= 3:
        interval = float(argv[2])
    else:
        interval = 1.0

    # duration input
    if len(argv) >= 4:
        duration = datetime.timedelta(minutes=int(argv[3]))
    else:
        duration = datetime.timedelta(minutes=1)

    print('interval=%f, duration=%s' % (interval, duration))
    main(pc_fh, line_fh, interval, duration)