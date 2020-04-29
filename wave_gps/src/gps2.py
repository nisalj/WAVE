#! /usr/bin/python
from time import sleep
from gps3.agps3threaded import AGPS3mechanism

agps_thread = AGPS3mechanism()
agps_thread.stream_data()
agps_thread.run_thread()

while True:
    print('{},{},{},{},{}'.format(agps_thread.data_stream.lat, agps_thread.data_stream.lon, agps_thread.data_stream.epx, agps_thread.data_stream.epy, agps_thread.data_stream.hdop))
    sleep(2)
