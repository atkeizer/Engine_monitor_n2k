# Engine_monitor_n2k
Use an Arduino DUE to measure basic engine sensors and send on CAN/NMEA2000
Currently only iplemented RPM, Oil pressure, Temperature.

The parameters defined are aplicable to a Volvo Peta MD17 but will be the same on MD11 and probaly many others. The teeth per revolution may vary. and the curves of the pressure and temperature sensors may be different as well. (here VDO sensors are equiped)

The hard work is done by the NMEA2000 and due_can libraries created by ttlappalainen:

https://github.com/ttlappalainen/NMEA2000
https://github.com/ttlappalainen/due_can

It also uses Aduino tc_lib by Antonio C. Domínguez Brito 

