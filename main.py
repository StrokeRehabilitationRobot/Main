from UDP.UDP import UDP


VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PI_CONFIG = 65

udp = UDP(9876)



packet = 15*[2,1,1]


print udp.send_packet(37,packet)

