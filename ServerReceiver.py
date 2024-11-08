import socket
import pickle
import struct
import cv2

NumOfCams = 3

ServerIp = socket.gethostname()

Socket = socket.socket()
Socket.bind((ServerIp, 8000))
Socket.listen()

ConnArr = []
AddrArr = []

def get():
    for Conn in ConnArr:
        Conn.send(pickle.dumps("Start Flag"))
    
    data = []
    for Conn in ConnArr:
        # read the size of this packet
        buf = b''
        
        while len(buf) < 4:
            recv = Conn.recv(4 - len(buf))
            if not len(recv):
                break
            buf += recv
        if len(buf) != 4:
            # we waited for a packet but there isn't anything
            return False
        packetsize = struct.unpack('<l', buf)[0]
        
        # read the whole packet
        buf = b''
        while len(buf) < packetsize:
            recv = Conn.recv(packetsize - len(buf))
            if not len(recv):
                break
            buf += recv
        if len(buf) != packetsize:
            return False
        
        data.append(pickle.loads(buf))
    return data


for _ in range(NumOfCams):
    Conn, Addr = Socket.accept()
    
    ConnArr.append(Conn)
    AddrArr.append(Addr)
    
    print([i[2] for i in get()])

print("\n\nStart\n\n")

# while True:
#     data = get()
#     for ind, CamData in enumerate(data):
#         print(f"{CamData[2]} --> {CamData[1]}")
#         cv2.imshow(f"{ind}", CamData[0])
#     cv2.waitKey(0)