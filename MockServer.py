import socket

UDP_IP = "192.168.1.129"
UDP_PORT = 5801

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    myAddr = (UDP_IP, UDP_PORT)
    sock.bind(myAddr)

    while True:
        data, distAddr = sock.recvfrom(1024)
        print("received message: %s" % data.hex())
        ret = data[:5] + b"0"

        if data[2] == 1:
            ret += b"FF01FF02"
        sock.sendto(ret, distAddr)