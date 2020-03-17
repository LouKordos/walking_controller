import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 6768

link = input("Enter the link name you want to apply the force to:\n")
force = input("Enter the force you want to apply to the link (in Newtons):\n")
duration = input("Enter the duration the force should be applied for (in seconds):\n")

# This should have a format like: "knee|1,2,3|1.4"

message = str(link) + "|" + str(force) + "|" + str(duration)

#message = "left_knee|0,0,10|1"

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(bytes(message, "utf-8"), (UDP_IP, UDP_PORT))

print("Sent message:\n\n")
print(message)