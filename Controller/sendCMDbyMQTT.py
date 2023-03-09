import paho.mqtt.client as mqtt
import time, sys, tty, termios

serverIP = '127.0.0.1'
qos = 1

#message = {key_input : [topic, payload]}
message = {'8':['direction', 'FORWARD'],
           '4':['direction', 'LEFT'],
           '6':['direction', 'RIGHT'],
           '2':['direction', 'BACKWARD'],
           '5':['direction', 'STOP'],
           's':['command', 'START'],
           'f':['command', 'FINISH'],
           'r':['command', 'RC'],
           'h':['command', 'HIDE'],
           }

##for MQTT
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)

def on_disconnect(client, userdata, flags, rc=0):
    print("disconnect the MQTT connection")
    
##for keyboard input
def getKeyPressEvent():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
    return ch


if __name__ == "__main__":    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    #client.on_publish = on_publish
    #address : localhost, port: 1883 에 연결
    
    client.connect(serverIP)
    client.loop_start()
    time.sleep(0.1)
    
    while True:
        key_input = getKeyPressEvent()
        #print("payload:", key_input)
        if key_input in message:
            msg = message[key_input]
            print("topic:", msg[0], "payload:", msg[1])
            client.publish(msg[0], msg[1], qos)
        elif key_input == 'q': #quit the program
            break
    
    # 연결 종료
    client.loop_stop()
    client.disconnect() 
