from paho.mqtt import client as mqtt_client
from datetime import datetime
import time
import csv
import random

broker = "10.13.103.28"
port = 1883
client_id = 'python'
username = "esp32"
password = "adminadmin"

data = datetime.now()
dataInicial = data
labels = ['LV1', 'SPVolt', 'Mode', 'Kp', 'Ki', 'Kd', 'Output', 'Date']

database = {'LV1': '0','SPVolt': '0', 'Mode': '0', 'Kp':'0', 'Ki':'0', 'Kd': '0', 'Output': '0', 'Date': data }

with open('dados.csv', 'w', newline='') as saida:
    escrever = csv.DictWriter(saida, fieldnames=labels)
    escrever.writeheader()

def appendData():
    while True:
        #global dataInicial
        #delta = database['Date'] - dataInicial
    
        #if str(delta) >= '0:10:00':
        #    dataInicial = database['Date']
        #    client.publish("/TkSys/W/PID/SPVolt", str(random.randint(0,150)))

        time.sleep(1)
        try:
            with open('dados.csv', 'a', newline='') as saida:
                escrever = csv.DictWriter(saida, fieldnames=labels)
                escrever.writerow(database)
        except IOError:
            print("I/O error")

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe("/TkSys/R/Lv1")
            client.subscribe("/TkSys/R/PID/SPVolt")
            client.subscribe("/TkSys/R/PID/Mode")
            client.subscribe("/TkSys/R/PID/Kp")
            client.subscribe("/TkSys/R/PID/Ki")
            client.subscribe("/TkSys/R/PID/Kd")
            client.subscribe("/TkSys/R/PID/output")


        else:
            print("Failed to connect, return code %d\n", rc)
    
    client = mqtt_client.Client(client_id)
    client.connect(broker, port)            
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    
    return client

def on_message_Lv1(client, userdata, msg):
    
    database['LV1'] = msg.payload.decode()
    database['Date'] = datetime.now()
       
def on_message_SPVolt(client, userdata, msg):
  
    database['SPVolt'] = msg.payload.decode()
    database['Date'] = datetime.now()

def on_message_mode(client, userdata, msg):
 
    database['Mode'] = msg.payload.decode()
    database['Date'] = datetime.now()

def on_message_kp(client, userdata, msg):
       
    database['Kp'] = msg.payload.decode()
    database['Date'] = datetime.now()

def on_message_ki(client, userdata, msg):

    database['Ki'] = msg.payload.decode()
    database['Date'] = datetime.now()

def on_message_kd(client, userdata, msg):
      
    database['Kd'] = msg.payload.decode()
    database['Date'] = datetime.now()

def on_message_out(client, userdata, msg):

    database['Output'] = msg.payload.decode()
    database['Date'] = datetime.now()
   
def on_message(client, userdata, msg):
    print("MESSAGES: " + msg.topic +" " + str(msg.payload))

def recive_menssage(client):
    client.message_callback_add("/TkSys/R/Lv1", on_message_Lv1) 
    client.message_callback_add("/TkSys/R/PID/SPVolt", on_message_SPVolt) 
    client.message_callback_add("/TkSys/R/PID/Mode", on_message_mode) 
    client.message_callback_add("/TkSys/R/PID/Kp", on_message_kp) 
    client.message_callback_add("/TkSys/R/PID/Ki", on_message_ki) 
    client.message_callback_add("/TkSys/R/PID/Kd", on_message_kd) 
    client.message_callback_add("/TkSys/R/PID/output", on_message_out) 
       
    client.on_message = on_message

client = connect_mqtt()
recive_menssage(client)
client.loop_start()
time.sleep(1)
appendData()