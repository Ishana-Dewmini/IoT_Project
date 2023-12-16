import paho.mqtt.client as mqtt
import time
import pandas as pd
import csv
import json

# Callback when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker\n")
    else:
        print("Connection failed with code {rc}")

# Create an MQTT client instance
client = mqtt.Client("Project_simulation_PH")    

# Set the callback function
client.on_connect = on_connect

broker_address = "test.mosquitto.org"  # broker's address
broker_port = 1883
keepalive = 5
qos = 2
publish_topic = "Group07_Project"

# Connect to the MQTT broker
client.connect(broker_address, broker_port, keepalive)

# Publish a message
while(True):
    try:
        file_path = r"F:\Sem5\2-IOT(EN3251)\Project\lois_continuous.csv"

        # Open the CSV file for reading
        with open(file_path, mode='r', encoding='utf-8') as csv_file:
            # Create a CSV reader object
            csv_reader = csv.DictReader(csv_file)
            
            # Convert CSV data to a list of dictionaries
            list_of_dicts = list(csv_reader)


        # Serialize the list of dictionaries to a JSON-formatted string
        json_obj = json.dumps(list_of_dicts, indent=4)

        data_list = json.loads(json_obj)


        client.loop_start()
        for row in data_list[1:]:
            # Publish the message
            client.publish(publish_topic, json.dumps(row), qos)
            print(row)
            # Wait for a moment to simulate some client activity
            time.sleep(4)
            #client.disconnect()
            #print("Disconnected from the MQTT broker")

    except KeyboardInterrupt:
        # Disconnect from the MQTT broker
        pass

client.loop_stop()
client.disconnect()
print("Disconnected from the MQTT broker")
