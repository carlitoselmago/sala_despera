import csv
from pythonosc import dispatcher
from pythonosc import osc_server
import time

# CSV file setup
csv_filename = "osc_messages.csv"
csv_header = ["Timestamp", "Address", "Args"]

# Create and initialize the CSV file with semicolon delimiter
with open(csv_filename, mode="w", newline='') as file:
    writer = csv.writer(file, delimiter=";")
    writer.writerow(csv_header)

# Function to handle incoming OSC messages
def osc_message_handler(address, *args):
    # Get the current timestamp with milliseconds
    timestamp = time.time()#.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # To include milliseconds

    # Log the message contents (for troubleshooting)
    print(f"Received message at {timestamp} from {address} with args: {args}")

    # Prepare the row with the address and the arguments of the OSC message
    row = [timestamp, address, args]

    # Append the row to the CSV file with semicolon delimiter
    with open(csv_filename, mode="a", newline='') as file:
        writer = csv.writer(file, delimiter=";")
        writer.writerow(row)

# Set up the dispatcher
dispatcher = dispatcher.Dispatcher()
dispatcher.set_default_handler(osc_message_handler)

# Set up the server
ip = "0.0.0.0"  # localhost
port = 54321  # Port number

# Create and start the server
server = osc_server.BlockingOSCUDPServer((ip, port), dispatcher)

print(f"Listening for OSC messages on {ip}:{port}...")
server.serve_forever()
