extends OSCMessage

var csv_data = []
var start_time : float
var current_time : float
var last_timestamp : float = 0.01  # Store the last timestamp
@export var active=false;

# Load CSV file and store data
func _ready():
	if active:
		var file = FileAccess.open("res://scripts/osc_messages.csv", FileAccess.READ)
		if file:
			var content = file.get_as_text()
			var lines = content.split("\n")
			for line in lines:
				if line != "":  # Skip empty lines
					var values = line.split(";", false)
					#print(values)
					# Parse the Timestamp and convert it to a float representing seconds
					var timestamp = values[0].strip_edges().to_float()
					#print(timestamp)
					var address = values[1].strip_edges()
					var args = values[2].strip_edges()  # Handle the OSC args
					
					csv_data.append({"timestamp": timestamp, "address": address, "args": args})
			
			start_time = Time.get_unix_time_from_system()  # Use current time as reference point
			current_time = start_time

		else:
			print("Failed to open CSV file")

# Update is called every frame. This checks the relative time and prints the OSC data accordingly.
func _process(delta):
	if active:
		if csv_data.size() > 0:
			current_time = Time.get_unix_time_from_system()
			#print("current"current_time)
			var relative_time = current_time - start_time
			
			for i in range(csv_data.size()):
				var osc_entry = csv_data[i]
				
				# Calculate the delay based on timestamp difference from the last processed one
				var delay = osc_entry["timestamp"] - last_timestamp
				#print(delay)
				if last_timestamp > 0:
					await get_tree().create_timer(delay).timeout
					#OS.delay_msec(delay*1000)  # Pause based on the delay in milliseconds
				
					if relative_time >= (osc_entry["timestamp"] - csv_data[0]["timestamp"]):
						# Remove the printed message from the list
						csv_data.remove_at(i)
						
						# Emit the OSC message
						var args = osc_entry["args"].split(",", false)
						var argsfloat = [0.0, 0.0, 0.0, 0.0]
						for ii in range(args.size()):
							var val = args[ii].replace("(", "").replace(")", "").strip_edges()
							argsfloat[ii] = val.to_float()
						
						target_client.send_message(osc_entry["address"], argsfloat)
						
						# Update the last timestamp after sending the message
				last_timestamp = osc_entry["timestamp"]
				break  # Avoid skipping over messages
