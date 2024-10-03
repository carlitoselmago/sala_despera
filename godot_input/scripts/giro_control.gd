extends OSCReceiver

#var smoothed_rotation : Vector3 = Vector3()
#var smoothing_factor : float = 0.1  # Adjust this value for more or less smoothing
var lastx=0.0
var lasty=0.0
var lastz=0.0
var move_threshold = 1.0  # Set the threshold to 5
var accumulation_time = 2.0  # Time period in seconds to accumulate changes (N seconds)
var accumulated_change = 0.0  # To track total accumulated change

# Timer for accumulation
var sleep_timer = null

func _ready():
	pass

func _process(delta):
	if target_server.incoming_messages.has(osc_address):
		
		var x = target_server.incoming_messages[osc_address][0]
		var y = target_server.incoming_messages[osc_address][1]
		var z = target_server.incoming_messages[osc_address][2]
		var status=target_server.incoming_messages[osc_address][3]
		#print("status ",status)
		var sleepind=get_node("../../../sleepindicator");
		sleepind.set_mode(status);
		#sleepind._draw(status);
		# Apply the rotation to the parent
		parent.rotation_degrees = Vector3(y, x * -1, z * -1)
		
		# Calculate the sum of absolute differences between current and last values
		var delta_x = abs(x - lastx)
		var delta_y = abs(y - lasty)
		var delta_z = abs(z - lastz)
		var total_change = delta_x + delta_y + delta_z

		# Accumulate the total change
		accumulated_change += total_change

		# Update the last values
		lastx = x
		lasty = y
		lastz = z


		
#TODO: muestra de datos guardados para no necesitar el sensor

