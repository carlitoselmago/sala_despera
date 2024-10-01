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
	# Create and configure the timer
	sleep_timer = Timer.new()
	sleep_timer.set_wait_time(accumulation_time)
	sleep_timer.set_one_shot(false)
	add_child(sleep_timer)
	sleep_timer.start()

	# Use Callable to connect the timer's timeout signal
	sleep_timer.connect("timeout", Callable(self, "_on_timer_timeout"))

func _process(delta):
	if target_server.incoming_messages.has(osc_address):
		
		var x = target_server.incoming_messages[osc_address][0]
		var y = target_server.incoming_messages[osc_address][1]
		var z = target_server.incoming_messages[osc_address][2]

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

# Function that runs when the timer finishes
func _on_timer_timeout():
	if accumulated_change >= move_threshold:
		print("awake")
	else:
		print("sleep")
	
	# Reset the accumulated change for the next period
	accumulated_change = 0.0
		
		
#TODO: muestra de datos guardados para no necesitar el sensor

