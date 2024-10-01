extends OSCReceiver

func _process(delta):
	
		if target_server.incoming_messages.has(osc_address):
			
			var cpu_degrees=target_server.incoming_messages[osc_address][0]
			var battery_percent=target_server.incoming_messages[osc_address][1]
			#print("cpu:",cpu_degrees,"battery:",battery_percent)
			parent.text=("cpu:"+str(snapped(cpu_degrees,0.01))+"º battery:"+str(int(battery_percent))+"%")

		else:
			pass
			#set_sleep()


func set_sleep():
	
	parent.text = "sleep"
