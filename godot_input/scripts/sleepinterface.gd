extends Node2D

var mode=0;

# Called when the node enters the scene tree for the first time.
func _ready():
	pass

func set_mode(newmode):
	mode=newmode
	queue_redraw();

func _draw():
	
	var rad=10;
	var cen=Vector2(DisplayServer.window_get_size()[0]-(rad+60),12);
	#print("mode ",mode)
	if mode:
		#active
		#print("mode ",bool(mode))
		#print("on")
		if (mode>1):
			#softsleep
			draw_circle(cen,rad,Color(1,0.5,0));
		else:
			#awake
			draw_circle(cen,rad,Color(0,1,0));
	
	else:
		#hard sleep
		#print("off")
		draw_circle(cen,rad,Color(1,0,0));

	
