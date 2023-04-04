/**
* Name: Arena Model
* Stimulation Model to test search and rescue algorithms
* Author: Aayush
* Tags: Firefly Algorithm, Multi-agent systems
*/


model arenaModel

global {
	
	list all_directions <- [
		[0,1], 		//N
		[1,-1],		//NW
		[-1,0],		//W
		[-1,-1],	//WS
		[0,-1],		//S
		[1,-1],		//SE
		[1,0],		//E
		[1,1]		//NE
	];
	
	int nb_target_init <- 20;
	int nb_drone_init <- 10;
	int distance_to_intercept <- 1;
	
	init {
		create target number: nb_target_init;
		create drone number: nb_drone_init;
	}
}

species target {
	float size <- 1.0;
	rgb color <- #blue;
	grid_cell target_cell <- one_of (grid_cell); 	
	
	init { 
		location <- target_cell.location;		
	}
	
	reflex basic_move {
		target_cell <- one_of(target_cell.neighbors);
		location <- target_cell.location;
		write location;	
	}
	
	action rescused{
		do die();
	}
	
	aspect base {
		draw triangle(size) color: color;
	}
} 

species drone {
	float size <- 1.0;
	rgb color <- #green;
	grid_cell target_cell <- one_of (grid_cell); 	
	
	init { 
		location <- target_cell.location;
	}
	
	reflex basic_move {
		target_cell <- one_of(target_cell.neighbors);
		location <- target_cell.location;
		ask target at_distance(distance_to_intercept) {
			color <- #red;
	        write "Rescused";
	        do rescused();
	    }
	}	   
	aspect base {
		draw "|👨‍🦱|" color: #white font: font('Default', 50, #bold) ;
		
	}
} 

grid grid_cell width: 50 height: 50 neighbors: 4 {
	rgb color <- rgb(int(220),int(220),int(220));
}

experiment search_and_rescue type: gui {
	parameter "Initial number of target: " var: nb_target_init min: 1 max: 1000 category: "Target";
	parameter "Initial number of drones: " var: nb_drone_init min: 1 max: 1000 category: "Drone";
	output {
		display main_display background:#red {
//			image drone file:"/home/aayush/Downloads/DroneLogo.png";
			grid grid_cell lines:#black;
			species target aspect: base;
			species drone aspect: base;
		}
	}
}
