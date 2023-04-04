/*
* Name: Ground Model
* Stimulation Model to test search and rescue algorithms 
* Author: Aayush Kumar Shandilya
* Tags: Firefly Algorithm, Multi-agent systems
*/

model Ground

global {
	
	/* General Parameters */
	list<point> all_directions <- [
		{0,1,0}, 		//N
		{1,-1,0},		//NW
		{-1,0,0},		//W
		{-1,-1,0},		//WS
		{0,-1,0},		//S
		{1,-1,0},		//SE
		{1,0,0},		//E
		{1,1,0}			//NE
	];
	
	/* Ground Parameters */
	int ground_height <- 100;
	int ground_width <- 100;
	int ground_cell_neighbors <- 4;
	rgb ground_cell_color <- rgb(220,220,220);
	rgb ground_cell_lines <- #black;
	
	/* Target Parameters */
	int nb_target_init <- 20;
	float direction_change_probabilty <- 0.20;
	float target_size <- 1.0;
	rgb target_color <- #blue;
	point target_init_location <- {50.0,50.0,0.0}; // Change with no of grid cells
	
	/* Drone Parameters */
	
	init{
		create target number:nb_target_init;
	}
	
}

species drone{
	
}

species target{
	
	float size <- target_size;
	rgb color <- target_color;	
	point cur_direction <- {0.0,0.0,0.0};
	
	init { 
		location <- target_init_location;	
	}
	
	//TO DO :: Handle Out of boundary cases.
	reflex basic_move {
		if (flip(direction_change_probabilty)){
			cur_direction <- one_of(all_directions);	
		}
		location <- location + cur_direction;
	}
	
	action rescused{
		do die();
	}
	
	aspect base {
		draw triangle(size) color: color;
	}
	
}

grid ground_cell height:ground_height width:ground_width neighbors:ground_cell_neighbors {
	rgb color <- ground_cell_color;
}

experiment sar_simulation type:gui{
	
	parameter "Ground Height: " var: ground_height min: 0 max: 1000 category: "Ground Parameters";
	parameter "Ground Width: " var: ground_width min: 0 max: 1000 category: "Ground Parameters";
	parameter "Ground Cell Neighbors: " var: ground_cell_neighbors among:[4,6] category: "Ground Parameters";	
	
	output{
		display ground_display {
			grid ground_cell lines:ground_cell_lines;
//			species target aspect:base;
			
		}
	}
}