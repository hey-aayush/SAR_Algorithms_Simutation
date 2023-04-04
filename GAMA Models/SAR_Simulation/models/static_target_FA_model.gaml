/**
* Name: Ground
* Simulation Model to test Firefly search and rescue algorithm for static target  
* Author: Aayush Kumar Shandilya
* Tags:  Firefly Algorithm, Multi-agent systems
*/


model Ground

global {
	
	int ground_side <-100;
	
	int minor_cell_side <- 3;
	int major_cell_side <- 5;
	int cell_side <- minor_cell_side * major_cell_side;
	
	float minor_cell_length <- float(ground_side)/float(cell_side);
	
	int no_of_drones <- 3;
	
	matrix major_cell_target_matrix <- 0.0 as_matrix({major_cell_side,major_cell_side});
	matrix cell_surveyed_matrix <- false as_matrix({cell_side,cell_side});
	list attraction_matrix;
	
	matrix dummy_mat <-[
		[1,2,3],
		[4,5,6],
		[7,8,9]
	];
	
	geometry shape <- rectangle(ground_side,ground_side);
	
	init{
		
//		write dummy_mat;
//		write dummy_mat[0,0];
//		write dummy_mat[1,0];
		loop i from:0 to:major_cell_side-1{
			list row;
			loop j from:0 to:major_cell_side-1{
					add j to:row;
			}
			add row to:attraction_matrix;
		} 
		write attraction_matrix ;
		write attraction_matrix[0][1];
		create drone number:no_of_drones;
	}

}

species entity{
		
	point get_grid_location(int grid_x,int grid_y){
		return {(grid_x+0.5)*minor_cell_length,(grid_y+0.5)*minor_cell_length,0};
	}
	
	float get_vector_magnitude(point vector){
		return sqrt(vector.x^2+vector.y^2+vector.z^2);
	}
	
	float get_distance(point drone_location,int grid_x,int grid_y){
		point destination_location <- get_grid_location(grid_x,grid_y);
		point distance_vector <- destination_location-drone_location;
		float distance <- get_vector_magnitude(distance_vector);
		return distance;
	}
	
	float get_time_req(float distance,float speed){
		return distance/speed;
	}
	
	int get_lock_period_req(float time_req){
		return int(ceil(time_req/step));
	}
	
}

species drone parent:entity skills:[moving]{
	
	int lockPeriod;
	
	float speed;
	float surveyTime;
	
	int target_grid_X;
	int target_grid_Y;
	point target_location;
	
	image_file my_icon <- image_file("../includes/DroneLogo.png");
	
	aspect icon {
		draw my_icon size: 2.5;
	}
	
	int get_lock_period(point drone_location,int grid_x,int grid_y,float drone_speed,float survey_time){
		float distance_bet_points <- get_distance(drone_location,grid_x,grid_y);
		float time_req <- get_time_req(distance_bet_points,drone_speed)+survey_time;
		int lock_period <- get_lock_period_req(time_req);
		return lock_period;
	}
	
	action goto_minor_cell(int grid_x,int grid_y){
		lockPeriod <- get_lock_period(location,grid_x,grid_y,speed,surveyTime);
		do goto(target_location);
	}
	
	reflex move {
		
		if(lockPeriod=0){
			do survey_nxt_cell();
			ask ground_cell {
				do make_target_cell(myself.target_grid_X,myself.target_grid_Y);
				do update_color;
			}
		}else{
			lockPeriod<-lockPeriod-1;
			do goto(target_location);
			if(lockPeriod=0){
				ask ground_cell {
					do remove_target_cell(myself.target_grid_X,myself.target_grid_Y);
					do update_color;
				}
			}
		}
	}
	
	point select_best_cell{
		return {rnd(cell_side-1),rnd(cell_side-1)};
	}
	
	action survey_nxt_cell{
		point best_cell <-select_best_cell();
		target_grid_X<-int(best_cell.x);
		target_grid_Y<-int(best_cell.y);
		target_location <- get_grid_location(target_grid_X,target_grid_Y);
		do goto_minor_cell(target_grid_X,target_grid_Y);
	}
	
	init{
		lockPeriod<-0;
		speed<-1.0;
		surveyTime<-1.0;
	}
	
}

grid ground_cell height:cell_side width:cell_side neighbors:4 {
	
	bool isTarget;
	
	action update_color{
		if(isTarget){
			color <- #red;
		}else{
			color <- #grey;
		}
	}
	
	action make_target_cell(int grid_X,int grid_Y){
		if(grid_x=grid_X and grid_y=grid_Y){
			isTarget<-true;
		}
	}
	
	action remove_target_cell(int grid_X,int grid_Y){
		if(grid_x=grid_X and grid_y=grid_Y){
			isTarget<-false;
		}
	}
	
	init{
		isTarget<-false;
		do update_color;
	}
	
}


experiment sar_simulation type:gui{
	output{
		display ground_display{
			grid ground_cell lines:#black;
			species drone aspect:icon;
		}
	}
}