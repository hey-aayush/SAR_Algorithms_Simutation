/**
* Name: Ground
* Simulation Model to test Firefly search and rescue algorithm for static target  
* Author: Aayush Kumar Shandilya
* Tags:  Firefly Algorithm, Multi-agent systems
*/


model Ground

global parent:agent{
	
	int ground_side <-100;
	
	int minor_cell_side <- 4;
	int major_cell_side <- 5;
	int cell_side <- minor_cell_side * major_cell_side;
	
	float minor_cell_length <- float(ground_side)/float(cell_side);
	
	int no_of_drones <- 5;
	int no_of_target <- 20;
	
	list target_map;
	list is_cell_allocated;
	list is_cell_surveyed;
		
	geometry shape <- rectangle(ground_side,ground_side);
	
	action initialise_matrix(list init_matrix,int cell_size){
		loop i from:0 to:cell_size-1{
			list row;
			loop j from:0 to:cell_size-1{
					add false to:row;
			}
			add row to:init_matrix;
		}	
	}
	
	init{
		do initialise_matrix(target_map,cell_side);
		do initialise_matrix(is_cell_allocated,cell_side);
		do initialise_matrix(is_cell_surveyed,cell_side);
		create drone number:no_of_drones;
		create target number:no_of_target;
	}
}

species utility{
	action put_in_matrix(list target_matrix,int target_grid_X,int target_grid_Y,bool value){
		list target_map_row <- target_matrix[target_grid_X];
		put value in:target_map_row at:target_grid_Y;
		put target_map_row in:target_matrix at:target_grid_X;
	}
}

species target parent:utility {
	
	image_file target_icon <- image_file("../includes/TargetLogo.png");
	ground_cell target_cell <- one_of(ground_cell);
	
	aspect icon {
		draw target_icon size: 2;
	}
	
	action add_target_map(int target_grid_X,int target_grid_Y){
		do put_in_matrix(target_map,target_grid_X,target_grid_Y,true);
	}
	
	action remove_target_map(int target_grid_X,int target_grid_Y){
		do put_in_matrix(target_map,target_grid_X,target_grid_Y,false);
	}
	
	action get_rescued{
		do remove_target_map(target_cell.grid_x,target_cell.grid_y);
		do die();
	}
	
	init{
		location <- target_cell.location;
		do add_target_map(target_cell.grid_x,target_cell.grid_y);
		ask ground_cell{
			do update_color;
		}
	}
}

species drone parent:utility skills:[moving]{
	
	int lockPeriod;
	
	float speed;
	float surveyTime;
	
	int target_grid_X;
	int target_grid_Y;
	point target_location;
	
	image_file drone_icon <- image_file("../includes/DroneLogo.png");
	
	aspect icon {
		draw drone_icon size: 2.5;
	}	
			
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
	
	float get_time_req(float distance,float given_speed){
		return distance/given_speed;
	}
	
	int get_lock_period_req(float time_req){
		return int(ceil(time_req/step));
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
	
	action allocate_cell(int grid_X,int grid_Y){
		do put_in_matrix(is_cell_allocated,grid_X,grid_Y,true);
	}
	
	action deallocate_cell(int grid_X,int grid_Y){
		write "Cell Deallocated";
		do put_in_matrix(is_cell_allocated,grid_X,grid_Y,false);
	}
	
	action mark_cell_surveyed(int grid_X,int grid_Y){
		write "Cell Surveyed";
		write {grid_X,grid_Y};
		do put_in_matrix(is_cell_surveyed,grid_X,grid_Y,true);
		write is_cell_surveyed;
	}
	
	reflex move {
		
		if(lockPeriod=0){
			do survey_nxt_cell();
		}else{
			lockPeriod<-lockPeriod-1;
			do goto(target_location);
			if(lockPeriod=0){
				do deallocate_cell(target_grid_X,target_grid_Y);
				do mark_cell_surveyed(target_grid_X,target_grid_Y);
			}
		}
		ask target at_distance(0.5){
			do get_rescued();
		}

		ask ground_cell{
			do update_color;
		}
	}
	
	point select_best_cell{
		loop i from:0 to:cell_side-1{
			loop j from:0 to:cell_side-1{
					if(is_cell_surveyed[i][j]=false and target_map[i][j]=true and is_cell_allocated[i][j]=false){
						return {i,j};
					}
			}
		}	
		return {rnd(cell_side-1),rnd(cell_side-1)};
	}
	
	action survey_nxt_cell{
		point best_cell <-select_best_cell();
		target_grid_X<-int(best_cell.x);
		target_grid_Y<-int(best_cell.y);
		target_location <- get_grid_location(target_grid_X,target_grid_Y);
		do allocate_cell(target_grid_X,target_grid_Y);
		do goto_minor_cell(target_grid_X,target_grid_Y);
	}
	
	init{
		lockPeriod<-0;
		speed<-1.0;
		surveyTime<-1.0;
	}
	
}

grid ground_cell height:cell_side width:cell_side neighbors:4 {
	
	action update_color{
		if(bool(is_cell_surveyed[grid_x][grid_y])){
			color <- #green;
		}else if(bool(target_map[grid_x][grid_y])){
			color <- #blue;
		}else{
			color <- #grey;
		}
	}
	
}


experiment sar_simulation type:gui{
	output{
		display ground_display{
			grid ground_cell border:#black;
			species target aspect:icon;
			species drone aspect:icon;
		}
	}
}
