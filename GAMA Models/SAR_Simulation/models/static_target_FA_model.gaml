/**
* Name: Ground
* Simulation Model to test Firefly search and rescue algorithm for static target  
* Author: Aayush Kumar Shandilya
* Tags:  Firefly Algorithm, Multi-agent systems
*/


model Ground

global parent:agent{
	
	int ground_side <-100;
	
	int minor_cell_side <- 5;
	int major_cell_side <- 7;
	int cell_side <- minor_cell_side * major_cell_side;
	
	float minor_cell_length <- float(ground_side)/float(cell_side);
	float major_cell_length <- float(ground_side)/float(major_cell_side);
	
	int no_of_drones <- 20;
	int no_of_target <- 50;
	
	list target_map;
	
	list attraction_matrix;	
	
	int IS_SURVEYED <- 0;
	int IS_ALLOCATED <- 1;
	int NO_OF_TARGETS <- 2;
	int SURVEILLANCE_MAT_HEIGHT <- 3;
	
	list ground_surveillance_mat;
	
	list<drone> allDrones;
	int droneBatteryLife <-2000;
	
	float attraction_constant <-1.0;
	
	geometry shape <- rectangle(ground_side,ground_side);
	
	action initialise_matrix_2d(list init_matrix,int cell_size){
		loop i from:0 to:cell_size-1{
			list row;
			loop j from:0 to:cell_size-1{
					add false to:row;
			}
			add row to:init_matrix;
		}	
	}
	
	action initialise_matrix_3d(list init_matrix,int base_cell_size,int height){
		loop row from:0 to:base_cell_size-1{
			list rows;
			loop col from:0 to:base_cell_size-1{
				list att_for_each_cell;
				loop d from:0 to:height-1{
					add 0.0 to:att_for_each_cell;
				}
				add att_for_each_cell to:rows;
			}
			add rows to:init_matrix;
		}	
	}
	
	bool is_surveillance_complete{
		loop i from:0 to:cell_side-1{
			loop j from:0 to:cell_side-1{
					if(ground_surveillance_mat[i][j][IS_SURVEYED]=0.0){
						return false;
					}
			}
		}
		return true;
	}
	
	reflex end_stimuation when:is_surveillance_complete(){
		do pause;
	}
	
	init{	
		do initialise_matrix_2d(target_map,cell_side);
		do initialise_matrix_3d(ground_surveillance_mat,cell_side,SURVEILLANCE_MAT_HEIGHT);
		do initialise_matrix_3d(attraction_matrix,major_cell_side,no_of_drones);
		create drone number:no_of_drones;
		create target number:no_of_target;
	}
}

species utility{
	
	action put_in_matrix_2d(list target_matrix,int target_grid_X,int target_grid_Y,bool value){
		list target_map_row <- target_matrix[target_grid_X];
		put value in:target_map_row at:target_grid_Y;
		put target_map_row in:target_matrix at:target_grid_X;
	}
	
	action put_in_matrix_3d(list target_matrix,int target_grid_X,int target_grid_Y,int height,float value){
		list target_map_row <- target_matrix[target_grid_X];
		list target_map_cell <- target_map_row[target_grid_Y];
		put value in:target_map_cell at:height;
		put target_map_cell in:target_map_row at:target_grid_Y;
		put target_map_row in:target_matrix at:target_grid_X;
	}
	
			
	point get_minor_grid_location(int grid_x,int grid_y){
		return {(grid_x+0.5)*minor_cell_length,(grid_y+0.5)*minor_cell_length,0};
	}
	
	point get_major_grid_location(int grid_X,int grid_Y){
		return {(grid_X+0.5)*major_cell_length,(grid_Y+0.5)*major_cell_length,0};
	}
	
	float get_vector_magnitude(point vector){
		return sqrt(vector.x^2+vector.y^2+vector.z^2);
	}
	
	float get_distance_bt_points(point pt1,point pt2){
		point distance_vector <- pt2-pt1;
		float distance <- get_vector_magnitude(distance_vector);
		return distance;		
	}
	
	float get_distance(point drone_location,int grid_x,int grid_y){
		point destination_location <- get_minor_grid_location(grid_x,grid_y);
		float distance_bet <- get_distance_bt_points(drone_location,destination_location);
		return distance_bet;
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
	
	// TODO :: Test these functions individually
	int get_no_of_targets(int major_grid_X,int major_grid_Y){
		int no_of_targets_found <-0;
		loop row from:0 to:minor_cell_side-1{
			loop col from:0 to:minor_cell_side-1{
				int minor_row <- major_grid_X+row;
				int minor_col <- major_grid_Y+col;
				no_of_targets_found<-no_of_targets_found+int(ground_surveillance_mat[minor_row][minor_col][NO_OF_TARGETS]);
			}
		}
		return no_of_targets_found;
	}
	
	float get_surveyed_percentage(int major_grid_X,int major_grid_Y){
		int no_of_cells_surveyed <-0;
		loop row from:0 to:minor_cell_side-1{
			loop col from:0 to:minor_cell_side-1{
				int minor_row <- minor_cell_side*major_grid_X+row;
				int minor_col <- minor_cell_side*major_grid_Y+col;
				no_of_cells_surveyed<-no_of_cells_surveyed+int(ground_surveillance_mat[minor_row][minor_col][IS_SURVEYED]);
			}
		}
		return no_of_cells_surveyed/minor_cell_side^2;
	}
	
	// TODO:: Make sure empty major cells have less attraction over other target present major cells
	float return_attraction_value(drone given_drone,int major_grid_X,int major_grid_Y){
		point drone_location <- given_drone.location;
		point major_cell_location <- get_major_grid_location(major_grid_X,major_grid_Y);
		float distance_bet <- get_distance_bt_points(drone_location,major_cell_location);
		float battery_life <- given_drone.batteryLife;
		int reputation <- 1+given_drone.targetRescued;
		float surveillance_factor <- get_surveyed_percentage(major_grid_X,major_grid_Y);
		
		float attraction_power <- (distance_bet)^2/(battery_life*reputation);
		float attraction <- attraction_constant*exp(-1*attraction_power)*(1-surveillance_factor);
		
		return attraction;
	}
	
	action update_attraction_mat(drone given_drone){
		int drone_idx <-given_drone.droneID;
		loop row from:0 to:major_cell_side-1{
			loop col from:0 to:major_cell_side-1{
				float attraction_val <- return_attraction_value(given_drone,row,col);
				do put_in_matrix_3d(attraction_matrix,row,col,drone_idx,attraction_val);
			}
		}
	}
}

species target parent:utility {
	
	image_file target_icon <- image_file("../includes/TargetLogo.png");
	ground_cell target_cell <- one_of(ground_cell);
	
	aspect icon {
		draw target_icon size: 2;
	}
	
	action add_target_map(int target_grid_X,int target_grid_Y){
		do put_in_matrix_2d(target_map,target_grid_X,target_grid_Y,true);
	}
	
	action remove_target_map(int target_grid_X,int target_grid_Y){
		do put_in_matrix_2d(target_map,target_grid_X,target_grid_Y,false);
	}
	
	action get_rescued{
		do remove_target_map(target_cell.grid_x,target_cell.grid_y);
		int no_target_at_cell <- int(ground_surveillance_mat[target_cell.grid_x][target_cell.grid_y][NO_OF_TARGETS]);
		do put_in_matrix_3d(ground_surveillance_mat,target_cell.grid_x,target_cell.grid_y,NO_OF_TARGETS,float(no_target_at_cell+1));
		no_of_target<-no_of_target-1;
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
	
	int droneID;
	
	int lockPeriod;
	float batteryLife;
	
	float speed;
	float surveyTime;
	
	int target_grid_X;
	int target_grid_Y;
	point target_location;
	int targetRescued;
	
	image_file drone_icon <- image_file("../includes/DroneLogo.png");
	
	aspect icon {
		draw drone_icon size: 2.5;
	}	

	action goto_minor_cell(int grid_x,int grid_y){
		lockPeriod <- get_lock_period(location,grid_x,grid_y,speed,surveyTime);
		do goto(target_location);
	}
	
	action allocate_cell(int grid_X,int grid_Y){
		do put_in_matrix_3d(ground_surveillance_mat,grid_X,grid_Y,IS_ALLOCATED,1.0);
	}
	
	action deallocate_cell(int grid_X,int grid_Y){
		do put_in_matrix_3d(ground_surveillance_mat,grid_X,grid_Y,IS_ALLOCATED,0.0);
	}
	
	action mark_cell_surveyed(int grid_X,int grid_Y){
		do put_in_matrix_3d(ground_surveillance_mat,grid_X,grid_Y,IS_SURVEYED,1.0);
	}
	
	reflex move {
		
		if(lockPeriod=0){
			do update_attraction_mat(self);
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
			myself.targetRescued<-myself.targetRescued+1;
		}

		ask ground_cell{
			do update_color;
		}
		
		batteryLife <- batteryLife - (1/droneBatteryLife);
	}
	
	point select_best_cell{
		float cur_attraction_val <- 0.0;
		int best_x <- rnd(major_cell_side-1);
		int best_y <- rnd(major_cell_side-1);
		int drone_IDX <- self.droneID;
		loop i from:0 to:major_cell_side-1{
			loop j from:0 to:major_cell_side-1{					
				float att_val <- float(attraction_matrix[i][j][drone_IDX]);
				if(cur_attraction_val < att_val or cur_attraction_val = att_val){
					cur_attraction_val <- att_val;
					best_x<-i;
					best_y<-j;
				}	
			}
		}
		
		list available_cell;
		loop i from:0 to:minor_cell_side-1{
			loop j from:0 to:minor_cell_side-1{	
				int grid_row <- minor_cell_side*best_x+i;
				int grid_col <- minor_cell_side*best_y+j;
				if(int(ground_surveillance_mat[grid_row][grid_col][IS_SURVEYED])=0 and int(ground_surveillance_mat[grid_row][grid_col][IS_ALLOCATED])=0){
					add {grid_row,grid_col} to:available_cell;
				}
			}
		}		
		
		if (length(available_cell)!=0){
			point best_cell <- one_of(available_cell);	
			return best_cell;			
		}
		
		return {minor_cell_side*best_x+rnd(minor_cell_side-1),minor_cell_side*best_y+rnd(minor_cell_side-1)};
	}
	
	action survey_nxt_cell{
		point best_cell <-select_best_cell();
		target_grid_X<-int(best_cell.x);
		target_grid_Y<-int(best_cell.y);
		target_location <- get_minor_grid_location(target_grid_X,target_grid_Y);
		do allocate_cell(target_grid_X,target_grid_Y);
		do goto_minor_cell(target_grid_X,target_grid_Y);
	}
	
	init{
		lockPeriod<-0;
		speed<-1.0;
		surveyTime<-1.0;
		batteryLife<-1.0;
		targetRescued<-0;
		droneID<-length(allDrones);
		
		add self to:allDrones;
	}
	
}

grid ground_cell height:cell_side width:cell_side neighbors:4 {
	
	action update_color{
		if(ground_surveillance_mat[grid_x][grid_y][IS_SURVEYED]=1.0){
			color <- #green;
		}else if(target_map[grid_x][grid_y]=true){
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
		display Parameters_Information refresh: every(5#cycles){
			chart "Targets Remaining" type: series size: {1,0.5} position: {0, 0.5} {
				data "number_of_target" value: no_of_target color: #blue;
			}
		}
	}
}
