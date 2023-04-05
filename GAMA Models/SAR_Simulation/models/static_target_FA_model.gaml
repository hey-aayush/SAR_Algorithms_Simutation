/**
* Name: Ground
* Simulation Model to test Firefly search and rescue algorithm for static target  
* Author: Aayush Kumar Shandilya
* Tags:  Firefly Algorithm, Multi-agent systems
*/


model Ground

global parent:agent{
	
	int ground_side <-100;
	
	int minor_cell_side <- 3;
	int major_cell_side <- 4;
	int cell_side <- minor_cell_side * major_cell_side;
	
	float minor_cell_length <- float(ground_side)/float(cell_side);
	
	int no_of_drones <- 4;
	int no_of_target <- 4;
	
	list target_map;

	list attraction_matrix;	
	
	int IS_SURVEYED <- 0;
	int IS_ALLOCATED <- 1;
	int NO_OF_TARGETS <- 2;
	int SURVEILLANCE_MAT_HEIGHT <- 3;
	
	list ground_surveillance_mat;
	
	list<drone> allDrones;
		
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
					if(int(ground_surveillance_mat[i][j][IS_SURVEYED])=0 and target_map[i][j]=true and int(ground_surveillance_mat[i][j][IS_ALLOCATED])=0){
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
//		display Parameters_Information refresh: every(5#cycles){
//			chart "Targets Remaining" type: series size: {1,0.5} position: {0, 0.5} {
//				data "number_of_target" value: no_of_target color: #blue;
//			}
//		}
	}
}
