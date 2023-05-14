/**
* Name: Ground
* Simulation Model to test Firefly search and rescue algorithm for static target  
* Author: Aayush Kumar Shandilya
* Tags:  Firefly Algorithm, Multi-agent systems
*/


model ground 

global {
	
	// Ground Parameters	
	
	int ground_side <-300;
	int minor_cell_side <- 3;
	int major_cell_side <- 7;	
	int cell_side <- minor_cell_side * major_cell_side;
	float minor_cell_length <- float(ground_side)/float(cell_side);
	float major_cell_length <- float(ground_side)/float(major_cell_side);
	geometry shape <- rectangle(ground_side,ground_side);
		
	// Drones Parameters	
	
	string DRONE_ALGO_MODE;
	string DRONE_ORIGIN_MODE;
	list<drone> allDrones;
	int droneBatteryCapacity <-2000;
	float cur_avg_target_hit_time <-INIFINITY;
	float cur_energy_consumed <-0.0;
	float rescue_sensitivity <- 0.5;
	
	//Algorithm Constants - FA
	
	float attraction_constant <-1.0;
	float INIFINITY <- float(0);		
	
	// Simulation Parameters
	
	string TARGET_MODE;
	int nb_drones_init <- 10;
	int nb_target_init <- 25;
	int no_of_drones <- nb_drones_init;
	int no_of_target <- nb_target_init;	
	
	// Ground Information Data
	
	list target_map;
	list attraction_matrix;	
	list ground_surveillance_mat;
	bool is_batch <- false;
	
	//Index
	
	int IS_SURVEYED <- 0;
	int IS_ALLOCATED <- 1;
	int NO_OF_TARGETS <- 2;
	int SURVEILLANCE_MAT_HEIGHT <- 3;
	
	action initialise_matrix_2d(list init_matrix,int cell_size){
		loop i from:0 to:cell_size-1{
			list row;
			loop j from:0 to:cell_size-1{
					add 0 to:row;
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
		
		if(TARGET_MODE!="Static"){
			return (no_of_target=0);
		}
		loop i from:0 to:cell_side-1{
			loop j from:0 to:cell_side-1{
					if(ground_surveillance_mat[i][j][IS_SURVEYED]=0.0){
						return false;
					}
			}
		}
		
		return true;
		
	}
	
	reflex end_stimuation when:(is_surveillance_complete() or cur_energy_consumed>=1.0 )and !is_batch{
		
		do pause();
	
	}
	
	reflex update_info{
		
		float netBatterylife <- 0.0;
		list<drone> drone_list;
		if(DRONE_ALGO_MODE='Firefly Algorithm'){
			drone_list <- list(drone_FA);
		}else if (DRONE_ALGO_MODE='Back N Fro'){
			drone_list <- list(drone_BF);
		}else if(DRONE_ALGO_MODE='Random'){
			drone_list <- list(drone_RND);	
		}else{
			do error;
		}
		loop droneModel over:drone_list{
			netBatterylife <- netBatterylife+1.0-droneModel.batteryLife;
		}
		cur_energy_consumed <-netBatterylife/no_of_drones;
	
	}
	
	init{	
				
		do initialise_matrix_2d(target_map,cell_side);
		do initialise_matrix_3d(ground_surveillance_mat,cell_side,SURVEILLANCE_MAT_HEIGHT);
		do initialise_matrix_3d(attraction_matrix,major_cell_side,no_of_drones);
		
		if(DRONE_ALGO_MODE='Firefly Algorithm'){
			create drone_FA number:nb_drones_init;	
		}else if (DRONE_ALGO_MODE='Back N Fro'){
			create drone_BF number:nb_drones_init;
		}else if(DRONE_ALGO_MODE='Random'){
			create drone_RND number:nb_drones_init;	
		}else{
			do error;
		}
		
		if (TARGET_MODE="Static"){
			create target_static number:nb_target_init;	
		}else if(TARGET_MODE="Dnyamic"){
			create target_Dynamic number:nb_target_init;
		}else if (TARGET_MODE="Sourced"){
			create target_src number:nb_target_init;	
		}else{
			do error;
		}
		
	}
}

species utility{
	
	action put_in_matrix_2d(list target_matrix,int target_grid_X,int target_grid_Y,int value){
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
	
	int get_lock_period(point drone_location,int grid_x,int grid_y,float drone_speed){
		float distance_bet_points <- get_distance(drone_location,grid_x,grid_y);
		float time_req <- get_time_req(distance_bet_points,drone_speed);
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
	
	bool is_cell_surveyed(int grid_X,int grid_Y){
		return bool(ground_surveillance_mat[grid_X][grid_Y][IS_SURVEYED]!=0.0);
	}
	
	bool is_surveillance_complete{
		if(TARGET_MODE!="Static"){
			return false;
		}
		loop i from:0 to:cell_side-1{
			loop j from:0 to:cell_side-1{
					if(ground_surveillance_mat[i][j][IS_SURVEYED]=0.0){
						return false;
					}
			}
		}
		return true;
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
	
	bool is_valid_cell(int minor_grid_x,int minor_grid_y){
		return ( ( minor_grid_x > -1 and minor_grid_x < cell_side ) and ( minor_grid_y > -1 and minor_grid_y < cell_side ) );
	}
	
}

species target parent:utility {
	
	image_file target_icon <- image_file("../includes/TargetLogo.png");
	
	int cur_grid_X;
	int cur_grid_Y;
	
	aspect icon {
		
		draw target_icon size: 2;
		
	}
	
	action add_target_map(int target_grid_X,int target_grid_Y){
		
		int cur_target <- int(target_map[target_grid_X][target_grid_Y]);
		do put_in_matrix_2d(target_map,target_grid_X,target_grid_Y,cur_target+1);
	
	}
	
	action remove_target_map(int target_grid_X,int target_grid_Y){
	
		int cur_target <- int(target_map[target_grid_X][target_grid_Y]);
		do put_in_matrix_2d(target_map,target_grid_X,target_grid_Y,cur_target-1);
	
	}
	
	action move_to(int target_grid_X,int target_grid_Y){
	
		do remove_target_map(cur_grid_X,cur_grid_Y);
		do add_target_map(target_grid_X,target_grid_Y);
		cur_grid_X <- target_grid_X;
		cur_grid_Y <- target_grid_Y;
		location <- get_minor_grid_location(cur_grid_X,cur_grid_Y);
	
	}
	
	action get_rescued{
	
		do remove_target_map(cur_grid_X,cur_grid_Y);
		int no_target_at_cell <- int(ground_surveillance_mat[cur_grid_X][cur_grid_Y][NO_OF_TARGETS]);
		do put_in_matrix_3d(ground_surveillance_mat,cur_grid_X,cur_grid_Y,NO_OF_TARGETS,float(no_target_at_cell+1));
		no_of_target<-no_of_target-1;
		cur_avg_target_hit_time <- (nb_target_init=no_of_target)?(INIFINITY):(cycle*step)/(nb_target_init-no_of_target);
		do die();
	
	}
	
}

species target_static parent:target {
	
	init{
		
		ground_cell target_cell <- one_of(ground_cell);
		cur_grid_X <- target_cell.grid_x;
		cur_grid_Y <- target_cell.grid_y;
		location <- target_cell.location;
		
		do add_target_map(target_cell.grid_x,target_cell.grid_y);
		ask ground_cell{
			do update_color;
		}
		
	}
	
}

species target_Dynamic parent:target {
	
	list<point> all_directions <- [
		
		{0,0,0},
		{0,1,0}, 		//N
		{1,-1,0},		//NW
		{-1,0,0},		//W
		{-1,-1,0},		//WS
		{0,-1,0},		//S
		{1,-1,0},		//SE
		{1,0,0},		//E
		{1,1,0}			//NE
		
	];
	
	point cur_direction <- {0,0,0};
	float direction_change_probabilty <- 0.20;
	
	reflex basic_move {
		
		if (flip(direction_change_probabilty)){
			cur_direction <- one_of(all_directions);	
		}
		
		point new_grid_cell <- {cur_grid_X,cur_grid_Y,0} + cur_direction;
		
		loop while:(!is_valid_cell(int(new_grid_cell.x),int(new_grid_cell.y))){
			cur_direction <- one_of(all_directions);
			new_grid_cell <- {cur_grid_X,cur_grid_Y} + cur_direction;
		}
		
		do move_to(int(new_grid_cell.x),int(new_grid_cell.y));

		ask ground_cell{
			do update_color;
		}
		
	}
	
	init{
		
		ground_cell target_cell <- one_of(ground_cell);
		cur_grid_X <- target_cell.grid_x;
		cur_grid_Y <- target_cell.grid_y;
		location <- target_cell.location;
		
		do add_target_map(target_cell.grid_x,target_cell.grid_y);
		ask ground_cell{
			do update_color;
		}
		
	}
}

species target_src parent:target {
	
	list<point> all_directions <- [
		{0,0,0},
		{0,1,0}, 		//N
		{1,-1,0},		//NW
		{-1,0,0},		//W
		{-1,-1,0},		//WS
		{0,-1,0},		//S
		{1,-1,0},		//SE
		{1,0,0},		//E
		{1,1,0}			//NE
	];
	
	point cur_direction <- {0,0,0};
	float direction_change_probabilty <- 0.40;
	
	reflex basic_move {
		
		if (flip(direction_change_probabilty)){
			cur_direction <- one_of(all_directions);	
		}
		
		point new_grid_cell <- {cur_grid_X,cur_grid_Y,0} + cur_direction;
		
		loop while:(!is_valid_cell(int(new_grid_cell.x),int(new_grid_cell.y))){
			cur_direction <- one_of(all_directions);
			new_grid_cell <- {cur_grid_X,cur_grid_Y} + cur_direction;
		}
		
		do move_to(int(new_grid_cell.x),int(new_grid_cell.y));

		ask ground_cell{
			do update_color;
		}
		
	}
	
	init{
		
		cur_grid_X <- int(cell_side/2);
		cur_grid_Y <- int(cell_side/2);
		
		location <- get_minor_grid_location(cur_grid_X,cur_grid_Y);
		
		do add_target_map(cur_grid_X,cur_grid_Y);
		ask ground_cell{
			do update_color;
		}
		
	}
}

species drone parent:utility skills:[moving]{
	
	int droneID;
	
	int speedLockPeriod;
	int surveyLockPeriod;
	
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
		speedLockPeriod <- get_lock_period(location,grid_x,grid_y,speed);
		surveyLockPeriod <- int(surveyTime);
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
	
	// Return {bool,point} false -> turn off or dont move
	action select_best_cell virtual:true type:point;
	
	action survey_nxt_cell{
		point best_cell <-select_best_cell();
		target_grid_X<-int(best_cell.x);
		target_grid_Y<-int(best_cell.y);
		target_location <- get_minor_grid_location(target_grid_X,target_grid_Y);
		do allocate_cell(target_grid_X,target_grid_Y);
		do goto_minor_cell(target_grid_X,target_grid_Y);
	}	
		
	// TODO:: Test Locking Mechanism on small cell size.
	reflex move when:(!is_surveillance_complete()) {	
		
		if(TARGET_MODE="Static"){
			ask target_static at_distance(rescue_sensitivity){
				do get_rescued();
				myself.targetRescued<-myself.targetRescued+1;
			}
		}else if(TARGET_MODE="Dnyamic"){
			ask target_Dynamic at_distance(rescue_sensitivity){
				do get_rescued();
				myself.targetRescued<-myself.targetRescued+1;
			}
		}else if(TARGET_MODE="Sourced"){
			ask target_src at_distance(rescue_sensitivity){
				do get_rescued();
				myself.targetRescued<-myself.targetRescued+1;
			}
		}else{
			do error;
		}
			
		if(speedLockPeriod>0){
			speedLockPeriod<-speedLockPeriod-1;
			batteryLife <- batteryLife - (speed/float(droneBatteryCapacity));
			do goto(target_location);
		}else if(surveyLockPeriod>0){
				surveyLockPeriod <- surveyLockPeriod-1;
				if(surveyLockPeriod=0){
					do deallocate_cell(target_grid_X,target_grid_Y);
					do mark_cell_surveyed(target_grid_X,target_grid_Y);
				}				
		}else{
			do survey_nxt_cell();
			batteryLife <- batteryLife - (1/float(droneBatteryCapacity));
		}

		ask ground_cell{
			do update_color;
		}
	}
	
	init{
		speedLockPeriod<-0;
		surveyLockPeriod<-0;
		speed<-5.0;
		surveyTime<-2.5;
		batteryLife<-1.0;
		targetRescued<-0;
		droneID<-length(allDrones);
		if(DRONE_ORIGIN_MODE="Sourced"){
			location<-get_minor_grid_location(int(cell_side/2),int(cell_side/2));
		}	
		add self to:allDrones;
	}
	
}

// TODO::Implement different Algo for dynamic Targets
species drone_FA parent:drone{
	float rand_factor <- 0.15;
	
	// TODO:: Make sure empty major cells have less attraction over other target present major cells
	float return_attraction_value(drone given_drone,int major_grid_X,int major_grid_Y){
		point drone_location <- given_drone.location;
		point major_cell_location <- get_major_grid_location(major_grid_X,major_grid_Y);
		float distance_bet <- get_distance_bt_points(drone_location,major_cell_location);
		float battery_life <- given_drone.batteryLife;
		int reputation <- 1+given_drone.targetRescued;
		float surveillance_factor <- get_surveyed_percentage(major_grid_X,major_grid_Y);
		
		float attraction_power <- (distance_bet^(0.5))*(rnd(1))/(battery_life*reputation);
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
	
	point select_best_cell {
		do update_attraction_mat(self);
		
		// TODO::Clean it up !
		if(flip(rand_factor)){
			return { minor_cell_side*rnd(major_cell_side-1)+rnd(minor_cell_side-1),minor_cell_side*rnd(major_cell_side-1)+rnd(minor_cell_side-1)};		
		}
		
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
				if((TARGET_MODE!="Static" or int(ground_surveillance_mat[grid_row][grid_col][IS_SURVEYED])=0) and int(ground_surveillance_mat[grid_row][grid_col][IS_ALLOCATED])=0){
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
}

species drone_BF parent:drone{
	
	bool fr_direction <- true;
		
	int return_dir{
		return (fr_direction)?(1):(-1);
	}
	
	point select_best_cell{
		
		point curLocation <- self.location;
		
		int grid_X <- int(curLocation.x/(ground_side/cell_side));
		int grid_Y <- int(curLocation.y/(ground_side/cell_side));
		
		int best_grid_X <- grid_X;
		int best_grid_Y <- grid_Y;
		
		if( grid_X<cell_side-1 and ((fr_direction and  mod(grid_Y,2)=0) or (!fr_direction and grid_X<cell_side-1 and mod(grid_Y,2)=1)) ) {
			best_grid_X <- grid_X+1;
			best_grid_Y <- grid_Y;
		}else if(grid_X>0 and ((fr_direction and mod(grid_Y,2)=1) or (!fr_direction and mod(grid_Y,2)=0)) ){
			best_grid_X <- grid_X-1;
			best_grid_Y <- grid_Y;
		}else if(fr_direction and grid_Y<cell_side-1 and ((grid_X=cell_side-1 and mod(grid_Y,2)=0) or (grid_X=0 and mod(grid_Y,2)=1)) ){
			best_grid_X <- grid_X;
			best_grid_Y <- grid_Y+1;
		}else if(!fr_direction and grid_Y>0 and ((grid_X=0 and mod(grid_Y,2)=0) or (grid_X=cell_side-1 and mod(grid_Y,2)=1)) ){
			best_grid_X <- grid_X;
			best_grid_Y <- grid_Y-1;
		}
		
		bool is_surveyed <- is_cell_surveyed(best_grid_X,best_grid_Y);
		
		if (is_surveyed){
			fr_direction <- false;
		}
		
		return {best_grid_X,best_grid_Y};
	}
	
}

species drone_RND parent:drone{
	
	point select_best_cell{
		
		if(TARGET_MODE!="Static"){
			return {rnd(cell_side-1),rnd(cell_side-1)};
		}
		
		loop times:cell_side^2{
			int i <- rnd(cell_side-1);
			int j <- rnd(cell_side-1);
			if(int(ground_surveillance_mat[i][j][IS_SURVEYED])=0 and int(ground_surveillance_mat[i][j][IS_ALLOCATED])=0){
				return {i,j};
			}
		}
		return {rnd(cell_side-1),rnd(cell_side-1)};		
	}
	
}


grid ground_cell height:cell_side width:cell_side neighbors:4 {
	
	action update_color{
		if(TARGET_MODE="Static" and ground_surveillance_mat[grid_x][grid_y][IS_SURVEYED]=1.0){
			color <- #green;
		}else if(int(target_map[grid_x][grid_y])>0){
			color <- #blue;
		}else{
			color <- #grey;
		}
		
	}
	
}

experiment SAR_simualtion type:gui{
	
	parameter "Ground Side" category:"Ground Parameters" var: ground_side min:10 max:1000 step:5; 
	parameter "minor cell side" category:"Ground Parameters" var: minor_cell_side min:2 max:100 step:5;
	parameter "major cell side" category:"Ground Parameters" var: major_cell_side min:2 max:100 step:5;
	
	parameter "Drones" category:"Drone Parameters" var: nb_drones_init min:1 max:1000 step:5;
	parameter "Drone Origin Mode" category:"Drone Parameters" var: DRONE_ORIGIN_MODE init:"Sourced" among:["Random","Sourced"];
	parameter "Drone Algo Mode" category:"Drone Parameters" var: DRONE_ALGO_MODE init:"Firefly Algorithm" among:["Firefly Algorithm","Back N Fro","Random"];
	parameter "Battery Capacity" category:"Drone Parameters" var: droneBatteryCapacity min:10 max:3000 step:5;
	parameter "Rescue Sensitivity" category:"Drone Parameters" var: rescue_sensitivity min:0.5 max:100.0 step:0.5;
	
	parameter "Targets" category:"Target Parameters" var: nb_target_init min:1 max:1000 step:5;
	parameter "Target Mode" category:"Target Parameters" var: TARGET_MODE init:"Sourced" among:["Static","Dnyamic","Sourced"];
	
	output{
		display ground_display{
			
			grid ground_cell border:#black;

			species drone_FA aspect:icon;
			species drone_BF aspect:icon;
			species drone_RND aspect:icon;
			
			species target_static aspect:icon;
			species target_Dynamic aspect:icon;
			species target_src aspect:icon;
			
		}
		display Parameters_Information refresh: every(2#cycles){
			chart "Targets and Battery Remaining" type: series size: {1,0.5} position: {0, 0} {
				data "% of target Rescued" value: no_of_target/nb_target_init color:(#blue);
				data "% of Energy Consumed" value: cur_energy_consumed color:(#green);
			}
			chart "Average Hit Time" type: series size: {1,0.5} position: {0, 0.5} {
				data "Average Hit Time" value: cur_avg_target_hit_time color: #red;
			}
		}
	}
}


experiment Batch_Stimualtions type: batch keep_seed:false repeat:1 until:(no_of_target=0 or cur_energy_consumed>=1.0){
	
//	parameter "Ground Side" category:"Ground Parameters" var: ground_side min:100 max:500 step:200; 
//	parameter "minor cell side" category:"Ground Parameters" var: minor_cell_side min:3 max:5 step:1;
//	parameter "major cell side" category:"Ground Parameters" var: major_cell_side min:4 max:8 step:2;
	
	parameter "Drones" category:"Drone Parameters" var: nb_drones_init min:10 max:30 step:10;
	parameter "Drone Origin Mode" category:"Drone Parameters" var: DRONE_ORIGIN_MODE init:"Sourced" among:["Random","Sourced"];
	parameter "Drone Algo Mode" category:"Drone Parameters" var: DRONE_ALGO_MODE init:"Firefly Algorithm" among:["Firefly Algorithm","Back N Fro","Random"];
//	parameter "Battery Capacity" category:"Drone Parameters" var: droneBatteryCapacity min:1000 max:3000 step:1000;
//	parameter "Rescue Sensitivity" category:"Drone Parameters" var: rescue_sensitivity min:0.5 max:1.0 step:0.5;
	
	parameter "Targets" category:"Target Parameters" var: nb_target_init min:10 max:40 step:10;
	parameter "Target Mode" category:"Target Parameters" var: TARGET_MODE init:"Sourced" among:["Static","Dnyamic","Sourced"];
	
	parameter "Batch mode:" var: is_batch <- true;
	
	reflex save_results_explo {
		ask simulations {
			save [int(self),DRONE_ORIGIN_MODE,DRONE_ALGO_MODE,TARGET_MODE,nb_drones_init,nb_target_init,no_of_target,minor_cell_side,major_cell_side,droneBatteryCapacity,rescue_sensitivity,cur_energy_consumed,cur_avg_target_hit_time] 
		   		to: "results_5.csv" type: "csv" rewrite: (int(self) = 0) ? true : false header: true;
		}		
	}
}
