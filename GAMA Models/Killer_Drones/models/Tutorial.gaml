model my_grid_model

global{
    float max_range <- 5.0;
    int number_of_agents <- 5;
    init {
    create my_species number: number_of_agents;
    }
    
    reflex update {
    ask my_species {
        do wander amplitude: 180.0; 
        ask my_grid at_distance(max_range) {
        if(self overlaps myself) {
            self.color_value <- 2;
        } else if (self.color_value != 2) {
            self.color_value <- 1;
        }
        }
    }
    ask my_grid {
        do update_color;
    }   
    }
}

species my_species skills:[moving] {
    float speed <- 2.0;
    aspect default {
    draw circle(1) color: #blue;
    }
}

grid my_grid width:30 height:30 {
    int color_value <- 0;
    action update_color {
    if (color_value = 0) {
        color <- #green;
    } else if (color_value = 1) {
        color <- #yellow;
    } else if (color_value = 2) {
        color <- #red;
    }
    color_value <- 0;
    }
}

experiment MyExperiment type: gui {
    output {
        display MyDisplay type: java2D {
            grid my_grid lines: #black;
            species my_species aspect: default; 
        }
    }
}