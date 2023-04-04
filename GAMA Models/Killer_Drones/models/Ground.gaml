/**
* Name: Ground
* Based on the internal empty template. 
* Author: aayush
* Tags: 
*/


model Ground

global {
	
	/* Ground Parameters */
	int ground_height <- 50;
	int ground_width <- 50;
	int ground_cell_neighbors <- 4;
	rgb ground_cell_color <- #black;
	rgb ground_cell_lines <- rgb(220,220,220);
	
}

species drone{
	
}

species target{
	
}

grid ground_cell height:ground_height width:ground_width neighbors:ground_cell_neighbors {
	rgb color <- ground_cell_color;
}

experiment sar_simulation type:gui{
	output{
		display ground_display {
			grid ground_cell lines:ground_cell_lines;
		}
	}
}