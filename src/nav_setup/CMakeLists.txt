cmake_minimum_required(VERSION 2.8.3)
project(nav_setup)
find_package(catkin REQUIRED
        	COMPONENTS
			costmap_2d
			base_local_planner
            dynamic_reconfigure
            nav_core
            ca_msgs
			move_base
			map_server
			urg_node
		    )


