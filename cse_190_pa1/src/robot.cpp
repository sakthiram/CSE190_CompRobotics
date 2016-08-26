#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include "read_config.cpp"
#include <math.h>
#include <iostream>
#include <list>
#include "cse_190_assi_1/moveService.h"
#include "cse_190_assi_1/requestTexture.h"
#include "cse_190_assi_1/RobotProbabilities.h"
#include "cse_190_assi_1/temperatureMessage.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"


std::vector<std::vector<int> > motions;
std::vector<std::vector<float> > heat_map;
std::vector<std::vector<std::string> > pipe_map;
std::vector<std::vector<float> > probability_matrix;
Json::Value config; 
bool end_sim;
int motion_index;

ros::Subscriber tempSub;
ros::ServiceClient textClient;
ros::ServiceClient moveClient;
ros::Publisher temp_activator_pub;
ros::Publisher temp_res_pub;
ros::Publisher tex_res_pub;
ros::Publisher prob_res_pub;
ros::Publisher shutdown_pub;
//using namespace std;

void Print (const std::vector<std::vector<float>>& v){
  for (int i=0; i<v.size();i++){
  for (int j=0; j<v[0].size();j++){
    std::cout << v[i][j] << " ";
  }
    std::cout << std::endl;
  }
    std::cout << std::endl;
}

void Print (const std::vector<std::vector<int>>& v){
  for (int i=0; i<v.size();i++){
  for (int j=0; j<v[0].size();j++){
    std::cout << v[i][j] << " ";
  }
    std::cout << std::endl;
  }
    std::cout << std::endl;
}

void initialize_maps() {
        float temp_cold = 20.0;
        float temp_hot  = 40.0;
        float temp_warm = 25.0;

	pipe_map.resize(config["pipe_map"].size());
        for (int i=0; i<config["pipe_map"].size(); i++) {
	pipe_map[i].resize(config["pipe_map"][0].size());
            for (int j=0; j<config["pipe_map"][0].size(); j++) {
		pipe_map[i][j] = config["pipe_map"][i][j].asString();  
	    }
	}

	heat_map.resize(config["pipe_map"].size());
        for (int i=0; i<pipe_map.size(); i++) {
	heat_map[i].resize(config["pipe_map"][0].size());
            for (int j=0; j<pipe_map[0].size(); j++) {
                if (pipe_map[i][j] == "C")
                    heat_map[i][j] = temp_cold;
                else if (pipe_map[i][j] == "H")
                    heat_map[i][j] = temp_hot;
                else
                    heat_map[i][j] = temp_warm;
	    }
	}	

}

void initialize_beliefs() {
	float init_prob = 1.0/(float((config["texture_map"]).size())*float((config["texture_map"][0]).size()));
	probability_matrix.resize(config["pipe_map"].size());
        for (int i=0; i<pipe_map.size(); i++) {
	probability_matrix[i].resize(config["pipe_map"][0].size());
            for (int j=0; j<pipe_map[0].size(); j++) {
		probability_matrix[i][j] = init_prob;
	    }
	}
}

void update_prob_after_temp_measurement(float temp_reading) {
	std::vector<std::vector<float> > temp_probability_matrix(probability_matrix);
	float total_prob = 0.0;
	
        for (int i=0; i<probability_matrix.size(); i++) {
            for (int j=0; j<probability_matrix[0].size(); j++) {
		float prob_hit;
		if (config["temp_noise_std_dev"] == 0)
		   prob_hit = (heat_map[i][j] == temp_reading);
		else 
		   prob_hit = (1/(sqrt(2*M_PI)*config["temp_noise_std_dev"].asFloat()))*exp(-0.5*(pow(float(heat_map[i][j]-temp_reading),2.0)/pow(config["temp_noise_std_dev"].asFloat(),2.0))); 

		temp_probability_matrix[i][j] = prob_hit*probability_matrix[i][j];
		total_prob += temp_probability_matrix[i][j];
	    }
	}
  std::cout << "Update Temp Measurement " << motion_index << std::endl;
	Print(temp_probability_matrix);
	
        if (total_prob == 0)
            std::cout << "sum of probabilities is zero \n";
        else {
           for (int i=0; i<probability_matrix.size(); i++) {
               for (int j=0; j<probability_matrix[0].size(); j++) {
                  probability_matrix[i][j] = temp_probability_matrix[i][j]/total_prob;
	       }
	   }
	}

  std::cout << "Update Temp Measurement NORMALIZED" << motion_index << std::endl;
	Print(probability_matrix);
}

void update_prob_after_texture_measurement(std::string texture_reading) {
	std::vector<std::vector<float> > temp_probability_matrix(probability_matrix);
	float total_prob = 0.0;
	
        for (int i=0; i<probability_matrix.size(); i++) {
            for (int j=0; j<probability_matrix[0].size(); j++) {
		int hit;
		   hit = int(config["texture_map"][i][j] == texture_reading);
		   temp_probability_matrix[i][j] = (config["prob_tex_correct"].asFloat()*hit+(1-config["prob_tex_correct"].asFloat())*(1-hit))*probability_matrix[i][j];
		   total_prob += temp_probability_matrix[i][j];
	    }
	}
  std::cout << "Update Texture Measurement " << motion_index << std::endl;
	Print(temp_probability_matrix);
	
        if (total_prob == 0)
            std::cout << "sum of probabilities is zero \n";
        else {
           for (int i=0; i<probability_matrix.size(); i++) {
               for (int j=0; j<probability_matrix[0].size(); j++) {
                  probability_matrix[i][j] = temp_probability_matrix[i][j]/total_prob;
	       }
	   }
	}
  std::cout << "Update Texture Measurement NORMALIZED" << motion_index << std::endl;
	Print(probability_matrix);
}

void move_and_update(std::vector<int> curr_motion) {

	cse_190_assi_1::moveService srv;

	//srv.request.move = curr_motion;
	//std::copy(curr_motion.begin(), curr_motion.end(), srv.request.move);
	srv.request.move[0] = curr_motion[0];
	srv.request.move[1] = curr_motion[1];

	if (moveClient.call(srv)) {
  	     ROS_INFO("Succesfully called service moveService");
	} else {
  	     ROS_ERROR("Failed to call service moveService");
	}


	std::vector<std::vector<float> > temp_probability_matrix(probability_matrix);
	float total_prob = 0.0;
	int num_rows = probability_matrix.size();
	int num_cols = probability_matrix[0].size();

        for (int i=0; i<probability_matrix.size(); i++) {
            for (int j=0; j<probability_matrix[0].size(); j++) {
                   std::vector<std::vector<int>> possible_starts {{(i-1), j},{i, j},{(i+1), j},{i, (j-1)},{i, (j+1)}};

		   std::vector<int> correct_start = {(i-curr_motion[0]), (j-curr_motion[1])};
                   std::vector<std::vector<int> > incorrect_starts(possible_starts);
		   incorrect_starts.erase(std::remove(incorrect_starts.begin(), incorrect_starts.end(), correct_start),incorrect_starts.end());

  //std::cout << "i=" << i << " j=" << j << " Curr Motion: " << curr_motion[0] << " , " << curr_motion[1] << std::endl;
  //std::cout << "Correct Start: " << correct_start[0] << " , " << correct_start[1] << std::endl;
  //std::cout << "Incorrect Starts" << std::endl;
  //		   Print(incorrect_starts);
			   int correct_start_row = ((correct_start[0])%num_rows + num_rows)%num_rows;
			   int correct_start_col = ((correct_start[1])%num_cols + num_cols)%num_cols;
			
		   temp_probability_matrix[i][j] = (config["prob_move_correct"].asFloat())*probability_matrix[correct_start_row][correct_start_col];
		   for(int k=0; k<incorrect_starts.size(); k++) { 
			   int incorrect_start_row = ((incorrect_starts[k][0])%num_rows + num_rows)%num_rows;
			   int incorrect_start_col = ((incorrect_starts[k][1])%num_cols + num_cols)%num_cols;
			   //temp_probability_matrix[i][j] += ((1-(config["prob_move_correct"].asFloat()))/4)*temp_probability_matrix[incorrect_start_row][incorrect_start_col];
			   temp_probability_matrix[i][j] += ((1-(config["prob_move_correct"].asFloat()))/4)*probability_matrix[incorrect_start_row][incorrect_start_col];
		   }

		   total_prob += temp_probability_matrix[i][j];
	    }
	}
  std::cout << "Update Move Prob " << motion_index << std::endl;
	Print(temp_probability_matrix);	

        if (total_prob == 0)
            std::cout << "sum of probabilities is zero " << std::endl;
        else {
           for (int i=0; i<probability_matrix.size(); i++) {
               for (int j=0; j<probability_matrix[0].size(); j++) {
                  probability_matrix[i][j] = temp_probability_matrix[i][j]/total_prob;
	       }
	   }
	}
	
  std::cout << "Update Move Prob NORMALIZED" << motion_index << std::endl;
	Print(temp_probability_matrix);	
}

void on_temp_topic(const cse_190_assi_1::temperatureMessage::ConstPtr& msg) {
	
	// Read Temperature from topic 
	float temp_reading = msg->temperature;
  std::cout << "temp reading = " << temp_reading << std::endl;


	// Update prob after temperature measurement
	update_prob_after_temp_measurement(temp_reading);
	std_msgs::Float32 temp_res_msg;
	temp_res_msg.data = temp_reading; 
	temp_res_pub.publish(temp_res_msg);


	//Send service request to read texture
	std::string texture_reading;
	cse_190_assi_1::requestTexture srv;
	if (textClient.call(srv)) {
	     texture_reading = srv.response.data;
	} else {
  	     ROS_ERROR("Failed to call service requestTexture");
	}

  std::cout << "texture reading = " << texture_reading << std::endl;
	// Update prob after texture measurement
	update_prob_after_texture_measurement(texture_reading);
	std_msgs::String tex_res_msg;
	tex_res_msg.data = texture_reading; 
	tex_res_pub.publish(tex_res_msg);

        move_and_update(motions[motion_index++]);

	cse_190_assi_1::RobotProbabilities prob_msg;
	prob_msg.data.resize(probability_matrix.size()*probability_matrix[0].size());
	for(int i=0; i<probability_matrix.size(); i++) {
	   for(int j=0; j<probability_matrix[0].size(); j++) {
	   	   prob_msg.data[i*probability_matrix[0].size()+j] = probability_matrix[i][j];
	   }
	}
        //std::cout << "Final Probability Matrix \n" << probability_matrix << "\n";        
	prob_res_pub.publish(prob_msg);

        if (motion_index == motions.size()) {
        std::cout << "Motion complete\n";
  std::cout << "Motion index " << motion_index << std::endl;
	    end_sim = true; 
	}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;
  config = read_config();
  std::cout << "Read Config Done" << std::endl;

  end_sim = false;
  motion_index = 0;
  //////////////////////////////////////////////
  // Define Publishers, Subscribers & Clients //
  //////////////////////////////////////////////

  // Subscribing for temperature topic
  tempSub = n.subscribe("/temp_sensor/data", 1, on_temp_topic);

 
  // Client for requestTexture service 
  textClient = n.serviceClient<cse_190_assi_1::requestTexture>("requestTexture");

  // Client for moveService 
  moveClient = n.serviceClient<cse_190_assi_1::moveService>("moveService");

  // Publisher for /temp_sensor/activation 
  temp_activator_pub = n.advertise<std_msgs::Bool>("/temp_sensor/activation", 10);

  // Publisher for  /results/temperature_data
  temp_res_pub = n.advertise<std_msgs::Float32>("/results/temperature_data", 10);

  // Publisher for  /results/texture_data
  tex_res_pub = n.advertise<std_msgs::String>("/results/texture_data", 10);

  // Publisher for  /results/probabilities
  prob_res_pub = n.advertise<cse_190_assi_1::RobotProbabilities>("/results/probabilities", 10);

  // Publisher for  /map_node/sim_complete
  shutdown_pub = n.advertise<std_msgs::Bool>("/map_node/sim_complete", 10);


  ////////////////////////////////////////////
  // Initialize heat maps & initial beliefs //
  ////////////////////////////////////////////

  initialize_maps();
  initialize_beliefs();

  std::cout << "Init maps & Beliefs Done" << std::endl;

  motions.resize(config["move_list"].size());
  //motions = config["move_list"];
  for (int i=0; i<config["move_list"].size(); i++) {
  	motions[i].resize(2);
  	motions[i][0] = config["move_list"][i][0].asInt();  
  	motions[i][1] = config["move_list"][i][1].asInt();  
  }

  std::cout << "Init Done" << std::endl;

  ros::Duration(2).sleep();

  ////////////////////// //////////////
  // Publish temp_activator_pub msg //
  ////////////////////////////////////

  std_msgs::Bool activation_msg;
  activation_msg.data = true;
  temp_activator_pub.publish(activation_msg);
  //ros::spin();
  ros::AsyncSpinner spinner(2);
  spinner.start();	

  while(end_sim == false) {
	// Run simulation
  ros::Duration(0.5).sleep();
  std::cout<<"Inside while testing End sim called" << std::endl;
  }

  std::cout<<"End sim called" << std::endl;
  
  activation_msg.data = false; 
  temp_activator_pub.publish(activation_msg);

  std_msgs::Bool shutdown_msg;
  shutdown_msg.data = true;
  shutdown_pub.publish(shutdown_msg);
  ros::Duration(1).sleep();
  ros::shutdown();

/*

  //ROS_INFO("WHEE");
  //ros::Duration(10).sleep();

  while (ros::ok())
  {
    geometry_msgs::Twist twist;   
    twist.linear.x = 10.0;
    twist.angular.z = 0.0;
    velocity_pub.publish(twist);

    ROS_INFO("Velocity: [%f %f]", twist.linear.x, twist.angular.z);

    ros::spinOnce();
  }
*/

  return 0;
}
