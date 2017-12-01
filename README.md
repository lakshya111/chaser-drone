# chaser-drone
		Project - Lakshya Priyadarshi : official.18lakshya@gmail.com
			  B.Tech 3rd Semester, Computer Science & Engineering
			  Institute of Engineering & Technology, Lucknow
		

		Overview- Chaser Drone based on ROS, ROS Whycon technology, simulated in Gazebo real-world physics simulator
			  Whycon employs flood-filling image-processing algorithm to detect markers
			  Proportional–integral–derivative control algorithm employed to control drone motion
	
			  Contributors : Lekhraj Singh (ECE), Yaduveer Singh (EE)

		Design -  Robot simulation in real-world environment simulator Gazebo
			  Codes + Simulator are integrated with ROS Core ( Robotics Operating System ) 
			  All nodes are published under Subscriber/Publisher scheme of ROScore / ROSTopic
			  The robot motion parameters are six-dimensional phase space [x,y,z,p_x,p_y,p_x]
			  Algorithm applies feedback control to linear velocity [x,y,z] and angular velocity [th_x, th_y, th_z]
			  Proportional–integral–derivative control algorithm employed to control drone motion				
        
A proportional–integral–derivative controller is a control loop feedback mechanism widely used in industrial control systems and applications requiring continuously modulated control. A PID controller continuously calculates an error value e ( t ) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name. In practical terms it automatically applies accurate and responsive correction to a control function. 
