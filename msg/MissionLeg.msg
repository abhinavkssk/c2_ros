C2_BHV m_bhv				#Mission Behavior
geometry_msgs/Pose2D m_pt	#UTM Frame: x->easting, y->northing, theta->heading
float32 altdepth 			#Altitude for air vehicle, Depth for underwater vehicle, both +ve value
float32 m_pt_radius			#radius within where the m_pt is considered reached, in meter
float32 desired_speed		#desired speed for the vehicle, in meter per second
KeyValue[] props 			#key/value properties for this m_pt