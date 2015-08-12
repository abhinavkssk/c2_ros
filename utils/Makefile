include Makefile.inc

RHOST = 172.16.177.175

#upload the local mission file to the remote machine
upload-mission:
	$(UPLOAD_MISSION)

# remove all the mission files in the remote machine
clean-rmission:
	$(RLOGIN) "cd $(RPATH_MISSIONDIR); rm *"

# list remote ros service
rosservice-list:
	$(RLOGIN) "$(RSOURCE_ROS); rosservice list"

# send start command to Captain in the remote machine
start:
	$(RLOGIN) "$(RSOURCE_ROS); rosservice call /CAPTAIN \"command: 0\""

abort:
	$(RLOGIN) "$(RSOURCE_ROS); rosservice call /CAPTAIN \"command: 1\""

abort-home:
	$(RLOGIN) "$(RSOURCE_ROS); rosservice call /CAPTAIN \"command: 2\""

abort-to-start:
	$(RLOGIN) "$(RSOURCE_ROS); rosservice call /CAPTAIN \"command: 3\""