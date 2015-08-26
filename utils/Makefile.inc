#basic settings

BASEDIR ?= .
SHELL ?= /bin/sh
MISSIONDIR = $(BASEDIR)/Missions
MISSIONFILE = $(MISSIONDIR)/mission.txt
SCP = scp
RSH = ssh
RUSER = odroid
RLOGIN = $(RSH) $(RUSER)@$(RHOST)
RPATH_MISSIONDIR = /home/$(RUSER)/Missions
UPLOAD_MISSION = $(SCP) $(MISSIONFILE) $(RUSER)@$(RHOST):$(RPATH_MISSIONDIR)

SOURCE = source
RPATH_C2 = /home/$(RUSER)/git/ws/devel/setup.bash
RSOURCE_ROS = $(SOURCE) /opt/ros/indigo/setup.bash; $(SOURCE) $(RPATH_C2)
