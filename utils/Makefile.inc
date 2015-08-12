#basic settings

BASEDIR ?= .
SHELL ?= /bin/sh
MISSIONDIR = $(BASEDIR)/mission
MISSIONFILE = $(MISSIONDIR)/mission.txt
SCP = scp
RSH = ssh
RUSER = william
RLOGIN = $(RSH) $(RUSER)@$(RHOST)
RPATH_MISSIONDIR = /home/william/Missions
UPLOAD_MISSION = $(SCP) $(MISSIONFILE) $(RUSER)@$(RHOST):$(RPATH_MISSIONDIR)

SOURCE = source
RPATH_C2 = /home/william/git/c2/devel/setup.bash
RSOURCE_ROS = $(SOURCE) /opt/ros/indigo/setup.bash; $(SOURCE) $(RPATH_C2)