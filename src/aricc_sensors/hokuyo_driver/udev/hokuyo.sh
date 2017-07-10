#!/bin/sh
. /opt/ros/indigo/setup.sh
. /home/aricc/catkin_ws/devel/setup.sh
id=$(rosrun hokuyo_node getID $1 --)
#id=$(rosrun hokuyo_node getID /dev/ttyACM1 --)
#echo "$id"
if [ "$id" = "H1309383" ] ; then
  echo "rear"
elif [ "$id" = "H1303034" ] ; then
  echo "front"
elif [ "$id" = "H1303037" ] ; then
  echo "rear"
elif [ "$id" = "H1303040" ] ; then
  echo "front"
fi
