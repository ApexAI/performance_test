#!/bin/bash


set -e

# help
if [ "$1" = "--help" ] || [ -z "$1" ] || [[ "$1" == "enable" && -z "$2" ]] || [[ "$1" != "disable" && "$1" != "enable" ]]
then
#  ; then
  echo "Usage: setup_security <enable/disable> <root_directory_to_keys>"
  echo "directory to keys is required when security is enabled"
  echo "Example usage: "
  echo "To enable the security: "
  echo "bash setup_security.sh enable '/ApexOS/apex_ws/demo_keys'"
  echo "To disable the security: "
  echo "bash setup_security.sh disable"
  exit 0
fi

if [ "$1" == "enable" ]
then 
  echo 'generating keys'

  max_nodes=5

  source ~/ApexOS/apex_ws/install/setup.bash  

  # create keys
  ros2 security create_keystore "$2"

  for ((i=0; i<=$max_nodes; i++))
  do
    node=performance_test$i
    echo "generating keys for "$node
    ros2 security create_key "$2" "$node"
    ros2 security create_permission "$2" "$node" policies.yaml;  
    echo " "
  done 

  #enable security
  source ~/ApexOS/apex_ws/install/setup.bash
  echo "run this line to enable security"
  echo "export ROS_SECURITY_ENABLE=true && export ROS_SECURITY_STRATEGY=Enforce && export ROS_SECURITY_ROOT_DIRECTORY=$2"
elif [ "$1" == "disable" ]
then 
  echo "run this line to disable security"
  #disable security
  echo "unset ROS_SECURITY_ENABLE && unset ROS_SECURITY_STRATEGY && unset ROS_SECURITY_ROOT_DIRECTORY"
else 
  echo 'incorrect parameter '$2
fi



