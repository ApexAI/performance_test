#!/bin/bash

set -e

# help
if [ "$1" = "--help" ] || [ -z "$1" ] ; then
  echo "Usage: enable_security <root_directory_to_keys> <enable/disable>"
  echo "Example usage: "
  echo "echo enable_security '~/ApexOS/apex_ws/demo_keys' enable"
  exit 0
fi


if [ $2 == 'enable' ]
then 
  echo 'enabling security'

  max_nodes=5

  source ~/ApexOS/apex_ws/install/setup.bash  

  # create keys
  ros2 security create_keystore "$1"

  for ((i=0; i<=$max_nodes; i++))
  do
    node=performance_test$i
    echo "generating keys for "$node
    echo $1
    ros2 security create_key "$1" "$node"
    ros2 security create_permission "$1" "$node" policies.yaml;  
    echo " "
  done 

  #enable security
  export ROS_SECURITY_ENABLE=true 
  export ROS_SECURITY_STRATEGY=Enforce 
  export ROS_SECURITY_ROOT_DIRECTORY=$1 
  source ~/ApexOS/apex_ws/install/setup.bash
else 
  echo 'diabling security'
  #disable security
  unset ROS_SECURITY_ENABLE 
  unset ROS_SECURITY_STRATEGY 
  unset ROS_SECURITY_ROOT_DIRECTORY
  echo 'security disabled'
fi



