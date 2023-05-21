#!/bin/bash

# Reference from https://unix.stackexchange.com/a/505342
helpFunction() {
   echo ""
   echo "Usage: $0 [-f workspace_folder]"
   echo -e "\t-f /path/to/workspace_folder"
   exit 1 # Exit script after printing help
}

while getopts "fh" opt; do
   case "$opt" in
   f) workspace_folder="$OPTARG" ;;
   h) helpFunction ;;
   ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

if [ -z "$workspace_folder" ]; then
   workspace_folder="${HOME}/catkin_ws"
fi

if [[ "$workspace_folder" == */ ]]; then
   workspace_folder=${workspace_folder::-1}
fi

package_folder=${PWD##*/}

mkdir -p ${workspace_folder}/src/${package_folder}

string=$(ls -d */)
array=($(echo "$string" | tr ',' '\n'))

count=0
for i in ${array[@]}; do
    array[$count]=${i::-1}
    rsync -a --delete ${array[$count]} ${workspace_folder}/src/${package_folder}
    count=$((count + 1))
done

cd ${workspace_folder}
catkin clean -y
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
