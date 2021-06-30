#!/bin/bash

echo "Make Sure you are in the OpenFace/build path"
workdir=`pwd`
if ! [[ $workdir =~ OpenFace/build$ ]]; then
    #statements
    echo "Directory wrong"
    exit
fi


# read -p "Enter your video path/folder path:" input_path
# read -p "Enter your output path(default is ./temp_path_processed):" output_path
input_path=$1
output_path=$2

output_path=${output_path:-temp_path_processed}
files=`ls -d $input_path/*`
for i in ${files[@]}; 
do 
    ./bin/FeatureExtraction -f $i -aus -out_dir $output_path
done