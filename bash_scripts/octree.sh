#!/bin/bash

for day in 11
do
    for i in {1..5}
    do 
    resolution=0.$i
    point_resolution=0.001
    root_data_folder=../sample_data
    data_folder=$root_data_folder/$day
    output_folder=octree/$resolution

    echo "running octree compression"
    time ./../octree/build/octree_compression_fast $data_folder $output_folder true 1 $resolution $point_resolution

    echo "running octree decompression"
    time ./../octree/build/octree_decompression_fast $data_folder $output_folder


    echo "running error calculation "
    input_folder_1=$output_folder/decompressed
    input_folder_2=pcds
    
    echo "running chamfer distance calculation "
    time ./../metrics/build/chamferDistance_fast $data_folder $input_folder_1 $input_folder_2 >>  octree_CD.txt

    echo "running PSNR calculation "
    time ./../metrics/build/PSNR_fast $data_folder $input_folder_1 $input_folder_2 >>  octree_PSNR.txt
    
    done
done 
