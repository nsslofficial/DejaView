#!/bin/bash

for day in 11
do
    for i in {8..11}
    do 
    quantization_parameter=$i
    root_data_folder=../sample_data
    data_folder=$root_data_folder/$day
    output_folder=draco/${quantization_parameter}

    echo "running draco compression"
    time ./../draco/build/draco_compression_fast $data_folder $output_folder $quantization_parameter
    
    echo "running draco decompression"
    time ./../draco/build/draco_decompression_fast $data_folder  $output_folder

    echo "running error calculation "
    input_folder_1=$output_folder/decompressed
    input_folder_2=pcds

    echo "running chamfer distance calculation "
    time ./../metrics/build/chamferDistance_fast $data_folder $input_folder_1 $input_folder_2 >>  draco_CD.txt
    
    echo "running PSNR calculation "
    time ./../metrics/build/PSNR_fast $data_folder $input_folder_1 $input_folder_2 >>  draco_PSNR.txt
    
    done
done 
