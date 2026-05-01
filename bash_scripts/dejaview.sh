#!/bin/bash


for j in 11
do
    for i in {1..5}
    do 
        data_folder=../sample_data
        output_folder=our/w_1to10_d_10_res_0.$i
        ref_day=11
        compress_day=$j

        multiple_day=1 #1 for multiple days. 0 for 1 day 
        # python lidcom/find_closest_frame.py $data_folder $ref_day $compress_day 1 2 3 2
        python3 ../dejaview/find_closest_frame_multiple_days.py $data_folder $compress_day 1 2 3 1

        # echo "running compression script"
        time ./../dejaview/build/compression $data_folder $ref_day $compress_day $output_folder 0.$i $multiple_day


        echo "running decompression script"
        time ./../dejaview/build/decompression $data_folder $ref_day $compress_day $output_folder $output_folder/decompressed $multiple_day


        echo "running error calculation "

        input_folder_1=$output_folder/decompressed
        input_folder_2=pcds

        echo "running chamfer distance calculation"  
        ../metrics/build/chamferDistance_fast $data_folder/$compress_day $input_folder_1 $input_folder_2 >> dejaview_CD.txt
        
        echo "running psnr calculation"
        ../metrics/build/PSNR_fast $data_folder/$compress_day $input_folder_1 $input_folder_2 >> dejaview_PSNR.txt
    done
done
