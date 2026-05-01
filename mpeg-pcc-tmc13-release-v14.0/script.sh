#!/bin/bash

# # Directories
CONFIG_BASE=/home/ak5013/workspace/3D_data_compression/final_code/mpeg-pcc-tmc13-release-v14.0/cfg/octree-predlift/lossy-geom-lossy-attrs/custom
BASE=/mydata/dataset/rit-small-loop
# BASE=/home/ak5013/dataset/rit-small-loop
# BASE=/home/ak5013/dataset/lidcom-carla/3D_object_detection/
# BASE=/home/ak5013/dataset/lidcom-carla/64/semantic_eval
# BASE=/home/ak5013/dataset/lidcom-carla/32/effect_of_dynamic_env/lane_wise_cars_w_gaps_filled/2_lanes_vehicle_wo_gap_filled


# /home/ak5013/dataset/lidcom-carla/32/no-noise/2

# LOG_DIR="./logs"
# CONFIG="./cfg/octree-predlift/lossy-geom-lossy-attrs/ford_01_q1mm/r01/encoder.cfg" # Path to your TMC13 config file


# Create output and log directories if they don't exist
# mkdir -p "$LOG_DIR"

TMC3_EXEC="./build/tmc3/tmc3" # Adjust path if necessary


# PCD to PLY

for j in 14 {20..37} {43..73}
do 
    input_folder=$BASE/$j/pcds
    output_folder=$BASE/$j/plys
    mkdir -p $output_folder

    for filename in $input_folder/*.pcd; 
    do
        #format 1 for bin output and 0 for asci output
        pcl_pcd2ply -format 1 $filename $output_folder/$(basename "$filename" .pcd).ply
    done
done 




#Compression 

for j in 14 {20..37} {43..73}
do 
    INPUT_DIR=$BASE/$j/plys
    for i in {2,3,5,7,10}
    do 
        OUTPUT_DIR=$BASE/$j/gpcc/r0$i/compressed
        mkdir -p "$OUTPUT_DIR"

        for INPUT_FILE in "$INPUT_DIR"/*
        do
            # Get the filename without the extension
            BASENAME=$(basename "$INPUT_FILE")
            FILENAME="${BASENAME%.*}"

            # Define output compressed file and log file
            COMPRESSED_FILE="$OUTPUT_DIR/${FILENAME}.bin"
            # LOG_FILE="$LOG_DIR/${FILENAME}.log"
            # --config=$CONFIG 

            $TMC3_EXEC --config=$CONFIG_BASE/r0$i/encoder.cfg --uncompressedDataPath=$INPUT_FILE --compressedStreamPath=$COMPRESSED_FILE

            echo "Compressed $INPUT_FILE -> $COMPRESSED_FILE"
        done
    done 
done



# Decompression 
for j in 14 {20..37} {43..73}
do 
    for i in {2,3,5,7,10}
    do 
        INPUT_DIR=$BASE/$j/gpcc/r0$i/compressed
        OUTPUT_DIR=$BASE/$j/gpcc/r0$i/decompressed

        mkdir -p "$OUTPUT_DIR"
        for INPUT_FILE in "$INPUT_DIR"/*; do
            # Get the filename without the extension
            BASENAME=$(basename "$INPUT_FILE")
            FILENAME="${BASENAME%.*}"

            # Define output compressed file and log file
            UNCOMPRESSED_FILE="$OUTPUT_DIR/${FILENAME}.ply"
            # LOG_FILE="$LOG_DIR/${FILENAME}_compression.log"
            # --config=$CONFIG 

            $TMC3_EXEC --config=$CONFIG_BASE/r0$i/decoder.cfg --reconstructedDataPath=$UNCOMPRESSED_FILE --compressedStreamPath=$INPUT_FILE  

            echo "Decompressed $INPUT_FILE -> $UNCOMPRESSED_FILE"
        done

    done 
done

# ply to pcd 
for j in 14 {20..37} {43..73}
do 
    for i in {2,3,5,7,10}
    do 
    input_folder=$BASE/$j/gpcc/r0$i/decompressed
    output_folder=$BASE/$j/gpcc/r0$i/decompressed_pcds

    mkdir -p $output_folder
    for filename in $input_folder/*.ply; 
    do
        #format 1 for bin output and 0 for asci output
        python ./../ply2pcd.py --input $filename --output $output_folder/$(basename "$filename" .ply).pcd
    done

    done 
done


# # for j in 2
# # #14 {20..25} 
# # #{26..37} 
# # # {43..73}
# # do 
# #     for i in {2,3,5,7,10}
# #     do 
# #         input_folder=$BASE/$j/gpcc/r0$i/decompressed
# #         output_folder=$BASE/$j/gpcc/r0$i/decompressed_pcds

# #         mkdir -p "$output_folder"
        
# #         for filename in $input_folder/*.ply; 
# #         do 
# #             python ./../ply2pcd.py --input "$filename" --output "$output_folder/$(basename "$filename" .ply).pcd"
# #         done 


# #         Find all .ply files and process them in parallel with 4 threads
# #         find "$input_folder" -name "*.ply" | xargs -I{} -P 12 bash -c '
# #             filename="{}"
# #             output_file="'$output_folder'/$(basename "$filename" .ply).pcd"
# #             python ./../ply2pcd.py --input "$filename" --output "$output_file"
# #             '

# #     done
# # done

# Error calculation 
for j in 14 {20..25} {26..37} {43..73}
do 
    for i in {2,3,5,7,10}
    do 
        input_folder_2=pcds
        input_folder_1=gpcc/r0$i/decompressed_pcds

        ./../metrics/build/chamferDistance $BASE/$j $input_folder_1 $input_folder_2 >> gpcc_CD.txt
        ./../metrics/build/PSNR $BASE/$j $input_folder_1 $input_folder_2 >> gpcc_PSNR.txt
    done 
done 



# # echo "running error calculation "
# # output_folder=draco/${quantization_parameter}
# # input_folder_1=$output_folder/decompressed
# # input_folder_2=pcds_xyz
# # ./metrics/build/chamferDistance $datset_path/$compress_day $input_folder_1 $input_folder_2 
# # ./metrics/build/PSNR $datset_path/$compress_day $input_folder_1 $input_folder_2 
# # done



