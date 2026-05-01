#!/bin/bash

# # Directories
CONFIG_BASE=../../mpeg-pcc-tmc13-release-v14.0/cfg/octree-predlift/lossy-geom-lossy-attrs/custom
BASE=../sample_data


TMC3_EXEC="../../mpeg-pcc-tmc13-release-v14.0/build/tmc3/tmc3" # Adjust path if necessary


# PCD to PLY
echo "running pcd to ply conversion"
for j in 11
do 
    input_folder=$BASE/$j/pcds
    output_folder=$BASE/$j/plys
    mkdir -p $output_folder

    python3 ../misc/convert_dir.py $input_folder $output_folder .pcd .ply
done 




#Compression 

echo "running compression"
for j in 11
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
echo "running decompression"
for j in 11
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

echo "running ply to pcd conversion"
for j in 11
do 
    for i in {2,3,5,7,10}
    do 
    input_folder=$BASE/$j/gpcc/r0$i/decompressed
    output_folder=$BASE/$j/gpcc/r0$i/decompressed_pcds

    mkdir -p $output_folder
    python3 ../misc/convert_dir.py $input_folder $output_folder .ply .pcd

    done 
done


# Error calculation 
echo "running error calculation"
for j in 11
do 
    for i in {2,3,5,7,10}
    do 
        input_folder_2=pcds
        input_folder_1=gpcc/r0$i/decompressed_pcds

        ./../metrics/build/chamferDistance_fast $BASE/$j $input_folder_1 $input_folder_2 >> gpcc_CD.txt
        ./../metrics/build/PSNR_fast $BASE/$j $input_folder_1 $input_folder_2 >> gpcc_PSNR.txt
    done 
done 




