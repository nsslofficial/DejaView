#!/bin/bash 


# # original PCD
# data_folder=/home/ak5013/dataset/lidcom-carla/64/semantic_eval/2

# ./build/RTE $data_folder pose pose_ndt


# # Draco 
# data_folder=/home/ak5013//dataset/lidcom-carla/64/semantic_eval/2/draco
# for i in {8..11}
# do
#     ./build/RTE $data_folder/$i pose  pose_ndt
    
# done 

# octree 
# data_folder=/home/ak5013//dataset/lidcom-carla/64/semantic_eval/2/octree
# for i in {1..5}
# do
#     ./build/RTE $data_folder/0.$i pose   pose_ndt

# done 


# # lidcom 
# data_folder=/home/ak5013//dataset/lidcom-carla/64/semantic_eval/2/our/original
# for i in {1..5}
# do
#     ./build/RTE $data_folder/w_1_d_10_res_0.$i pose  pose_ndt
    
# done 

##gpcc
data_folder=/home/ak5013/dataset/lidcom-carla/64/semantic_eval/2/gpcc
for i in {2,3,5,7,10}
do
    ./build/RTE $data_folder/r0$i pose pose_ndt

done 