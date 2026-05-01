This readme files list the logic used in finding the lean represenatation of point cloud in differnt fiels. 

1- compression_kdtree_only.cpp
    Performs two way diff operation using KDtree NN search only. 
2- compression_kdtree_radius_search.cpp 
    Replaces two way diff operation with one way by using KDtree radius search
3- compression_octree_radius_search.cpp
    Replaces two way diff operation with one way by using OCtree radius search
4- compression_kdtree_vs_octree_accuracy.cpp
    Compare the RMSE (both ways) of reconstructed point cloud from following three methods
        a- Performs two way diff operation using KDtree NN search only. (compression_kdtree_only.cpp)
        b- Performs two way diff operation using OCtree Change Detector only. (intermediate state of our method)
        c- Performs two way diff operation using first Octree change detector followed by KDtree NN search. (our method)
5- compression.cpp
    The proposed compression algorithm that is used in paper. It is a two way diff operation using both octree change detector and kdtree nn search
6- compression_w_duplicate_points.cpp
    This is same as 5 except the duplicate points are not removed. 