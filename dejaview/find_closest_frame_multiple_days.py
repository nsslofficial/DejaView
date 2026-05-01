import pandas as pd
import numpy as np
import os
import sys


#   Usage 
#   python find_closest_frame.py [dataset folder path] [day1 path (reference day) ] [day2 path (day to be compressed)] [x] [y] [z]

def remove_extension(integer):
    return integer[:-4]


def list_folders(directory):
    items = os.listdir(directory)
    folders = [item for item in items if os.path.isdir(os.path.join(directory, item))]
    return folders

#   Load command line arguments 
data_path = sys.argv[1]
com_day = sys.argv[2]
x = int(sys.argv[3])
y = int(sys.argv[4])
z = int(sys.argv[5])
data_type = sys.argv[6]

# global variables
remove_pcd_extension = False
seperator = ","
folders = []
    

# folders.remove(com_day)
if 'basemap' in folders:
    folders.remove('basemap')

if 'ignore' in folders:
    folders.remove('ignore')


# setting ref data for carla 
if (data_type =='1'):
    folders = list_folders(data_path)
    folders = list(range(1,11))
    

# setting ref data for real world data
ref_data_1 = [15,16,17,18,19]
ref_data_2 = [38,39,40,41,42]
if (data_type == '2'):
    if (int(com_day) >=43):
        folders = ref_data_2
    else:
        folders = ref_data_1

folders = [str(i) for i in folders]
print(folders)
com_path = data_path + '/' + com_day
com_pose_file = com_path + '/pose.txt'
output_file = com_path+ '/association.txt'

#   find if the frame names have .pcd with them or not

com_frame_names = np.loadtxt(com_pose_file, delimiter=seperator, usecols=(0), unpack=True, dtype='str')
if (len(com_frame_names[0]) > 4):
    if(com_frame_names[0][-4:] == ".pcd"):
        remove_pcd_extension = True

#   load position of to be compressed frames
if (remove_pcd_extension):
    # compress_df = pd.read_csv(com_pose_file, sep=",", header=None, float_precision='round_trip')
    compress_df = pd.read_csv(com_pose_file, sep=seperator, header=None, float_precision='round_trip', dtype= {0: 'object'})
    compress_df[compress_df.columns[0]] = compress_df[compress_df.columns[0]].apply(remove_extension)
    compress_df[compress_df.columns[0]] = compress_df[compress_df.columns[0]].astype('object')
    com_frame_names = compress_df[compress_df.columns[0]].to_numpy()
    com_location_data = compress_df[compress_df.columns[x:z+1]].to_numpy()
    com_location_data= np.transpose(com_location_data)
     
else:
    com_frame_names = np.loadtxt(com_pose_file, delimiter=seperator, usecols=(0), unpack=True, dtype='object')
    com_location_data  = np.loadtxt(com_pose_file, delimiter=seperator, usecols=(x,y,z), unpack=True)

com_location_data = np.reshape(com_location_data, (3, -1,1))
ones = -1*np.ones((3,com_location_data.shape[1],1))
com_location_data = np.append(com_location_data, ones, 2)
print(com_location_data.shape)

final_df = pd.DataFrame()
combined_df = pd.DataFrame()
for ref_day in folders:
    ref_path = data_path + '/' + ref_day
    ref_pose_file = ref_path + '/pose.txt'

    #   load position of ref frames

    if (remove_pcd_extension):
        # reference_df[reference_df.columns[0]] = reference_df[reference_df.columns[0]].apply(remove_extension)
        reference_df = pd.read_csv(ref_pose_file, sep=seperator, header=None, float_precision='round_trip', dtype= {0: 'object'})
        reference_df[reference_df.columns[0]] = reference_df[reference_df.columns[0]].astype('object')
        ref_frame_names = reference_df[reference_df.columns[0]].to_numpy()
        ref_location_data = reference_df[reference_df.columns[x:z+1]].to_numpy()
        ref_location_data = np.transpose(ref_location_data)
    else:
        ref_frame_names = np.loadtxt(ref_pose_file, delimiter=seperator, usecols=(0), unpack=True,  dtype='object')
        ref_location_data  = np.loadtxt(ref_pose_file, delimiter=seperator, usecols=(x,y,z), unpack=True)

    ref_frame_names_dict = dict(enumerate(ref_frame_names))
    ref_location_data = np.reshape(ref_location_data, (3,1,-1))
    ones = np.ones((3,1, ref_location_data.shape[2]))
    ref_location_data = np.append(ones,ref_location_data,  1)
    print(ref_location_data.shape)

    #   find the distance from colsest frame and its index
    a = np.matmul(com_location_data, ref_location_data)
    dist = np.sqrt(np.sum(np.square(np.matmul(com_location_data, ref_location_data)),0))
    print(dist.shape)
    correspondence_dist = np.min(dist, 1)
    correspondence_index = np.argmin(dist,1)

    dataframe_array = np.stack((com_frame_names, correspondence_index, correspondence_dist),axis=1)
    df = pd.DataFrame(dataframe_array, columns = ['frame','associated_frame','distance'])
    df['associated_frame'] = df['associated_frame'].map(ref_frame_names_dict)
    df['associated_frame'] = df['associated_frame'].apply(lambda x: ref_day+"/pcds/" + x)
    # print(df.tail(5))
    #   save resutls 
    # output_file = com_path+ '/' + ref_day+'_association.txt'
    # df.to_csv(output_file, header=None, index=None, sep=',', mode='w')

    if (combined_df.shape[0] == 0):
        combined_df=df.add_suffix('_'+ref_day) 
    else:
        combined_df = pd.concat([combined_df, df.add_suffix('_'+ref_day)], axis=1)



    if (final_df.shape[0] == 0):
        final_df=df 
    else:
        temp_df = pd.DataFrame({
        'frame': np.where(df['distance'] < final_df['distance'], df['frame'], final_df['frame']),
        'associated_frame': np.where(df['distance'] < final_df['distance'], df['associated_frame'], final_df['associated_frame']),
        'distance': np.where(df['distance'] < final_df['distance'], df['distance'], final_df['distance'])   
        })

        final_df = temp_df


# print(final_df.tail(5))
column_order = ['frame', 'associated_frame', 'distance']

# Reorder the DataFrame columns
final_df = final_df[column_order]

output_file = com_path+ '/association.txt'
final_df.to_csv(output_file, header=None, index=None, sep=',', mode='w')

# output_file = com_path+ '/combined_association.txt'
# combined_df.to_csv(output_file, header=None, index=None, sep=',', mode='w')
    #control the type of frame names
    #df['associated_frame'] = df['associated_frame'].astype(int)
    #df['frame'] = df['frame'].astype(int)
    #print(df.shape)


