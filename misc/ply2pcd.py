import open3d as o3d
import argparse

def convert_ply_to_pcd(input_file, output_file):
    # Load PLY file
    point_cloud = o3d.io.read_point_cloud(input_file)
    
    # Save as PCD file
    o3d.io.write_point_cloud(output_file, point_cloud)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PLY to PCD format.")
    parser.add_argument("--input", required=True, help="Path to the input PLY file")
    parser.add_argument("--output", required=True, help="Path to the output PCD file")
    
    args = parser.parse_args()
    convert_ply_to_pcd(args.input, args.output)
