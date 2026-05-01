import sys
import open3d as o3d
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--input', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()

pcd = o3d.io.read_point_cloud(args.input)
o3d.io.write_point_cloud(args.output, pcd, write_ascii=False)
