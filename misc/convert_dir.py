import sys, os, glob
import open3d as o3d
import numpy as np

if len(sys.argv) != 5:
    print("Usage: python3 convert_dir.py <in_dir> <out_dir> <ext_in> <ext_out>")
    sys.exit(1)

in_dir = sys.argv[1]
out_dir = sys.argv[2]
ext_in = sys.argv[3]
ext_out = sys.argv[4]

os.makedirs(out_dir, exist_ok=True)

files = glob.glob(os.path.join(in_dir, f"*{ext_in}"))
for f in files:
    base = os.path.basename(f)
    name = os.path.splitext(base)[0]
    out_f = os.path.join(out_dir, name + ext_out)
    
    pcd = o3d.io.read_point_cloud(f)
    
    if ext_out == '.ply':
        points = np.asarray(pcd.points).astype(np.float32)
        with open(out_f, 'wb') as fout:
            header = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
            fout.write(header.encode('ascii'))
            fout.write(points.tobytes())
    else:
        # Saving as PCD or other format
        o3d.io.write_point_cloud(out_f, pcd, write_ascii=False)
