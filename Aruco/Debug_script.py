import os
import yaml

# for error Xe../
with open("Visualizes_PoseEstimation.py") as fp:
    for i, line in enumerate(fp):
        if "\xe2" in line:
            print i, repr(line), '1.0'

# if there is a file in the folder

# debug
if os.access("test", os.R_OK):
    with open("test") as fp: print fp.read()
print "some default data"


# find - open - load --Doesnt work on my file..
# with open('true.yaml') as f:
#     data_dict = yaml.load(f, Loader=yaml.FullLoader)
#     print(data_dict)

# camera_matrix = np.asarray(data_dict.get('camera_matrix'))
# dist_coeffs = np.asarray(data_dict.get('distortion_coefficients'))