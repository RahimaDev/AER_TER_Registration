# Aerial/terrestrial registration framework based on 3D segments by Rahima Djahel (ENPC), Pascal Monasse (ENPC) and Buno VAllet (IGN)

# Required dependencies: PCL, Eigen, OpenCV.

AT_Registration: an efficient algorithm to register Aerial and Terrestrial data.


# Test:

./AT_Registration ../data/Terrestre.txt ../data/Aer.txt 0.2 3

where:

Terrestre.txt: txt file contains informations about 3D segments reconstructed from a terrestrial image sequence.

Aer:  txt file contains informations about 3D segments reconstructed from an ortho-image.

0.2: is a distance threshold above which a line is just considered an outlier (can be adapted by the users).

3: is a distance threshold to avoid selecting coplanar segment pairs (can be adapted by the users).

# To visualize the result:

cloudcompare.CloudCompare TER_Lines.objTRRR.obj

# If you use our algorithm in any of your publications or projects, please cite our paper:

DJAHEL, Rahima, MONASSE, Pascal, et VALLET, Bruno. A 3D SEGMENTS BASED ALGORITHM FOR HETEROGENEOUS DATA REGISTRATION. The International Archives of Photogrammetry, Remote Sensing and Spatial Information Sciences, 2022, vol. 43, p. 129-136.

#If you have any questions, you can send an email to :

rahima.djahel@enpc.fr

rdjahel@gmail.com


