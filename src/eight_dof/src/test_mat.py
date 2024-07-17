import numpy as np


def Construct_Inertia_Matrix(i_v):
    ixx=i_v[0]
    ixy=i_v[1]
    ixz= i_v[2]
    iyy= i_v[3]
    iyz=i_v[4]
    izz=i_v[5]

    Matrix=np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]])
    return Matrix

def test_matrix(matrix):
    eigen_values=np.linalg.eigvals(matrix)
    return np.all(eigen_values>0)

def reorient_inertial_values(inertia_values):
    inertia_values_list= inertia_values.split(" ")
    inertia_values_list=[inertia_values_list[0],inertia_values_list[]]

    

i_v=[0.01012 ,0.000038096 ,-0.000079858, 0.0185 ,-0.001408 ,0.01433]
IM=Construct_Inertia_Matrix(i_v)
print(test_matrix(IM))




                    