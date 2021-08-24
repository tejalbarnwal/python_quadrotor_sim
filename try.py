import numpy as np

# a = np.array([40,40,30])
# b = np.array([25,25,35])

# compare_mat = a > b
# print(compare_mat * b + (1-compare_mat) * a)


def Rot_v_to_v1(psi):
    R = np.array([
        [ np.cos(psi), np.sin(psi), 0],
        [-np.sin(psi), np.cos(psi), 0],
        [     0,           0    ,   1]
    ])
    return R

def Rot_v1_to_v2(theta):
    R = np.array([
        [np.cos(theta), 0, -np.sin(theta)],
        [      0      , 1,        0      ],
        [np.sin(theta), 0,  np.cos(theta)]
    ])
    return R

def Rot_v2_to_b(phi):
    # R = np.array([  [1,       0     ,      0     ],
    #         [0,  np.cos(phi), np.sin(phi)],
    #           [0, -1 * np.sin(phi), np.cos(phi)]  
    #         ])
    R = np.array([
        [1,0,0],
        [0,np.cos(phi),np.sin(phi)],
        [0,-np.sin(phi),np.cos(phi)]
      ])
    return R

def Rot_v_to_b(phi, theta, psi):
    return np.dot( np.dot(Rot_v2_to_b(phi),Rot_v1_to_v2(theta)) , Rot_v_to_v1(psi) )

def Rot_i_to_b(phi, theta, psi):
    return Rot_v_to_b(phi, theta, psi)

orientation = np.zeros((3,1),dtype=float)
[phi, theta, psi] = orientation

# vel = np.ones((3,1),dtype=float)
# a = np.dot(Rot_i_to_b(phi, theta, psi).transpose(), vel)
# print(a)