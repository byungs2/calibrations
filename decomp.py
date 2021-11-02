import cv2
import numpy
import math

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return numpy.array([x, y, z])

def euler_to_rotVec(yaw, pitch, roll):
    # compute the rotation matrix
    Rmat = euler_to_rotMat(yaw, pitch, roll)

    theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
        ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
        rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
    return rx, ry, rz

def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = numpy.array([
        [numpy.cos(yaw), -numpy.sin(yaw), 0],
        [numpy.sin(yaw),  numpy.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = numpy.array([
        [ numpy.cos(pitch), 0, numpy.sin(pitch)],
        [             0, 1,             0],
        [-numpy.sin(pitch), 0, numpy.cos(pitch)]])
    Rx_roll = numpy.array([
        [1,            0,             0],
        [0, numpy.cos(roll), -numpy.sin(roll)],
        [0, numpy.sin(roll),  numpy.cos(roll)]])
    # R = RzRyRx
    rotMat = numpy.dot(Rz_yaw, numpy.dot(Ry_pitch, Rx_roll))
    return rotMat

def rot_mat_to_rod_vec(R) :
    eur = rotationMatrixToEulerAngles(R)
    rod2 = euler_to_rotVec(eur[2], eur[1], eur[0])
    print("Rodrigues form", rod2)

def get_rotation_mat_and_translation_vec(yaw, pitch, roll, tx, ty, tz):
    roll_mat = numpy.array([[1, 0, 0, 0], [0, math.cos(roll),-math.sin(roll),0], [0, math.sin(roll), math.cos(roll)], [0, 0,0,1]])
    pitch_mat = numpy.array([[math.cos(pitch), 0, math.sin(pitch), 0], [0, 1, 0, 0], [-math.sin(pitch), 0, math.cos(pitch), 0], [0,0,0,1]])
    yaw_mat = numpy.array([[math.cos(yaw), -math.sin(yaw), 0, 0],[math.sin(yaw), math.cos(yaw), 0, 0], [ 0, 0, 1, 0], [0, 0, 0, 1]])
    trans_mat = numpy.array([[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0, 0,0, 1]])
    rot_mat = yaw_mat @ pitch_mat @ roll_mat @ trans_mat
    return rot_mat

def get_camera_location(rod_vec, t_vec):
    rod_vec_np = numpy.array(rod_vec)
    t_vec_np = numpy.array(t_vec)
    rot_mat = numpy.zeros(shape=(3,3))
    cv2.Rodrigues(rod_vec_np, rot_mat)
    point = -numpy.linalg.inv(rot_mat) @ t_vec_np
    print("cam_point", point)
    return point

def get_distance(a, b):
    a_ = numpy.array(a)
    b_ = numpy.array(b)
    dist = numpy.linalg.norm(a_-b_)
    print(dist)

def rod_to_rot_mat(rod):
    rod_vec = numpy.array(rod)
    rot_mat = numpy.zeros(shape=(3,3))
    cv2.Rodrigues(rod_vec, rot_mat)
    return rot_mat

def world_point_to_t_vec(world_point, rot_mat):
    w_point_np = numpy.array(world_point)
    t_vec = -rot_mat @ world_point
    print("T_VEC")
    print(t_vec)
    return t_vec

rod_vec_array = [[-0.9052410125732422, -0.3208119869232178,
    -0.25825899839401245], [-0.34401899576187134, -0.7848190069198608,
        -2.1209309101104736],[-1.0454959869384766, 0.2603999972343445,
            0.3319210112094879], [-0.8710970282554626, -0.3608259856700897,
                -0.25987499952316284], [-0.3267410099506378, -0.74788898229599,
                    -2.121756076812744],[-1.0038950443267822,
                        0.2336760014295578, 0.3497700095176697]]
t_vec_array = [[-153.6964111328125, 41.12293243408203, 375.68475341796875],
        [-0.10194999724626541, 173.8535919189453, 220.1582489013672],
        [-11.225990295410156, 45.41680908203125,
            489.9926452636719],[-155.0535125732422, 34.5850944519043,
                369.1050720214844], [-11.585250854492188, 174.8994598388672,
                    196.32400512695312], [-10.956890106201172,
                        34.2403564453125, 479.7378234863281]]

gt_1 = (6.5, 300, -270)
gt_2 = (0, 0, -270)
gt_3 = (180, 360, -270)
gt_array = [gt_1, gt_2, gt_3]
points = []
cnt = 0
for i in range(len(rod_vec_array)):
    points.append(get_camera_location(rod_vec_array[i],
        t_vec_array[i]).flatten())


for i in range(len(points)):
    get_distance(points[i], gt_array[cnt%3])
    cnt+=1

cam_2_gt = (0, 0, -270)
cam_2_rod_vec_ex_in_fix = [[-0.34401899576187134], [-0.7848190069198608],
        [-2.1209309101104736]]
cam_2_rod_vec_sdk = [[-0.3158052647404374], [-0.8168505729333261], [-2.1170787310913557]]
rot_mat = rod_to_rot_mat(cam_2_rod_vec_ex_in_fix)
rot_mat_2 = rod_to_rot_mat(cam_2_rod_vec_sdk)

world_point_to_t_vec(cam_2_gt, rot_mat)
world_point_to_t_vec(cam_2_gt, rot_mat_2)








