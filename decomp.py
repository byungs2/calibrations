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
"""
rot_vec = numpy.array([[-0.8907040732899438], [-0.2922251787112129],[-0.2609131578431293]])
rot_mat = numpy.zeros(shape=(3,3))
cv2.Rodrigues(rot_vec,rot_mat)

t_vec = numpy.array([[-76.9658812814217], [66.26125684386133],[344.3542079495192]])
print(rot_mat)
#print(numpy.linalg.inv(rot_mat))
print(numpy.linalg.inv(rot_mat) @ t_vec)
"""
rot_vec = numpy.array([[-0.7676100134849548], [-0.3693520128726959], [-0.2177170068025589]])
rot_mat = numpy.zeros(shape=(3,3))
cv2.Rodrigues(rot_vec,rot_mat)

t_vec = numpy.array([[-146.2354278564453], [81.89420318603516], [332.9459533691406]])

print(rot_mat)
#print(numpy.linalg.inv(rot_mat))
print(-numpy.linalg.inv(rot_mat) @ t_vec)


"""
print(cv2.__version__)

rvec=numpy.array([[-0.9042028868446967], [-0.3130032960424806],[-0.25788580230550934]])
tvec=numpy.array([[-153.48245509344966], [42.10181990635741],[378.54655767797425]])
rvec_matrix = numpy.zeros(shape=(3,3))
cv2.Rodrigues(rvec, rvec_matrix)
print("Rotation vector form", rvec_matrix)

euler_angles = rotationMatrixToEulerAngles(rvec_matrix)
print("Euler angle radian form", euler_angles)
de_pi = euler_angles * 180/3.1415926
print("Euler angle plain angle form", de_pi)

rotMat = euler_to_rotMat(euler_angles[2], euler_angles[1], euler_angles[0])

rot_mat_to_rod_vec(rotMat)
"""

"""
gtMat =numpy.array([[0.650977,-0.758717,0.024027],[-0.018862,-0.04781,-0.998678],[0.758863,0.649664,-0.045434]])
gtMat2 = numpy.array([
      [
        -0.016771,
        -0.999835,
        0.006926
      ],
      [
        -0.029435,
        -0.006431,
        -0.999546
      ],
      [
        0.999426,
        -0.016967,
        -0.029322
      ]
    ])
gtMat3 = numpy.array([
      [
        -0.789986,
        -0.610527,
        0.05638
      ],
      [
        -0.370413,
        0.401962,
        -0.837389
      ],
      [
        0.488586,
        -0.68241,
        -0.543691
      ]
    ])
gtMat4 = numpy.array([
      [
        -0.970568,
        0.235647,
        -0.049676
      ],
      [
        0.09763,
        0.196438,
        -0.975644
      ],
      [
        -0.22015,
        -0.951779,
        -0.213663
      ]
    ])
gtMat5 = numpy.array([
      [
        -0.194109,
        0.980554,
        -0.028888
      ],
      [
        0.233045,
        0.017488,
        -0.972309
      ],
      [
        -0.952896,
        -0.195466,
        -0.231908
      ]
    ])


rot_mat_to_rod_vec(gtMat)
rot_mat_to_rod_vec(gtMat2)
rot_mat_to_rod_vec(gtMat3)
rot_mat_to_rod_vec(gtMat4)
rot_mat_to_rod_vec(gtMat5)

tvec1 = numpy.array([[1,0,0,-1586.4496077989998], [0, 1, 0, -2109.46905869], [0, 0, 1, 1104.209800652]])
tvec2 = numpy.array([[1,0,0,-3512.391424833], [0, 1, 0, 311.47771461800005], [0, 0, 1, 964.5481307480001]])
tvec3 = numpy.array([[1,0,0,-1420.944211509], [0, 1, 0, 2546.574076866], [0, 0, 1, 2688.8728944060003]])
tvec4 = numpy.array([[1,0,0,963.489306486], [0, 1, 0, 3408.674914882], [0, 0, 1, 1422.035001899]])
tvec5 = numpy.array([[1,0,0,3832.020978729], [0, 1, 0, 273.55271850000014], [0, 0, 1, 1439.4616998990002]])

rvec_matrix = numpy.zeros(shape=(3,3))
rvec = numpy.array([[1.515903666496034, -0.6757946059741218,0.6804104540008696]])
cv2.Rodrigues(rvec, rvec_matrix)
print("Rotation vector form", rvec_matrix)
"""


