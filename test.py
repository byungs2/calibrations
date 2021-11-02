import numpy
import math

gt1_1 = numpy.array([90,90,-60])
gt1_2 = numpy.array([90,90,-120])
gt2_1 = numpy.array([90,150,-60])
gt2_2 = numpy.array([90,150,-120])
gt3_1 = numpy.array([0, 90, -60])
gt3_2 = numpy.array([0, 90, -120])
gt4_1 = numpy.array([0, 150, -60])
gt4_2 = numpy.array([0, 150, -120])
gt5_1 = numpy.array([0, 60, -120])
gt5_2 = numpy.array([0, 60, -60])
gt6_1 = numpy.array([60, 0, -60])
gt6_2 = numpy.array([60, 0, -120])

array2 = [gt1_1, gt1_2, gt2_1, gt2_2, gt3_1, gt3_2, gt4_1, gt4_2, gt5_1, gt5_2,gt6_1, gt6_2]

def get_distance(a, b):
    a_ = numpy.array(a)
    b_ = numpy.array(b)
    dist = numpy.linalg.norm(a_-b_)
    print(dist)

array = [(90.10820116087157, 90.9555022107847, -58.90766243725637),
(89.88693900563483, 91.03591425603243, -60.16842556397279),
(89.91078100839569, 90.6087790687744, -59.607369800688964),
(90.01750591010936, 90.96117400683366, -118.36837004942483),
(91.11001962997003, 90.66735506217744, -119.12700948500225),
(90.83180506926993, 98.42058328637658, -123.21006908487561),
(89.56649814616324, 149.61726091661686, -59.600674164361486),
(91.40799130836898, 149.67133504389273, -59.164607559238895),
(90.72907137242842, 150.76298179757768, -60.29893765927645),
(89.302116419436, 150.18219081612776, -118.36699728111977),
(91.5845047305991, 149.22613486007356, -118.93456497038457),
(91.06810499988339, 151.64569533381717, -120.05941909822067),
(-2.155268982443566, 89.18281881283134, -56.89848963548939),
(-0.6547012593399231, 89.2928939271177, -57.0511959982517),
(0.9824158284595437, 95.02939712577592, -61.3599506597348),
(-3.4108063698937885, 89.26272719658589, -118.48553401048642),
(-0.945497821409539, 88.20819845674976, -119.9173359462075),
(2.6625208824025073, 98.55117388501434, -123.1203257818303),
(-0.4503259348237564, 149.6520193240073, -56.82261167251898),
(0.038579965310701216, 150.22995994000004, -57.6541721050587),
(-0.495330761801957, 148.7861522769416, -57.791256840692384),
(-1.8001651549998239, 149.6038167303654, -117.38887496923208),
(-0.19533663723600067, 149.7263233292418, -118.1881280263657),
(1.0134599739596741, 152.92299485829116, -119.64893939050577),
(-2.432141346627435, 58.07207917117497, -118.13873086736918),
(-1.6870004963286953, 57.32122257262312, -118.3431127431881),
(-1.5449411433642442, 59.33239071810079, -119.04574343152835),
(-0.7103769537773523, 59.708504948071294, -59.13963877246493),
(0,0,0),
(0,0,0),
(59.38582135295206, -3.248960937428235, -56.56867029811791),
(60.92141466402539, -3.4119352887485657, -56.55041819830018),
(61.754241294842046, 6.283214849306305, -62.11086592367676),
(61.21736477328337, -3.993058832733547, -115.18453832316992),
(62.37750831071625, -4.603066263026803, -117.31106666495525),
(64.10191341998473, 19.938659984392693, -125.7420791169564)]
cnt = 0
for i in array:
    get_distance(i, array2[cnt//3])
    cnt+=1