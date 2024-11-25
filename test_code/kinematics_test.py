import kinematics_l2delta as kl2d 
import kinematics_delta2x as kd2x

# l1 = 0.123
# l2 = 0.456
# l3 = 0.789
# d  = 0.027

# L = [l1, l2, l3]

# print(kl2d.FK_L2S_jones(L,d))
# print(kl2d.FK_L2S_cosimo_old(L,d))
# print(kl2d.FK_L2S_cosimo_new(L,d))

# s = 0.789
# deltax = 1.234
# deltay = 1.567

# print(kd2x.FK_S2X_jones(s,deltax,deltay))
# print(kd2x.FK_S2X_cosimo_old(s,deltax,deltay))
# print(kd2x.FK_S2X_cosimo_new(s,deltax,deltay))

# l1 = 0.123
# l2 = 0.456
# l3 = 0.789
# l4 = 0.123
# l5 = 0.456
# l6 = 0.789
# l7 = 0.123
# l8 = 0.456
# l9 = 0.789

# L = [l1, l2, l3, l4, l5, l6, l7, l8, l9]

# d  = 0.027

# sdxdy = kl2d.FK_L2S_new_3sections(L,d)
# print(sdxdy.shape)

S = [0.456, 0.789, 0.123]
Deltax = [1.1, 1.2, 1.3]
Deltay = [2.1, 2.2, 2.3]

print(kd2x.FK_realrobot(S, Deltax, Deltay))