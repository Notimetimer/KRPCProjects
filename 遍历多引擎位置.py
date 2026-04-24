import krpc  
import numpy as np 

conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
engines = vessel.parts.engines  

# 获取体轴系参考系（原点在质心）   

print("引擎相对质心位置（体轴系）:")  
print("格式：引擎索引: (x, y, z) - 引擎名称")  
print("坐标系：x=右, y=前, z=下")  

# 右前下转前右下
def RFD2FRD(input_vector):
    input_vector = np.array(input_vector) # 不管进来是什么一律转成array
    output_vector = np.zeros_like(input_vector)
    output_vector[0] = input_vector[1] # 前
    output_vector[1] = input_vector[0] # 右
    output_vector[2] = input_vector[2] # 下
    return output_vector

engines_dict = {}
# 获取部件位置
for i, engine in enumerate(engines):  
    # 获取引擎部件在体轴系中的位置（相对于质心）  
    position = engine.part.position(vessel.reference_frame)
    engines_dict[i] = RFD2FRD(engine.part.position(vessel.reference_frame))
    print(f"{i}: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) - {engine.part.title}")

print(engines_dict)

# # 获取引擎质心位置
# for i, engine in enumerate(engines):  
#     # 使用引擎部件的质心位置  
#     com_position = engine.part.center_of_mass(vessel.reference_frame)  
#     print(f"{i}: ({com_position[0]:.2f}, {com_position[1]:.2f}, {com_position[2]:.2f}) - {engine.part.title}")

# 获取飞行器的体轴系  

  
# 遍历所有发动机  
for engine in vessel.parts.engines:  
    for thruster in engine.thrusters:  
        # 获取推力位置和方向（在体轴系中，右前下顺序）  
        position = thruster.thrust_position(vessel.reference_frame)  
        direction = thruster.thrust_direction(vessel.reference_frame)  
          
        # 获取发动机力矩  
        torque = engine.available_torque

        print(engine, '位置', position, '方向', direction, '力矩', torque)
