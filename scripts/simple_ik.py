import ikpy.chain
import os
import numpy as np

# 1. 加载链条
urdf_path = os.path.expanduser('~/panda_ws/src/panda_description/urdf/panda_fixed.urdf')
my_chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["world"])

# 2. 锁定关节
for i, link in enumerate(my_chain.links):
    if i < 2 or i > 8:
        link.is_kinematic = False
    else:
        link.is_kinematic = True

# 3. 【核心改进】给一个“热启动”初始姿态
# 这是 Panda 机器人比较自然的弯曲姿态（弧度制）
# 对应关节 1-7：[0, -0.785, 0, -2.356, 0, 1.571, 0.785]
# 我们需要把它补全到 12 个元素的数组中
initial_guess = [0.0] * 12
initial_guess[2] = 0.0      # joint 1
initial_guess[3] = -0.785   # joint 2
initial_guess[4] = 0.0      # joint 3
initial_guess[5] = -2.356   # joint 4
initial_guess[6] = 0.0      # joint 5
initial_guess[7] = 1.571    # joint 6
initial_guess[8] = 0.785    # joint 7

# 4. 设定目标（先只管位置，不管朝向，降低难度）
target_pos = [0.431, -0.401, 0.814]

print(f"正在以‘热启动’模式冲刺目标点: {target_pos}")

# 5. 计算逆解
joint_angles = my_chain.inverse_kinematics(
    target_position=target_pos,
    initial_position=initial_guess  # 告诉它从这个姿态开始算
)

# 6. 验证
real_frame = my_chain.forward_kinematics(joint_angles)
print(f"\n--- 验证结果 ---")
print(f"目标坐标: {target_pos}")
print(f"实际坐标: [{real_frame[0, 3]:.3f}, {real_frame[1, 3]:.3f}, {real_frame[2, 3]:.3f}]")

# 计算误差
error = np.linalg.norm(real_frame[:3, 3] - target_pos)
print(f"直线距离误差: {error*1000:.2f} 毫米")

if error < 0.01:
    print("\n✅ 逆解成功！这串角度可以用了：")
    print(list(joint_angles[2:9]))
else:
    print("\n❌ 依然失败，尝试放宽搜索范围...")