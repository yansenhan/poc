x_map = 5
y_map = 4

local_boundary_points = [
    (0, y) for y in range(y_map)  # 左边界上的点
] + [
    (x_map - 1, y) for y in range(y_map)  # 右边界上的点
] + [
    (x, 0) for x in range(x_map)  # 上边界上的点
] + [
    (x, y_map - 1) for x in range(x_map)  # 下边界上的点
]
import pdb;pdb.set_trace()

print("local_boundary_points:", local_boundary_points)
