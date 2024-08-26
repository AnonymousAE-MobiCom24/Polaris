
import numpy as np

import math as m

# 计算两点之间的距离
def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# 找到距离最远的两个点
def find_vertex_points(points):
    max_dist = 0
    p1, p2 = points[0], points[1]
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            dist = distance(points[i], points[j])
            if dist > max_dist:
                max_dist = dist
                p1, p2 = points[i], points[j]
    candidates = find_third_vertex(points, p1, p2)

    if candidates is None:
        return p1, p2, None
    else:
        return list(p1), list(p2), list(candidates)

# 假设 p1 和 p2 是已经找到的两):
def find_third_vertex(points, p1, p2):
            # 计算角度，检查是否接近直角
    candidate = None
    for point in points:
            if np.allclose(point, p1) or np.allclose(point, p2):
                continue
            angle = calculate_angle(p1, point, p2)  # 这需要一个计算角度的函数
            if np.isclose(angle, 90, atol=20):  # 允许一定的误差范围
                candidate = point
    
    # print(candidates)
    return candidate
    

def calculate_angle(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p2
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))) * 180 / np.pi



import numpy as np

# 假设 points_measured 和 points_true 是两个包含对应点的numpy数组
# 形式如：points_measured = np.array([[x1, y1], [x2, y2], [x3, y3]])
# points_true = np.array([[X1, Y1], [X2, Y2], [X3, Y3]])

def compute_rotation_angle(points_measured, points_true):
    # 计算两组点的质心
    centroid_measured = np.mean(points_measured, axis=0)
    centroid_true = np.mean(points_true, axis=0)
    
    # 平移到质心
    points_measured_centred = points_measured - centroid_measured
    points_true_centred = points_true - centroid_true
    
    # 计算旋转矩阵
    H = points_measured_centred.T @ points_true_centred
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # 确保旋转矩阵是正交的
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T
    
    # 从旋转矩阵中提取角度
    angle_rad = np.arctan2(R[1, 0], R[0, 0])
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg





# import numpy as np

# # 假设 points_measured 是测量得到的三个点坐标，形式如：[[x1, y1], [x2, y2], [x3, y3]]
# # 假设 points_true 是已知的三个点坐标，形式相同
# # 这里假设 points_measured 和 points_true 中的第一个点是直角顶点

# def compute_rotation_angle(points_measured, points_true):
#     # 将三角形平移到原点，使得直角顶点在原点
#     origin_measured = np.array(points_measured[0])
#     origin_true = np.array(points_true[0])
#     translated_measured = points_measured - origin_measured
#     translated_true = points_true - origin_true
    
#     # 仅使用斜边上的一个点来计算旋转矩阵
#     # 假设斜边上的点是第二个点
#     vector_measured = translated_measured[1]
#     vector_true = translated_true[1]
    
#     # 单位化向量
#     unit_vector_measured = vector_measured / np.linalg.norm(vector_measured)
#     unit_vector_true = vector_true / np.linalg.norm(vector_true)
    
#     # 计算两个向量之间的旋转矩阵
#     dot_product = np.dot(unit_vector_measured, unit_vector_true)
#     det = np.linalg.det([unit_vector_measured, unit_vector_true])
#     angle = np.arctan2(det, dot_product)
    
#     # 将弧度转换为度
#     angle_degrees = np.degrees(angle)
    
#     return angle_degrees

# # 示例坐标
# points_measured = np.array([[1, 1], [4, 1], [1, 4]])
# points_true = np.array([[2, 2], [5, 2], [2, 5]])




import numpy as np

def calculate_rotation_angle(triangle1, triangle2):
    # 计算两个三角形的中心点
    center1 = np.mean(triangle1, axis=0)
    center2 = np.mean(triangle2, axis=0)
    
    # 将三角形平移到原点
    triangle1_centered = triangle1 - center1
    triangle2_centered = triangle2 - center2
    
    # 计算两个三角形的向量
    vector1 = triangle1_centered[0]  # 取第一个点作为向量
    vector2 = triangle2_centered[0]  # 同上
    
    # 计算两个向量的夹角
    unit_vector1 = vector1 / np.linalg.norm(vector1)
    unit_vector2 = vector2 / np.linalg.norm(vector2)
    dot_product = np.dot(unit_vector1, unit_vector2)
    angle = np.arccos(dot_product)
    
    # 计算叉积确定旋转方向（顺时针或逆时针）
    cross_product = np.cross(unit_vector1, unit_vector2)
    
    # 根据叉积符号调整角度正负
    if cross_product < 0:
        angle = -angle
    
    # 将弧度转换为度
    angle_degrees = np.degrees(angle)
    
    return angle_degrees


if __name__ == "__main__":

    # 假设 points 是一个包含所有点坐标的 NumPy 数组
    points = [(2, 0), (6, 1), (1, 4), (5.5, 5), (10, 4), (2, 8), (6, 8), (11, 8)]  # Replace with your actual points
    points = np.array(points)
    p1, p2, can = find_vertex_points(points)
    print(p1, p2, can)

    # # 使用示例：
    # # points_measured = np.array([[8*m.sqrt(2), 4*m.sqrt(2)], [0, 4*m.sqrt(2)], [4*m.sqrt(2), 0]])
    # points_measured = np.array([[0, 0], [8, 8], [0, 8]])
    # points_true = np.array([[8, 0], [0, 0], [8, 8]])

    # # 计算旋转角度
    # angle = compute_rotation_angle(points_measured, points_true)
    # print("旋转角度（度）:", angle)

   
    
    # 定义两个三角形的顶点坐标
    triangle_reference = np.array([[0, 8], [0, 0], [8, 8]])
    # triangle_measured = np.array([[8, 0], [8, 8], [0, 0]])

    triangle_measured = np.array([can, p1, p2])


    # 计算旋转角度
    rotation_angle = calculate_rotation_angle(triangle_reference, triangle_measured)
    print(f"测量三角形需要旋转 {rotation_angle:.2f} 度来对齐到参考三角形。")


   