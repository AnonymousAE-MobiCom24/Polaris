import numpy as np
import magpylib as magpy
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math
import pandas as pd
import pickle
from sklearn import preprocessing
import sys
from decimal import Decimal, ROUND_HALF_UP


class Simulator:
    def __init__(self) -> None:
        self.magnets = []
        self.sensors = {}

    def addMagnet(self, *srcs):
        """
        parameters:
            src : magpylib.magnet objects
        """
        for src in srcs:
            self.magnets.append(src)

    def addSensor(self, name, sen):
        """
        parameters:
            name : the sensor's name, a string
            sens : a magpylib.Sensor object
        """
        self.sensors[name] = sen

    def simulate(self, x_offset, sensor_height, dis_before_tag, dis_after_tag, deg, speed, sample_rate):
        """
        parameters:         unit
            x_offset:       mm
            sensor_height:  mm
            dis_before_tag: mm
            dis_after_tag:  mm
            deg:            degree
            speed:          km/h
            sample_rate:    Hz
        return:
            res: A dictionary
            { "the sensor's name": [the sensor's reading] }
        """
        rotation_vector = np.array((math.cos(deg / 180 * math.pi), math.sin(deg / 180 * math.pi),
                                    0))  # 存在误差，如sensor从4Ntag中间竖直穿过时x轴会有异常读数，不过加噪后应该可以忽略
        rotation_object = R.from_euler('z', deg - 90, degrees=True)
        for sensor in self.sensors.values():
            sensor.rotate(rotation_object, anchor=(0, 0, 0))
            sensor.move((x_offset, 0, sensor_height))
          
            sensor.move(-dis_before_tag * rotation_vector)
            # print(sensor.position)
            # magpy.show(src, sensor)

        res = {}
        for name in self.sensors.keys():
            res[name] = []

        sensor_shift = 0
        shift_step = 1 / sample_rate * speed * 100 / 3.6
        shift_range = dis_before_tag + dis_after_tag

        while sensor_shift <= shift_range:
            for name in self.sensors.keys():
                res[name].append(magpy.getB(self.magnets, self.sensors[name], sumup=True))
                self.sensors[name].move(shift_step * rotation_vector)
            sensor_shift += shift_step
            # magpy.show(self.magnets[0], self.sensors['Sensor 0'])
        #   magpy.show(self.magnets[0], self.sensors['Sensor 0'], animation=True)

        return res, self.sensors


def draw2Dfigure_single(x_data, title, label, color='blue', fs=20):  # copied from simulation.py
    plt.figure(figsize=[10, 8])
    x = [i for i in range(1, len(x_data) + 1)]
    plt.plot(x, x_data, color=color, label=label, linewidth=2.5)
    plt.tick_params(labelsize=20)
    plt.legend(fontsize=20)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    # plt.title(title, fontsize=fs)
    plt.tight_layout()
    plt.show()
    return


def draw2Dfigure_xyz(x_data, y_data, z_data, fs=20):  # copied from simulation.py
    plt.figure(figsize=[12, 8])
    x = [i for i in range(1, len(x_data) + 1)]
    plt.plot(x, x_data, color='red', label='x-axis data', linewidth=3)
    plt.plot(x, y_data, color='blue', label='y-axis data', linewidth=3)
    plt.plot(x, z_data, color='green', label='z-axis data', linewidth=3)
    plt.tick_params(labelsize=fs)
    plt.legend(fontsize=fs)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    plt.tight_layout()
    plt.show()
    return


def draw2Dfigure_subplot(x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, fs=20):  # copied from simulation.py
    # plt.figure(figsize=[10, 8])
    plt.subplot(3, 1, 1)
    x = [i for i in range(1, len(x_1) + 1)]
    plt.plot(x, x_1, color='red', label='x-axis data', linewidth=3)
    plt.plot(x, y_1, color='blue', label='y-axis data', linewidth=3)
    plt.plot(x, z_1, color='green', label='z-axis data', linewidth=3)
    plt.tick_params(labelsize=fs)
    plt.legend(fontsize=fs)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    # plt.tight_layout()
    # plt.show()

    plt.subplot(3, 1, 2)
    x = [i for i in range(1, len(x_2) + 1)]
    plt.plot(x, x_2, color='red', label='x-axis data', linewidth=3)
    plt.plot(x, y_2, color='blue', label='y-axis data', linewidth=3)
    plt.plot(x, z_2, color='green', label='z-axis data', linewidth=3)
    plt.tick_params(labelsize=fs)
    # plt.legend(fontsize=fs)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    # plt.tight_layout()
    # plt.show()

    plt.subplot(3, 1, 3)
    x = [i for i in range(1, len(x_3) + 1)]
    plt.plot(x, x_3, color='red', label='x-axis data', linewidth=3)
    plt.plot(x, y_3, color='blue', label='y-axis data', linewidth=3)
    plt.plot(x, z_3, color='green', label='z-axis data', linewidth=3)
    plt.tick_params(labelsize=fs)
    # plt.legend(fontsize=fs)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    # plt.tight_layout()
    plt.show()

    return

def draw2Dfigure(x_data, y_data, z_data, all_data, angle, offset, fs=25):  # copied from simulation.py
    plt.figure(figsize=[12, 8])
    x = [i for i in range(1, len(x_data) + 1)]
    plt.plot(x, x_data, color='red', label='x-axis data', linewidth=3)
    plt.plot(x, y_data, color='blue', label='y-axis data', linewidth=3)
    plt.plot(x, z_data, color='green', label='z-axis data', linewidth=3)
    # plt.plot(x, all_data, color='black', label='vector sum', linewidth=3.5)
    plt.tick_params(labelsize=20)
    plt.legend(fontsize=20)
    plt.xlabel('Sample', fontsize=fs)
    plt.ylabel('Sensor reading (uT)', fontsize=fs)
    # plt.title(title, fontsize=fs)
    plt.tight_layout()
    # save
    # plt.savefig('Codes/CouplingSimulation/Simulate_Varying_Orientation/figs_1030/F_' + str(angle) + "_" + str(offset) + '.pdf')
    # plt.close()
    plt.show()
    return



def simulation_process(rotation_angle, lateral_offset = 0, deg = 90):
    magpy.defaults.display.style.magnet.update(
    magnetization_show=True,
    magnetization_color_middle='grey',
    magnetization_color_mode='bicolor',
)
    simulator = Simulator()

    # Magnet configuration
    # src = magpy.magnet.Cuboid(magnetization=(0,  10000000, 0), dimension=[3, 3, 3], position=(0, 0, 0))
    src = magpy.magnet.Cylinder(magnetization=(0, 2000000, 0), dimension=(2, 1), position=(0, 0, 0))
    # src = magpy.magnet.Sphere(magnetization=(0, 10000000, 0), diameter=15, position=(0, 0, 0))


    sens = magpy.Sensor(pixel=[(0, 0, 0)], position=(lateral_offset, 0, 15), orientation=R.from_euler('x', 180, degrees=True))

    rotation_object = R.from_euler('z', rotation_angle, degrees=True)
    src.rotate(rotation_object, anchor=(0, 0, 0))

    # rotation_object = R.from_euler('z', rotation_angle, degrees=True)
    # sens.rotate(rotation_object, anchor=(0, 0, 0))

    simulator.addMagnet(src)

    # 组成传感器组sensor bar
    # sensor_pos = np.arange(-900, 900, 150)
    # for i in range(len(sensor_pos)):
    #     sens = magpy.Sensor(pixel=[(0, 0, 0)], position=(sensor_pos[i], 0, 0))
    #     simulator.addSensor('sensor'+str(i), sens)

    simulator.addSensor('Sensor 0', sens)

    # 给定运动参数进行模拟
    offset = 0

    # magpy.show(sens, backend="plotly")
    # magpy.show(src, sens)

    result, sensors = simulator.simulate(x_offset=offset, sensor_height=0, dis_before_tag=150, dis_after_tag=150, deg=deg, speed=3.6,
                                sample_rate=100)
    # for name in sensors.keys():
    #     sensors[name].move(np.linspace((0, -200, 0), (0, 0, 0), 100))

    # magpy.show(src, sensors['Sensor 0'])

    # rotate the magnet

    # 取sensor0的读数
    # sen_name_list = ['sensor 0')]
    sen_name_list = ['Sensor 0']
    all_x_data, all_y_data, all_z_data = [], [], []
    total_B = []
    for sen_name in sen_name_list:
        B_x = [result[sen_name][i][0] for i in range(len(result[sen_name]))] 
        B_y = [result[sen_name][i][1] for i in range(len(result[sen_name]))] 
        B_z = [result[sen_name][i][2] for i in range(len(result[sen_name]))]  # 直接取了sensor0的z轴读数
        
        for i in range(len(B_x)):
            x_num = Decimal(B_x[i])
            B_x[i] = float(x_num.quantize(Decimal('0.0001'), rounding=ROUND_HALF_UP))
            y_num = Decimal(B_y[i])
            B_y[i] = float(y_num.quantize(Decimal('0.0001'), rounding=ROUND_HALF_UP))
            z_num = Decimal(B_z[i])
            B_z[i] = float(z_num.quantize(Decimal('0.0001'), rounding=ROUND_HALF_UP))

        for i in range(len(B_x)):
            if(abs(B_x[i]) < 1):
                B_x[i] = 0
        for i in range(len(B_y)):
            if(abs(B_y[i]) < 1):
                B_y[i] = 0
        for i in range(len(B_z)):
            if(abs(B_z[i]) < 1):
                B_z[i] = 0
        

        all_x_data.append(B_x)
        all_y_data.append(B_y)
        all_z_data.append(B_z)
        # add gauss noise to the data
        # B_x = np.array(B_x) + np.random.normal(0, 5, len(B_x))
        # B_y = np.array(B_y) + np.random.normal(0, 5, len(B_y))
        # B_z = np.array(B_z) + np.random.normal(0, 5, len(B_z))




        # draw the sum of the three axis
        
        for i in range(len(B_x)):
            # total_B.append(math.sqrt(B_x[i]**2 + B_y[i]**2 + B_z[i]**2))
            total_B.append([B_x[i], B_y[i], B_z[i]])
        # draw2Dfigure_single(total_B, "Vector sum", 'vector sum', color='black')

        # draw2Dfigure_single(B_x, "X-axis data of %s" % sen_name, 'x-axis data', color="red")
        # draw2Dfigure_single(B_y, "Y-axis data of %s" % sen_name, 'y-axis data', color='blue')
        # draw2Dfigure_single(B_z, "Z-axis data of %s" % sen_name, 'z-axis data', color='green')
        
        # draw2Dfigure(B_x, B_y, B_z, total_B, angle=rotation_angle, offset=lateral_offset)     
     
    return B_x, B_y, B_z, total_B



def xyz_sensor(file_name, sensor_num, flag=0):
    file = pd.read_csv(file_name)
    df = pd.DataFrame(file)
    total_sensor_list = []
    sensor_x, sensor_y, sensor_z = [], [], []
    for i in range(len(df)):
        document = df[i:i+1]
        sensor = list(map(float, document[sensor_num][i][1:-1].split(', ')))
        '''
        sensor2 = list(map(float, document['Sensor 2'][i][1:-1].split(', ')))
        sensor3 = list(map(float, document['Sensor 3'][i][1:-1].split(', ')))
        sensor4 = list(map(float, document['Sensor 4'][i][1:-1].split(', ')))
        sensor5 = list(map(float, document['Sensor 5'][i][1:-1].split(', ')))
        sensor6 = list(map(float, document['Sensor 6'][i][1:-1].split(', ')))
        sensor7 = list(map(float, document['Sensor 7'][i][1:-1].split(', ')))
        sensor8 = list(map(float, document['Sensor 8'][i][1:-1].split(', ')))
        '''
        sensor_x.append(sensor[0])
        sensor_y.append(sensor[1])
        sensor_z.append(sensor[2])
        total_sensor_list.append([sensor[0], sensor[1], sensor[2]])
    if(flag):
        sensor_x = preprocessing.scale(sensor_x)
        sensor_y = preprocessing.scale(sensor_y)
        sensor_z = preprocessing.scale(sensor_z)
    return sensor_x, sensor_y, sensor_z, total_sensor_list


if __name__ == "__main__":
    gt_datas = []
    for i in range(0, 18):
        for j in range(-2, 3):
            Bx, By, Bz, Tb = simulation_process(rotation_angle=-i*20, lateral_offset=j*5)
            gt_datas.append(Tb[70:230:2])

    # 将列表保存到文件中
    # with open('template_8_1.pkl', 'wb') as f:
    #     pickle.dump(gt_datas, f)


    with open("template_18_5_80.txt", "w") as file:
        for layer in gt_datas:
            for row in layer:
                file.write(" ".join(map(str, row)) + "\n")
            file.write("\n")
