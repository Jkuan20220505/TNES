import os
import numpy as np
import random
import math

INF = 999999
D = 300

# 车辆和任务初始化
class Node():
    def __init__(self):
        self.st = 0
        self.ft = 0
        self.w = 0
        self.I = 0
        self.O = 0
        self.ind = 0
        self.outd = 0
        self.preds = []
        self.succs = []
        self.visit = 0
        self.EST = [0 for _ in range(M)]
        self.ranku = 0
        self.AverW = 0


class Car():
    def __init__(self,i, N ):
        self.tab = 0
        self.N = N
        self.id = i
        self.pos = 0
        self.vel = 0
        self.F = 0
        self.cyc = 0
        self.node = [Node() for _ in range(N)]  # 执行数据大小和输出率
        self.C = getDAG(rho,N)
        self.arr_t = 0
        self.R_exe = 0


    def reset(self):
        # self.pos = np.random.randint(1,50)
        # self.acc = np.random.randint(1,5)
        # self.vel = np.random.randint(1,10)
        self.F = np.random.randint(20,50)  # GHz
        self.cyc = 1000    # Cycles/bit
        self.arr_t = np.random.randint(0, 10) # /10
        self.R_exe = np.random.randint(20, 60)
        for i in range(self.N):
            self.node[i].w = np.random.randint(100, 300) # Mbit
            self.node[i].I = np.random.randint(50, 100) # Mbit
            self.node[i].O = np.random.randint(50, 100) # Mbit


def cross(matrix,start,end, n):
    if end - start <= 1:
        return
    half = int((end - start) / 2)
    for i in range(start,start+half):
        matrix[i - n][i] = np.random.randint(50, 100)
        matrix[i - n + half][i] = np.random.randint(50, 100)

    for i in range(start+half,end):
        matrix[i - n][i] = np.random.randint(50, 100)
        matrix[i - n - half][i] = np.random.randint(50, 100)

    cross(matrix, start - n, start + half - n, rho)
    cross(matrix, start + half - n, end - n, n)

def getDAG(rho,N):
    # ========================傅里叶DAG============================
    # 顶部二叉树
    matrix = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(rho):
        leftChild = i * 2 + 1
        rightChild = i * 2 + 2
        matrix[i][leftChild] = np.random.randint(50, 100)
        matrix[i][rightChild] = np.random.randint(50, 100)

    cross(matrix, N-rho, N, rho)

    # for i in range(N):
    #     print(matrix[i])
    return matrix

    # ========================高斯DAG==============================
    # communicationMatrix = [ [0 for _ in range(N)] for _ in range(N)]
    # end = rho - 1
    # k = 2
    # level = rho - 1
    #
    # for i in range(N):
    #     for j in range(N):
    #         if i == 0 and j > 0 and j < rho:
    #             communicationMatrix[i][j] = np.random.randint(50, 100)
    #
    # for i in range(N):
    #     for j in range(i,N):
    #         if i == end + 1:
    #             for a in range(i+1,i+1+rho-k):
    #                 communicationMatrix[i][a] = np.random.randint(50, 100)
    #             end = end + 1 + (rho - k)
    #             k += 1
    #             level -= 1
    #             break
    #         if i>0 and j == i+level:
    #             communicationMatrix[i][j] = np.random.randint(50, 100)
    #
    # # for i in range(N):
    # #     print(communicationMatrix[i])
    # return communicationMatrix


def build_graph(cars):
    graph = [[INF for _ in range(len(cars))] for _ in range(len(cars))]
    for i in range(len(cars)):
        for j in range(i + 1, len(cars)):
            distance = abs(V[i].pos - V[j].pos)
            if distance <= D:
                graph[i][j] =  1 #round(distance,2)
                graph[j][i] =  1 #round(distance,2)
    return graph


def txt_to_matrix(filename):
    data = []
    with open(filename, 'r') as f:  # with语句自动调用close()方法
        line = f.readline()
        while line:
            eachline = line.split()  ###按行读取文本文件，每行数据以列表形式返回
            read_data = [int(x) for x in eachline[0:N]]  # TopN概率字符转换为float型
            # read_data = list(map(float, eachline))
            data.append(read_data)
            line = f.readline()
        return data  # 返回数据为双列表形式


def random_indices(A, k):
    n = len(A)
    indices = random.sample(range(n), k)
    return indices




if __name__ == '__main__':
    max_N = 30
    M = 4
    nums = 100

    # rho = 4
    # task_number = int(2 * rho + rho * (math.log(rho) / math.log(2))-1)
    # getDAG(rho, task_number)

    for num in range(nums):
        path = "D:/MyPapers/[4]Paper/FFTCar" + str(M) + "/" + "Game" + str(num) + "/"
        if os.path.exists(path) == False:
            os.mkdir(path)

        # 车的信息
        local = [0 for _ in range(M)]
        for i in range(M):
            if i == 0:
                local[i] = 0
            else:
                local[i] = local[i - 1] + np.random.randint(51, 299)
        # rho = np.random.randint(5, 10)
        # task_number = int((rho * rho + rho - 2) / 2 )
        rho = 8
        task_number = int(int(2 * rho + rho * (math.log(rho) / math.log(2)))-1)
        V = [Car(i, task_number) for i in range(M)]

        # 示例用法
        A = [i for i in range(M)]
        # NUM_DAG = max(2, np.random.randint(M/3, M/2))
        NUM_DAG = np.random.randint(2, M)
        selected_indices = random_indices(A, NUM_DAG)
        print(selected_indices)

        for i in range(M):
            if i in selected_indices:
                V[i].tab = 1
            V[i].reset()
            V[i].pos = local[i]

        CAR = path + "car.txt"
        with open(CAR, "w") as f:  # 设置文件对象
            for v in V:  # 对于双层列表中的数据
                i = [v.id, v.pos, v.F, v.cyc, v.tab, v.N, v.R_exe]
                i = str(i).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'  # 将其中每一个列表规范化成字符串
                f.write(i)

        # 通信网络
        graph = build_graph(V)
        CAR_path = path + "carpath.txt"
        with open(CAR_path, "w") as f:  # 设置文件对象
            for v in range(M):  # 对于双层列表中的数据
                i = str(graph[v]).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'
                f.write(i)

        # 通信网络中的可靠性参数
        r = [[0 for _ in range(M)] for _ in range(M)]
        for i in range(M):
            for j in range(i, M):
                if graph[i][j] < INF:
                    r[j][i] = r[i][j] = np.random.randint(20, 50)

        CAR_R = path + "carR.txt"
        with open(CAR_R, "w") as f:  # 设置文件对象
            for v in range(M):  # 对于双层列表中的数据
                i = str(r[v]).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'
                f.write(i)

        # DAG任务的信息
        for v in V:
            if v.id in selected_indices:
                DAGs = path + "DAG_Car" + str(v.id) + ".txt"
                with open(DAGs, "w") as f:  # 设置文件对象
                    f.write(str(v.arr_t) + '\n')
                    w = []
                    I = []
                    O = []
                    for i in range(v.N):
                        w.append(v.node[i].w)
                        I.append(v.node[i].I)
                        O.append(v.node[i].O)

                    I = str(I).strip('[').strip(']').replace(',', '').replace('\'',
                                                                              '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(I)

                    w = str(w).strip('[').strip(']').replace(',', '').replace('\'',
                                                                              '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(w)

                    O = str(O).strip('[').strip(']').replace(',', '').replace('\'',
                                                                              '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(O)

                    for i in v.C:
                        i = str(i).strip('[').strip(']').replace(',', '').replace('\'',
                                                                                  '') + '\n'  # 将其中每一个列表规范化成字符串
                        f.write(i)

# 0 0 [0, 0, 0, 1, 0, 0] [0.002885831845584169] [0.17513463567333537] [0.17731051344301274]
# 0 2 [0, 0, 1, 1, 0, 0] [0.1802620817771068, 0.17879711279520633] [0.413754904743614, 0.43164685372784883] [0.41508935546036474, 0.43231896880244863]
# 0 1 [0, 1, 0, 0, 0, 0] [0.17859836265267498] [0.45663304473359984] [0.45795238169899954]
# 0 3 [1, 0, 0, 0, 1, 0] [0.17731051344301274, 0.1799653227975639] [0.4108434475747493, 0.4252483416654884] [0.4108434475747493, 0.4276329414165229]
# 0 4 [0, 0, 0, 1, 0, 0] [0.46041092636926945] [0.5494061416802742] [0.5524234465028782]
# Myal 0.37896297156366004 7 0.9997509010688032 1

# 5 0 [0, 0, 0, 1, 0, 0] [0.5494061416802742] [0.599645376130035] [0.6001424297885213]
# 5 1 [0, 0, 0, 1, 0, 0] [0.6007756625315244] [0.7026895381296105] [0.7031627604483064]
# 5 3 [0, 0, 1, 1, 0, 0] [0.7044871020452321, 0.7045641392383809] [0.9231473891265718, 0.9413517040052203] [0.924325338207642, 0.942598174599991]
# 5 2 [0, 1, 0, 0, 0, 0] [0.6021797438189226] [0.7391739634721017] [0.7418104875114446]
# 5 4 [0, 0, 0, 1, 0, 0] [0.9438203818835291] [1.124681625902668] [1.1255531720435754]
# Myal 0.742237619736802 6 0.99979151756316 1



# 0 0 [0, 0, 0, 1, 0, 0] [0.002885831845584169] [0.17513463567333537] [0.17731051344301274]
# 5 0 [0, 0, 1, 0, 0, 0] [0.0006448503944044924] 0.055048995472124704 [0.0555749628329239]
# 5 1 [0, 0, 1, 0, 0, 0] [0.0562450308405174] 0.16660772514103553 [0.16710847488864572]
# 0 2 [0, 0, 1, 1, 0, 0] [0.1802620817771068, 0.17879711279520633] [0.413754904743614, 0.43164685372784883] [0.41508935546036474, 0.43231896880244863]
# 0 1 [0, 1, 0, 0, 0, 0] [0.17859836265267498] [0.45663304473359984] [0.45795238169899954]
# 0 3 [1, 0, 0, 0, 1, 0] [0.17731051344301274, 0.1799653227975639] [0.4108434475747493, 0.4252483416654884] [0.4108434475747493, 0.4276329414165229]
# 0 4 [0, 0, 0, 1, 0, 0] [0.46041092636926945] [0.5494061416802742] [0.5524234465028782]
# 5 3 [0, 0, 1, 0, 0, 1] [0.16710847488864572, 0.43164685372784883] [0.4958854533059119, 0.6684344184946882] [0.4958854533059119, 0.6696808890894589]
# 5 2 [1, 0, 0, 0, 0, 0] [0.4108434475747493] [0.55275961523942] [0.5554973326749078]
# 5 4 [0, 0, 0, 1, 0, 0] [0.670903096372997] [0.8517643403921358] [0.8526358865330432]
# Myal 0.575120868971874 13 0.9995321073962878 2