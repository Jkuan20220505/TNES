import os
import numpy as np
import random

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
        self.C = getDAG(N)
        self.arr_t = 0
        self.R_exe = 0


    def reset(self):
        # self.pos = np.random.randint(1,50)
        # self.acc = np.random.randint(1,5)
        # self.vel = np.random.randint(1,10)
        self.F = np.random.randint(20,50)  # GHz
        self.cyc = 1000    # Cycles/bit【
        self.arr_t = np.random.randint(0, 10) # /10
        self.R_exe = np.random.randint(20, 30) #/10000
        for i in range(self.N):
            self.node[i].w = np.random.randint(100, 300) # Mbit
            self.node[i].I = np.random.randint(50, 100) # Mbit
            self.node[i].O = np.random.randint(50, 100) # Mbit


def getDAG(N):
    A1 = [1, 2, 3, 4] # 出度数
    A2 = [1, 2, 3, 4] # 入度数
    A3 = [3, 4, 5, 6] # 连边概率
    # n1 = 10  # 节点数量
    def prob_value(p):  # 一个按概率连边的函数
        q = int(10 * p)
        l1 = [1] * q + [0] * (10 - q)
        item = random.sample(l1, 1)[0]
        return item

    into_degree = [0] * N  # 节点入度列表
    out_degree = [0] * N  # 节点出度列表
    edges = []  # 存储边的列表
    # 拓扑序就按[1,n]的顺序，依次遍历加边
    for i in range(N - 1):
        for j in range(i + 1, N):
            a = random.choice(A1)
            b = random.choice(A2)
            p = random.choice(3)
            if i == 0 and j == N - 1:  # 不直连入口和出口
                continue
            prob = prob_value(0.1*p)  # 连边的概率取0.4
            if prob:
                if out_degree[i] < a and into_degree[j] < b:  # 限制节点的入度和出度不大于2
                    edges.append((i, j))  # 连边
                    into_degree[j] += 1
                    out_degree[i] += 1
    for node, Id in enumerate(into_degree):  # 给所有没有入边的节点添加入口节点作父亲
        if node != 0:
            if Id == 0:
                edges.append((0, node))
                out_degree[0] += 1
                into_degree[node] += 1
    for node, Od in enumerate(out_degree):  # 给所有没有出边的节点添加出口节点作儿子
        if node != N - 1:
            if Od == 0:
                edges.append((node, N - 1))
                out_degree[node] += 1
                into_degree[N - 1] += 1

    K = [[0 for _ in range(N)] for _ in range(N)]
    K = np.array(K)
    for node in edges:
        # print(node[0],node[1])
        K[node[0]][node[1]] = random.uniform(20, 100)  # Mbit
    C = []
    for num in K:
        C.append(list(num))
    print("C = ",C)

    return C


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
    M = 8
    nums = 100

    for num in range(nums):
        path = "D:/MyPapers/[4]Paper/P/" + str(M) + str(max_N) + "/" + "Game" + str(num) + "/"
        # path = "D:/MyPapers/[4]Paper/Car"+str(M)+"/" + "Game" + str(num) + "/"
        if os.path.exists(path) == False:
            os.mkdir(path)

        # 车的信息
        local = [0 for _ in range(M)]
        for i in range(M):
            if i == 0:
                local[i] = 0
            else:
                local[i] = local[i - 1] + np.random.randint(51, 299)

        V = [Car(i, np.random.randint(max_N - 5, max_N + 5)) for i in range(M)]

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
        CAR_path = path +  "carpath.txt"
        with open(CAR_path, "w") as f:  # 设置文件对象
            for v in range(M):  # 对于双层列表中的数据
                i = str(graph[v]).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'
                f.write(i)

        # 通信网络中的可靠性参数
        r = [[0 for _ in range(M)] for _ in range(M)]
        for i in range(M):
            for j in range(i, M):
                if graph[i][j] < INF:
                    r[j][i] = r[i][j] = np.random.randint(20, 30)

        CAR_R = path +  "carR.txt"
        with open(CAR_R, "w") as f:  # 设置文件对象
            for v in range(M):  # 对于双层列表中的数据
                i = str(r[v]).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'
                f.write(i)


        # DAG任务的信息
        for v in V:
            if v.id in selected_indices:
                DAGs = path + "DAG_Car" + str(v.id) + ".txt"
                with open(DAGs, "w") as f:  # 设置文件对象
                    f.write(str(v.arr_t)+ '\n')
                    w = []
                    I = []
                    O = []
                    for i in range(v.N):
                        w.append(v.node[i].w)
                        I.append(v.node[i].I)
                        O.append(v.node[i].O)

                    I = str(I).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(I)

                    w = str(w).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(w)

                    O = str(O).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'  # 将其中每一个列表规范化成字符串
                    f.write(O)

                    for i in v.C:
                        i = str(i).strip('[').strip(']').replace(',', '').replace('\'', '') + '\n'  # 将其中每一个列表规范化成字符串
                        f.write(i)








