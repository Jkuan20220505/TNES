import math
import os
import numpy as np
from math import *
import heapq
import matplotlib.pyplot as plt
from scipy.integrate import quad
import sympy as sp
import random

INF = 99999
def txt_to_matrix(filename):
    data = []
    with open(filename, 'r') as f:  # with语句自动调用close()方法
        line = f.readline()
        while line:
            eachline = line.split()  ###按行读取文本文件，每行数据以列表形式返回
            read_data = [int(x) for x in eachline[0:1000]]  # TopN概率字符转换为float型
            data.append(read_data)
            line = f.readline()
        return data  # 返回数据为双列表形式


def find_paths(adj_matrix, a, b, k):
    n = len(adj_matrix)
    path_count = 0
    paths = []
    def backtrack(node, length, path):
        nonlocal path_count
        nonlocal paths
        if length == k and node == b:
            path_count += 1
            paths.append(path[:])
        if length < k:
            for i in range(n):
                if adj_matrix[node][i] == 1:
                    path.append(i)
                    backtrack(i, length + 1, path)
                    path.pop()



    path = [a]
    backtrack(a, 0, path)
    PATHS = []
    for path in paths:
        PATH = []
        for i in range(len(path)-1):
            PATH.append([path[i],path[i+1]])
        PATHS.append(PATH)
    # print(PATHS)
    return path_count, paths

class NODE():
    def __init__(self):
        self.ind = 0
        self.outd = 0
        self.preds = []
        self.succs = []
        self.W = 0
        self.I = 0
        self.O = 0

class HEFT():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.st = []
        self.ft = []
        self.runcpu = [0 for _ in range(M)]
        self.backup = 0
        self.R = 1

class MAXR():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.st = []
        self.ft = []
        self.runcpu = [0 for _ in range(M)]
        self.backup = 0
        self.R = 1

class LOCA():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.backup = 0
        self.R = 1

class LOCAL():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.backup = 0
        self.R = 1

class Rand():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.st = []
        self.ft = []
        self.runcpu = [0 for _ in range(M)]
        self.backup = 0
        self.R = 1
        self.visit = 0
        self.indegree = 0

class MYAL():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.st = []
        self.ft = []
        self.runcpu = [0 for _ in range(M)]
        self.OCT = [0 for _ in range(M)]
        self.rankOCT = 0
        self.averW = []

        self.cpuR = [1 for _ in range(M)]
        self.minR = 1
        self.newR = 1
        self.backup = 0
        self.num = 0
        self.R = 1
        self.ni_fail_R = 1

class MYAL1():
    def __init__(self):
        self.ranku = 0
        self.R_lb_req = 0
        self.st = []
        self.ft = []
        self.runcpu = [0 for _ in range(M)]
        self.OCT = [0 for _ in range(M)]
        self.rankOCT = 0
        self.averW = []

        self.cpuR = [1 for _ in range(M)]
        self.minR = 1
        self.newR = 1
        self.backup = 0
        self.num = 0
        self.R = 1
        self.ni_fail_R = 1


class DAGS():
    def __init__(self,DAGN):
        self.C = []
        self.node = [NODE() for _ in range(DAGN)]
        self.heft = [HEFT() for _ in range(DAGN)]
        self.maxr = [HEFT() for _ in range(DAGN)]
        self.loca = [LOCA() for _ in range(DAGN)]
        self.local = [LOCAL() for _ in range(DAGN)]
        self.rand = [Rand() for _ in range(DAGN)]
        self.myal = [MYAL() for _ in range(DAGN)]
        self.myal1 = [MYAL1() for _ in range(DAGN)]
        self.SL = 0
        self.n = 0


class Car():
    def __init__(self):
        self.arr = []
        self.pos = 0
        self.F = 0
        self.namoda = 0
        self.dags = []
        self.Task = []

    def resetCar(self, Car_data):
        # [v.pos, v.f, v.namoda,v.Netnamoda]
        self.pos = Car_data[0]
        self.F = Car_data[1] / 100   # GHz
        self.namoda = Car_data[2] / 100000

    def resetDAGS(self,CARDAGSDATAS):
        for k in range(len(CARDAGSDATAS)):
            DAGN = len(CARDAGSDATAS[k][0])
            self.dags.append( DAGS(DAGN) ) # 初始化类

        for k in range( len(CARDAGSDATAS) ):
            CARDAGSDATA = CARDAGSDATAS[k]
            DAGN = len(CARDAGSDATA[0])
            I = [c / 1000 for c in CARDAGSDATA[0]]
            W = [c / 1000 for c in CARDAGSDATA[1]]
            O = [c / 1000 for c in CARDAGSDATA[2]]
            C = []
            for l in CARDAGSDATA[3:]:
                C.append([ c for c in l])
            self.dags[k].C = C.copy()
            for i in range(DAGN):
                self.dags[k].node[i].I = I[i]
                self.dags[k].node[i].W = W[i]
                self.dags[k].node[i].O = O[i]
                for j in range(DAGN):
                    if C[i][j] > 0:
                        self.dags[k].node[j].preds.append(i)
                        self.dags[k].node[j].ind += 1
                    if C[i][j] > 0:
                        self.dags[k].node[i].succs.append(j)
                        self.dags[k].node[i].outd += 1

def I_T_R(ma, cara, k, ni):
    if ma == cara:
        T, R = 0, 1
    else:
        _, path = find_paths(net, ma, cara, pathnum[ma][cara])
        minsendtime, thesendR = INF, INF
        for a in range(len(path)):
            sendtime = 0
            sendR = 1
            for i in range(len(path[a])-1):
                h = A * pow(1 / abs(V[path[a][i]].pos - V[path[a][i+1]].pos), 2)
                rate = bandwidth * log((1 + P * h / noise), 2)
                sendtime += V[cara].dags[k].node[ni].I  / rate
                sendR *= math.exp(-Netnomoda[path[a][i]][path[a][i+1]] * V[cara].dags[k].node[ni].I / rate)
            if minsendtime > sendtime:
                minsendtime = sendtime
                thesendR = sendR
        T, R = minsendtime, thesendR
    return T, R

def W_T_R(ma, cara, k, ni):
    localtime = V[cara].dags[k].node[ni].W * bit * cyc / V[ma].F / cyc_s
    localR = math.exp(-V[ma].namoda  * localtime)
    # print(localtime,localR,"执行可靠性====")
    return localtime,localR

def O_T_R(ma, cara, k, ni):
    if ma == cara:
        T, R = 0, 1
    else:
        _, path = find_paths(net, ma, cara, pathnum[ma][cara])
        minsendtime, thesendR = INF, 1
        # print(len(path))
        for a in range(len(path)):
            sendtime = 0
            sendR = 1
            for i in range(len(path[a])-1):
                h = A * pow(1 / abs(V[path[a][i]].pos - V[path[a][i+1]].pos), 2)
                rate = bandwidth * log((1 + P * h / noise), 2)
                sendtime += V[cara].dags[k].node[ni].O  / rate
                sendR *= math.exp(-(Netnomoda[path[a][i]][path[a][i+1]]) * V[cara].dags[k].node[ni].O / rate)

            if minsendtime > sendtime:
                minsendtime = sendtime
                thesendR = sendR
        T, R = minsendtime, thesendR
        # print(T, R,"传输可靠性")
    return T, R


def Ranku(car, k, DAG):
    # 计算平均速率
    rate = []
    for cara in range(M):
        for carb in range(M):
            if net[cara][carb] > 0:
                h = A * pow(1 / abs(V[cara].pos - V[carb].pos), 2)
                rate.append(bandwidth * log((1 + P * h / noise), 2))
    averate = sum(rate) / len(rate)
    # 计算任务优先级
    N = len(DAG)
    order = []
    ranku = []
    for i in range(N - 1, -1, -1):
        if len(V[car].dags[k].node[i].succs) > 0:
            MAX = 0
            for succ in V[car].dags[k].node[i].succs:
                temp = V[car].dags[k].heft[succ].ranku
                if MAX < temp:
                    MAX = temp
            V[car].dags[k].heft[i].ranku = MAX + V[car].dags[k].node[i].I / averate + V[car].dags[k].node[i].O / averate
            sumw = 0
            for ma in range(M):
                sumw += V[car].dags[k].node[i].W * bit * cyc / V[ma].F / cyc_s
            V[car].dags[k].heft[i].ranku += sumw / M
        else:
            V[car].dags[k].heft[i].ranku = V[car].dags[k].node[i].I / averate + V[car].dags[k].node[i].O / averate
            sumw = 0
            for ma in range(M):
                sumw += V[car].dags[k].node[i].W * bit * cyc / V[ma].F / cyc_s
            V[car].dags[k].heft[i].ranku += sumw / M
        order.append(i)
        ranku.append(V[car].dags[k].heft[i].ranku)
    order_ranku = dict(zip(order, ranku))
    res = sorted(order_ranku.items(), key=lambda d: d[1], reverse=True)  # 按开始时间降序排列
    order_ranku_queue = []
    for i in range(len(order_ranku)):
        l = res[i][0]
        order_ranku_queue.append(l)
    # print(order_ranku_queue)
    return order_ranku_queue


def LOCALMaxR():
    RG = [[1 for _ in range(K)] for _ in range(M)]  # 记录所有DAG的可靠性
    Arrival = [arr[i]/100 for i in range(M)]
    times = []
    backups = 0
    for cara in range(M):
        Time = 0

        for k in range(K):
            DAG = V[cara].dags[k].C
            DAG = np.array(DAG)
            N = len(DAG)
            eachR = 1
            for ni in range(N):
                t =  V[cara].dags[k].node[ni].W * bit * cyc / V[cara].F / cyc_s
                V[cara].dags[k].local[ni].R = math.exp(-V[cara].namoda * t)
                V[cara].dags[k].local[ni].backup += 1
                V[cara].dags[k].local[ni].R_lb_req = pow(R_req, 1 / N)
                while V[cara].dags[k].local[ni].R < V[cara].dags[k].local[ni].R_lb_req:
                    V[cara].dags[k].local[ni].R = 1 - ((1 - V[cara].dags[k].local[ni].R) * (1 - V[cara].dags[k].local[ni].R))
                    V[cara].dags[k].local[ni].backup += 1
                eachR *= V[cara].dags[k].local[ni].R
                Time += t * V[cara].dags[k].local[ni].backup
                backups += V[cara].dags[k].local[ni].backup
            RG[cara][k] *= eachR
        times.append( Time + Arrival[cara] )
    AverRG = []
    for i in range(M):
        AverRG.append( sum(RG[i]) / len(RG[i]))
    print("Local+MaxR", sum(times)/M, backups-NUM, sum(AverRG)/len(AverRG))

# RR
def LOCALAL():
    RG = [[1 for _ in range(K)] for _ in range(M)]  # 记录所有DAG的可靠性
    Arrival = [arr[i]/100 for i in range(M)]
    times = []
    backups = 0
    for cara in range(M):
        Time = 0
        for k in range(K):
            DAG = V[cara].dags[k].C
            DAG = np.array(DAG)
            N = len(DAG)
            eachR = 1
            each = 0
            for ni in range(N):
                t =  V[cara].dags[k].node[ni].W * bit * cyc / V[cara].F / cyc_s
                V[cara].dags[k].loca[ni].R = math.exp(-V[cara].namoda * t)
                V[cara].dags[k].loca[ni].backup += 1
                V[cara].dags[k].loca[ni].R_lb_req = pow(R_req / eachR, 1 / (N - each))
                while V[cara].dags[k].loca[ni].R < V[cara].dags[k].loca[ni].R_lb_req:
                    V[cara].dags[k].loca[ni].R = 1 - ((1 - V[cara].dags[k].loca[ni].R) * (1 - V[cara].dags[k].loca[ni].R))
                    V[cara].dags[k].loca[ni].backup += 1
                eachR *= V[cara].dags[k].loca[ni].R
                each += 1
                Time += t * V[cara].dags[k].loca[ni].backup
                backups += V[cara].dags[k].loca[ni].backup
            RG[cara][k] *= eachR
        times.append( Time + Arrival[cara] )
    AverRG = []
    for i in range(M):
        AverRG.append(sum(RG[i]) / len(RG[i]))
    print("Local+RR", sum(times)/M, backups-NUM, sum(AverRG)/len(AverRG))


def HEFTMaxR():
    RG = [[1 for _ in range(K)] for _ in range(M)]  # 记录所有DAG的可靠性
    Avail = [0 for _ in range(M)]
    carK = [0 for _ in range(M)]

    HEFTArrival = [arr[i]/100 for i in range(M)]
    numofdags = 0  # 调度的DAG数量
    cara = 0
    maxft = 0
    while True:
        if carK[cara] >= K:
            HEFTArrival[cara] = INF
        cara = HEFTArrival.index(min(HEFTArrival))  # 得到要调度的车，和它的DAG
        if HEFTArrival[cara] >= INF:
            # print(numofdags)
            break
        k = carK[cara]
        DAG = V[cara].dags[k].C
        DAG = np.array(DAG)

        ranku = Ranku(cara, k, DAG)
        eachR = 1
        for ni in ranku:
            ni_R = 0
            V[cara].dags[k].maxr[ni].R_lb_req = pow(R_req, 1 / (len(DAG)))
            while ni_R < V[cara].dags[k].maxr[ni].R_lb_req:
                EST = [0 for _ in range(M)]
                EFT = [0 for _ in range(M)]
                O_t_ = []
                I_R_, W_R_, O_R_ = [], [], []
                max_pred_FT = 0
                for pred, val in enumerate(DAG[:, ni]):
                    if val > 0:
                        for ft in V[cara].dags[k].maxr[pred].ft:
                            if max_pred_FT < ft:
                                max_pred_FT = ft
                for ma in range(M):
                    I_t, I_R = I_T_R(ma, cara, k, ni)
                    W_t, W_R = W_T_R(ma, cara, k, ni)
                    O_t, O_R = O_T_R(ma, cara, k, ni)
                    O_t_.append(O_t)
                    I_R_.append(I_R)
                    W_R_.append(W_R)
                    O_R_.append(O_R)
                    maxWaitPredReady = max_pred_FT + I_t
                    EST[ma] = max(Avail[ma], maxWaitPredReady, HEFTArrival[cara] )
                    EFT[ma] = EST[ma] + W_t + O_t
                minEFT = INF
                mincar = cara
                for mb in range(M):
                    if EFT[mb] < minEFT and V[cara].dags[k].maxr[ni].runcpu[mb] == 0:
                        minEFT = EFT[mb]
                        mincar = mb
                Avail[mincar] = minEFT - O_t_[mincar]
                V[cara].dags[k].maxr[ni].st.append(EST[mincar])
                V[cara].dags[k].maxr[ni].ft.append(EFT[mincar])
                V[cara].dags[k].maxr[ni].runcpu[mincar] = 1
                V[cara].dags[k].maxr[ni].backup += 1

                if maxft < EFT[mincar]:  # 记录时间轴
                    maxft = EFT[mincar]

                I_R = I_R_[mincar]
                W_R = W_R_[mincar]
                O_R = O_R_[mincar]
                ni_R = 1 - ((1 - ni_R) * (1 - I_R * W_R * O_R))
            V[cara].dags[k].maxr[ni].R *= ni_R
            eachR *= V[cara].dags[k].maxr[ni].R

        if eachR < R_req:
            print([cara, k], "可靠性不满足要求")
        RG[cara][k] *= eachR
        HEFTArrival[cara] = max(HEFTArrival[cara], maxft)
        carK[cara] += 1
        numofdags += 1
    backups = 0
    AverRG = []
    for i in range(M):
        AverRG.append(sum(RG[i]) / len(RG[i]))
        for j in range(K):
            DAG = np.array(V[i].dags[j].C)
            for ni in range(len(DAG)):
                backups += V[i].dags[j].maxr[ni].backup

    print("HEFT+MaxR", sum(Avail) / M, backups-NUM, sum(AverRG)/len(AverRG), numofdags)

def HEFTAl():
    carK = [0 for _ in range(M)]
    HEFTArrival = [ arr[i]/100 for i in range(M) ]  # 记录DAG的到达时间，即前一个DAG的完成时间
    RG = [[1 for _ in range(K)] for _ in range(M)]  # 记录所有DAG的可靠性
    numofdags = 0  # 调度的DAG数量
    maxft = 0
    Avail = [0 for _ in range(M)]  # 处理器的空闲时间
    # Avail = [0.1368846452015068, 0.1368846452015068, 0.1368846452015068, 0.1368846452015068]
    for cara in range(K):
        if carK[cara] >= K:
            HEFTArrival[cara] = INF
        cara = HEFTArrival.index(min(HEFTArrival))  # 得到要调度的车，和它的DAG
        if HEFTArrival[cara] >= INF:
            # print(numofdags)
            break
        k = carK[cara]

        # k = 0
        DAG = V[cara].dags[k].C
        DAG = np.array(DAG)
        ranku = Ranku(cara, k, DAG)
        eachR = 1
        each = 0
        for ni in ranku:
            ni_R = 0
            V[cara].dags[k].heft[ni].R_lb_req = pow( R_req / eachR, 1 / (len(DAG) - each))
            eft = []
            while ni_R < V[cara].dags[k].heft[ni].R_lb_req:
                EST = [0 for _ in range(M)]
                EFT = [0 for _ in range(M)]
                O_t_ = []
                I_R_, W_R_, O_R_ = [], [], []
                max_pred_FT = 0
                for pred, val in enumerate(DAG[:, ni]):
                    if val > 0:
                        for ft in V[cara].dags[k].heft[pred].ft:
                            if max_pred_FT < ft:
                                max_pred_FT = ft
                for ma in range(M):
                    I_t, I_R = I_T_R(ma, cara, k, ni)
                    W_t, W_R = W_T_R(ma, cara, k, ni)
                    O_t, O_R = O_T_R(ma, cara, k, ni)
                    O_t_.append(O_t)
                    I_R_.append(I_R)
                    W_R_.append(W_R)
                    O_R_.append(O_R)
                    maxWaitPredReady = max_pred_FT + I_t
                    EST[ma] = max(Avail[ma], maxWaitPredReady, HEFTArrival[cara] )
                    EFT[ma] = EST[ma] + W_t + O_t
                minEFT = INF
                mincar = cara
                for mb in range(M):
                    if EFT[mb] < minEFT and V[cara].dags[k].heft[ni].runcpu[mb] == 0:
                        minEFT = EFT[mb]
                        mincar = mb
                Avail[mincar] = EFT[mincar] - O_t_[mincar]
                eft.append(Avail[mincar])
                V[cara].dags[k].heft[ni].st.append(EST[mincar])
                V[cara].dags[k].heft[ni].ft.append(EFT[mincar])
                V[cara].dags[k].heft[ni].runcpu[mincar] = 1
                V[cara].dags[k].heft[ni].backup += 1

                if maxft < EFT[mincar]:  # 记录时间轴
                    maxft = EFT[mincar]
                I_R = I_R_[mincar]
                W_R = W_R_[mincar]
                O_R = O_R_[mincar]
                ni_R = 1 - ((1 - ni_R) * (1 - I_R * W_R * O_R))

            V[cara].dags[k].heft[ni].R *= ni_R
            eachR *= V[cara].dags[k].heft[ni].R
            each += 1

            print(cara, ni,  V[cara].dags[k].heft[ni].runcpu, V[cara].dags[k].heft[ni].st, eft, V[cara].dags[k].heft[ni].ft)

        if eachR < R_req:
            print([cara,k], "可靠性不满足要求")
        RG[cara][k] *= eachR
        HEFTArrival[cara] = max(HEFTArrival[cara], maxft)
        carK[cara] += 1
        numofdags += 1
    backups = 0
    AverRG = []
    for i in range(M):
        AverRG.append(sum(RG[i])/len(RG[i]))
        for j in range(K):
            DAG = np.array(V[i].dags[j].C)
            for ni in range(len(DAG)):
                backups += V[i].dags[j].heft[ni].backup
    print("HEFT+RR",sum(Avail)/M, backups-NUM, sum(AverRG)/len(AverRG),numofdags)



# the minimum backups of the worst case
def Minbackups(car, k, DAG):
    # 车 car 的第 k 个 DAG
    for ni in range(len(DAG)):
        for ma in range(M):
            I_t, I_R = I_T_R(ma, car, k, ni)
            W_t, W_R = W_T_R(ma, car, k, ni)
            O_t, O_R = O_T_R(ma, car, k, ni)
            V[car].dags[k].myal[ni].cpuR[ma] = I_R * W_R * O_R
        V[car].dags[k].myal[ni].minR = min(V[car].dags[k].myal[ni].cpuR)
        V[car].dags[k].myal[ni].newR = min(V[car].dags[k].myal[ni].cpuR)
        V[car].dags[k].myal[ni].num += 1
        V[car].dags[k].myal1[ni].minR = V[car].dags[k].myal[ni].minR
        V[car].dags[k].myal1[ni].newR = V[car].dags[k].myal[ni].newR
        V[car].dags[k].myal1[ni].num = V[car].dags[k].myal[ni].num


    RG = 0
    newR = [V[car].dags[k].myal[ni].newR for ni in range(len(DAG))]
    while RG < R_req:
        RiG = []
        for ni in range(len(DAG)):
            newR[ni] = 1 - (1-newR[ni])*(1-V[car].dags[k].myal[ni].minR)
            rig = 1
            for i in range(len(DAG)):
                rig *= newR[i]
            RiG.append(rig)
            newR[ni] = V[car].dags[k].myal[ni].newR
        ni = RiG.index(max(RiG))
        V[car].dags[k].myal[ni].newR = 1 - ( 1 - V[car].dags[k].myal[ni].newR )*( 1 - V[car].dags[k].myal[ni].minR )
        V[car].dags[k].myal[ni].num += 1

        V[car].dags[k].myal1[ni].newR = V[car].dags[k].myal[ni].newR
        V[car].dags[k].myal1[ni].num = V[car].dags[k].myal[ni].num
        RG = max(RiG)

def DSD2():
    Avail = [0 for _ in range(M)]  # 处理器的空闲时间
    carK = [0 for _ in range(M)]   # 用来表示第几个DAG
    Arrival = [ arr[i]/100 for i in range(M) ]  # 记录DAG的到达时间，即前一个DAG的完成时间
    RG = [ [1 for _ in range(K)] for _ in range(M) ]  # 记录所有DAG的可靠性
    numofdags = 0  # 调度的DAG数量
    cara = 0
    maxft = 0
    sumET  = 0
    while True:
        if carK[cara] >= K:
            Arrival[cara] = INF
        # print(Arrival)
        cara = Arrival.index(min(Arrival))  # 得到要调度的车，和它的DAG
        if Arrival[cara] >= INF:
            # print(numofdags)
            break
        k = carK[cara]
        if k >= K:
            continue
        DAG = np.array(V[cara].dags[k].C)  # 选定的当前要去调度的DAG，不能影响其正常调度
        task = V[cara].Task[k].copy()
        # task 保存车辆 cara 中的第 k 个 DAG 的调度顺序

        # 开始调度task中的所有任务
        for i, ni in enumerate(task):
            # 得到任务ni前驱任务中最大的完成时间
            max_pred_FT = 0
            for pred, val in enumerate(DAG[:, ni]):
                if val > 0:
                    for ft in V[cara].dags[k].myal1[pred].ft:
                        if max_pred_FT < ft:
                            max_pred_FT = ft
            # 计算当前任务的可靠性要求
            laterR = 1  # 得到后面任务的总可靠性
            for nj in task[i + 1::]:
                laterR *= V[cara].dags[k].myal1[nj].newR
            ni_req = R_req / ( RG[cara][k] * laterR ) # 考虑要不要对laterR更新
            # print(V[cara].dags[k].heft[ni].R_lb_req - ni_req) # ni可靠性要求要低于MaxR 和 RR

            while V[cara].dags[k].myal1[ni].num:
                EST = [0 for _ in range(M)]
                EFT = [0 for _ in range(M)]
                O_t_, W_t_ = [], []
                I_R_, W_R_, O_R_ = [], [], []
                derta = [0 for _ in range(M)]
                for ma in range(M):
                    I_t, I_R = I_T_R(ma, cara, k, ni)
                    W_t, W_R = W_T_R(ma, cara, k, ni)
                    O_t, O_R = O_T_R(ma, cara, k, ni)
                    O_t_.append(O_t)
                    W_t_.append(W_t)
                    I_R_.append(I_R)
                    W_R_.append(W_R)
                    O_R_.append(O_R)
                    maxWaitPredReady = max_pred_FT + I_t
                    EST[ma] = max(Avail[ma], maxWaitPredReady, Arrival[cara] )
                    EFT[ma] = EST[ma] + W_t + O_t
                    derta[ma] = EST[ma] - Avail[ma]
                minEFT,minEFT2 = INF, INF
                for mb in range(M):
                    if EFT[mb] < minEFT and V[cara].dags[k].myal1[ni].runcpu[mb] == 0:
                        minEFT2 = minEFT
                        minEFT = EFT[mb]
                    elif EFT[mb] < minEFT2:
                        minEFT2 = EFT[mb]
                if minEFT == INF:
                    print("任务备份数量超过上限,超过部分已全部放在本地计算")
                    mincar = cara
                else:
                    mincar = EFT.index(minEFT)

                buffer = 0

                if buffer == 0:
                    V[cara].dags[k].myal1[ni].st.append(EST[mincar])
                    V[cara].dags[k].myal1[ni].ft.append(EFT[mincar])
                    sumET += EFT[mincar] - EST[mincar]
                    Avail[mincar] = EFT[mincar] - O_t_[mincar]
                else:
                    EST[mincar] = max(EST[mincar], Avail[mincar])  # ni开始时间有更新情况
                    EFT[mincar] = EST[mincar] + W_t_[mincar] + O_t_[mincar]
                    V[cara].dags[k].myal1[ni].st.append(EST[mincar])
                    V[cara].dags[k].myal1[ni].ft.append(EFT[mincar])
                    sumET += EFT[mincar] - EST[mincar]
                    Avail[mincar] = EFT[mincar] - O_t_[mincar]
                V[cara].dags[k].myal1[ni].runcpu[mincar] = 1
                V[cara].dags[k].myal1[ni].backup += 1
                V[cara].dags[k].myal1[ni].num -= 1
                if maxft < EFT[mincar]:  # 记录一个DAG的完成时间，下一个DAG在这个时间到达
                    maxft = EFT[mincar]
                I_R = I_R_[mincar]
                W_R = W_R_[mincar]
                O_R = O_R_[mincar]
                V[cara].dags[k].myal1[ni].ni_fail_R *= 1 - I_R * W_R * O_R

                if 1 - V[cara].dags[k].myal1[ni].ni_fail_R >= ni_req:
                    # print("备份数量得到第 2 次优化")
                    break

            # ni 已经调度完，更新可靠性
            V[cara].dags[k].myal1[ni].R *= 1 - V[cara].dags[k].myal1[ni].ni_fail_R
            RG[cara][k] *= V[cara].dags[k].myal1[ni].R
            # 通过计数的方法判断是否执行完
            V[cara].dags[k].n += 1
            carK[cara] += int( V[cara].dags[k].n / len(DAG) )

        if RG[cara][k] < R_req:
            print([cara,k], "可靠性不满足要求")
        Arrival[cara] = max(Arrival[cara], maxft)

        # print("=====================================")
        numofdags += 1
        if numofdags > NUM:
            break

    backups = 0
    AverRG = []
    for i in range(M):
        AverRG.append( sum(RG[i])/len(RG[i]) )
        for j in range(K):
            V[i].dags[j].n = 0
            DAG = np.array(V[i].dags[j].C)
            for ni in range(len(DAG)):

                backups += V[i].dags[j].myal1[ni].backup

    print("Myal-", sum(Avail) / M, backups - NUM, sum(AverRG)/len(AverRG), sumET/sum(Avail), numofdags )

def DSD1():
    T = 0   # 时间轴
    wait = [-1 for _ in range(M)]  # 存放等候任务
    Avail = [0.5494061416802742 for _ in range(M)]  # 处理器的空闲时间
    carK = [0 for _ in range(M)]   # 用来表示第几个DAG
    Arrival = [ arr[i]/100 for i in range(M) ]  # 记录DAG的到达时间，即前一个DAG的完成时间
    RG = [ [1 for _ in range(K)] for _ in range(M) ]  # 记录所有DAG的可靠性
    numofdags = 0  # 调度的DAG数量
    maxft = 0
    sumET = 0
    k = 0
    for cara in range(M):
        # if carK[cara] >= K:
        #     Arrival[cara] = INF
        # # print(Arrival)
        # cara = Arrival.index(min(Arrival))  # 得到要调度的车，和它的DAG
        # if Arrival[cara] >= INF:
        #     # print(numofdags)
        #     break
        # k = carK[cara]
        if cara == 5:
            DAG = np.array(V[cara].dags[k].C)  # 选定的当前要去调度的DAG，不能影响其正常调度
            task = V[cara].Task[k].copy()
            # task 保存车辆 cara 中的第 k 个 DAG 的调度顺序

            # 开始调度task中的所有任务
            for i, ni in enumerate(task):
                # 得到任务ni前驱任务中最大的完成时间
                max_pred_FT = 0
                for pred, val in enumerate(DAG[:, ni]):
                    if val > 0:
                        for ft in V[cara].dags[k].myal[pred].ft:
                            if max_pred_FT < ft:
                                max_pred_FT = ft
                # 计算当前任务的可靠性要求
                laterR = 1  # 得到后面任务的总可靠性
                for nj in task[i + 1::]:
                    laterR *= V[cara].dags[k].myal[nj].newR
                ni_req = R_req / ( RG[cara][k] * laterR ) # 考虑要不要对laterR更新
                # print(V[cara].dags[k].heft[ni].R_lb_req - ni_req) # ni可靠性要求要低于MaxR 和 RR
                eft = []
                while V[cara].dags[k].myal[ni].num:
                    EST = [0 for _ in range(M)]
                    EFT = [0 for _ in range(M)]
                    O_t_, W_t_ = [], []
                    I_R_, W_R_, O_R_ = [], [], []
                    derta = [0 for _ in range(M)]
                    for ma in range(M):
                        I_t, I_R = I_T_R(ma, cara, k, ni)
                        W_t, W_R = W_T_R(ma, cara, k, ni)
                        O_t, O_R = O_T_R(ma, cara, k, ni)
                        O_t_.append(O_t)
                        W_t_.append(W_t)
                        I_R_.append(I_R)
                        W_R_.append(W_R)
                        O_R_.append(O_R)
                        maxWaitPredReady = max_pred_FT + I_t
                        EST[ma] = max(Avail[ma], maxWaitPredReady, Arrival[cara] )

                        EFT[ma] = EST[ma] + W_t + O_t
                        derta[ma] = EST[ma] - Avail[ma]
                    minEFT,minEFT2 = INF, INF
                    for mb in range(M):
                        if EFT[mb] < minEFT and V[cara].dags[k].myal[ni].runcpu[mb] == 0:
                            minEFT2 = minEFT
                            minEFT = EFT[mb]
                        elif EFT[mb] < minEFT2:
                            minEFT2 = EFT[mb]
                    if minEFT == INF:
                        print("任务备份数量超过上限,超过部分已全部放在本地计算")
                        mincar = cara
                    else:
                        mincar = EFT.index(minEFT)
                    if minEFT2 >= INF or minEFT >= INF:
                        buffer = 0
                    else:
                        buffer =  abs(minEFT2 - minEFT)
                    # buffer = 0
                    # 记录时间轴，以便其他DAG任务加入等候集合中
                    if T < EST[mincar]:
                        T = EST[mincar]
                    for h in range(M):
                        if h != cara and carK[h] < K:
                            if T >= Arrival[h] and wait[h] < 0 and len(V[h].Task[carK[h]])>0:
                                wait[h] = V[h].Task[carK[h]][0]
                        else:
                            wait[h] = -1
                    # print(derta[mincar] + buffer)
                    wait[1] = wait[3] = -1
                    if derta[mincar] + buffer > 0 and sum(wait) > -M:
                        while True:
                            # 找出wait中最早完成的任务
                            waitnift = [INF for _ in range(M)]
                            waitnist = [0 for _ in range(M)]
                            waitnir = [1 for _ in range(M)]
                            for waitcar, waitni in enumerate(wait):
                                if waitni != -1:
                                    waitDAG = np.array(V[waitcar].dags[carK[waitcar]].C)
                                    max_pred_FT_a = 0
                                    for pred_a, val_a in enumerate(waitDAG[:, waitni]):
                                        if val_a > 0:
                                            for ft_a in V[waitcar].dags[carK[waitcar]].myal[pred_a].ft:
                                                if max_pred_FT_a < ft_a:
                                                    max_pred_FT_a = ft_a
                                    wait_I_t, wait_I_R = I_T_R(mincar, waitcar, carK[waitcar], waitni)
                                    wait_W_t, wait_W_R = W_T_R(mincar, waitcar, carK[waitcar], waitni)
                                    waitPredReady = max_pred_FT_a + wait_I_t
                                    waitnist[waitcar] = max(Avail[mincar], waitPredReady, Arrival[waitcar])
                                    waitnift[waitcar] = waitnist[waitcar] + wait_W_t
                                    waitnir[waitcar] *= wait_I_R * wait_W_R
                            minwaitft = min(waitnift)
                            wait_car = waitnift.index(minwaitft)
                            wait_ni = wait[wait_car]
                            wait_k = carK[wait_car]
                            if minwaitft >= INF or wait_k >= K:
                                break
                            # 判断是否可以填充
                            # 能的话，执行该任务，更新参数（wait、RG、任务的信息）
                            if EST[mincar] + buffer >= minwaitft:  # EST[mincar] + buffer 为定值
                                # print("===OK===", wait_car, wait_k, wait_ni)
                                wait_O_t, wait_O_R = O_T_R(mincar, wait_car, wait_k, wait_ni)
                                V[wait_car].dags[wait_k].myal[wait_ni].st.append(waitnist[wait_car])
                                V[wait_car].dags[wait_k].myal[wait_ni].ft.append(waitnift[wait_car] + wait_O_t)
                                sumET += waitnift[wait_car] - waitnist[wait_car]

                                Avail[mincar] = waitnift[wait_car]   # Avail[mincar]的值会不断增大

                                V[wait_car].dags[wait_k].myal[wait_ni].runcpu[mincar] = 1
                                V[wait_car].dags[wait_k].myal[wait_ni].backup += 1
                                V[wait_car].dags[wait_k].myal[wait_ni].num -= 1
                                waitnir[wait_car] *= wait_O_R
                                V[wait_car].dags[wait_k].myal[wait_ni].ni_fail_R *= 1 - waitnir[wait_car]

                                wait_num = V[wait_car].dags[wait_k].myal[wait_ni].num
                                wait_ni_fail_R = V[wait_car].dags[wait_k].myal[wait_ni].ni_fail_R

                                # 计算等候任务的可靠性要求
                                wait_other_R = 1  # 得到后面任务的总可靠性
                                for nj in V[wait_car].Task[wait_k][1::]:
                                    wait_other_R *= V[wait_car].dags[wait_k].myal[nj].newR  # 可以去更新newR
                                wait_ni_req = R_req / (RG[wait_car][wait_k] * wait_other_R)  # 当前任务的可靠性要求

                                print(wait_car, wait_ni, V[wait_car].dags[wait_k].myal[wait_ni].runcpu, V[wait_car].dags[wait_k].myal[wait_ni].st,waitnift[wait_car],
                                      V[wait_car].dags[wait_k].myal[wait_ni].ft)

                                # 当前任务已经调度完成
                                if wait_num == 0 or 1 - wait_ni_fail_R > wait_ni_req:
                                    V[wait_car].dags[wait_k].myal[wait_ni].num  = 0
                                    V[wait_car].dags[wait_k].myal[wait_ni].R = 1 - V[wait_car].dags[wait_k].myal[wait_ni].ni_fail_R
                                    RG[wait_car][wait_k] *= V[wait_car].dags[wait_k].myal[wait_ni].R
                                    V[wait_car].Task[wait_k].pop(0)
                                    if len(V[wait_car].Task[wait_k]) == 0:
                                        wait[wait_car] = -1
                                    else:
                                        wait[wait_car] = V[wait_car].Task[wait_k][0]

                                    V[wait_car].dags[wait_k].n += 1
                                    waitDAG = np.array(V[wait_car].dags[wait_k].C)
                                    if int( V[wait_car].dags[wait_k].n / len(waitDAG) ) == 1:
                                        carK[wait_car] += 1
                                        numofdags += 1

                            else: # 不能的话，则跳出while循环
                                break

                    if buffer == 0:
                        V[cara].dags[k].myal[ni].st.append(EST[mincar])

                        V[cara].dags[k].myal[ni].ft.append(EFT[mincar])
                        sumET += EFT[mincar] - EST[mincar]
                        Avail[mincar] = EFT[mincar] - O_t_[mincar]
                        eft.append(Avail[mincar] )
                    else:
                        EST[mincar] = max(EST[mincar], Avail[mincar])  # ni开始时间有更新情况
                        EFT[mincar] = EST[mincar] + W_t_[mincar] + O_t_[mincar]
                        V[cara].dags[k].myal[ni].st.append(EST[mincar])
                        V[cara].dags[k].myal[ni].ft.append(EFT[mincar])
                        sumET += EFT[mincar] - EST[mincar]
                        Avail[mincar] = EFT[mincar] - O_t_[mincar]
                        eft.append(Avail[mincar])
                    V[cara].dags[k].myal[ni].runcpu[mincar] = 1
                    V[cara].dags[k].myal[ni].backup += 1
                    V[cara].dags[k].myal[ni].num -= 1
                    if maxft < EFT[mincar]:  # 记录一个DAG的完成时间，下一个DAG在这个时间到达
                        maxft = EFT[mincar]
                    I_R = I_R_[mincar]
                    W_R = W_R_[mincar]
                    O_R = O_R_[mincar]
                    V[cara].dags[k].myal[ni].ni_fail_R *= 1 - I_R * W_R * O_R

                    if 1 - V[cara].dags[k].myal[ni].ni_fail_R >= ni_req:
                        # print("备份数量得到第 2 次优化")
                        break

                # ni 已经调度完，更新可靠性
                V[cara].dags[k].myal[ni].R *= 1 - V[cara].dags[k].myal[ni].ni_fail_R
                RG[cara][k] *= V[cara].dags[k].myal[ni].R
                # 通过计数的方法判断是否执行完
                V[cara].dags[k].n += 1
                carK[cara] += int( V[cara].dags[k].n / len(DAG) )
                print(cara, ni, V[cara].dags[k].myal[ni].runcpu, V[cara].dags[k].myal[ni].st, eft, V[cara].dags[k].myal[ni].ft)

            if RG[cara][k] < R_req:
                print([cara,k], "可靠性不满足要求")
            Arrival[cara] = max(Arrival[cara], maxft)

            # print("=====================================")
            numofdags += 1
            if numofdags > NUM:
                break

    backups = 0
    AverRG = []
    for i in range(M):
        AverRG.append( sum(RG[i])/len(RG[i]) )
        for j in range(K):
            DAG = np.array(V[i].dags[j].C)
            for ni in range(len(DAG)):
                backups += V[i].dags[j].myal[ni].backup

    print("Myal", sum(Avail) / M, backups, sum(AverRG)/len(AverRG), numofdags )


def DSD():
    aver_F = INF
    for car in range(M):
        aver_F += V[car].F
    aver_F /= M

    Avail = [0 for _ in range(M)]  # 处理器的空闲时间
    DAGs = [0 for _ in range(M)]   # 用来表示第几个DAG
    # Arival = [ arr[0][i]/10000 for i in range(M) ]  # 记录DAG的到达时间，即前一个DAG的完成时间
    RG = [ [1 for _ in range(K)] for _ in range(M) ]  # 记录所有DAG的可靠性
    localfree = [0 for _ in range(M)]
    sumET = 0

    readytasks = [[] for _ in range(M)]
    for car in range(M):
        dagk = DAGs[car]
        readytasks[car] = Ranku(car, dagk, np.array(V[car].dags[dagk].C)).copy()  # 得到调度顺序
        for ni in readytasks[car]:
            V[car].dags[dagk].node[ni].lft = localfree[car] + V[car].dags[dagk].node[ni].W * bit * cyc / V[car].F / cyc_s
            localfree[car] = V[car].dags[dagk].node[ni].lft

    Tab = [0 for _ in range(M)]
    while True:
        # 当前就绪 DAG
        proritys = []
        for car in range(M):
            # 从 M 个队首任务中选择出一个紧迫程度最大的任务进行调度
            if DAGs[car] >= K:
                proritys.append(INF)
                Tab[car] = 1
            else:
                DAG = np.array(V[car].dags[DAGs[car]].C)
                ni = readytasks[car][0]
                max_pred_FT = 0
                for pred, val in enumerate(DAG[:, ni]):  # 前驱
                    if val > 0:
                        for ft in V[car].dags[DAGs[car]].myal[pred].ft:  # 备份
                            if max_pred_FT < ft:
                                max_pred_FT = ft
                prority = 0
                for car1 in range(M):
                    I_t, _ = I_T_R(car1, car, DAGs[car], ni)
                    ex = V[car].dags[DAGs[car]].node[ni].W * bit * cyc / V[car1].F / cyc_s
                    mm = (V[car].dags[DAGs[car]].node[ni].lft - max(max_pred_FT + I_t, Avail[car1]))
                    if mm == 0:
                        prority += 0
                    else:
                        prority += ex / mm
                proritys.append( prority / M )

        if sum(Tab) == M:
            break

        cpu = proritys.index(min(proritys))
        task = readytasks[cpu][0]
        print(cpu,DAGs[cpu],task)





        DAG = np.array(V[cpu].dags[DAGs[cpu]].C)
        # 处理器分配
        # 得到任务ni前驱任务中最大的完成时间
        max_pred_FT = 0
        for pred, val in enumerate(DAG[:, task]):
            if val > 0:
                for ft in V[cpu].dags[DAGs[cpu]].myal[pred].ft:
                    if max_pred_FT < ft:
                        max_pred_FT = ft

        # 计算当前任务的可靠性要求
        laterR = 1  # 得到后面任务的总可靠性
        for nj in readytasks[cpu][1::]:
            # print(cpu,nj,"-----")
            laterR *= V[cpu].dags[ DAGs[cpu] ].myal[nj].newR
        ni_req = R_req / (RG[cpu][DAGs[cpu]] * laterR)  # 考虑要不要对laterR更新

        while V[cpu].dags[DAGs[cpu]].myal[task].num:
            EST = [0 for _ in range(M)]
            EFT = [0 for _ in range(M)]
            O_t_, W_t_ = [], []
            I_R_, W_R_, O_R_ = [], [], []

            for ma in range(M):
                I_t, I_R = I_T_R(ma, cpu, DAGs[cpu], task)
                W_t, W_R = W_T_R(ma, cpu, DAGs[cpu], task)
                O_t, O_R = O_T_R(ma, cpu, DAGs[cpu], task)
                O_t_.append(O_t)
                W_t_.append(W_t)
                I_R_.append(I_R)
                W_R_.append(W_R)
                O_R_.append(O_R)
                maxWaitPredReady = max_pred_FT + I_t
                EST[ma] = max(Avail[ma], maxWaitPredReady)
                EFT[ma] = EST[ma] + W_t + O_t

            minEFT = INF
            mincar = M
            for mb in range(M):
                if EFT[mb] < minEFT and V[cpu].dags[DAGs[cpu]].myal[task].runcpu[mb] == 0:
                    minEFT = EFT[mb]
                    mincar = mb

            EFT[mincar] = EST[mincar] + W_t_[mincar] + O_t_[mincar]
            V[cpu].dags[DAGs[cpu]].myal[task].st.append(EST[mincar])
            V[cpu].dags[DAGs[cpu]].myal[task].ft.append(EFT[mincar])
            Avail[mincar] = EFT[mincar] - O_t_[mincar]
            V[cpu].dags[DAGs[cpu]].myal[task].runcpu[mincar] = 1
            V[cpu].dags[DAGs[cpu]].myal[task].backup += 1
            V[cpu].dags[DAGs[cpu]].myal[task].num -= 1

            sumET += EFT[mincar] - EST[mincar]

            I_R = I_R_[mincar]
            W_R = W_R_[mincar]
            O_R = O_R_[mincar]
            V[cpu].dags[DAGs[cpu]].myal[task].ni_fail_R *= 1 - I_R * W_R * O_R
            if 1 - V[cpu].dags[DAGs[cpu]].myal[task].ni_fail_R >= ni_req:
                # print("备份数量得到第 2 次优化")
                break

            # ni 已经调度完，更新可靠性
        RG[cpu][DAGs[cpu]] *=  1 - V[cpu].dags[DAGs[cpu]].myal[task].ni_fail_R
        readytasks[cpu].remove(task)
        # 通过计数的方法判断是否执行完
        # V[cpu].dags[DAGs[cpu]].n += 1
        # DAGs[cpu] += int(V[cpu].dags[DAGs[cpu]].n / len(DAG))
        if len(readytasks[cpu]) == 0:
            DAGs[cpu] += 1
            dagk = DAGs[cpu]
            if DAGs[cpu] >= K:
                continue
            else:
                readytasks[cpu] = Ranku(cpu, dagk, np.array(V[cpu].dags[dagk].C)).copy()  # 得到调度顺序
                for ni in readytasks[cpu]:
                    V[cpu].dags[dagk].node[ni].lft = localfree[cpu] + V[cpu].dags[dagk].node[ni].W * bit * cyc /  V[cpu].F  / cyc_s
                    localfree[cpu] = V[cpu].dags[dagk].node[ni].lft

    backups = 0
    AverRG = []
    for i in range(M):
        AverRG.append(sum(RG[i]) / len(RG[i]))
        for j in range(K):
            DAG = np.array(V[i].dags[j].C)
            for ni in range(len(DAG)):
                backups += V[i].dags[j].myal[ni].backup

    print("Myal", sum(Avail) / M, backups - NUM, sum(AverRG) / len(AverRG), sumET / sum(Avail))

if __name__ == '__main__':
    # 各种参数，有待优化
    bandwidth = 10
    P = 0.1
    noise = pow(10, -13)
    A = 17.8
    bit = 1000000
    cyc = 1000
    cyc_s = 1000000000

    R_req = 0.998

    M = 6
    K = 1
    max_N = 110

    path = "D:/MyPapers/[4]Paper/Example/"

    # path = "D:/MyPapers/[4]Paper/CAR" + str(M) + "/"
    # path = "D:/MyPapers/[4]Paper/N/" + str(M) + str(max_N) + "/"
    # path = "D:/MyPapers/[4]Paper/EG" + str(M) + "/"
    # path = "D:/MyPapers/[4]Paper/FFT" + str(M) + "/"
    CAR = path + "car.txt"
    cardata = txt_to_matrix(CAR)
    # cardata 数组：是 .txt 中一行一行的数组
    # 实例化车辆信息
    V = [Car() for _ in range(M)]
    Netnomoda = []
    arr = []
    net = [[0 for _ in range(M)] for _ in range(M)]
    for i in range(len(cardata)):
        if i < M:
            V[i].resetCar(cardata[i])
        elif i < 2*M:
            arr.append(cardata[i][0])
            # arr.append(0)
        else:
            Netnomoda.append(cardata[i])

    for i in range(M):
        for j in range(M):
            if Netnomoda[i][j] > 0:
                Netnomoda[i][j] = Netnomoda[i][j]/100000
                net[i][j] = net[j][i] = 1
    print(np.array(net))

    pathnum = [[0 for _ in range(M)] for _ in range(M)]  # 得到任意车之间的最短路径
    for cara in range(M):
        for carb in range(cara+1,M):
            pathcount = 0
            for i in range(1,INF):
                pathcount, _ = find_paths(net, cara, carb, i)
                if pathcount > 0:
                    pathnum[cara][carb] = pathnum[carb][cara] = i
                    break
    # ============================至此，我们得到车和网络信息=====================================

    for m, v in enumerate(V):  # 对于双层列表中的数据
        CARDAGSDATAS = []
        for k in range(K):
            CARDAGS = path + "car" + str(m) + "/" + "DAG" + str(k) + ".txt"
            CARDAGSDATA = txt_to_matrix(CARDAGS)
            CARDAGSDATAS.append(CARDAGSDATA)
        v.resetDAGS(CARDAGSDATAS)

    NUM = 0
    kkkkkkk = 0
    for i in range(M):
        for j in range(K):
            DAG = np.array(V[i].dags[j].C)
            NUM += len(DAG)
            V[i].Task.append(Ranku(i, j, DAG))
            Minbackups(i, j, DAG)
            kkkkkkk += 1
            print(kkkkkkk,"车"+str(i)+"的第"+str(j)+"个DAG已被初始化")

    print("所有 DAGs 获取完毕")
    DSD1()
    DSD()
    LOCALMaxR()
    LOCALAL()
    HEFTMaxR()
    HEFTAl()



