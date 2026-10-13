import math
import os
import numpy as np
from math import *
import heapq
import matplotlib.pyplot as plt
from scipy.integrate import quad
import sympy as sp
import random

INF = 999999
D = 300
# 车辆和任务初始化
class Node():
    def __init__(self):
        self.st = [ 0 for _ in range(M)]
        self.ft = [ 0 for _ in range(M)]
        self.W = 0
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
        self.R_lb_req = 0
        self.num = 0
        self.R = 0
        self.R_cpu = [0 for _ in range(M)]
        self.max_runcpu = 0
        self.R_new = 0
        self.R_i_G = 0
        self.runcpu = [0 for _ in range(M)]

class ERRMnode:
    def __init__(self):
        self.ST = []
        self.FT = []
        self.W = 0
        self.I = 0
        self.O = 0
        self.runcpu = [0 for _ in range(M)]
        self.AverW = 0
        self.preds = []
        self.succs = []


        self.R_lb_req = 0
        self.num = 0
        self.R = 0
        self.R_cpu = [0 for _ in range(M)]
        self.max_runcpu = 0
        self.R_new = 0
        self.R_i_G = 0
        self.opti_strategy = []

class Localnode:
    def __init__(self):
        self.W = 0
        self.I = 0
        self.O = 0
        self.R_lb_req = 0
        self.num = 0
        self.R = 0
        self.R_cpu = [0 for _ in range(M)]
        self.max_runcpu = 0
        self.R_new = 0
        self.R_i_G = 0

class Myalnode:
    def __init__(self):
        self.ind = 0
        self.outd = 0
        self.preds = []
        self.succs = []
        self.visit = 0
        self.W = 0.0
        self.I = 0.0
        self.O = 0.0
        self.ind_ = 0
        self.visit_ = 0
        self.backups_cpu = []
        self.FT = []
        self.ST = []
        self.backups_num = 0
        self.num = 0
        self.NUM = 0
        self.R = 0.0
        self.min_R = 1
        self.R_new = 0
        self.rankd_R = 1
        self.ni_fail_R = 1
        self.R_cpu = [1 for _ in range(M)]
        self.mincpu = [0 for _ in range(M)]
        self.runcpu = [0 for _ in range(M)]

        self.OCT = [0 for _ in range(M)]
        self.ETR = [0 for _ in range(M)]



class MyalNODE:
    def __init__(self):
        self.ind = 0
        self.outd = 0
        self.preds = []
        self.succs = []
        self.visit = 0
        self.W = 0.0
        self.I = 0.0
        self.O = 0.0
        self.ind_ = 0
        self.visit_ = 0
        self.backups_cpu = []
        self.FT = []
        self.ST = []
        self.backups_num = 0
        self.num = 0
        self.NUM = 0
        self.R = 0.0
        self.min_R = 1
        self.R_new = 0
        self.rankd_R = 1
        self.ni_fail_R = 1
        self.R_cpu = [1 for _ in range(M)]
        self.mincpu = [0 for _ in range(M)]
        self.runcpu = [0 for _ in range(M)]

        self.OCT = [0 for _ in range(M)]
        self.ETR = [0 for _ in range(M)]



class Random:
    def __init__(self):
        self.R_lb_req = 0
        self.ind = 0
        self.outd = 0
        self.preds = []
        self.succs = []
        self.visit = 0
        self.W = 0
        self.I = 0
        self.O = 0
        self.ind_ = 0
        self.visit_ = 0
        self.backups_cpu = []
        self.FT = []
        self.ST = []
        self.num = 0
        self.R = 0


class Car():
    def __init__(self):
        self.tab = 0
        self.id = 0
        self.N = 0
        self.pos = 0
        self.vel = 0
        self.F = 0
        self.cyc = 0
        # 算法用到的变量
        self.C = None
        self.r = []
        self.R_exe = 0.0
        self.arr_t = 0.0

        self.myal_num = 0

        self.j = 0

        self.Task = []
        self.Tasks = []
        self.HEFTTasks = []

        # HEFT
        self.node = []  # 执行数据大小和输出率

        # ERRM
        self.errm = []

        # Loca
        self.loca = []
        # Myal
        self.myal = []

        # Myal
        self.MYAL = []

        # Rand
        self.rand = []



    def resetCar(self, Car_data):
        # [v.id, v.pos, v.F, v.cyc, v.N, v.R_exe]
        self.id = Car_data[0]
        self.pos = Car_data[1]
        self.F = Car_data[2] / 10   # GHz
        self.cyc = Car_data[3] # 1000 Cycles/bit
        self.tab = Car_data[4]
        self.N = Car_data[5]
        self.R_exe = Car_data[6] / 100000
        self.diaodu = [0 for _ in  range(self.N) ]



    def resetHEFT(self, arr_t, I, W, O, C):
        self.node = [Node() for _ in range(self.N)]  # 执行数据大小和输出率
        self.carcpu = [M for _ in range(self.N)]
        self.r = [M for _ in range(self.N)]
        self.arr_t = arr_t
        self.C = C
        for i in range(self.N):
            self.node[i].I = I[i]
            self.node[i].W = W[i]
            self.node[i].O = O[i]

        for i in range(self.N):
            for j in range(self.N):
                if C[i][j] > 0:
                    self.node[j].preds.append(i)
                    self.node[j].ind += 1
                if C[i][j] > 0:
                    self.node[i].succs.append(j)
                    self.node[i].outd += 1

    def resetERRM(self,arr_t, I, W, O, C):
        self.errm = [ERRMnode() for _ in range(self.N)]
        for i in range(self.N):
            self.errm[i].I = I[i]
            self.errm[i].W = W[i]
            self.errm[i].O = O[i]
        for i in range(self.N):
            for j in range(self.N):
                if self.C[i][j] > 0:
                    self.errm[j].preds.append(i)
                if self.C[i][j] > 0:
                    self.errm[i].succs.append(j)

    def resetLocal(self,arr_t,I, W, O, C):
        self.loca = [Localnode() for _ in range(self.N)]
        for i in range(self.N):
            self.loca[i].I = I[i]
            self.loca[i].W = W[i]
            self.loca[i].O = O[i]

    def resetRand(self,arr_t,I, W, O, C):
        self.rand = [ Random() for _ in range(self.N)]
        for i in range(self.N):
            self.rand[i].I = I[i]
            self.rand[i].W = W[i]
            self.rand[i].O = O[i]
        for i in range(self.N):
            for j in range(self.N):
                if C[i][j] > 0:
                    self.rand[j].preds.append(i)
                    self.rand[j].ind += 1
                if C[i][j] > 0:
                    self.rand[i].succs.append(j)
                    self.rand[i].outd += 1


    def resetMyal(self,arr_t,I, W, O, C):
        self.myal = [Myalnode() for _ in range(self.N)]
        for i in range(self.N):
            self.myal[i].I = I[i]
            self.myal[i].W = W[i]
            self.myal[i].O = O[i]
        for i in range(self.N):
            for j in range(self.N):
                if C[i][j] > 0:
                    self.myal[j].preds.append(i)
                    self.myal[j].ind += 1
                if C[i][j] > 0:
                    self.myal[i].succs.append(j)
                    self.myal[i].outd += 1

    def resetMYAL(self,arr_t,I, W, O, C):
        self.MYAL = [MyalNODE() for _ in range(self.N)]
        for i in range(self.N):
            self.MYAL[i].I = I[i]
            self.MYAL[i].W = W[i]
            self.MYAL[i].O = O[i]
        for i in range(self.N):
            for j in range(self.N):
                if C[i][j] > 0:
                    self.MYAL[j].preds.append(i)
                    self.MYAL[j].ind += 1
                if C[i][j] > 0:
                    self.MYAL[i].succs.append(j)
                    self.MYAL[i].outd += 1

def floyd(graph):
    n = len(graph)
    dist = [[graph[i][j] for j in range(n)] for i in range(n)]
    path = [[j if graph[i][j] is not None else None for j in range(n)] for i in range(n)]
    for k in range(n):
        for i in range(n):
            for j in range(n):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    path[i][j] = path[i][k]
    return path,dist


def Path(start, end):
    com_paths = []
    if paths[start][end] is None:
        print(f"No path from node {start} to {end}")
    else:
        k = paths[start][end]
        path = [start]
        while k != end:
            path.append(k)
            k = paths[k][end]
        path.append(end)
        # print(path)
        for i in range(len(path) - 1):
            com_paths.append([path[i], path[i + 1]])
    # print(com_paths)
    return com_paths



def Distance(m1, m2):
    return abs(V[m1].pos - V[m2].pos)


def low_R_path(data):
    all_R_path = []
    for car in range(M):
        for car1 in range(car+1,M):
            com_paths = Path(car, car1)
            sum_time = 1
            for path in com_paths:
                distance = Distance(path[0], path[1])
                h = A * pow(1 / distance, 2)
                rate = bandwidth * log((1 + P * h / noise), 2)
                sum_time *= math.exp( -r[path[0]][path[1]] * data / rate)
            all_R_path.append(sum_time)
    return min(all_R_path)

def averrate():
    all_r_path = []
    for car in range(M):
        for car_a in range(car + 1, M):
            com_paths = Path(car, car_a)
            rate = 0
            for path in com_paths:
                distance = Distance(path[0], path[1])
                h = A * pow(1 / distance, 2)
                rate += bandwidth * log((1 + P * h / noise), 2)
            all_r_path.append( rate / len(com_paths) )
    # print(sum(all_r_path) / len(all_r_path))
    aver_rate = sum(all_r_path) / len(all_r_path)
    return aver_rate


def I_T_R(m, car, ni):
    if m == car:
        T, R = 0, 1
    else:
        I_paths = Path(m, car)
        I_t = 0
        I_r = 1
        for path in I_paths:
            distance = Distance(path[0], path[1])
            h = A * pow(1 / distance, 2)
            rate = bandwidth * log((1 + P * h / noise), 2)
            I_t += V[car].myal[ni].I / rate
            I_r *= math.exp(-r[path[0]][path[1]] * V[car].node[ni].I / rate)
        T, R = I_t, I_r
    return T, R

def W_T_R(m, car, ni):
    l_t = V[car].myal[ni].W * bit * V[m].cyc / V[m].F / cyc_s
    l_r = math.exp(-V[m].R_exe * l_t)
    T, R = l_t,l_r
    return T, R


def O_T_R(m, car, ni):
    if m == car:
        T, R = 0, 1
    else:
        O_paths = Path(m, car)
        O_t = 0
        O_r = 1
        for path in O_paths:
            distance = Distance(path[0], path[1])
            h = A * pow(1 / distance, 2)
            rate = bandwidth * log((1 + P * h / noise), 2)
            O_t += V[car].myal[ni].O / rate
            O_r *= math.exp(-r[path[0]][path[1]] * V[car].node[ni].O / rate)
        T, R = O_t, O_r
    return T, R

def Local():
    all_local_delay = []
    R_G = []
    nums = 0
    CAR = [i for i in range(M) if V[i].tab == 1]

    for car in CAR:
        sum_w = 0
        each_R = 1
        each = 0
        R_G_i = 1
        for i in range(V[car].N):
            V[car].loca[i].R = math.exp(-V[car].R_exe * V[car].node[i].W * bit * V[car].cyc / V[car].F / cyc_s)
            V[car].loca[i].R_lb_req = pow( R_req / each_R, 1 / (V[car].N - each))
            # V[car].loca[i].num += 1
            while V[car].loca[i].R < V[car].loca[i].R_lb_req:
                V[car].loca[i].R = 1 - ((1 - V[car].loca[i].R) * (1 - V[car].loca[i].R))
                V[car].loca[i].num += 1
            each_R *= V[car].loca[i].R
            each += 1
            R_G_i *= V[car].loca[i].R
            sum_w += V[car].loca[i].W * (V[car].loca[i].num+1)

        local_delay = sum_w * bit * V[car].cyc / V[car].F / cyc_s
        all_local_delay.append( local_delay )

        # 本地执行可靠性计算
        R_G.append(R_G_i)

        for i in range(V[car].N):
            nums += V[car].loca[i].num

    print("local平均延迟:", round(  sum(all_local_delay) / len(CAR), 2), nums, R_G, all_local_delay, len(CAR) / M )

    return sum(all_local_delay)/len(CAR), nums, sum(R_G)/len(CAR), len(CAR) / M

def Rand():
    Task = []
    Indegree = []
    CAR = [i for i in range(M) if V[i].tab == 1]
    for v in CAR:
        for ni in range(V[v].N):
            if V[v].rand[ni].ind == 0:
                Indegree.append(str(v) + "+" + str(ni))
    Q = Indegree.copy()

    each_R = [1 for _ in range(M)]
    each = [0 for _ in range(M)]
    Avail = [0 for _ in range(M)]

    Use_car = [0 for _ in range(M)]

    while Q:
        # 得出此时Q中最大可靠性的任务--------------------------------
        B = []
        for car_ni in Q:
            car, ni = car_ni.split("+")
            B.append([int(car), int(ni)])
        A = [b for b in B if V[b[0]].rand[b[1]].ind == 0]
        A_choice = random.choice(A)  # 随机选择一个候选任务调度
        Task.append(A_choice)  # 保存任务调度集合
        car, ni = A_choice[0], A_choice[1]
        Tran_C = np.array(V[car].C)

        # RR 算法结合
        V[car].rand[ni].R_lb_req = pow(R_req / each_R[car], 1 / (V[car].N - each[car]))
        ni_R = 0
        while ni_R < V[car].rand[ni].R_lb_req:

            cpu = random.choice( [i for i in range(M)] )

            maxWaitPredReady = 0
            if V[car].rand[ni].ind == 0:
                maxWaitPredReady = V[car].arr_t

            # 前驱任务最早完成时间
            max_pred_FT = 0
            for pred, val in enumerate(Tran_C[:, ni]):
                if val > 0:
                    for ft in V[car].rand[pred].FT:
                        if max_pred_FT < ft:
                            max_pred_FT = ft

            I_t, I_r = I_T_R(cpu, car, ni)
            W_t, W_r = W_T_R(cpu, car, ni)
            O_t, O_r = O_T_R(cpu, car, ni)
            waitPredReady = max_pred_FT + I_t
            if waitPredReady > maxWaitPredReady:
                maxWaitPredReady = waitPredReady

            V[car].rand[ni].ST.append(max(Avail[cpu], maxWaitPredReady))
            Avail[cpu] = max(Avail[cpu], maxWaitPredReady)  + W_t
            Use_car[cpu]  += W_t
            V[car].rand[ni].FT.append( Avail[cpu] + O_t)
            V[car].rand[ni].backups_cpu.append(cpu)
            V[car].rand[ni].num += 1

            ni_R = 1 - ((1 - ni_R) * (1 - I_r * W_r * O_r))

        V[car].rand[ni].R = ni_R
        each_R[car] *= ni_R
        each[car] += 1

        Q.remove(str(car) + "+" + str(ni))

        # 就绪任务正式加入就绪集合--------------------
        if V[car].rand[ni].outd > 0:
            # 将车car中任务ni的后继加入Q中，同时让其后继任务的入度减一
            for succ in V[car].rand[ni].succs:
                V[car].rand[succ].ind -= 1
                if V[car].rand[succ].visit == 0:  # 找没有被访问过的后继
                    V[car].rand[succ].visit = 1
                    Q.append(str(car) + "+" + str(succ))
    # ====================总体调度情况输出===============================
    car_ft = []
    nums = 0
    NUM = 0
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    USE = [0 for _ in range(M)]
    for car in CAR:
        max_ft = 0.0
        if (Avail[car] - min(ARR)) > 0:
            USE[car] = Use_car[car] / (Avail[car] - min(ARR))
        else:
            USE[car] = 0
        for ni in range(V[car].N):
            nums += V[car].rand[ni].num
            NUM += 1
            for ft in V[car].rand[ni].FT:
                if max_ft < ft:
                    max_ft = ft
        car_ft.append(max_ft - V[car].arr_t)

    for i in range(M):
        if each_R[i] == 1:
            each_R[i] = 0

    print("Rand平均延迟:", round(sum(car_ft) / len(CAR), 2), nums - NUM, each_R, car_ft, sum(USE) / M)
    return sum(car_ft) / len(CAR), nums - NUM, sum(each_R)/len(CAR), sum(USE) / M

def LBR():
    NR_G = [0 for _ in range(M)]
    R_G = [ 1 for _ in range(M)]
    CAR = [i for i in range(M) if V[i].tab == 1]
    for car in CAR:
        for ni in range(V[car].N):
            V[car].errm[ni].R_lb_req = R_req
            while V[car].errm[ni].R < V[car].errm[ni].R_lb_req:
                for s in range(M):
                    I_t, I_r = I_T_R(s, car, ni)
                    W_t, W_r = W_T_R(s, car, ni)
                    O_t, O_r = O_T_R(s, car, ni)
                    V[car].errm[ni].R_cpu[s] = I_r * W_r * O_r

                max_R_cpu = max(V[car].errm[ni].R_cpu)
                V[car].errm[ni].runcpu[ V[car].errm[ni].R_cpu.index(max_R_cpu) ] = 1
                V[car].errm[ni].runcpu_ = V[car].errm[ni].R_cpu.index(max_R_cpu)
                V[car].errm[ni].num += 1
                V[car].errm[ni].R = 1 - ((1 - V[car].errm[ni].R) * (1 - max_R_cpu))
            NR_G[car] += V[car].errm[ni].num
            R_G[car] *= V[car].errm[ni].R
    return R_G

def ERRM():
    R_G = LBR()
    NR_G_M = []
    R_G_M = []
    CAR = [i for i in range(M) if V[i].tab == 1]
    for car in CAR:
        while R_G[car] < R_req:
            for ni in range(V[car].N):
                R_cpu = V[car].errm[ni].R_cpu.copy()
                for car_ in range(M):
                    if V[car].errm[ni].runcpu[car_] != 0:
                        R_cpu[car_] = 0
                max_R_cpu = max(R_cpu)
                V[car].errm[ni].max_runcpu = R_cpu.index(max_R_cpu)
                V[car].errm[ni].R_new = 1 - ((1 - V[car].errm[ni].R) * (1 - max_R_cpu))
                R_task_G = 1
                for i in range(V[car].N):
                    if i != ni:
                        R_task_G *= V[car].errm[ni].R
                V[car].errm[ni].R_i_G = R_task_G * V[car].errm[ni].R_new
            select_task = V[car].N
            max_R_i_G = 0
            for ni in range(V[car].N):
                if max_R_i_G < V[car].errm[ni].R_i_G:
                    max_R_i_G = V[car].errm[ni].R_i_G
                    select_task = ni
            V[car].errm[select_task].runcpu[ V[car].errm[select_task].max_runcpu ] += 1
            V[car].errm[select_task].num += 1
            V[car].errm[select_task].R = V[car].errm[select_task].R_new
            R_G[car] = V[car].errm[select_task].R_i_G
        # 记录总数
        NR_G = 0
        for ni in range(V[car].N):
            NR_G += V[car].errm[ni].num
            # print("===>ERRM", car, ni, V[car].errm[ni].R_cpu, V[car].errm[ni].runcpu, V[car].errm[ni].num, V[car].errm[ni].R)
        NR_G_M.append( NR_G - V[car].N )
        R_G_M.append( R_G[car] )



    # =================== ERRM 调度分配===================
    Avail = [0 for _ in range(M)]  # 记录空闲时间
    CAR = [i for i in range(M) if V[i].tab == 1]

    Use_car = [0 for _ in range(M)]

    for v in CAR:
        for ni in range(V[v].N):

            Tran_C = np.array(V[v].C)
            for i in range(M):
                if V[v].errm[ni].runcpu[i] > 0:
                    V[v].errm[ni].opti_strategy.append(i)

            max_pred_FT = 0
            for pred, val in enumerate(Tran_C[:, ni]):
                if val > 0:
                    for ft in V[v].errm[pred].FT:
                        if max_pred_FT < ft:
                            max_pred_FT = ft

            fail_R = 1
            for s in V[v].errm[ni].opti_strategy:
                maxWaitPredReady = 0
                if len(V[v].errm[ni].preds) == 0:
                    maxWaitPredReady = V[v].arr_t
                I_t, I_R = I_T_R(s, v, ni)
                W_t, W_R = W_T_R(s, v, ni)
                O_t, O_R = O_T_R(s, v, ni)

                waitPredReady = max_pred_FT + I_t
                maxWaitPredReady = max(waitPredReady, maxWaitPredReady)
                V[v].errm[ni].ST.append(max(Avail[s], maxWaitPredReady))
                Avail[s] = max(Avail[s], maxWaitPredReady) + W_t

                Use_car[s] += W_t

                V[v].errm[ni].FT.append(Avail[s] + O_t)
                fail_R *= (1 - I_R * W_R * O_R)

            V[v].errm[ni].R = 1 - fail_R
            R_G[v] *= V[v].errm[ni].R

    car_ft = []
    CAR = [i for i in range(M) if V[i].tab == 1]
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    USE = [0 for _ in range(M)]
    for car in CAR:
        max_ft = 0.0
        if Avail[car] != 0:
            USE[car] = Use_car[car] / Avail[car]

        for ni in range(V[car].N):
            for ft in V[car].errm[ni].FT:
                if max_ft < ft:
                    max_ft = ft
        car_ft.append(max_ft - V[car].arr_t)

    print(Use_car,USE)

    print("ERRM平均延迟:", round(sum(car_ft) /len(CAR), 2), sum(NR_G_M), R_G_M, car_ft, sum(USE) / M)
    return sum(car_ft) /len(CAR), sum(NR_G_M), sum(R_G_M)/len(CAR), sum(USE) / M


# 得到DAG的调度顺序=====================================================================
def Ranku():
    # 车之间的平均通信速率
    aver_rate = averrate()
    # 计算每个节点的优先级
    CAR = [i for i in range(M) if V[i].tab == 1]
    for car in CAR:
        for i in range(V[car].N):
            sumw = 0
            for car1 in range(M):
                sumw += V[car].node[i].W * bit * V[car1].cyc / V[car1].F / cyc_s
            V[car].node[i].AverW = sumw / M
    order = []
    ranku = []
    CAR = [i for i in range(M) if V[i].tab == 1]
    for car in CAR:
        order_ = []
        ranku_ = []
        for i in range(V[car].N-1,-1,-1):
            if len(V[car].node[i].succs) > 0:
                max_ = 0
                for succ in V[car].node[i].succs:
                    temp =  V[car].node[succ].ranku   # 平均传输速率？？？？？？？？？
                    if max_ < temp:
                        max_ = temp
                V[car].node[i].ranku = V[car].node[i].I / aver_rate + V[car].node[i].AverW + V[car].node[i].O / aver_rate  + max_
            else:
                V[car].node[i].ranku = V[car].node[i].I / aver_rate + V[car].node[i].AverW + V[car].node[i].O / aver_rate
            order.append( str(car) + "+" + str(i) )
            ranku.append( V[car].node[i].ranku )
            order_.append(str(car) + "+" + str(i))
            ranku_.append(V[car].node[i].ranku)

        order_ranku = dict(zip(order_, ranku_))
        res = sorted(order_ranku.items(), key=lambda d: d[1], reverse=True)  # 按开始时间降序排列
        order_ranku_queue = []
        for i in range(len(order_ranku)):
            k, l = res[i][0].split("+")
            order_ranku_queue.append([int(k), int(l)])
        V[car].Task = order_ranku_queue.copy()
        V[car].Tasks = V[car].Task.copy()

    order_ranku = dict(zip(order, ranku))
    res = sorted(order_ranku.items(), key=lambda d: d[1], reverse=True)  # 按开始时间降序排列
    order_ranku_queue = []
    for i in range(len(order_ranku)):
        k, l = res[i][0].split("+")
        order_ranku_queue.append([int(k), int(l)])

    return order_ranku_queue

# DAG调度
def HEFT():
    order_ranku_queue = Ranku()
    each_R = [1 for _ in range(M)]
    each = [0 for _ in range(M)]
    Avail = [0 for _ in range(M)]

    Use_car = [0 for _ in range(M)]

    for v_ni in order_ranku_queue:
        v, ni = v_ni[0], v_ni[1]
        Tran_C = np.array(V[v].C)

        ni_R = 0
        V[v].node[ni].R_lb_req = pow( R_req / each_R[v], 1 / (V[v].N - each[v]))

        while ni_R < V[v].node[ni].R_lb_req:
            EST = [0 for _ in range(M)]
            EFT = [0 for _ in range(M)]
            O_t_ = [0 for _ in range(M)]
            for s in range(M):

                maxWaitPredReady = 0
                if V[v].node[ni].ind == 0:
                    maxWaitPredReady = V[v].arr_t

                max_pred_FT = 0
                for pred, val in enumerate(Tran_C[:, ni]):
                    if val > 0:
                        for ft in V[v].node[pred].ft:
                            if max_pred_FT < ft:
                                max_pred_FT = ft
                # print(max_pred_FT)

                I_t, _ = I_T_R(s, v, ni)
                W_t, _ = W_T_R(s, v, ni)
                O_t, _ = O_T_R(s, v, ni)

                waitPredReady = max_pred_FT + I_t
                if waitPredReady > maxWaitPredReady:
                    maxWaitPredReady = waitPredReady

                EST[s] = max(Avail[s], maxWaitPredReady)
                EFT[s] = max(Avail[s], maxWaitPredReady) + W_t + O_t
                O_t_[s] = O_t

            minEFT = INF
            mincar = M
            for car in range(M):
                if EFT[car] < minEFT and V[v].node[ni].runcpu[car] == 0:
                    minEFT = EFT[car]
                    mincar = car
            Avail[mincar] = minEFT - O_t_[mincar]
            Use_car[mincar] += Avail[mincar] - EST[mincar]
            V[v].node[ni].st[mincar] = EST[mincar]
            V[v].node[ni].ft[mincar] = minEFT
            V[v].node[ni].runcpu[mincar] = 1
            V[v].node[ni].num += 1

            _, I_R = I_T_R(mincar, v, ni)
            _, W_R = W_T_R(mincar, v, ni)
            _, O_R = O_T_R(mincar, v, ni)

            ni_R = 1 - ( (1 - ni_R) * (1 - I_R * W_R * O_R) )

        V[v].node[ni].R = ni_R
        each_R[v] *= ni_R
        each[v] += 1
        # print(v, ni, V[v].node[ni].st, V[v].node[ni].ft, V[v].node[ni].num, V[v].node[ni].runcpu, V[v].node[ni].R)


    # ====================总体调度情况输出===============================
    car_exe = []
    nums = 0
    CAR = [i for i in range(M) if V[i].tab == 1]
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    USE = [ 0 for _ in range(M) ]
    for car in CAR:
        max_ft = 0.0
        if (Avail[car]-min(ARR)) > 0:
            USE[car] = Use_car[car] / ( Avail[car] - min(ARR) )
        else:
            USE[car] = 0
        for ni in range(V[car].N):
            nums += V[car].node[ni].num
            for i in range(M):
                if max_ft < V[car].node[ni].ft[i]:
                    max_ft = V[car].node[ni].ft[i]
        car_exe.append(max_ft - V[car].arr_t)

    for i in range(M):
        if each_R[i] == 1:
            each_R[i] = 0
    print("HEFT平均延迟:", round(sum(car_exe)/len(CAR),2), nums-len(order_ranku_queue), each_R,  sum(USE) / M)
    return sum(car_exe) / len(CAR), nums-len(order_ranku_queue),sum(each_R)/len(CAR),  sum(USE) / M



# 计算每个节点最小可靠性。同时，得到最差情况下的【初始备份数】====================================OK
def Get_num_up():
    R_G = [1 for _ in range(M)]
    CAR = [i for i in range(M) if V[i].tab == 1]
    for v in CAR:
        R = []
        for ni in range(V[v].N):
            # 可靠性最差路径 和 最低可靠性CPU 为下限得到每个节点的最低可靠性
            for car in range(M):
                _, R_zhixing = W_T_R(car, v, ni)
                V[v].myal[ni].R_cpu[car] = low_R_path(V[v].myal[ni].I) * R_zhixing *  low_R_path(V[v].myal[ni].O)
            V[v].myal[ni].min_R = V[v].myal[ni].R_new = min( V[v].myal[ni].R_cpu)
            V[v].myal[ni].num += 1
            R.append(V[v].myal[ni].R_new)
            R_G[v] *= V[v].myal[ni].R_new

        while R_G[v] < R_req:
            ni = R.index(min(R))
            V[v].myal[ni].R_new = 1 - ((1 - V[v].myal[ni].R_new) * (1 - V[v].myal[ni].min_R))
            V[v].myal[ni].num += 1  # 该节点确定备份，但处理器选择未确定，处理器都可以选择
            R[ni] = V[v].myal[ni].R_new
            R_task_G = 1
            for i in range(V[v].N):
                R_task_G *= V[v].myal[i].R_new
            R_G[v] = R_task_G

    for v in CAR:
        for ni in range(V[v].N):
            V[v].myal[ni].NUM = V[v].myal[ni].num
    return 0

# 得到DAG的调度顺序
def Get_Task( ):
    # 计算每个节点最小可靠性。同时，得到最差情况下的【初始备份数】=========================================================
    NUM = Get_num_up()
    # 入口任务加入就绪任务集合中
    CAR = [i for i in range(M) if V[i].tab == 1]
    ARR = [ INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    CPU = []
    for _ in CAR:
        v = ARR.index(min(ARR))
        CPU.append(v)
        ARR[v] = INF
        Indegree = []
        for ni in range(V[v].N):
            if V[v].myal[ni].ind == 0:
                Indegree.append(str(v)+"+"+str(ni))
        Q = Indegree.copy()
        while Q:
            # 得出此时Q中最大可靠性的任务--------------------------------
            B = []
            for car_ni in Q:
                car, ni = car_ni.split("+")
                B.append([int(car),int(ni)])
            A = [b for b in B if V[b[0]].myal[b[1]].ind == 0]

            A_min_R = []
            for a in A:
                # A_min_R.append( pow(V[a[0]].myal[a[1]].min_R, (V[a[0]].myal[a[1]].num + 1)) )
                A_min_R.append( V[a[0]].myal[a[1]].min_R )

            max_A_min_R = 0
            ni_id = len(A_min_R)
            for i, a_min_R in enumerate(A_min_R):
                if max_A_min_R < a_min_R:
                    max_A_min_R = a_min_R
                    ni_id = i
            car, ni = A[ni_id][0],A[ni_id][1]

            V[v].Task.append(A[ni_id])  # 保存任务调度集合

            Q.remove( str(car) + "+" + str(ni) )

            # 就绪任务正式加入就绪集合--------------------
            if V[car].myal[ni].outd > 0:
                # 将车car中任务ni的后继加入Q中，同时让其后继任务的入度减一
                for succ in V[car].myal[ni].succs:
                    V[car].myal[succ].ind -= 1
                    if V[car].myal[succ].visit == 0:  # 找没有被访问过的后继
                        V[car].myal[succ].visit = 1
                        Q.append(str(car) + "+" + str(succ))
        V[v].Tasks = V[v].Task.copy()
    return CPU, NUM

def Youhua(car, RG):
    # 优化剩余节点的R_new, 减少备份数
    RRGG = 1
    while RRGG > R_req / RG:
        kk = 0
        RRG_max = 0
        for nj in V[car].Tasks[ V[car].j + 1::]:
            RRG = 1
            if V[car].myal[nj[1]].num > 1:
                for nk in V[car].Tasks[V[car].j + 1::]:
                    if nk[1] != nj[1]:
                        RRG *= V[car].myal[nk[1]].R_new
                    else:
                        RRG *= 1 - pow(1 - V[car].myal[nj[1]].min_R, V[car].myal[nj[1]].num - 1)
            else:
                RRG = 0

            if RRG_max < RRG and RRG > R_req / RG:
                RRG_max = RRG
                kk = nj[1]
        if RRG_max > 0:
            V[car].myal[kk].R_new = 1 - pow(1 - V[car].myal[kk].min_R, V[car].myal[kk].num - 1)
            V[car].myal[kk].num -= 1
            print("备份数量得到第 1 次优化", V[car].myal[kk].num, V[car].myal[kk].NUM)
            RRGG = RRG_max
        else:
            V[car].myal[kk].NUM = V[car].myal[kk].num
            break

def MyAl_():
    T = 0
    CAR, NUM = Get_Task()
    wait_task = [ -1 for _ in range(M)]
    each_R = []
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    Avail = [min(ARR) for _ in range(M)]
    RG = [1 for _ in range(M)]
    _CAR_  = CAR.copy()

    Use_car = [0 for _ in range(M)]
    for cpu in CAR:
        _CAR_.remove(cpu)  # 方便记录后续未正式调度的车,不包括当前车
        V_Task = V[cpu].Task.copy()
        V_Task_ = V[cpu].Task.copy()
        for _ in range( len(V_Task) ):
            car_ni = V_Task_.pop(0)   # 主要调度当前DAG的任务，不能影响其本来的调度长度
            car, ni = car_ni[0], car_ni[1]
            Tran_C = np.array(V[car].C)
            # 得到前驱任务中最大的完成时间
            max_pred_FT = 0
            for pred, val in enumerate(Tran_C[:, ni]):
                if val > 0:
                    for ft in V[car].myal[pred].FT:
                        if max_pred_FT < ft:
                            max_pred_FT = ft
            # 不包括当前任务，是得出当前任务的可靠性要求，如果达到要求，可以避免不必要的备份
            other_R = 1   # 得到后面任务的总可靠性
            for nj in V[car].Task[ V[car].j + 1::]:
                other_R *= V[nj[0]].myal[nj[1]].R_new
            ni_req = R_req / ( RG[car]  *  other_R ) # 当前任务的可靠性要求

            while V[car].myal[ni].num:
                EST = [0 for _ in range(M)]
                EFT = [0 for _ in range(M)]
                O_t_ = [0 for _ in range(M)]
                kongxi = [0 for _ in range(M)]
                M_R = [1 for _ in range(M)]
                for s in range(M):
                    maxWaitPredReady = 0
                    if V[car].myal[ni].ind == 0:
                        maxWaitPredReady = V[car].arr_t
                    I_t, I_r = I_T_R(s, car, ni)
                    W_t, W_r = W_T_R(s, car, ni)
                    O_t, O_r = O_T_R(s, car, ni)
                    waitPredReady = max_pred_FT + I_t
                    if waitPredReady > maxWaitPredReady:
                        maxWaitPredReady = waitPredReady
                    EST[s] = max(Avail[s], maxWaitPredReady)
                    EFT[s] = EST[s] + W_t + O_t

                    M_R[s] *= I_r * W_r * O_r
                    O_t_[s] = O_t
                    kongxi[s] = EST[s] - Avail[s]
                mincar = M
                minEFT = INF
                for car_a in range(M):
                    if EFT[car_a] < minEFT and V[car].myal[ni].runcpu[car_a] == 0:
                        minEFT = EFT[car_a]
                        mincar = car_a

                # 其他车的候补任务加入集合
                for h in _CAR_:
                    if T > V[h].arr_t and wait_task[h] < 0:
                        wait_task[h] = V[h].Task[0][1]
                # print([ [h,wait_task[h]] for h in _CAR_ if wait_task[h] >= 0])

                # 处理器存在空隙，并且有候补车中的任务正在等待
                if kongxi[mincar] > 0 and sum(wait_task) > -M:
                    for car_a in _CAR_:
                        Tab = False
                        if wait_task[car_a] >= 0: # 该车的有候补任务
                            Tab = True
                        while Tab:
                            wait_car, wait_ni = car_a, wait_task[car_a]
                            if V[wait_car].myal[wait_ni].runcpu[mincar] != 0:  # 该处理器已分配该类任务，我们不做重调度
                                break
                            wait_Tran_C = np.array( V[wait_car].C )
                            max_pred_FT_a = 0
                            for pred_a, val_a in enumerate(wait_Tran_C[:, wait_ni]):
                                if val_a > 0:
                                    for ft_a in V[wait_car].myal[pred_a].FT:
                                        if max_pred_FT_a < ft_a:
                                            max_pred_FT_a = ft_a
                            # 不包括当前任务，是得出当前任务的可靠性要求，如果达到要求，可以避免不必要的备份
                            wait_other_R = 1  # 得到后面任务的总可靠性
                            for nj in V[wait_car].Task[ V[wait_car].j + 1::]:
                                wait_other_R *= V[nj[0]].myal[ nj[1] ].R_new
                            wait_ni_req = R_req / ( RG[wait_car] * wait_other_R)  # 当前任务的可靠性要求

                            maxWaitPredReady = 0
                            if V[wait_car].myal[wait_ni].ind == Derta:
                                maxWaitPredReady = V[wait_car].arr_t
                            wait_I_t, wait_I_r = I_T_R(mincar, wait_car, wait_ni)
                            wait_W_t, wait_W_r = W_T_R(mincar, wait_car, wait_ni)
                            waitPredReady = max_pred_FT_a + wait_I_t
                            wait_ni_EST = max( Avail[mincar], waitPredReady, maxWaitPredReady )
                            wait_car_Avail = wait_ni_EST + wait_W_t
                            # 说明该车的候补任务可以插入空隙中, 这里是真实调度
                            if EST[mincar] - wait_car_Avail >= 0.005:

                                wait_O_t, wait_O_r = O_T_R(mincar, wait_car, wait_ni)
                                V[wait_car].myal[wait_ni].ST.append(wait_ni_EST)
                                V[wait_car].myal[wait_ni].FT.append( wait_car_Avail + wait_O_t)
                                Avail[mincar] = wait_car_Avail

                                Use_car[mincar] += wait_W_t  # 计算利用率

                                V[wait_car].myal[wait_ni].runcpu[mincar] = 1
                                V[wait_car].myal[wait_ni].backups_cpu.append(mincar)
                                V[wait_car].myal[wait_ni].backups_num += 1
                                V[wait_car].myal[wait_ni].num -= 1
                                V[wait_car].myal[wait_ni].ni_fail_R *= 1 - wait_I_r * wait_W_r * wait_O_r

                                print("OK", [wait_car,wait_ni], wait_ni_EST, wait_car_Avail + wait_O_t, mincar)

                                # 已经调度完一个等候任务。
                                if V[wait_car].myal[wait_ni].num == 0 or 1 - V[wait_car].myal[wait_ni].ni_fail_R > wait_ni_req:

                                    V[wait_car].myal[wait_ni].R = 1 - V[wait_car].myal[wait_ni].ni_fail_R
                                    RG[wait_car] *= V[wait_car].myal[wait_ni].R
                                    V[wait_car].Task.pop(0)
                                    wait_task[wait_car] = V[wait_car].Task[0][1]

                                    Youhua(wait_car, RG[wait_car])
                                    V[wait_car].j += 1  # 当前任务在调度序列中的位置

                                    if 1 - V[wait_car].myal[wait_ni].ni_fail_R > wait_ni_req:
                                        V[wait_car].myal[wait_ni].num = 0
                                        break
                            else:
                                # Tab = False
                                break

                EST[mincar] = max(EST[mincar], Avail[mincar])  # 无用
                V[car].myal[ni].ST.append(EST[mincar])
                V[car].myal[ni].FT.append(minEFT)
                Avail[mincar] = minEFT - O_t_[mincar]
                Use_car[mincar] += Avail[mincar] - EST[mincar]  # 计算利用率
                V[car].myal[ni].runcpu[mincar] = 1
                V[car].myal[ni].backups_cpu.append(mincar)
                V[car].myal[ni].backups_num += 1
                V[car].myal[ni].num -= 1
                # 如果当前任务达到该任务的可靠性要求，则跳出循环，保存实际的备份数
                V[car].myal[ni].ni_fail_R *= 1 - M_R[mincar]

                # 记录时间轴，以便其他DAG任务加入等候集合中
                if T < Avail[mincar]:
                    T = Avail[mincar]

                if 1 - V[car].myal[ni].ni_fail_R > ni_req and V[car].myal[ni].backups_num < V[car].myal[ni].NUM:
                    # print("备份数量得到第 2 次优化", V[car].myal[ni].backups_num, V[car].myal[ni].NUM)
                    break

            # 计算当前任务的实际可靠性和已调度任务的总可靠性,(包括当前任务的可靠性)
            V[car].myal[ni].R = 1 - V[car].myal[ni].ni_fail_R
            RG[car] *= V[car].myal[ni].R
            Youhua(car, RG[car])
            V[car].j += 1  # 当前任务在调度序列中的位置
            # print(car_ni,V[car].myal[ni].backups_cpu , V[car].myal[ni].ST, V[car].myal[ni].FT)

        myal_num = 0
        for i in V[cpu].Task:
            myal_num += V[cpu].myal[i[1]].backups_num - 1
        # print(kkkkk)
        each_R.append(RG[cpu])
        V[cpu].myal_num = myal_num

    car_exe = []
    nums = 0
    max_max_ft = 0
    USE = [0 for _ in range(M)]
    for car in CAR:
        if (Avail[car]-min(ARR)) > 0:
            USE[car] = Use_car[car] / (Avail[car]-min(ARR))
        else:
            USE[car] = 0
        max_ft = 0.0
        nums += V[car].myal_num
        for ni in range(V[car].N):
            for ft in V[car].myal[ni].FT:
                if max_ft < ft:
                    max_ft = ft
        if max_max_ft < max_ft:
            max_max_ft = max_ft
        car_exe.append(max_ft - V[car].arr_t)
    print("Myal平均延迟:", round(sum(car_exe) / len(CAR), 2), nums, each_R, car_exe, sum(USE) / M )


    return sum(car_exe) / len(CAR), nums, sum(each_R)/len(CAR), sum(USE) / M


# 计算每个节点最小可靠性。同时，得到最差情况下的【初始备份数】====================================OK
def Get_num_up_():
    R_G = [1 for _ in range(M)]
    CAR = [i for i in range(M) if V[i].tab == 1]
    for v in CAR:
        R = []
        for ni in range(V[v].N):
            # 可靠性最差路径 和 最低可靠性CPU 为下限得到每个节点的最低可靠性
            for car in range(M):
                _, R_zhixing = W_T_R(car, v, ni)
                V[v].MYAL[ni].R_cpu[car] = low_R_path(V[v].MYAL[ni].I) * R_zhixing * low_R_path(V[v].MYAL[ni].O)
            V[v].MYAL[ni].min_R = V[v].MYAL[ni].R_new = min(V[v].MYAL[ni].R_cpu)
            V[v].MYAL[ni].num += 1
            R.append(V[v].MYAL[ni].R_new)
            R_G[v] *= V[v].MYAL[ni].R_new

        while R_G[v] < R_req:
            ni = R.index(min(R))
            V[v].MYAL[ni].R_new = 1 - ((1 - V[v].MYAL[ni].R_new) * (1 - V[v].MYAL[ni].min_R))
            V[v].MYAL[ni].num += 1  # 该节点确定备份，但处理器选择未确定，处理器都可以选择
            R[ni] = V[v].MYAL[ni].R_new
            R_task_G = 1
            for i in range(V[v].N):
                R_task_G *= V[v].MYAL[i].R_new
            R_G[v] = R_task_G

    for v in CAR:
        for ni in range(V[v].N):
            V[v].MYAL[ni].NUM = V[v].MYAL[ni].num
    return 0
# 得到DAG的调度顺序
def Get_Task_():
    # 计算每个节点最小可靠性。同时，得到最差情况下的【初始备份数】=========================================================
    NUM = Get_num_up_()
    # 入口任务加入就绪任务集合中
    CAR = [i for i in range(M) if V[i].tab == 1]
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    CPU = []
    for _ in CAR:
        v = ARR.index(min(ARR))
        CPU.append(v)
        ARR[v] = INF
        Indegree = []
        for ni in range(V[v].N):
            if V[v].MYAL[ni].ind == 0:
                Indegree.append(str(v) + "+" + str(ni))
        Q = Indegree.copy()
        while Q:
            # 得出此时Q中最大可靠性的任务--------------------------------
            B = []
            for car_ni in Q:
                car, ni = car_ni.split("+")
                B.append([int(car), int(ni)])
            A = [b for b in B if V[b[0]].MYAL[b[1]].ind == 0]

            A_min_R = []
            for a in A:
                # A_min_R.append( pow(V[a[0]].MYAL[a[1]].min_R, (V[a[0]].MYAL[a[1]].num + 1)) )
                A_min_R.append(V[a[0]].MYAL[a[1]].min_R)

            max_A_min_R = 0
            ni_id = len(A_min_R)
            for i, a_min_R in enumerate(A_min_R):
                if max_A_min_R < a_min_R:
                    max_A_min_R = a_min_R
                    ni_id = i
            car, ni = A[ni_id][0], A[ni_id][1]

            V[v].Task.append(A[ni_id])  # 保存任务调度集合

            Q.remove(str(car) + "+" + str(ni))

            # 就绪任务正式加入就绪集合--------------------
            if V[car].MYAL[ni].outd > 0:
                # 将车car中任务ni的后继加入Q中，同时让其后继任务的入度减一
                for succ in V[car].MYAL[ni].succs:
                    V[car].MYAL[succ].ind -= 1
                    if V[car].MYAL[succ].visit == 0:  # 找没有被访问过的后继
                        V[car].MYAL[succ].visit = 1
                        Q.append(str(car) + "+" + str(succ))
        V[v].Tasks = V[v].Task.copy()
    return CPU, NUM
def Youhua_(car, RG):
    # 优化剩余节点的R_new, 减少备份数
    RRGG = 1
    while RRGG > R_req / RG:
        kk = 0
        RRG_max = 0
        for nj in V[car].Tasks[V[car].j + 1::]:
            RRG = 1
            if V[car].MYAL[nj[1]].num > 1:
                for nk in V[car].Tasks[V[car].j + 1::]:
                    if nk[1] != nj[1]:
                        RRG *= V[car].MYAL[nk[1]].R_new
                    else:
                        RRG *= 1 - pow(1 - V[car].MYAL[nj[1]].min_R, V[car].MYAL[nj[1]].num - 1)
            else:
                RRG = 0
            if RRG_max < RRG and RRG > R_req / RG:
                RRG_max = RRG
                kk = nj[1]
            # 对任务x的备份数减一，根据Eq.(21),计算出集合Z的可靠性值R'_x
            # 如果 R'_x> R_req / RG：
                # 保存任务x和R'_x

        if RRG_max > 0:
            V[car].MYAL[kk].R_new = 1 - pow(1 - V[car].MYAL[kk].min_R, V[car].MYAL[kk].num - 1)
            V[car].MYAL[kk].num -= 1
            print("备份数量得到第 1 次优化", V[car].MYAL[kk].num, V[car].MYAL[kk].NUM)
            RRGG = RRG_max
        else:
            V[car].MYAL[kk].NUM = V[car].MYAL[kk].num
            break
def MYAL():
    for i in range(M):
        V[i].myal_num = 0
        V[i].j = 0
        V[i].Task = []
        V[i].Tasks = []

    T = 0
    CAR, NUM = Get_Task_()
    wait_task = [-1 for _ in range(M)]
    each_R = []
    ARR = [INF for _ in range(M)]
    for i in CAR:
        ARR[i] = V[i].arr_t
    Avail = [min(ARR) for _ in range(M)]
    RG = [1 for _ in range(M)]
    _CAR_ = CAR.copy()

    Use_car = [0 for _ in range(M)]
    for cpu in CAR:
        _CAR_.remove(cpu)  # 方便记录后续未正式调度的车,不包括当前车
        V_Task = V[cpu].Task.copy()
        V_Task_ = V[cpu].Task.copy()
        for _ in range(len(V_Task)):
            car_ni = V_Task_.pop(0)  # 主要调度当前DAG的任务，不能影响其本来的调度长度
            car, ni = car_ni[0], car_ni[1]
            Tran_C = np.array(V[car].C)
            # 得到前驱任务中最大的完成时间
            max_pred_FT = 0
            for pred, val in enumerate(Tran_C[:, ni]):
                if val > 0:
                    for ft in V[car].MYAL[pred].FT:
                        if max_pred_FT < ft:
                            max_pred_FT = ft
            # 不包括当前任务，是得出当前任务的可靠性要求，如果达到要求，可以避免不必要的备份
            other_R = 1  # 得到后面任务的总可靠性
            for nj in V[car].Tasks[V[car].j + 1::]:
                other_R *= V[nj[0]].MYAL[nj[1]].R_new
            ni_req = R_req / (RG[car] * other_R)  # 当前任务的可靠性要求

            while V[car].MYAL[ni].num:
                EST = [0 for _ in range(M)]
                EFT = [0 for _ in range(M)]
                O_t_ = [0 for _ in range(M)]
                kongxi = [0 for _ in range(M)]
                M_R = [1 for _ in range(M)]
                for s in range(M):
                    maxWaitPredReady = 0
                    if V[car].MYAL[ni].ind == 0:
                        maxWaitPredReady = V[car].arr_t
                    I_t, I_r = I_T_R(s, car, ni)
                    W_t, W_r = W_T_R(s, car, ni)
                    O_t, O_r = O_T_R(s, car, ni)
                    waitPredReady = max_pred_FT + I_t
                    if waitPredReady > maxWaitPredReady:
                        maxWaitPredReady = waitPredReady
                    EST[s] = max(Avail[s], maxWaitPredReady)
                    EFT[s] = EST[s] + W_t + O_t

                    M_R[s] *= I_r * W_r * O_r
                    O_t_[s] = O_t
                    kongxi[s] = EST[s] - Avail[s]
                mincar = M
                minEFT = INF
                for car_a in range(M):
                    if EFT[car_a] < minEFT and V[car].MYAL[ni].runcpu[car_a] == 0:
                        minEFT = EFT[car_a]
                        mincar = car_a

                # 按G_a的调度顺序实时地将任务添加到集合WT_a
                for h in _CAR_:
                    if T > V[h].arr_t and wait_task[h] < 0:
                        wait_task[h] = V[h].Task[0][1]

                # 计算节点存在空隙，并且wait_task ！= 空
                if kongxi[mincar] > 0 and sum(wait_task) > -M:
                    for car_a in _CAR_:
                        Tab = False
                        if wait_task[car_a] >= 0:  # 该车的有候补任务
                            Tab = True
                        while Tab:
                            # 选择集合WT中的第一个任务t进行调度
                            # 如果v不是有效计算节点
                                # break

                            # 计算任务t的可靠性要求
                            # 计算任务t在v上的开始时间、完成时间和计算节点的空闲时间 S_x;
                            #if ET-S_x > kkk；
                                # 调度任务t

                            wait_car, wait_ni = car_a, wait_task[car_a]
                            if V[wait_car].MYAL[wait_ni].runcpu[mincar] != 0:  # 该处理器已分配该类任务，我们不做重调度
                                break

                            wait_Tran_C = np.array(V[wait_car].C)
                            max_pred_FT_a = 0
                            for pred_a, val_a in enumerate(wait_Tran_C[:, wait_ni]):
                                if val_a > 0:
                                    for ft_a in V[wait_car].MYAL[pred_a].FT:
                                        if max_pred_FT_a < ft_a:
                                            max_pred_FT_a = ft_a

                            # 不包括当前任务，是得出当前任务的可靠性要求，如果达到要求，可以避免不必要的备份
                            wait_other_R = 1  # 得到后面任务的总可靠性
                            for nj in V[wait_car].Tasks[V[wait_car].j + 1::]:
                                wait_other_R *= V[nj[0]].MYAL[nj[1]].R_new
                            wait_ni_req = R_req / (RG[wait_car] * wait_other_R)  # 当前任务的可靠性要求

                            maxWaitPredReady = 0
                            if V[wait_car].MYAL[wait_ni].ind == Derta:
                                maxWaitPredReady = V[wait_car].arr_t
                            wait_I_t, wait_I_r = I_T_R(mincar, wait_car, wait_ni)
                            wait_W_t, wait_W_r = W_T_R(mincar, wait_car, wait_ni)
                            waitPredReady = max_pred_FT_a + wait_I_t
                            wait_ni_EST = max(Avail[mincar], waitPredReady, maxWaitPredReady)
                            wait_car_Avail = wait_ni_EST + wait_W_t

                            # 说明该车的候补任务可以插入空隙中, 这里是真实调度
                            if EST[mincar] - wait_car_Avail >= 0.005:

                                wait_O_t, wait_O_r = O_T_R(mincar, wait_car, wait_ni)
                                V[wait_car].MYAL[wait_ni].ST.append(wait_ni_EST)
                                V[wait_car].MYAL[wait_ni].FT.append(wait_car_Avail + wait_O_t)
                                Avail[mincar] = wait_car_Avail

                                Use_car[mincar] += wait_W_t  # 计算利用率

                                V[wait_car].MYAL[wait_ni].runcpu[mincar] = 1
                                V[wait_car].MYAL[wait_ni].backups_cpu.append(mincar)
                                V[wait_car].MYAL[wait_ni].backups_num += 1
                                V[wait_car].MYAL[wait_ni].num -= 1
                                V[wait_car].MYAL[wait_ni].ni_fail_R *= 1 - wait_I_r * wait_W_r * wait_O_r

                                print("OK", [wait_car, wait_ni], wait_ni_EST, wait_car_Avail + wait_O_t, mincar)

                                # 已经调度完一个等候任务。
                                if V[wait_car].MYAL[wait_ni].num == 0:  # or 1 - V[wait_car].myal[wait_ni].ni_fail_R > wait_ni_req:

                                    V[wait_car].MYAL[wait_ni].R = 1 - V[wait_car].MYAL[wait_ni].ni_fail_R
                                    RG[wait_car] *= V[wait_car].MYAL[wait_ni].R
                                    V[wait_car].Task.pop(0)
                                    wait_task[wait_car] = V[wait_car].Task[0][1]

                                    # Youhua(wait_car, RG[wait_car])
                                    V[wait_car].j += 1  # 当前任务在调度序列中的位置

                                    if 1 - V[wait_car].MYAL[wait_ni].ni_fail_R > wait_ni_req:
                                        V[wait_car].MYAL[wait_ni].num = 0
                                        break
                            else:
                                # Tab = False
                                break

                EST[mincar] = max(EST[mincar], Avail[mincar])  # 无用
                V[car].MYAL[ni].ST.append(EST[mincar])
                V[car].MYAL[ni].FT.append(minEFT)
                Avail[mincar] = minEFT - O_t_[mincar]
                Use_car[mincar] += Avail[mincar] - EST[mincar]  # 计算利用率
                V[car].MYAL[ni].runcpu[mincar] = 1
                V[car].MYAL[ni].backups_cpu.append(mincar)
                V[car].MYAL[ni].backups_num += 1
                V[car].MYAL[ni].num -= 1
                # 如果当前任务达到该任务的可靠性要求，则跳出循环，保存实际的备份数
                V[car].MYAL[ni].ni_fail_R *= 1 - M_R[mincar]

                # 记录时间轴，以便其他DAG任务加入等候集合中
                if T < Avail[mincar]:
                    T = Avail[mincar]

                # if 1 - V[car].MYAL[ni].ni_fail_R > ni_req and V[car].MYAL[ni].backups_num < V[car].MYAL[ni].NUM:
                #     # print("备份数量得到第 2 次优化", V[car].MYAL[ni].backups_num, V[car].MYAL[ni].NUM)
                #     break

            # 计算当前任务的实际可靠性和已调度任务的总可靠性,(包括当前任务的可靠性)
            V[car].MYAL[ni].R = 1 - V[car].MYAL[ni].ni_fail_R
            RG[car] *= V[car].MYAL[ni].R
            # Youhua_(car, RG[car])
            V[car].j += 1  # 当前任务在调度序列中的位置
            # print(car_ni,V[car].MYAL[ni].backups_cpu , V[car].MYAL[ni].ST, V[car].MYAL[ni].FT)

        MYAL_num = 0
        for i in V[cpu].Tasks:
            MYAL_num += V[cpu].MYAL[i[1]].backups_num - 1
            # print(kkkkk)
        each_R.append(RG[cpu])
        V[cpu].myal_num = MYAL_num


    car_exe = []
    nums = 0
    max_max_ft = 0
    USE = [0 for _ in range(M)]
    for car in CAR:
        if (Avail[car] - min(ARR)) > 0:
            USE[car] = Use_car[car] / (Avail[car] - min(ARR))
        else:
            USE[car] = 0
        max_ft = 0.0
        nums += V[car].myal_num
        for ni in range(V[car].N):
            for ft in V[car].MYAL[ni].FT:
                if max_ft < ft:
                    max_ft = ft
        if max_max_ft < max_ft:
            max_max_ft = max_ft
        car_exe.append(max_ft - V[car].arr_t)
    print("MYAL平均延迟:", round(sum(car_exe) / len(CAR), 2), nums, each_R, car_exe, sum(USE) / M)

    return sum(car_exe) / len(CAR), nums, sum(each_R) / len(CAR), sum(USE) / M


def txt_to_matrix(filename):
    data = []
    with open(filename, 'r') as f:  # with语句自动调用close()方法
        line = f.readline()
        while line:
            eachline = line.split()  ###按行读取文本文件，每行数据以列表形式返回
            read_data = [int(x) for x in eachline[0:1000]]  # TopN概率字符转换为float型
            # read_data = list(map(float, eachline))
            data.append(read_data)
            line = f.readline()
        return data  # 返回数据为双列表形式



if __name__ == '__main__':

    R_req = 0.995
    M = 4
    max_N = 40
    nums = 1

    # 各种参数，有待优化
    bandwidth = 10
    P = 0.1
    noise = pow(10,-13)
    A = 17.8
    bit = 1000000
    cyc_s = 1000000000
    Derta = 0

    K = 0
    e = []

    local_delay = []
    Rand_delay = []
    HEFT_delay = []
    ERRM_delay =[]
    Myal_delay = []
    Myal_delay_ = []

    sum_l = 0
    sum_r = 0
    sum_h = 0
    sum_e = 0
    sum_m = 0
    sum_m_ = 0

    local_beifen = []
    Rand_beifen = []
    HEFT_beifen = []
    ERRM_beifen = []
    Myal_beifen = []

    beifen_l = 0
    beifen_r = 0
    beifen_h = 0
    beifen_e = 0
    beifen_m = 0
    beifen_m_ = 0

    kekaoxing_l = 0
    kekaoxing_r = 0
    kekaoxing_h = 0
    kekaoxing_e = 0
    kekaoxing_m = 0
    kekaoxing_m_ = 0

    liyonglv_l = 0
    liyonglv_r = 0
    liyonglv_h = 0
    liyonglv_e = 0
    liyonglv_m = 0
    liyonglv_m_ = 0


    for num in range(nums):
        # path = "D:/MyPapers/[4]Paper/N/" + str(M) + str(max_N) + "/" + "Game" + str(num) + "/"
        path = "D:/MyPapers/[4]Paper/Car"+str(M)+"/" + "Game" + str(num) + "/"
        # 实例化车辆
        CAR = path + "car.txt"
        Car_data = txt_to_matrix(CAR)
        # 实例化车辆信息
        V = [Car() for _ in range(M)]
        # 对车进行赋值：初始化
        for i in range(M):
            V[i].resetCar(Car_data[i])

        # 主要求得辆车之间的最短路径 跳转数量较少为主
        CAR_path =  path + "carpath.txt"
        CAR_path_data = txt_to_matrix(CAR_path)
        graph = []
        for i in range(M):
            graph.append(CAR_path_data[i])
        paths, dists = floyd(graph)

        # 传输可靠性参数
        CAR_R = path + "carR.txt"
        CAR_R_data = txt_to_matrix(CAR_R)
        r = []
        for i in range(M):
            r.append([ float(c /100000) for c in CAR_R_data[i]])
        # 车之间的平均通信速率
        rate_set = []
        for car in range(M):
            for car_ in range(M):
                if car != car_:
                    distance = Distance(car, car_)
                    h = A * pow(1 / distance, 2)
                    rate_set.append(bandwidth * log((1 + P * h / noise), 2))
        aver_rate = sum(rate_set) / len(rate_set)
        print(aver_rate)

        CAR = [i for i in range(M) if V[i].tab == 1]
        for i in CAR:
            DAGs = path + "DAG_Car" + str(i) + ".txt"
            Car_DAGs_data = txt_to_matrix(DAGs)
            # print(Car_DAGs_data)
            arr_t = [ c / 10 for c in Car_DAGs_data[0]]
            I = [c / 10000 for c in Car_DAGs_data[1]]
            W = [ c / 1000 for c in Car_DAGs_data[2]]
            O = [ c / 10000 for c in Car_DAGs_data[3]]
            C = []
            for k in Car_DAGs_data[4:]:
                C.append( [c/100 for c in k] )
            # 对车上任务进行赋值：初始化

            V[i].resetHEFT( arr_t[0],I, W, O, C )
            V[i].resetERRM(arr_t[0],I, W, O, C)
            V[i].resetLocal(arr_t[0],I, W, O, C)
            V[i].resetMyal(arr_t[0],I, W, O, C)
            V[i].resetRand(arr_t[0], I, W, O, C)
            V[i].resetMYAL(arr_t[0], I, W, O, C)


        e.append(num)
        # ============ local ========
        l,num_l,R_l,o_l = Local()
        sum_l += l
        beifen_l += num_l
        local_delay.append(sum_l)
        local_beifen.append(beifen_l)

        kekaoxing_l += R_l
        liyonglv_l += o_l

        # ============ random ========
        r_, num_r,R_r,o_r = Rand()
        sum_r += r_
        beifen_r += num_r
        Rand_delay.append(sum_r)
        Rand_beifen.append(beifen_r)

        kekaoxing_r += R_r
        liyonglv_r += o_r



        # =========== ERRM ==========
        e_,num_e,R_e,o_e = ERRM()
        sum_e += e_
        beifen_e += num_e
        ERRM_delay.append(sum_e)
        ERRM_beifen.append(beifen_e)

        kekaoxing_e += R_e
        liyonglv_e += o_e

        # # =========== HEFT ==========

        h,num_h,R_h,o_h = HEFT()
        sum_h += h
        beifen_h += num_h
        HEFT_delay.append(sum_h)
        HEFT_beifen.append(beifen_h)
        kekaoxing_h += R_h
        liyonglv_h += o_h


        # # ============= 优化 ==========
        m, num_m,R_m,o_m = MyAl_()
        sum_m += m
        beifen_m += num_m
        Myal_delay.append(sum_m)
        Myal_beifen.append(beifen_m)

        kekaoxing_m += R_m
        liyonglv_m += o_m

        # ============ 消融实验 ===========
        # m_, num_m_, R_m_, o_m_ = MYAL()
        # sum_m_ += m_
        # beifen_m_ += num_m_
        # Myal_delay_.append(sum_m_)
        # Myal_beifen.append(beifen_m_)
        #
        # kekaoxing_m_ += R_m_
        # liyonglv_m_ += o_m_


        if m < h:
            K += 1
        print(num, "\n")

    print(K)
    print('local_delay',sum(local_delay)/nums/nums, beifen_l/nums, kekaoxing_l/nums, liyonglv_l/nums)
    print('Rand_delay',sum(Rand_delay)/nums/nums, beifen_r/nums, kekaoxing_r/nums, liyonglv_r/nums)
    print('ERRM_delay',sum(ERRM_delay)/nums/nums, beifen_e/nums, kekaoxing_e/nums, liyonglv_e/nums)
    print('HEFT_delay',sum(HEFT_delay)/nums/nums, beifen_h/nums, kekaoxing_h/nums, liyonglv_h/nums)
    print('Myal_delay',sum(Myal_delay)/nums/nums, beifen_m/nums, kekaoxing_m/nums, liyonglv_m/nums)
    # print('Myal_delay_', sum(Myal_delay_) / nums / nums, beifen_m_ / nums, kekaoxing_m_ / nums, liyonglv_m_ / nums)

    # # 输入统计数据
    # plt.figure(dpi=300)
    #
    # index = ('M=4', 'M=8', 'M=12', 'M=16')
    #
    # buy_number_female4 = [98.04738109903232, 98.85734576180012, 100.12920892141585, 97.9215425178624]
    # buy_number_female5 = [84.70901416211696, 137.36609083703385, 175.04980442383012, 231.8354873470919]
    # buy_number_female6 = [111.73609461069977, 122.28185106438073, 131.46348489527216, 143.02194333868152]
    # buy_number_female7 = [65.15864477481985, 61.45424179326724, 57.65158579907583,  60.92003628852229]
    # buy_number_female8 = [42.02857955716591, 34.123397042415164, 31.421072420337012, 31.995195206755735]
    #
    #
    #
    # bar_width = 0.15  # 条形宽度
    # Idx1 = np.arange(len(index))  # --rounds 2
    # Idx2 = Idx1 + bar_width
    # Idx3 = Idx1 + bar_width * 2
    # Idx4 = Idx1 + bar_width * 3
    # Idx5 = Idx1 + bar_width * 4
    #
    # # 使用两次 bar 函数画出两组条形图
    # plt.bar(Idx1, height=buy_number_female4, width=bar_width, color='#43747B', edgecolor='black', label='local_delay')
    # plt.bar(Idx2, height=buy_number_female5, width=bar_width, color='#EED5B9', edgecolor='black', label='ERRM_delay')
    # plt.bar(Idx3, height=buy_number_female6, width=bar_width, color='#679DBD', edgecolor='black', label='Rand_delay')
    # plt.bar(Idx4, height=buy_number_female7, width=bar_width, color='#FEF0D7', edgecolor='black', label='HEFT_delay')
    # plt.bar(Idx5, height=buy_number_female8, width=bar_width, color='#D54C3F', edgecolor='black', label='Myal_delay')
    #
    # plt.legend(frameon=False, loc='upper left')  # 显示图例
    # plt.xticks((Idx1 + Idx5) / 2, index)
    # plt.xlabel('Number of Vehicles')  # 横坐标轴标题
    # plt.ylabel('Delay')  # 纵坐标轴标题
    # plt.title('Arr = 0')  # 图形标题
    # plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    #
    # plt.show()
    #
    plt.figure()
    plt.plot(e, local_delay, 'y', label='local_delay')
    plt.plot(e, Rand_delay, 'k', label='Rand_delay')
    plt.plot(e, ERRM_delay, 'g', label='ERRM_delay')
    plt.plot(e, HEFT_delay, 'b', label='HEFT_delay')
    plt.plot(e, Myal_delay, 'r', label='Myal_delay')
    plt.legend()
    plt.title("Deley")
    plt.show()

    plt.figure()
    plt.plot(e, local_beifen, 'y', label='local_beifen')
    plt.plot(e, Rand_beifen, 'k', label='Rand_beifen')
    plt.plot(e, ERRM_beifen, 'g', label='ERRM_beifen')
    plt.plot(e, HEFT_beifen, 'b', label='HEFT_beifen')
    plt.plot(e, Myal_beifen, 'r', label='Myal_beifen')
    plt.legend()
    plt.title("beifen")
    plt.show()










