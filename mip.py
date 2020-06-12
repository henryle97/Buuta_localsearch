import time

from ortools.sat.python import cp_model
from ortools.linear_solver import pywraplp
import numpy as np
from math import sqrt


class Dataset(object):
    def __init__(self, path='./data_10'):
        # read data from path
        self.path = path
        self.points, self.N = self.getData(self.path)

    # return points 2D, number points and demand of each point
    def getData(self, path):
        points, n, demand = [], 0, []
        # open and read file
        with open(path) as file:
            n = int(file.readline())
            # read points
            for i in range(n):
                line = file.readline().rstrip('\n').split()
                points.append([float(line[0]), float(line[1])])
            file.readline()

        return np.array(points), n


class PostMIP():
    def __init__(self, dataset, k, M=1000000):
        self.dataset = dataset
        self.N = dataset.N
        self.k, self.M = k, M
        self.dataset.points= list(self.dataset.points)
        for i in range(2 * k):
            self.dataset.points.append([0, 0])
        self.dataset.points = np.array(self.dataset.points)
        self.dict_ = {}
        for i in range(self.N+2*k+1):
            if i > self.N:
                self.dict_[i] = 0
            else:
                self.dict_[i] = i;

        # declare MIP solver
        self.solver = pywraplp.Solver("BK Post Mixed Integer Programing", pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

        # define variables
        self.X = np.empty((dataset.N + k,
                           dataset.N + 2 * k))  # (n+k) x (n+2k) , X[i, j] = 1 if j+1_th point is next point of i+1_th point
        self.router = np.empty((1, dataset.N + 2 * k))  # router of each point
        self.distances = np.empty((1, dataset.N + 2 * k))  # distances[i]: from root route to i

    def stateModel(self):
        # init variables
        self.X = np.array(
            [[self.solver.IntVar(0, 1, 'x[{0},{1}]'.format(i, j)) for j in range(self.X.shape[1])] for i in
             range(self.X.shape[0])])
        self.router = np.array(
            [self.solver.IntVar(1, self.k, 'Router of point {0}'.format(i + 1)) for i in range(self.router.shape[1])])
        self.distances = np.array(
            [self.solver.NumVar(0.0, self.solver.infinity(), 'Time of point {0}'.format(i + 1)) for i in
             range(self.distances.shape[1])])

        # add contraints
        N, k, M = self.dataset.N, self.k, self.M

        # distance[N+i] = 0, i = 0,1,..k-1
        for i in range(k):
            self.solver.Add(self.distances[N + i] == 0)
        # -----------------------------------------------

        # router[N+i] = router(N+i+k) = i+1 , i = 0, 1, ..k-1
        for i in range(self.k):
            self.solver.Add(self.router[N + i] - self.router[N + i + k] == 0)
            self.solver.Add(self.router[N + i] - i - 1 == 0)
        # -----------------------------------------------

        # sum(X[i, j]) = 1, i=0,1,...,N+k-1 , each point is not end depot, it has next point
        for i in range(N + k):
            self.solver.Add(self.solver.Sum(self.X[i, :]) == 1)
        # -----------------------------------------------------

        # sum(X[i, j]) = 1, j = 0,1,2,...N-1, N+k, ...N+2k-1
        for j in range(N + 2 * k):
            if j < N or j >= N + k:
                self.solver.Add(self.solver.Sum(self.X[:, j]) == 1)
        # ---------------------------------------------------

        # X[i, j] =1 -> X[j, i] == 0
        for i in range(self.N):
            for j in range(self.N):
                self.solver.Add(self.X[i, j] + self.X[j,i] <= 1)

        # sum(X[i, j]) = 0 , j = N,N-1, ..., N+k-1, next point of each point is not start depot
        for j in range(N, N + k):
            self.solver.Add(self.solver.Sum(self.X[:, j]) == 0)
        # -----------------------------------------------------

        # X[i, i] = 0 , i = 0,1, ..N+k-1
        for i in range(N + k):
            self.solver.Add(self.X[i, i] == 0)

        # distance[N+K+k] > 0 --> distances[i] > 0
        for i in range(self.k):
             self.solver.Add(self.distances[self.N + self.k + i] + 1 >= 1.1)

        # X[i, j] = 1 --> router[i] = router[j] , use big M
        for i in range(self.X.shape[0]):
            for j in range(self.X.shape[1]):
                self.solver.Add(self.router[i] + M * (1 - self.X[i, j]) >= self.router[j])
                self.solver.Add(self.router[i] - M * (1 - self.X[i, j]) <= self.router[j])

        # -----------------------------------------------------------

        def distance(p1, p2):
            return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        # X[i, j]= 1 -> distance[j] = distance[i] + d[i,j]
        for i in range(self.X.shape[0]):
            for j in range(self.X.shape[1]):
                d = 0
                if j < N or j >= N+k:
                    d = distance(self.dataset.points[i], self.dataset.points[j])
                self.solver.Add(self.distances[j] + M * (1 - self.X[i, j]) >= self.distances[i] + d)
                self.solver.Add(self.distances[j] - M * (1 - self.X[i, j]) <= self.distances[i] + d)
        # -----------------------------------------------------------

        # minimaze max distances
        for i in range(N + k + 1, N + 2 * k):
            self.solver.Add(self.distances[i] <= self.distances[N + k])
        self.solver.Minimize(self.distances[N + k])
        # self.solver.Minimize((self.k + k) * self.distances[N + k] + self.solver.Sum(self.distances[N+k:]))

    def search(self):
        t1 = time.time()
        status = self.solver.Solve()
        t2 = time.time() - t1
        print('Number Contraints :{0}, status : {1}'.format(self.solver.NumConstraints(), status))
        if status == pywraplp.Solver.OPTIMAL:
            print("Solution is found in {:2f}s".format(t2))

            # start: N+k -- end : N+K+k ( k =0..K-1)
            # for i in range(self.X.shape[0]):
            #     for j in range(self.X.shape[1]):
            #         print(self.X[i, j].solution_value(), end=' ')
            #     print()
            # print('router')
            # for r in self.router:
            #     print(r.solution_value(), end=' ')
            total_cost = 0
            for i in range(self.k):
                print("Route {}: ".format(i), end=" ")
                root_route = self.N + i
                u = root_route
                print(self.dict_[u+1]," -- ", end="")
                j = 0
                while j < self.X.shape[1]:
                    if self.X[u, j].solution_value() == 1:
                        print(self.dict_[j+1]," -- ", end="")

                        u = j
                        if u >= self.X.shape[0]: break
                        j = 0
                    else:
                        j += 1

                print()

                print("Cost = {:.4f}".format(self.distances[self.N + self.k + i].solution_value()))
                total_cost += self.distances[self.N + self.k + i].solution_value()
            print("Total cost = {:.4f}".format(total_cost))




dataset = Dataset(path='./data_4_mip')
app = PostMIP(dataset, 2)
app.stateModel()
app.search()
