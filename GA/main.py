import numpy as np
from enum import Enum
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from motor_team.interface import Interface
from motor_team.smc_controller import SMCController
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.pid_controller import PIDController
from motor_team.mpc_controller import MPCController
import matplotlib.pyplot as plt
from unit.main import Unit
import csv, time
from datetime import datetime

class Controller(Enum):
    PID = 1
    SMC = 2
    MPC = 3

class GeneticAlgorithm:
    def __init__(self, controller):
        self.utils = Utils()
        self.dynamics = Dynamics()
        self.motor = Interface(utils=self.utils)

        # Parameters
        self.generations = 50
        self.N = 20
        self.N_P = 16
        self.mutation_sol_prob = 0.05
        self.mutation_gene_prob = 0.1
        self.controller = controller
        self.count = 0
        self.genetic_num = 6
        self.genetic_digit_num = 15

        self.file = open(f'ga_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', encoding='utf-8')
        
        self.wr = csv.writer(self.file, delimiter=',', lineterminator='\n')
        # self.wr = csv.writer(self.file)
        

    def initial_population(self):
        population = np.zeros((self.N, 6, 15), dtype=int)

        # 0과 3번째 범주를 전체 길이(15비트)로 무작위 초기화
        population[:, 0, :] = np.random.randint(0, 2, (self.N, 15))
        population[:, 1, -10:] = np.random.randint(0, 2, (self.N, 10))
        population[:, 2, -12:] = np.random.randint(0, 2, (self.N, 12))

        population[:, 3, -11:] = np.random.randint(0, 2, (self.N, 11))
        population[:, 4, -9:] = np.random.randint(0, 2, (self.N, 9))
        population[:, 5, -9:] = np.random.randint(0, 2, (self.N, 9))

        return population

    def evaluate_fitness(self, solution):
        self.count += 1
        print(f'Count: {self.count}')
        Kp1 = sum([2 ** (self.genetic_digit_num-1-i) * solution[0][i] for i in range(self.genetic_digit_num)])
        Ki1 = sum([2 ** (self.genetic_digit_num-1-i) * solution[1][i] for i in range(self.genetic_digit_num)])
        Kd1 = sum([2 ** (self.genetic_digit_num-1-i) * solution[2][i] for i in range(self.genetic_digit_num)])
        Kp2 = sum([2 ** (self.genetic_digit_num-1-i) * solution[3][i] for i in range(self.genetic_digit_num)])
        Ki2 = sum([2 ** (self.genetic_digit_num-1-i) * solution[4][i] for i in range(self.genetic_digit_num)])
        Kd2 = sum([2 ** (self.genetic_digit_num-1-i) * solution[5][i] for i in range(self.genetic_digit_num)])

        controller = PIDController(motor=self.motor, dynamics=self.dynamics, utils=self.utils, kp1=Kp1, ki1=Ki1, kd1=Kd1, kp2=Kp2, ki2=Ki2, kd2=Kd2)
        # controller = SMCController(motor=motor, dynamics=dynamics, utils=utils)

        test = Unit(utils=self.utils, dynamics=self.dynamics, motor=self.motor, controller=controller)
        try:
            test.execute(500, 200)
            test.execute(300, 200)
            test.execute(150, 100)
            test.execute(100, 70)
            test.execute(50, 30)
            test.execute(0, 0)

            # self.motor.disableTorque()
            fitness = test.getRMSE()

            print(f'Current Log >> Fitness: {fitness} >> Kp1: {Kp1}, Ki1: {Ki1}, Kd1: {Kd1}, Kp2: {Kp2}, Ki2: {Ki2}, Kd2: {Kd2}')
            self.wr.writerow([fitness, Kp1, Ki1, Kd1, Kp2, Ki2, Kd2])
            
            return fitness
        except:    # 예외가 발생했을 때 실행됨
            print('예외가 발생했습니다1.')
            return 10000
       

       

    def crossover(self, solution1, solution2):
        # np.random.randint(1, 4, size=2) = 1, 2, 3
        p0 = np.random.randint(1, self.genetic_digit_num-1, size=1)[0]
        p1 = np.random.randint(6, self.genetic_digit_num-1, size=1)[0]
        p2 = np.random.randint(4, self.genetic_digit_num-1, size=1)[0]

        p3 = np.random.randint(5, self.genetic_digit_num-1, size=1)[0]
        p4 = np.random.randint(7, self.genetic_digit_num-1, size=1)[0]
        p5 = np.random.randint(7, self.genetic_digit_num-1, size=1)[0]

        # 부모 2개의 형질을 일정 비율로 잘라서 자식에게 물려줌
        child = np.zeros((self.genetic_num, self.genetic_digit_num), dtype=int)
        child[0, :p0] = solution1[0, :p0]
        child[0, p0:] = solution2[0, p0:]
        child[1, :p1] = solution1[1, :p1]
        child[1, p1:] = solution2[1, p1:]
        child[2, :p2] = solution1[2, :p2]
        child[2, p2:] = solution2[2, p2:]

        child[3, :p3] = solution1[3, :p3]
        child[3, p3:] = solution2[3, p3:]
        child[4, :p4] = solution1[4, :p4]
        child[4, p4:] = solution2[4, p4:]
        child[5, :p5] = solution1[5, :p5]
        child[5, p5:] = solution2[5, p5:]

        return child

    def mutation(self, child, p):
        for i in range(self.genetic_num):
            for j in range(self.genetic_digit_num):
                if np.random.random() < p:
                    if i == 0:
                        child[i, j] = 1 - child[i, j]
                    elif i == 1:
                        if j > 4:
                            child[i, j] = 1 - child[i, j]
                    elif i == 2:
                        if j > 2:
                            child[i, j] = 1 - child[i, j]

                    elif i == 3:
                        if j > 3:
                            child[i, j] = 1 - child[i, j]
                    elif i == 4:
                        if j > 5:
                            child[i, j] = 1 - child[i, j]
                    elif i == 5:
                        if j > 5:
                            child[i, j] = 1 - child[i, j]

        return child

    def bestLog(self, best, score):
        Kp1 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[0][i] for i in range(self.genetic_digit_num)])
        Ki1 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[1][i] for i in range(self.genetic_digit_num)])
        Kd1 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[2][i] for i in range(self.genetic_digit_num)])
        Kp2 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[3][i] for i in range(self.genetic_digit_num)])
        Ki2 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[4][i] for i in range(self.genetic_digit_num)])
        Kd2 = sum([2 ** (self.genetic_digit_num - 1 - i) * best[5][i] for i in range(self.genetic_digit_num)])

        print(f'Local Best Log >> Kp1: {Kp1}, Ki1: {Ki1}, Kd1: {Kd1}, Kp2: {Kp2}, Ki2: {Ki2}, Kd2: {Kd2}, Score: {score}')
        # print(f'Kp2: {Kp2}, Ki2: {Ki2}, Kd2: {Kd2}')

    def execute(self):
        # 현재 세대 초기화(1세대)
        current_population = self.initial_population()
        best_score = 100000
        best_solution = None

        for _ in range(self.generations):
            fitness_values = np.array([self.evaluate_fitness(sol) for sol in current_population])
            # print(f'Current Log >> Fitness: {fitness_values.min()}')

            if fitness_values.min() < best_score:
                best_score = fitness_values.min()
                best_solution = current_population[fitness_values.argmin()]
                self.bestLog(best_solution, best_score)

            # 이번 세대 중 가장 우월한 N_P 명을 뽑아 new_population 에 저장
            parents = current_population[np.argsort(fitness_values)[:self.N_P]]
            new_population = np.empty((self.N, self.genetic_num, self.genetic_digit_num), dtype=int)  # Reinitialize for new generation
            new_population[:self.N_P] = parents[:self.N_P]

            for i in range(self.N_P, self.N):
                # 이전 세대 에서 부모 2명 뽑기
                parent_1, parent_2 = current_population[np.random.choice(self.N_P, 2)]

                # 교차를 통해 다음 세대 1명 만들기
                child = self.crossover(parent_1, parent_2)

                # 확률적 으로 변이 나타남
                if np.random.random() < self.mutation_sol_prob:
                    child = self.mutation(child, self.mutation_gene_prob)

                # 다음 세대에 저장
                new_population[i] = child

            # 다음 세대로 이동
            current_population = new_population

        # 최적해와 적합도 출력
        print("Best Solution:", best_solution)
        print("Best Fitness:", best_score)
        print('count', self.count)
        self.bestLog(best_solution, best_score)

if __name__ == '__main__':
    algorithm = GeneticAlgorithm(Controller.PID)
    algorithm.execute()