import numpy as np
import pandas as pd
import random
import math
import copy


class Task:
    def __init__(self, id):
        self.id = id
        self.task_processors = []  # llist of tuples (best_processor, task.finish_time)
        self.position = None  # a tuple of  app user x_coordinate and y_coordinate
        self.cpu_capacity = 0
        self.ram = 0
        self.bandwidth = 0
        self.storage = 0
        self.coverage = []


class UAV:
    def __init__(self, id):
        self.id = id
        self.name = 'UAV'
        self.position = None
        self.cpu_capacity = 0
        self.ram = 0
        self.bandwidth = 0
        self.storage = 0
        self.range = 0
        self.queue = []
        self.active = False

    def enqueue_task(self, task):
        self.queue.append(task)

    def dequeue_task(self):
        if self.queue:
            return self.queue.pop(0)
        else:
            return None

    def queue_length(self):
        return len(self.queue)


class EdgeServer:
    def __init__(self, id):
        self.id = id
        self.name = 'EdgeServer'
        self.position = None  # a tuple of edgeserver x_coordinate and y_coordinate
        self.cpu_capacity = 0
        self.ram = 0
        self.bandwidth = 0
        self.storage = 0
        self.range = 0
        self.queue = []
        self.active = False

    def enqueue_task(self, task):
        self.queue.append(task)

    def dequeue_task(self):
        if self.queue:
            return self.queue.pop(0)
        else:
            return None

    def queue_length(self):
        return len(self.queue)


def initialize_tasks(df):
    task = [Task(i) for i in range(no_of_tasks)]
    resources = df.iloc[:, 1:5].values
    position_array = df.iloc[:, 5:].values

    for i in range(no_of_tasks):
        task[i].cpu_capacity = resources[i][0]
        task[i].ram = resources[i][1]
        task[i].bandwidth = resources[i][2]
        task[i].storage = resources[i][3]
        task[i].position = (position_array[i][0], position_array[i][1])

    return task


def initialize_servers(df):
    server = [EdgeServer(i) for i in range(no_of_servers)]
    resources = df.iloc[:, 1:5].values
    position_array = df.iloc[:, 5:].values

    for i in range(no_of_servers):
        server[i].cpu_capacity = resources[i][0]
        server[i].ram = resources[i][1]
        server[i].bandwidth = resources[i][2]
        server[i].storage = resources[i][3]
        server[i].position = (position_array[i][0], position_array[i][1])
        server[i].range = position_array[i][2]

    return server


def initialize_uavs(df):
    uav = [UAV(i + no_of_servers) for i in range(no_of_uavs)]
    resources = df.iloc[:, 1:5].values
    position_array = df.iloc[:, 5:].values

    for i in range(no_of_uavs):
        uav[i].cpu_capacity = resources[i][0]
        uav[i].ram = resources[i][1]
        uav[i].bandwidth = resources[i][2]
        uav[i].storage = resources[i][3]
        uav[i].position = (position_array[i][0], position_array[i][1])
        uav[i].range = position_array[i][2]

    return uav


def initialize_processors(servers, uavs):
    p = []
    for s in servers:
        p.append(s)

    for u in uavs:
        p.append(u)

    return p


def distance(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def initialize_coverages(tasks, processors):
    for task in tasks:
        x_1, y_1 = task.position
        for p in processors:
            x_2, y_2 = p.position
            r = p.range

            if distance(x_1, x_2, y_1, y_2) < r:
                task.lamda = True
                task.coverage.append(p)


def create_lambda_matrix(processors, tasks):
    lam_matrix = np.zeros((len(tasks), len(processors)), dtype=int)

    for i, task in enumerate(tasks):
        for j, processor in enumerate(processors):
            if processor in task.coverage:
                if processor.cpu_capacity >= task.cpu_capacity and \
                        processor.ram >= task.ram and \
                        processor.bandwidth >= task.bandwidth and \
                        processor.storage >= task.storage:
                    lam_matrix[i, j] = 1

    return lam_matrix


def calculate_sigma_u(b):
    return np.power(
        (math.gamma(1 + b) * np.sin((np.pi * b) / 2)) / (math.gamma((1 + b) / 2) * b * np.power(2, (b - 1) / 2)), 1 / b)


def calculate_step_size():
    u = random.uniform(0, 1) * sigma_u ** 2
    v = random.uniform(0, 1) * sigma_v ** 2
    s = u / np.power(v, 1 / beta)
    return s


def bits_needed(n):
    if n == 0:
        return 1  # Special case for 0, which requires 1 bit

    bits = 0
    while n > 0:
        bits += 1
        n = n // 2

    return bits


def observing(quantum):
    binary = []
    for q in quantum:
        rand = random.uniform(0, 1)
        if rand < q[0] ** 2:
            binary.append(0)
        else:
            binary.append(1)
    return binary


def decoding(binary, ndim):  # row
    decode = []
    num = 0
    Base = int(2 ** (no_of_bits - 1))
    t = 0
    for i in range(ndim):
        num += binary[i] * Base
        Base /= 2

        if (i + 1) % no_of_bits == 0:
            num = hashing(t, int(num))  # Apply hashing algorithm
            Base = int(2 ** (no_of_bits - 1))
            decode.append(int(num))
            num = 0
            t = t + 1

    return decode


def hashing(t, p):
    flag = 0
    cov_len = len(tasks[t].coverage)
    if p > (cov_len - 1) or p < 0 or lambda_matrix[t, p] == 0:
        while flag != 1 and len(tasks[t].coverage) != 0:
            h = int(p % cov_len)
            j = tasks[t].coverage[h].id
            if lambda_matrix[t, j] == 1:
                flag = 1
                p = j
            else:
                del tasks[t].coverage[h]
                cov_len -= cov_len
                p -= 1
        if flag != 1:
            p = -1
    else:
        p = tasks[t].coverage[p].id

    return p


def find_angular_position(quantum):
    v = []

    for x, y in quantum:
        theta = math.atan(y / x)
        v.append(theta)

    return v


def initialize_quantum_population(N, ndim):
    Q_pop = []
    for i in range(N):
        q = []
        for j in range(ndim):
            x = random.uniform(-1, 1)
            y = math.sqrt(1 - x ** 2)
            q.append([x, y])
        Q_pop.append(q)

    return Q_pop


def initialize_binary_population(Q_pop, N, ndim):  # quantum row
    B = []
    for i in range(N):
        b = observing(Q_pop[i])
        B.append(b)
    return B


def initialize_decoded_array(B, N, ndim):
    D = []
    for i in range(N):
        d = decoding(B[i], ndim)
        D.append(d)

    return D


def initialize_angular_position(Q_pop, N, ndim):
    A = []
    for i in range(N):
        ang = find_angular_position(Q_pop[i])
        A.append(ang)

    return A


def clear_all_queues():
    for processor in processors:
        processor.queue.clear()


def calculate_flying_energy():
    blade_energy = P_0 * (1 + 3 * (V_U_xy ** 2) / (V_tip ** 2))
    parasite_energy = 1 / 2 * drag_ratio * solidity * air_density * disc_area * (V_U_xy ** 3)
    induced_energy = P_1 * math.sqrt(math.sqrt(1 + (V_U_xy ** 4) / (4 * V_0 ** 2)) - (V_U_xy ** 2) / (2 * V_0 ** 2))
    vertical_energy = P_2 * V_U_z

    return blade_energy + parasite_energy + induced_energy + vertical_energy


def fitness(allocated_processors):
    global maximum_energy, maximum_delay
    total_delay, energy_offloading, energy_propulsion = 0, 0, 0
    processor_allocation = 0
    user_allocation_rate = 0
    phi = [0 for _ in range(no_of_processors)]

    for task_id, processor_id in zip(range(no_of_tasks), allocated_processors):
        if processor_id != -1:
            processor = processors[processor_id]
            task = tasks[task_id]
            processor.enqueue_task(task)
            queue_length = processor.queue_length()
            user_allocation_rate += 1 / no_of_tasks
            if phi[processor_id] == 0:
                phi[processor_id] = 1
                processor_allocation += 1 / no_of_processors
            f_b = processors[processor_id].cpu_capacity * 1000  # processor_cpu_capacity
            transmission_delay = L_in / R_jb
            receiving_delay = L_out / R_jb
            computation_delay = L_in * C_b / f_b
            waiting_delay = (queue_length - 1) * computation_delay
            total_delay += transmission_delay + receiving_delay + computation_delay + waiting_delay
            transmission_energy = P_jb * transmission_delay
            receiving_energy = P_jb * receiving_delay
            computaion_energy = k * (f_b ** 3) * computation_delay
            energy_offloading += transmission_energy + receiving_energy + computaion_energy

            # checking whether it is a UAV or Edge server
            if isinstance(processor, UAV):
                hovering_energy = (P_0 + P_1) * total_delay
                flying_energy = calculate_flying_energy()
                energy_propulsion += hovering_energy + flying_energy

    clear_all_queues()
    total_energy = energy_propulsion + energy_offloading
    maximum_energy = max(maximum_energy, total_energy)
    maximum_delay = max(maximum_delay, total_delay)
    f = w_1 * user_allocation_rate + w_2 * processor_allocation + w_3 * total_delay / maximum_delay \
        + w_4 * total_energy / maximum_energy
    return f


def initialize_fitness_matrix(pop):
    fitness_matrix = []
    for i in pop:
        fitness_matrix.append(fitness(i))

    return fitness_matrix


def calculate_new_quantum_bit(qbit, theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    angle = [[cos_theta, -sin_theta],
             [sin_theta, cos_theta]]
    angle = np.array(angle)
    qbit = np.array(qbit)

    updated_qbit = np.dot(angle, qbit)

    return list(updated_qbit)


def update_angular_position(theta_i, theta_d1, theta_d2):

    new_theta = [0 for _ in range(N_dim)]
    for j in range(N_dim):
        new_theta[j] = theta_i[j] + epsilon * (theta_d1[j] - theta_d2[j])
    return new_theta


def fitness_parameters(allocated_processors):
    global maximum_energy, maximum_delay
    total_delay, energy_offloading, energy_propulsion = 0, 0, 0
    processor_allocation = 0
    user_allocation_rate = 0
    phi = [0 for _ in range(no_of_processors)]

    for task_id, processor_id in zip(range(no_of_tasks), allocated_processors):
        if processor_id != -1:
            processor = processors[processor_id]
            task = tasks[task_id]
            processor.enqueue_task(task)
            queue_length = processor.queue_length()
            user_allocation_rate += 1 / no_of_tasks
            if phi[processor_id] == 0:
                phi[processor_id] = 1
                processor_allocation += 1 / no_of_processors
            f_b = processors[processor_id].cpu_capacity * 1000  # processor_cpu_capacity
            transmission_delay = L_in / R_jb
            receiving_delay = L_out / R_jb
            computation_delay = L_in * C_b / f_b
            waiting_delay = (queue_length - 1) * computation_delay
            total_delay += transmission_delay + receiving_delay + computation_delay + waiting_delay
            transmission_energy = P_jb * transmission_delay
            receiving_energy = P_jb * receiving_delay
            computaion_energy = k * (f_b ** 3) * computation_delay
            energy_offloading += transmission_energy + receiving_energy + computaion_energy

            # checking whether it is a UAV or Edge server
            if isinstance(processor, UAV):
                hovering_energy = (P_0 + P_1) * total_delay
                flying_energy = calculate_flying_energy()
                energy_propulsion += hovering_energy + flying_energy

    clear_all_queues()
    total_energy = energy_propulsion + energy_offloading
    maximum_energy = max(maximum_energy, total_energy)
    maximum_delay = max(maximum_delay, total_delay)

    normalized_energy = total_energy/maximum_energy
    normalized_delay = total_delay/maximum_delay
    un_normalized_fitness =  f = w_1 * user_allocation_rate + w_2 * processor_allocation + w_3 * total_delay +  w_4 * total_energy

    return user_allocation_rate, processor_allocation, total_energy, total_delay, normalized_energy, normalized_delay, un_normalized_fitness


# import all the datasets
server_data = pd.read_csv('server_9.csv')
task_data = pd.read_csv('user_500.csv')
uav_data = pd.read_csv('uav_6.csv')

# initialize constants
no_of_tasks = len(task_data)
no_of_servers = len(server_data)
no_of_uavs = len(uav_data)
no_of_processors = no_of_uavs + no_of_servers
B_0 = 1000000  # (hz) Bandwidth of the channel
SNC = 20  # (db)
R_jb = B_0 * math.log2(1 + SNC)
L_in, L_out = 3000000, 3000000  # (Mb)
C_b = 1000  # cycles/bit
P_jb, P_bj = 0.2, 0.2  # (Watt)  is the required power per unit of time to transmit a task from user to server
P_0 = 158.76  # is energy consumed per unit time due to the rotation of the rotary blade
P_1 = 88.63  # is energy consumed per unit time due to the induction
P_2 = 11.46  # power per unit time associated with UAV’s vertical movement
V_tip = 120  # m/s, rotor blade tip speed.
V_0 = 4.03  # m/s, mean rotor induced velocity in hover
solidity = 0.05  #
air_density = 1.225  # kg/m^3
drag_ratio = 0.3
disc_area = 0.503  # m^2 rotator disc area
flying_time = 3600  # seconds
V_U_xy = 20  # m/s
V_U_z = 5  # m/s
k = 1e-27  # computational Energy constant
w_1 = 0.2  # weight of the UAR
w_2 = 0.1  # weight of the Edge server allocation
w_3 = 0.4  # weight of the delay
w_4 = 0.3  # weight of the energy
beta = 1.5
sigma_u = calculate_sigma_u(beta)
sigma_v = 1
P_a = 0.5
epsilon = 0.002

# initialize max and min energy delay values
maximum_delay = 0
maximum_energy = 0

# initializing Parameters
G_max = 2000
N_pop = 100
no_of_bits = bits_needed(no_of_processors)
N_dim = no_of_bits * no_of_tasks

# initialize resources
tasks = initialize_tasks(task_data)
servers = initialize_servers(server_data)
uavs = initialize_uavs(uav_data)
processors = initialize_processors(servers, uavs)

# adding the processors that are in range of tasks
initialize_coverages(tasks, processors)

# initializing Lamda matrix
lambda_matrix = create_lambda_matrix(processors, tasks)

# initializing population
quantum_population = initialize_quantum_population(N_pop, N_dim)

# initializing Binary population
binary_population = initialize_binary_population(quantum_population, N_pop, N_dim)

# initializing Decoded array
decoded_array = initialize_decoded_array(binary_population, N_pop, N_dim)

# initializing Angular_position
angular_position = initialize_angular_position(quantum_population, N_pop, N_dim)


# initializing fitness matrix
F = initialize_fitness_matrix(decoded_array)
g_best_row = np.argmin([ind for ind in F])  # finding G_best row
g_best = F[g_best_row]
g_best_angular = angular_position[g_best_row]
g_best_decoded = decoded_array[g_best_row]
print(g_best_decoded)
print("Intial G_best : ", g_best)

# initializing Main function
for g in range(G_max):
    new_angular_position = [[0 for _ in range(N_dim)] for _ in range(N_pop)]
    new_quantum_population = [[[0, 0] for _ in range(N_dim)] for _ in range(N_pop)]

    for i in range(N_pop):
        for j in range(no_of_tasks):
            s = calculate_step_size()
            new_angular_position[i][j] = angular_position[i][j] + s * (angular_position[i][j] - g_best_angular[j])
            new_quantum_population[i][j] = calculate_new_quantum_bit(quantum_population[i][j],
                                                                     new_angular_position[i][j])

        temp_binary = observing(new_quantum_population[i])
        temp_decode = decoding(temp_binary, N_dim)
        decoded_array[i] = temp_decode
        new_fitness = fitness(temp_decode)
        if new_fitness < F[i]:
            angular_position[i] = new_angular_position[i]
            F[i] = new_fitness

        r = random.uniform(0, 1)
        if r < P_a:
            d1, d2 = np.random.choice(N_pop, 2, replace=False)
            new_angular_position[i] = update_angular_position(angular_position[i], angular_position[d1],angular_position[d2])
        for j in range(N_dim):
            new_quantum_population[i][j] = calculate_new_quantum_bit(quantum_population[i][j],
                                                                     new_angular_position[i][j])

        temp_binary_2 = observing(new_quantum_population[i])
        temp_decode_2 = decoding(temp_binary, N_dim)
        decoded_array[i] = temp_decode_2
        new_fitness = fitness(temp_decode_2)
        F[i] = new_fitness
        # print("New fitness : ", new_fitness)
        if new_fitness < g_best:
            print("*****************G best has been changed****************")
            g_best_row = i
            g_best = new_fitness
            g_best_angular = new_angular_position[i]
            g_best_decoded = temp_decode_2
        if g_best > fitness(g_best_decoded):
            print("******G_BEST CHANGED******* : ", g_best)
            g_best = fitness(g_best_decoded)
    quantum_population = new_quantum_population
    print("Generation ",g," : ", g_best)
    angular_position = new_angular_position

print("Actual Best Fitness : ", g_best)
print("Observed Best Fitness : ", fitness(g_best_decoded))

uar, proc_alloc, energy, delay, norm_energy, norm_delay, un_norm_fitness = fitness_parameters(g_best_decoded)

with open(f'Results_{no_of_tasks}_{no_of_processors}_gen_{G_max}.txt', 'w') as file:
    file.write(f'NO OF ITERATIONS: \t')
    file.write(str(G_max))

    file.write(f'\nNO OF USERS: \t')
    file.write(str(no_of_tasks))

    file.write(f'\nNO OF PROCESSORS: \t')
    file.write(str(no_of_servers) + " + " + str(no_of_uavs) + " = " + str(no_of_processors))

    file.write(f'\nUSER ALLOCATION RATE : \t')
    file.write(str(uar))

    file.write(f'\nPROCESSOR ALLOCATION : \t')
    file.write(str(proc_alloc))

    file.write(f'\nENERGY CONSUMPTION: \t')
    file.write(str(energy))

    file.write(f'\nDELAY CONSUMPTION : \t')
    file.write(str(delay))

    file.write(f'\n FITNESS VALUE : \t')
    file.write(str(un_norm_fitness))

    file.write(f'\nNORMALIZED ENERGY: \t')
    file.write(str(norm_energy))

    file.write(f'\nNORMALIZED DELAY: \t')
    file.write(str(norm_delay))

    file.write(f'\nNORMALIZED FITNESS: \t')
    file.write(str(g_best))

    file.write(f'\nMAXIMUM ENERGY: \t')
    file.write(str(maximum_energy))

    file.write(f'\nMAXIMUM DELAY: \t')
    file.write(str(maximum_delay))


















