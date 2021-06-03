import numpy as np

file = open("controller_log.txt", 'r')

iteration_times = []

for line in file:
    if "full mpc iteration" in line.lower():
        iteration_times.append(float(line.split("duration:")[1].split("µ")[0]))

print(iteration_times)

iteration_times = iteration_times[1:]

print("Solver time mean:", np.mean(iteration_times))
print("Solver time standard deviation:", np.std(iteration_times))