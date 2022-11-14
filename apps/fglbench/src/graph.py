#!/usr/bin/env python3

import sys
import csv
import matplotlib.pyplot as plt

line_names = {}
benchmark_data = {}
max_cores = 0

for i in range(1, len(sys.argv), 2):
    line_id = (i - 1) // 2
    line_names[line_id] = sys.argv[i+1]
    with open(sys.argv[i]) as data_file:
        tsv_iter = csv.reader(data_file, delimiter='\t')
        for line in tsv_iter:
            benchmark_data[(line_id, int(line[0]), int(line[1]))] = int(line[2])
            max_cores = max(max_cores, int(line[1]))

def plot_ipc_benchmark_0():
    plt.clf()
    xticks = [1, 2, 3, 4]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 0, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("Total IPC pingpong throughput across N cores")
    plt.xlabel('Cores')
    plt.ylabel('IPC/s')
    plt.savefig("ipc0.png")

def plot_ipc_benchmark_1():
    plt.clf()
    xticks = [0, 1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 1, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("IPC pingpong throughput of 1 core with N contender cores")
    plt.xlabel('Number of contender cores')
    plt.ylabel('IPC/s')
    plt.savefig("ipc1.png")

def plot_ipc_benchmark_2():
    plt.clf()
    xticks = [1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(2, max_cores + 1):
            data.append(benchmark_data[(i, 2, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("Total IPC pingpong throughput\nacross N cores with 1 contender core")
    plt.xlabel('Cores')
    plt.ylabel('IPC/s')
    plt.savefig("ipc2.png")

def plot_notification_benchmark_0():
    plt.clf()
    xticks = [1, 2, 3, 4]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 3, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("Total signal throughput across N cores")
    plt.xlabel('Cores')
    plt.ylabel('Signal/s')
    plt.savefig("ntfn0.png")

def plot_notification_benchmark_1():
    plt.clf()
    xticks = [0, 1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 4, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("Signal throughput\nof 1 core with N contender cores")
    plt.xlabel('Number of contender cores')
    plt.ylabel('Signal/s')
    plt.savefig("ntfn1.png")

def plot_notification_benchmark_2():
    plt.clf()
    xticks = [1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(2, max_cores + 1):
            data.append(benchmark_data[(i, 5, core_count)])
        plt.plot(xticks, data, label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.title("Total signal throughput\nacross N cores with 1 contender core")
    plt.xlabel('Cores')
    plt.ylabel('Signal/s')
    plt.savefig("ntfn2.png")

plot_ipc_benchmark_0()
plot_ipc_benchmark_1()
plot_ipc_benchmark_2()
plot_notification_benchmark_0()
plot_notification_benchmark_1()
plot_notification_benchmark_2()
