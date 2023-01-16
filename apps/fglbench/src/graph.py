#!/usr/bin/env python3

import sys
import csv
import matplotlib
import matplotlib.pyplot as plt

line_names = {}
benchmark_data = {}
max_cores = 0

matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'sans',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

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
            data.append(benchmark_data[(i, 0, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Cores')
    plt.ylabel('IPC/µs')
    plt.savefig("ipc0.pgf")

def plot_ipc_benchmark_1():
    plt.clf()
    xticks = [0, 1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 1, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Number of contender cores')
    plt.ylabel('IPC/µs')
    plt.savefig("ipc1.pgf")

def plot_ipc_benchmark_2():
    plt.clf()
    xticks = [1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(2, max_cores + 1):
            data.append(benchmark_data[(i, 2, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Cores')
    plt.ylabel('IPC/µs')
    plt.savefig("ipc2.pgf")

def plot_notification_benchmark_0():
    plt.clf()
    xticks = [1, 2, 3, 4]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 3, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Cores')
    plt.ylabel('Signals/µs')
    plt.savefig("ntfn0.pgf")

def plot_notification_benchmark_1():
    plt.clf()
    xticks = [0, 1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(1, max_cores + 1):
            data.append(benchmark_data[(i, 4, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Number of contender cores')
    plt.ylabel('Signals/µs')
    plt.savefig("ntfn1.pgf")

def plot_notification_benchmark_2():
    plt.clf()
    xticks = [1, 2, 3]
    for i in range(2):
        data = []
        for core_count in range(2, max_cores + 1):
            data.append(benchmark_data[(i, 5, core_count)] / 1000000.0)
        plt.plot(xticks, data, '-o', label=line_names[i])

    plt.xticks(xticks)
    plt.legend()
    plt.ylim(0)
    plt.xlabel('Cores')
    plt.ylabel('Signals/µs')
    plt.savefig("ntfn2.pgf")

plot_ipc_benchmark_0()
plot_ipc_benchmark_1()
plot_ipc_benchmark_2()
plot_notification_benchmark_0()
plot_notification_benchmark_1()
plot_notification_benchmark_2()
