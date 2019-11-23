#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from MetricTSP import MetricTSP

def main():
	print('Введите кол-во вершин n: ')
	num_nodes = int(input())
	graph = np.zeros((num_nodes, num_nodes))

	print('Введите ребра в формате u v w, где (u, v) - ребро, w - вес:')
	print('(всего должно быть n(n-1)/2 строчек)')
	for i in np.arange(num_nodes):
		for j in np.arange(i + 1, num_nodes):
			u, v, w = np.array(input().split(' ')).astype('int')
			graph[u, v] = w
			graph[v, u] = w

	tsp_solver = MetricTSP(num_nodes, graph)

	tsp_solver.ham_cycle()


if __name__ == '__main__':
	main()