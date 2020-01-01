#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from heapq import heappush, heappop, heapify
import os


class MetricTSP:

	def __init__(self, num_nodes, graph):
		self.graph = graph
		self.num_nodes = num_nodes

	# алгоритм Прима
	def build_mst(self):
		# расстояние от i-й вершины до дерева
		d = np.ones(self.num_nodes) * np.inf
		d[0] = 0

		# p[i] - номер самого близкого предка вершины i в дереве
		p = np.zeros(self.num_nodes)

		# приоритетная очередь с ключами d[i]
		q = []
		for i in range(self.num_nodes):
			heappush(q, (d[i], i))

		# сет с вершинами остова
		mst_nodes = set()

		mst = []

		v = heappop(q)[1]  # самая близкая к остову вершина
		mst_nodes.add(v)
		while len(q) > 0:
			# релаксируем ребра для соседей
			for u in range(self.num_nodes):
				if (u in mst_nodes) == False and self.graph[u, v] < d[u]:
					q.remove((d[u], u))
					d[u] = self.graph[u, v]
					p[u] = v
					heappush(q, (d[u], u))
					heapify(q)
			v = heappop(q)[1]
			mst_nodes.add(v)
			mst.append((int(p[v]), v))

		print('mst: ', mst)
		self.mst = mst

	# вершины из mst  с нечетными степенями
	def odd_mst_nodes(self):
		deg = np.zeros(self.num_nodes)
		for edge in self.mst:
			deg[edge[0]] += 1
			deg[edge[1]] += 1
		res = []
		for i in range(self.num_nodes):
			if deg[i] % 2 == 1:
				res.append(i)
		res = sorted(res)
		return res

	# минимальное совершенное паросочетание
	def min_perfect_matching(self):
		odd_nodes = self.odd_mst_nodes()
		arr = self.graph[odd_nodes, :][:, odd_nodes]  # подграф на нечетных вершинах
		
		v = len(arr)
		res = str(v) + '\n'
		res += str(int(v*(v-1)/2)) + '\n'
		for i in range(v):
			for j in range(v):
				if i < j:
					res += str(i) + ' ' + str(j) \
							+ ' ' + str(arr[i, j]) + '\n'

		with open('../min_cost_perfect_matching/input.txt', 'w') as f:
			f.write(res)

		os.system('g++ -O3 ../min_cost_perfect_matching/Example.cpp \
					../min_cost_perfect_matching/BinaryHeap.cpp \
					../min_cost_perfect_matching/Matching.cpp \
					../min_cost_perfect_matching/Graph.cpp \
					-o ../min_cost_perfect_matching/example')

		os.system('./../min_cost_perfect_matching/example \
					-f ../min_cost_perfect_matching/input.txt \
					--minweight')

		edges = []

		with open('output.txt', 'r') as f:
			line = f.readline()
			while line:
				u = int(line.split(' ')[0])
				v = int(line.split(' ')[1][:-1])
				edges.append((u, v))
				line = f.readline()

		edges = list(set(edges))
		print('min_perfect_matching: ', edges)
		return edges

	# поиск гамильтонова цикла
	def ham_cycle(self):
		self.build_mst()
		matching = self.min_perfect_matching()	

		# граф из остова и паросочетания
		graph = {} 
		for edge in self.mst + matching:
			if edge[0] in graph.keys():
				graph[edge[0]].append(edge[1])
			else:
				graph[edge[0]] = [edge[1]]
			if edge[1] in graph.keys():
				graph[edge[1]].append(edge[0])
			else:
				graph[edge[1]] = [edge[0]]
				
		# ищем эйлеров цикл
		eul_cycle = []
		stack = [0]
		while len(stack) > 0:
			v = stack[-1]
			if len(graph[v]) == 0:
				eul_cycle.append(v)
				stack = stack[:-1]
			else:
				u = graph[v][-1]
				graph[v].remove(u)
				graph[u].remove(v)
				stack.append(u)
		print('eul_cycle: ', eul_cycle)

		# удаляем повторяющиеся вершины, получаем гам. цикл		
		ham_cycle = list(dict.fromkeys(eul_cycle)) + [eul_cycle[0]]
		print('ham_cycle: ', ham_cycle)

		return ham_cycle