#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from heapq import heappush, heappop, heapify

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
		graph = self.graph[odd_nodes, :][:, odd_nodes]  # подграф на нечетных вершинах
		n = len(graph)
		for i in range(n):
			graph[i, i] = np.inf
		graph = np.hstack((np.zeros((n, 1)), graph))
		graph = np.concatenate((np.zeros((1, n + 1)), graph))

		u = np.zeros(n + 1)
		v = np.zeros(n + 1)
		p = np.zeros(n + 1)
		way = np.zeros(n + 1)

		for i in np.arange(1, n + 1):
			p[0] = i
			j0 = 0
			minv = np.ones(n + 1) * np.inf
			used = np.zeros(n + 1).astype('bool')

			while True:
				used[j0] = True
				i0 = p[j0]
				delta = np.inf
				j1 = 0
				for j in np.arange(1, n + 1):
					if used[j] == False:
						i0 = int(i0)
						cur = graph[i0][int(j)] - u[i0] - v[int(j)]
						if cur < minv[j]:
							minv[j] = cur
							way[j] = j0
						if minv[j] < delta:
							delta = minv[j]
							j1 = j
				for j in np.arange(0, n + 1):
					if used[j] == True:
						u[int(p[j])] += delta
						v[j] -= delta
					else:
						minv[j] -= delta
				j0 = j1

				if p[j0] == 0:
					break

			while True:
				j0 = int(j0)
				j1 = way[j0]
				p[j0] = p[int(j1)]
				j0 = j1

				if j0 == 0:
					break

		res = np.zeros(n + 1)
		for j in np.arange(1, n + 1):
			res[int(p[j])] = j

		edges = []
		for j in np.arange(1, n + 1):
			edge = sorted([j, int(res[j])])
			res[int(res[j])] = j
			edges.append((odd_nodes[edge[0] - 1],
						  odd_nodes[edge[1] - 1]))
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
