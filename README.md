# Задача коммивояжёра

## task.pdf

1) Описание двух постановок задачи: стандартная задача коммивояжёра и метрическая

2) Доказательство теоремы, что если P != NP, то для стандартной задачи коммивояжёра не существует константных алгоритмов приближения.

3) Описание алгоритма, дающего 2-приближение для метрической задачи на основе остовного дерева.

4) Описание алгоритма, дающего 1.5-приближение для метрической задачи на основе остовного дерева и паросочетания.

## src

Имплементация алгоритма, дающего 1.5-приближение для метрической задачи.

Зависимости: numpy

Использование: 

1) Загрузить файлы MetricTSP.py и main.py в одну папку.

2) Установить numpy: pip install numpy.

3) Запустить main.py: python3 main.py.

4) Ввести кол-во вершин и рёбра графа.

5) На выходе получить: mst - минимальное остовное дерево, min_perfect_matching - минимальное совершенное паросочетание в подграфе из вершин с нечетными степенями в остовном дереве, eul_cycle - построенный по mst + matching эйлеров цикл, ham_cycle - полученный из эйлерова цикла гамильтонов цикл - ответ на задачу.

## min_cost_perfect_matching

Имплементация алгоритма сжатия соцветий, находящего совершенное паросочетание минимального веса в произвольном графе. Алгоритм был взаимствован с github.com/dilsonpereira/Minimum-Cost-Perfect-Matching и изменён под конкретную задачу.

## test.pdf

IPython-notebook с проведением тестов работы реализованного алгоритма.
