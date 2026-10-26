"""Optimal assignment (Hungarian / Kuhn-Munkres) for the task dispatcher.

`hungarian(cost)` takes a rectangular cost matrix (rows = robots, columns =
tasks; math.inf = not allowed) and returns the (row, column) pairs of a
minimum-cost matching that assigns min(rows, columns) pairs.  O(n^3), pure
Python: the fleet has a few dozen robots and tasks at most.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

BIG = 1e9


def hungarian(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    n_rows = len(cost)
    n_cols = len(cost[0]) if n_rows else 0
    if n_rows == 0 or n_cols == 0:
        return []
    n = max(n_rows, n_cols)
    # square matrix, padding and forbidden pairs as a large finite cost
    a = [[0.0] * (n + 1) for _ in range(n + 1)]
    for i in range(n):
        for j in range(n):
            c = cost[i][j] if i < n_rows and j < n_cols else 0.0
            a[i + 1][j + 1] = BIG if c == math.inf else c
    # e-maxx formulation (1-based, potentials u / v, matching p)
    u = [0.0] * (n + 1)
    v = [0.0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [math.inf] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], math.inf, 0
            for j in range(1, n + 1):
                if not used[j]:
                    cur = a[i0][j] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    out = []
    for j in range(1, n + 1):
        i = p[j]
        if i and i <= n_rows and j <= n_cols and cost[i - 1][j - 1] != math.inf:
            out.append((i - 1, j - 1))
    return sorted(out)
