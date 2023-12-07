from random import random

from .utils import irange, argmin, RRT_ITERATIONS, INF

__all__ = [
    'rrt',
]

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def draw(self, draw_fn, valid=True):
        segment = [] if self.parent is None else [self.config, self.parent.config]
        draw_fn(self.config, segment, valid, valid)
        # https://github.mit.edu/caelan/lis-openrave
        # from manipulation.primitives.display import draw_node, draw_edge
        # self.node_handle = draw_node(env, self.config, color=color)
        # if self.parent is not None:
        #     self.edge_handle = draw_edge(
        #         env, self.config, self.parent.config, color=color)

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))


def rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn,
        goal_test=lambda q: False, max_iterations=RRT_ITERATIONS, goal_probability=.2, max_time=INF, draw_fn=None):
    if collision_fn(start):
        return None
    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)]
    for i in irange(max_iterations):
        goal = random() < goal_probability or i == 0
        s = goal_sample() if goal else sample_fn()

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        for q in extend_fn(last.config, s):
            if collision_fn(q):
                break
            last = TreeNode(q, parent=last)
            nodes.append(last)
            if draw_fn:
                last.draw(draw_fn)
            if goal_test(last.config):
                return configs(last.retrace())
        else:
            if goal:
                return configs(last.retrace())
    return None
