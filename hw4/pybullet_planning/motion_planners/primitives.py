from itertools import takewhile

from .rrt import TreeNode
from .utils import argmin, negate, get_pairs

ASYMETRIC = True


def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward and ASYMETRIC:
        return reversed(list(extend_fn(q2, q1))) # Forward model
    return extend_fn(q1, q2)


def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap=False, tree_frequency=2,
        sweep_collision_fn=None, **kwargs):
    """Takes current tree and extend it towards a new node (`target`).
    """
    assert tree_frequency >= 1
    # the nearest node in the tree to the target
    # the segments by connecting last to the target using the given extend fn
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    # check if the extended path collision-free, stop until find a collision
    if sweep_collision_fn is None:
        safe = list(takewhile(negate(collision_fn), extend))
    else:
        safe = []
        for q0, q1 in get_pairs(extend):
            if not collision_fn(q0):
                safe.append(q0)
                if sweep_collision_fn(q0, q1):
                    break
            else:
                break
        else:
            if not collision_fn(extend[-1]):
                safe.append(extend[-1])
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success

##################################

def distance_fn_from_extend_fn(extend_fn):
    # TODO: can compute cost between waypoints from extend_fn
    def distance_fn(q1, q2):
        path = list(extend_fn(q1, q2)) # TODO: cache
        return len(path) # TODO: subtract endpoints?
    return distance_fn
