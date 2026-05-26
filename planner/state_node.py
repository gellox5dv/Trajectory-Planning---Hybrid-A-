from models.models import *
from anytree import NodeMixin
from motion_primitives import MotionPrimitive


class StateNode(NodeMixin):
    def __init__(self,
                 id: int,
                 state_stamped: EgoStateStamped,
                 node_cost: float,
                 heuristic_cost: float, 
                 path_cost: float,
                 total_cost: float,
                 detailed_costs: Dict[str, float],
                 target_region_reached: bool,
                 depth: float,
                 parent: NodeMixin = None,
                 motion_primitive: MotionPrimitive = None):
        super().__init__()
        self.id = id
        self.state_stamped = state_stamped
        self.node_cost = node_cost
        self.heuristic_cost = heuristic_cost
        self.path_cost = path_cost
        self.total_cost = total_cost
        self.detailed_costs = detailed_costs
        self.target_region_reached = target_region_reached
        self.depth = depth
        self.parent = parent
        self.motion_primitive = motion_primitive

    def __lt__(self, other):
        return self.total_cost < other.total_cost
    
    def __repr__(self):
        return (
            f"StateNode(id={self.id}, depth={self.depth}, "
            f"node_cost={self.node_cost:.2f}, "
            f"f={self.total_cost:.2f}, g={self.path_cost:.2f}, h={self.heuristic_cost:.2f}, "
            f"target_region_reached={self.target_region_reached})"
        )
    