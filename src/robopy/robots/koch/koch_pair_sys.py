from .koch_follower import KochFollower
from .koch_leader import KochLeader


class KochPairSys:
    """Class representing a pair of Koch robotic arms: Leader and Follower"""

    def __init__(self, leader: KochLeader, follower: KochFollower):
        self.leader = leader
        self.follower = follower

    def connect(self) -> None:
        """Connect both the leader and follower arms"""
        self.leader.connect()
        self.follower.connect()

    def disconnect(self) -> None:
        """Disconnect both the leader and follower arms"""
        self.leader.disconnect()
        self.follower.disconnect()
