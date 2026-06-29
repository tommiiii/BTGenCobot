import sys
import rclpy
from rclpy.node import Node
import ikpy.chain

# We can't do this easily if ikpy isn't installed here, but the user is running the node inside docker!
# We can just run docker exec.
