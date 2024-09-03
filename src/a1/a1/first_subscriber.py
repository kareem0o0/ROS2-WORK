import rclpy as rp
from std_msgs.msg import String

def main():
    global node
    rp.init()
    node = rp.create_node("subsciber")
    sub = node.create_subscription(String,"topic",callback,10)
    msg = String
    rp.spin(node)
def callback(msg):
    global node
    node.get_logger().info(f"mes is {msg.data}")
if __name__ == '__main__' :
    main()

