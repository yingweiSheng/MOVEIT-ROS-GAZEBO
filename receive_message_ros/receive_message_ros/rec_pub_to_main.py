import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import zmq

class FeedbackBridge(Node):
    def __init__(self, ip="localhost", port=5556):
        super().__init__('zmq_bridge_to_main')

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        
        address = f"tcp://{ip}:{port}"
        print(f"正在尝试连接到: {address}")
        self.zmq_socket.connect(address)
        
        self.subscriber = self.create_subscription(
            String,
            "/bridge/cmd_feedback",
            self.send_feedback_pose, 
            1
        )

    def send_feedback_pose(self, msg):
        raw_data = msg.data

        try:
            print(f"准备发送反馈数据...")
            self.zmq_socket.send_string(raw_data)
            # 阻塞等待回复
            reply = self.zmq_socket.recv_string()
            print(f"SUCCESS: 收到 main 回复: {reply}")
            return True
        except Exception as e:
            print(f"ERROR: 通信出错: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zmq_socket.close()
        node.zmq_context.term()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()