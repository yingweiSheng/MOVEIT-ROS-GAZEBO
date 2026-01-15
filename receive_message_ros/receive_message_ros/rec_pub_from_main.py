# 性质：网关节点，外部命令的Publisher，ROS2与外部的通信接口
# 作用：轮询网络端口，将json文件转换为ROS2读取的消息，并发布到话题
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String 
import zmq
import json

class ZmqBridgeNode(Node):
    def __init__(self):
        super().__init__('zmq_bridge_from_main')
        
        # --- ROS 2 初始化 ---
        # 创建发布者，发布到双臂控制节点监听的 Topic
        self.publisher_ = self.create_publisher(String, '/bridge/master_arm_cmd', 1)
        
        # --- ZMQ 初始化 ---
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REP) # REP (Reply) 对应 Client 的 REQ
        self.zmq_socket.bind("tcp://*:5555") # 绑定端口
        
        self.timer = self.create_timer(0.01, self.zmq_callback)
        
        self.get_logger().info("ROS 2 ZMQ Bridge 已启动，正在监听端口 5555...")

    def zmq_callback(self):
        try:
            # 1. 尝试非阻塞接收消息
            # flags=zmq.NOBLOCK 是关键，如果没有消息，进入异常处理：zmq.Again 
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            
            # 2. 如果收到消息，进行处理
            self.process_message(message)
            
            # 3. 回复 Client (必须回复，否则 Client 下次发送会卡住)
            self.zmq_socket.send_string("ROS_OK")
            
        except zmq.Again:
            # 没有收到消息，直接跳过，让 ROS 继续处理其他事情
            pass
        except Exception as e:
            self.get_logger().error(f"处理消息出错: {e}")
            # 发生错误也要回复，防止 Client 死锁
            self.zmq_socket.send_string(f"ERR: {str(e)}")

    def process_message(self, json_msg):
        """
        解析 JSON 并发布 ROS 消息
        """
        try:
            data = json.loads(json_msg)
            
            # 这里的逻辑完全取决于你双臂节点需要什么格式的数据
            # 比如，我们可以直接把 JSON 原样转发，让双臂节点去解析
            ros_msg = String()
            ros_msg.data = json_msg
            self.publisher_.publish(ros_msg)
            
            self.get_logger().info(f"已转发指令, timestamp: {data.get('timestamp')}")

        except json.JSONDecodeError:
            self.get_logger().warn("收到非 JSON 格式数据")

def main(args=None):
    rclpy.init(args=args)
    node = ZmqBridgeNode()
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