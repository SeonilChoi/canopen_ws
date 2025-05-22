import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import os
import json
import time
import subprocess
import traceback
from common_canopen.motor_manager.motor_controller import MotorController
from common_canopen.motor_manager.motor_factory import MotorFactory

def runCommand(command):
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return True, result.stdout
    except subprocess.CalledProcessError as e:
        return False, e.stderr

def isCANInterfaceUp(interface):
    success, output = runCommand(f"ip link show {interface}")
    if not success:
        return False
    return "UP" in output and "state UP" in output

class CANOpenManager(Node):
    def __init__(self):
        super().__init__('canopen_manager')
        self.initializeMotors()
        
        QOS_REKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.joint_state_pub = self.create_publisher(JointState, 'canopen/joint_states', QOS_REKL10V)
        self.joint_state_timer = self.create_timer(0.01, self.publishJointState)
        self.motor_status_serv = self.create_service(Trigger, 'canopen/check_motors', self.checkMotorsStatus)
        self.motor_errors_serv = self.create_service(Trigger, 'canopen/get_all_motors_errors', self.getAllMotorsErrors)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.JointStateCallback, QOS_REKL10V)
        self.count = 0;
        try:
            self.motor_controller = MotorController(channel=self.can_interface)
            for motor_info in self.motors_info:
                motor = MotorFactory.create_motor(motor_info)
                self.motor_controller.add_motor(motor)

                motor_name = motor_info['name']
                node_id = motor_info['node_id']

                self.create_subscription(
                    Float64,
                    f'canopen/{motor_name}/command',
                    lambda msg, id=node_id: self.SingleJointCallback(msg, id),
                    qos_profile=QOS_REKL10V
                )

            SYNC_INTERVAL = 0.05
            self.motor_controller.all_motors_init_start(interval=SYNC_INTERVAL)
            self.get_logger().info("[CANOpenManager::init] Motor controller started")
        except Exception as e:
            self.get_logger().error(f"[CANOpenManager::init] Failed to initialize motors: {e}")
            self.motor_controller = None

        self.get_logger().info("[CANOpenManager::init] CANOpen manager node initialized")
        for motor_info in self.motors_info:
            self.get_logger().info(f"[CANOpenManager::init] Motor {motor_info['name']} NodeID: {motor_info['node_id']} initialized")
        time.sleep(3)

    def initializeMotors(self):
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('heartbeat_interval', 1000) # ms
        self.declare_parameter('can_bitrate', 1000000) # 1Mbps
        self.declare_parameter('can_txqueue_len', 1000)
        self.declare_parameter('motors_info_path', None)

        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').get_parameter_value().integer_value
        self.can_bitrate = self.get_parameter('can_bitrate').get_parameter_value().integer_value
        self.can_txqueue_len = self.get_parameter('can_txqueue_len').get_parameter_value().integer_value
        self.motors_info_path = self.get_parameter('motors_info_path').get_parameter_value().string_value

        self.get_logger().info(f"[CANOpenManager::initializeMotors] CAN interface: {self.can_interface}")
        self.get_logger().info(f"[CANOpenManager::initializeMotors] Heartbeat interval: {self.heartbeat_interval} ms")
        self.get_logger().info(f"[CANOpenManager::initializeMotors] CAN bitrate: {self.can_bitrate} kbps")
        self.get_logger().info(f"[CANOpenManager::initializeMotors] CAN TX queue length: {self.can_txqueue_len}")

        self.motors_status_ok = True
        self.motors_status_cache = {}
        self.last_joint_state_publish_time = self.get_clock().now()
        self.status_check_interval = rclpy.duration.Duration(seconds=0.5)

        if self.setCANInterface(self.can_interface, self.can_txqueue_len):
            self.get_logger().info(f"[CANOpenManager::initializeMotors] CAN Interface {self.can_interface} setup complete")
        else:
            self.get_logger().warn(f"[CANOpenManager::initializeMotors] CAN Interface {self.can_interface} setup failed, continue...")

        self.motors_info = self.loadMotorsInfo(self.motors_info_path)

    def setCANInterface(self, interface, txqueuelen):
        is_root = os.geteuid() == 0
        if not is_root:
            self.get_logger().warn("[CANOpenManager::setCANInterface] CAN Interface setup requires root privileges.")
            self.get_logger().warn("[CANOpenManager::setCANInterface] Please run the node with sudo or manually execute the following commands:")
            self.get_logger().warn("[CANOpenManager::setCANInterface] sudo slcand -o -s8 /dev/ttyACM0 can0")
            self.get_logger().warn(f"[CANOpenManager::setCANInterface] sudo ip link set {interface} up")
            self.get_logger().warn(f"[CANOpenManager::setCANInterface] sudo ip link set {interface} txqueuelen {txqueuelen}")
        
            if isCANInterfaceUp(interface):
                self.get_logger().info(f"[CANOpenManager::setCANInterface] CAN Interface {interface} is already up")
                return True
            else:
                self.get_logger().warn(f"[CANOpenManager::setCANInterface] CAN Interface {interface} is not up")
                return False

        if isCANInterfaceUp(interface):
            self.get_logger().info(f"[CANOpenManager::setCANInterface] CAN Interface {interface} is already up")
            return True
        
        success, output = runCommand("slcand -o -s8 /dev/canable can0")
        if not success:
            self.get_logger().warn(f"[CANOpenManager::setCANInterface] slcand execution failed: {output}")
            self.get_logger().info("[CANOpenManager::setCANInterface] It may be already running or there is a problem with the device.")
        else:
            self.get_logger().info("[CANOpenManager::setCANInterface] slcand execution successful")
            time.sleep(1)
        
        commands = [
            f"ip link set {interface} up",
            f"ip link set {interface} txqueuelen {txqueuelen}"
        ]        
        for cmd in commands:
            success, output = runCommand(cmd)
            if not success:
                self.get_logger().error(f"[CANOpenManager::setCANInterface] CAN Interface setup failed: {output}")
                return False
        
        success, output = runCommand("ip link show")
        if success:
            self.get_logger().info(f"[CANOpenManager::setCANInterface] CAN Interface setup result:\n{output}")        
        return isCANInterfaceUp(interface)

    def loadMotorsInfo(self, motors_info_path):
        try:
            with open(motors_info_path, 'r') as f:
                motors_info = json.load(f)
            return motors_info.get('motors', [])
        except Exception as e:
            self.get_logger().error(f"[CANOpenManager::loadMotorsInfo] Failed to load motors info: {e}")
            return []
        
    def isMotorStatusOK(self):
        motor_ok = True
        for motor_info in self.motors_info:
            node_id = motor_info['node_id']
            try:
                status = self.motor_controller.get_motor_status(node_id)
                self.motors_status_cache[node_id] = status
                if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                    motor_ok = False

                    break
            except Exception as e:
                self.get_logger().error(f"[CANOpenManager::isMotorStatusOK] Failed to get motor status for node ID {node_id}: {e}")
                motor_ok = False
                break

        return motor_ok

    def publishJointState(self):
        if not self.motor_controller:
            self.get_logger().warn("[CANOpenManager::publishJointState] Motor controller not initialized, skipping joint state publish")
            return
        
        current_time = self.get_clock().now()
        if current_time - self.last_joint_state_publish_time >= self.status_check_interval:
            self.last_joint_state_publish_time = current_time

            self.motors_status_ok = self.isMotorStatusOK()

            if self.motors_status_ok:
                joint_state = JointState()
                joint_state.header.stamp = current_time.to_msg()
                try:
                    positions = self.motor_controller.get_positions()
                    velocities = self.motor_controller.get_velocities()
                    efforts = self.motor_controller.get_torques()

                    for motor_info in self.motors_info:
                        node_id = motor_info['node_id']

                        motor_name = motor_info['name']
                        position = positions[node_id]
                        velocity = velocities[node_id]
                        effort = efforts[node_id]

                        joint_state.name.append(motor_name)
                        joint_state.position.append(position)
                        joint_state.velocity.append(velocity)
                        joint_state.effort.append(effort)

                    self.joint_state_pub.publish(joint_state)
                except Exception as e:
                    self.get_logger().error(f"[CANOpenManager::publishJointState] Failed to read motor info: {e}")
            else:
                self.get_logger().warn("[CANOpenManager::publishJointState] Motors are not ready, skipping joint state publish")
                return

    def checkMotorsStatus(self, request, response):
        if not self.motor_controller:
            self.get_logger().warn("[CANOpenManager::checkMotorsStatus] Motor controller not initialized, skipping motors status check")
            return
        
        try:
            if self.isMotorStatusOK():
                response.success = True
                response.message = "Motors are ready"
            else:
                response.success = False
                response.message = "Motors are not ready"
        except Exception as e:
            self.get_logger().error(f"[CANOpenManager::checkMotorsStatus] Failed to check motors status: {e}")
            return
        
        return response
        
    def getAllMotorsErrors(self, request, response):
        if not self.motor_controller:
            response.success = False
            response.message = "Motor controller not initialized"
            return response
        
        try:
            errors = {}
            for motor_info in self.motors_info:
                node_id = motor_info['node_id']
                motor_name = motor_info['name']
                
                try:
                    error_code = self.motor_controller.get_error_code(node_id)
                    motor_status = self.motor_controller.get_motor_status(node_id)
                    
                    # 타입 확인 및 처리
                    formatted_hex_code = "Unknown"
                    if error_code is not None:
                        if isinstance(error_code, int):
                            formatted_hex_code = f"0x{error_code:04X}"

                        elif isinstance(error_code, dict):
                            # 딕셔너리에서 필요한 필드 추출
                            if "hex_code" in error_code:
                                formatted_hex_code = error_code["hex_code"]
                            elif "code" in error_code and isinstance(error_code["code"], int):
                                formatted_hex_code = f"0x{error_code['code']:04X}"
                            else:
                                formatted_hex_code = "Unknown"         
                        else:
                            formatted_hex_code = str(error_code)
                    errors[motor_name] = {
                        "node_id": node_id,
                        "error_code": formatted_hex_code,
                        "status": motor_status,
                    }
                except Exception as e:
                    errors[motor_name] = {
                        "node_id": node_id,
                        "error_code": "Error reading",
                        "message": str(e)
                    }
            
            import json
            response.success = True
            response.message = json.dumps(errors)
        except Exception as e:
            self.get_logger().error(f"[CANOpenManager::getAllMotorsErrors] Failed to get motors errors: {e}")
            response.success = False
            response.message = f"Failed to get motors errors: {str(e)}"
        
        return response
        
    def JointStateCallback(self, msg):
        if not self.motor_controller:
            self.get_logger().warn("[CANOpenManager::JointStateCallback] Motor controller not initialized, skipping joint state callback")
            return
        
        self.motors_status_ok = self.isMotorStatusOK()
        if not self.motors_status_ok:
            self.get_logger().warn("[CANOpenManager::JointStateCallback] Motors are not ready, skipping joint state callback")
            self.motor_controller.disable_all_motors(error_motor_id=node_id, error_reason="motor status is not ready")
            return
        
        for i, joint_name in enumerate(msg.name):
            motor_info = next((m for m in self.motors_info if m['name'] == joint_name), None)
            if motor_info:
                node_id = motor_info['node_id']
                goal_position = msg.position[i]
                try:
                    self.motor_controller.set_position(node_id, goal_position)
                except Exception as e:
                    self.get_logger().error(f"[CANOpenManager::JointStateCallback] Failed to set goal position for {joint_name}: {e}")
                    
        self.count += 1
        self.get_logger().info(f"count: {self.count}")

    def SingleJointCallback(self, msg, node_id):
        if not self.motor_controller:
            self.get_logger().warn("[CANOpenManager::SingleJointCallback] Motor controller not initialized, skipping single joint callback")
            return
        
        if not self.motors_status_ok:
            self.get_logger().warn("[CANOpenManager::SingleJointCallback] Motors are not ready, skipping single joint callback")
            return
        
        try:
            self.motor_controller.set_position(node_id, msg.data)
        except Exception as e:
            motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
            self.get_logger().error(f"[CANOpenManager::SingleJointCallback] Failed to set {motor_info['name']} to {msg.data} rad: {e}")

def main(args=None):
    rclpy.init(args=args)
    canopen_manager = CANOpenManager()
    try:
        rclpy.spin(canopen_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(canopen_manager, 'motor_controller') and canopen_manager.motor_controller:
            try:
                canopen_manager.motor_controller.disable_all_motors(error_motor_id=None, error_reason="keyboard interrupt")
                canopen_manager.get_logger().info("[main] All motors disabled")
                
            except Exception as e:
                canopen_manager.get_logger().error(f"[main] Failed to disable motors: {e}")
        canopen_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
