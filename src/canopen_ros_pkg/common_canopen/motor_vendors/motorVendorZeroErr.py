from ..motor_manager.abstract_motor import AbstractMotor
import time
import csv
from datetime import datetime
from math import pi
import rclpy
from rclpy.logging import get_logger

# 로깅 메소드를 위한 클래스 - ROS2 로깅 시스템을 사용하도록 수정
class Logger:
    def __init__(self, name="MotorVendorZeroErr"):
        self.name = name
        self.logger = get_logger(name)

    def info(self, msg):
        self.logger.info(msg)

    def warn(self, msg):
        self.logger.warn(msg)

    def error(self, msg):
        self.logger.error(msg)

    def debug(self, msg):
        self.logger.debug(msg)

class MotorVendorZeroErr(AbstractMotor):
    """제조사 A 모터에 대한 구체 구현."""
    PULSE_PER_REVOLUTION = 524288  # 한 바퀴당 펄스 수

    
    
    def __init__(self, node_id, eds_path, zero_offset=0, operation_mode='PROFILE_POSITION',
                 profile_velocity=1.0, profile_acceleration=1.0, profile_deceleration=1.0, name=None):  # name 파라미터 추가
        super().__init__(node_id, eds_path, zero_offset, operation_mode,
                        profile_velocity, profile_acceleration, profile_deceleration,
                        name)  # name을 부모 클래스 생성자에 전달
        self._logger = Logger(f"MotorVendorZeroErr_Node{node_id}")
        self.dt = 0.01  # 기본 시간 간격 (10ms)
        self.current_velocity_old = 0.0  # 이전 속도 값 초기화
        
    def get_logger(self):
        """로거 반환"""
        return self._logger
        
    def init(self, operation_mode=None):
        if operation_mode:
            self.operation_mode = operation_mode.upper()
        
        if self.operation_mode not in self.OPERATION_MODES:
            raise ValueError(f"지원하지 않는 동작 모드입니다: {self.operation_mode}")
            
        self.get_logger().info(f"Init motor node: {self.node_id}")
        
        # 모터 상태 변수 초기화
        self.motor_status = {
            'statusword': 0,
            'ready_to_switch_on': False,
            'switched_on': False,
            'operation_enabled': False,
            'fault': False,
            'voltage_enabled': False,
            'quick_stop': False,
            'switch_on_disabled': False,
            'warning': False
        }
        
        # 모드 설정
        mode_value = self.OPERATION_MODES[self.operation_mode]
        self.node.sdo['Modes of operation'].raw = mode_value
        self.get_logger().info(f'Modes of operation: {hex(mode_value)} ({self.operation_mode})')

        self.ModeOfOperationDisplay = self.node.sdo['Modes of operation display'].raw
        self.get_logger().info(f'Modes of operation display: {self.ModeOfOperationDisplay}')
        
        # 모드별 초기화
        self._init_mode_specific_parameters()

        self.plusToRad = 2 * pi / self.PULSE_PER_REVOLUTION
        
    def try_auto_reset(self):
        """모터 에러 발생 시 자동 리셋을 시도하는 함수"""
        if self.motor_status.get('fault', False):
            self.get_logger().info(f"Attempting to auto-reset motor {self.node_id} after fault")
            self.reset()
            return True
        return False
    


    def _convert_rad_to_pulse(self, rad_value):
        """라디안 값을 펄스 카운트로 변환"""
        return int((rad_value * self.PULSE_PER_REVOLUTION) / (2 * pi))

    def _init_mode_specific_parameters(self):
        """모드별 특정 파라미터 초기화"""
        if self.operation_mode == 'PROFILE_POSITION':
            # 라디안 단위를 펄스 단위로 변환
            velocity_pulse = self._convert_rad_to_pulse(self.profile_velocity)
            acceleration_pulse = self._convert_rad_to_pulse(self.profile_acceleration)
            deceleration_pulse = self._convert_rad_to_pulse(self.profile_deceleration)
            
            self.node.sdo['Profile velocity'].raw = velocity_pulse  #0x6081
            self.node.sdo['Profile acceleration'].raw = acceleration_pulse  #0x6083
            self.node.sdo['Profile deceleration'].raw = deceleration_pulse  #0x6084
            self.get_logger().info(f'Profile parameters set for Position mode:')
            self.get_logger().info(f'  Velocity: {self.profile_velocity} rad/s -> {velocity_pulse} pulse/s')
            self.get_logger().info(f'  Acceleration: {self.profile_acceleration} rad/s² -> {acceleration_pulse} pulse/s²')
            self.get_logger().info(f'  Deceleration: {self.profile_deceleration} rad/s² -> {deceleration_pulse} pulse/s²')
            
        elif self.operation_mode == 'PROFILE_TORQUE':
            self.node.sdo['Target torque'].raw = 0 #0x6071
            self.get_logger().info(f'Profile parameters set for Torque mode')

        else:
            self.get_logger().warn(f"지원하지 않는 동작 모드입니다: {self.operation_mode}")
            
    def reset(self):
        self.get_logger().info(f"Reset motor node: {self.node_id}")
        self.node.sdo[0x6040].raw = 0x27
        time.sleep(0.1)
        self.node.sdo[0x6040].raw = 0x26    
        time.sleep(0.1)
        self.node.sdo[0x6040].raw = 0x80  # 에러 클리어
        time.sleep(0.1)
        pass

    def log_start(self):
        """로그 시작"""
        self.logging = True
        self.start_time = time.time()
        
        # 현재 시간을 이용한 파일명 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"motor_log_{self.node_id}_{timestamp}.csv"
        
        # CSV 파일 생성 및 헤더 작성
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time(ms)', 'Position(rad)', 'Torque(Nm)', 'Velocity(rad/s)', 'Acceleration(rad/s^2)'])

    def log_stop(self):
        """로그 종료"""
        if hasattr(self, 'logging') and self.logging:
            self.logging = False
            self.log_file.close()

    def pdo_mapping(self):
        self.get_logger().info(f"PDO mapping for node: {self.node_id}")
        # Read PDO configuration from node
        self.node.tpdo.read()
        self.node.rpdo.read()

        # master <- motor
        # 읽기 : 상태 값, 토크 센서 값
        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable('Statusword')
        self.node.tpdo[1].add_variable('Position actual value') #aPosition actual value
        self.node.tpdo[1].cob_id = 0x180 + self.node_id
        self.node.tpdo[1].trans_type = 1
        #self.node.tpdo[1].trans_type = 254  # SYNC마다 전송
        #self.node.tpdo[1].event_timer = 10
        self.node.tpdo[1].enabled = True

        # 읽기 : 속도, 위치
        self.node.tpdo[2].clear()
        self.node.tpdo[2].add_variable('Torque sensor') #0x3B69, mN.m
        self.node.tpdo[2].add_variable('Velocity actual value') #0x606C, plus/s            
        self.node.tpdo[2].cob_id = 0x280 + self.node_id
        self.node.tpdo[2].trans_type = 1  # SYNC마다 전
        #self.node.tpdo[2].trans_type = 254  # SYNC마다 전송
        #self.node.tpdo[2].event_timer = 10
        self.node.tpdo[2].enabled = True

        # motor <- master
        # 쓰기 : 위치 목표값
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable('Controlword')
        self.node.rpdo[1].add_variable('Target Position')
        self.node.rpdo[1].cob_id = 0x200 + self.node_id
        self.node.rpdo[1].trans_type = 0  # 즉시 적용
        #self.node.rpdo[1].event_timer = 255   # 이벤트 타이머 비활성화
        self.node.rpdo[1].enabled = True

        # 쓰기 : 토크 목표값
        self.node.rpdo[2].clear() 
        self.node.rpdo[2].add_variable('Controlword')
        self.node.rpdo[2].add_variable('Target torque') #0x6071 
        self.node.rpdo[2].cob_id = 0x300 + self.node_id
        self.node.rpdo[2].trans_type = 0  # 즉시 적용
        #self.node.rpdo[1].event_timer = 255   # 이벤트 타이머 비활성화
        self.node.rpdo[2].enabled = True

        self.motor_rated_current = self.node.sdo['Motor rated current'].raw #0x6075 모터 정격 전류 mA
        self.get_logger().info(f'Motor rated current: {self.motor_rated_current}')
        
        # Save new configuration (node must be in pre-operational)
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.tpdo.save()
        self.node.rpdo.save()

        # Start remote node
        self.node.nmt.state = 'OPERATIONAL'
        pass


    def pdo_callback_register(self):
        self.network.subscribe(self.node.tpdo[1].cob_id, self.node.tpdo[1].on_message)
        self.node.tpdo[1].add_callback(self.tpdo1_callback)

        self.network.subscribe(self.node.tpdo[2].cob_id, self.node.tpdo[2].on_message)
        self.node.tpdo[2].add_callback(self.tpdo2_callback)

    def get_motor_status(self):
        """모터 상태 반환"""
        return self.motor_status
        
    def get_position(self):        
        # self.current_position = self.node.sdo['Position actual value'].raw
        # print(f"[MotorVendorZeroErr] Get position, node: {self.node_id}, position: {self.current_position}")
        return self.current_position
    
 

    def get_torque(self):        
        # CANopen 노드에서 토크 값을 읽어옴
        # self.current_torque_sensor = self.node.sdo['Torque sensor'].raw / 1000  # mN.m을 N.m으로 변환
        return self.current_torque_sensor
    
    def get_velocity(self):
        return self.velocity_actual_value
    
    def get_acceleration(self):
        return self.current_acceleration

    def get_status(self):
        """모터의 전체 상태 정보를 반환하는 함수"""
        status = {
            'node_id': self.node_id,
            'name': self.name,
            'position': self.current_position if hasattr(self, 'current_position') else 0,
            'torque': self.current_torque_sensor if hasattr(self, 'current_torque_sensor') else 0,
            'velocity': self.velocity_actual_value if hasattr(self, 'velocity_actual_value') else 0
        }
        
        # 모터 상태 정보 추가
        if hasattr(self, 'motor_status'):
            status.update({
                'error': self.motor_status.get('fault', False),
                'disabled': self.motor_status.get('switch_on_disabled', False),
                'active': self.motor_status.get('operation_enabled', False),
                'statusword': self.motor_status.get('statusword', 0),
                'warning': self.motor_status.get('warning', False),
                'ready': self.motor_status.get('ready_to_switch_on', False)
            })
        
        return status
    

    def get_error_code(self):
        """
        모터의 에러 코드를 확인하고 에러 메시지를 반환합니다.
        
        Returns:
            dict: 에러 코드, 16진수 코드, 메시지를 포함하는 사전
        """
        try:
            error_code = self.node.sdo[0x603F].raw
            
            # 에러 코드에 따른 메시지 매핑 (이미지에 표시된 값과 정확히 일치)
            error_messages = {
                0x0000: "준비 완료 (Ready to switch on)",
                0x2214: "전기 전류 과다 (Motor current is over current)",
                0x2250: "전동기 삼상 전류 한계치 (Sum of motor three phase current exceed the limit)",
                0x2241: "U 상 과전류 (The U phase over current)",
                0x2242: "V 상 과전류 (The V phase over current)",
                0x2243: "W 상 과전류 (The W phase over current)",
                0x3210: "모선 전압 과전압 (The bus voltage is overvoltage)",
                0x3220: "모선 전압 저전압 (The bus voltage is undervoltage)",
                0x4110: "쿨러 온도 이상 (The temperature of the power component is too high)",
                0x7121: "전동기 구동 차단 (Blocked motor rotation)",
                0x730D: "전지 경고 오류 (Battery warning error)",
                0x730F: "전지 전압 낮음 (Battery low voltage)",
                0x7311: "모터 엔드 위치 오류 (The position error of the sampled motor end exceeds the limit)",
                0x7314: "전원 오류 감지 (The power failure is detected)",
                0x7315: "부하 위치 오류 (The sampling load position error exceeds the upper limit)",
                0x7350: "엔코더 미지원 (Motor side encoder type is not supported)",
                0x7374: "멀티턴 위치 오류 (Multi-turn position error)",
                0x8130: "CAN 통신 오류 (CAN Heartbeat error)",
                0x8400: "속도 오류 (The velocity error exceeds the limit value)",
                0x8401: "전동기 속도 과다 (Motor velocity exceeds the limit value)",
                0x8500: "위치 오류 (The position error exceeds the limit value)",
                0xA000: "마스터 오프라인 (The master station goes offline)",
                0xF004: "EtherCAT 초기화 오류 (EtherCAT initialization error)",
                0xF005: "STO 기능 활성화 (The STO function is activated)",
                0xF006: "다중 회전 오류 (Multi-turn circle count error)"
            }
            
            # 이미지에 표시된 16진수 값 형식으로 변환 (예: 0x0000(0))
            formatted_hex_code = f"0x{error_code:04X}({error_code})"
            
            # 에러 메시지 조회 (없으면 "알 수 없는 에러" 반환)
            error_message = error_messages.get(error_code, f"알 수 없는 에러 코드: {formatted_hex_code}")

           
            # ROS2 메시지 데이터로 반환할 사전
            error_data = {
                "code": error_code,
                "hex_code": formatted_hex_code,
                "message": error_message,
            }
            
            # 에러 코드가 0이 아닌 경우만 출력 (실제 에러가 있는 경우)
            if error_code != 0:
                self.get_logger().error(f"모터 {self.node_id} 에러: {formatted_hex_code} - {error_message}")
            else:
                self.get_logger().info(f"모터 {self.node_id} 상태: {error_message}")
            
            return error_data
            
        except Exception as e:
            self.get_logger().error(f"모터 {self.node_id} 에러 코드 읽기 실패: {str(e)}")
            return {
                "code": -1,
                "hex_code": "0xFFFF(-1)",
                "message": f"에러 코드 읽기 실패: {str(e)}"
            }


    def set_velocity(self, value):
        """모터 속도 명령"""
        self.get_logger().info(f"Set velocity to {value}, node: {self.node_id}")

    def set_acceleration(self, value):
        """모터 가속도 명령"""
        self.get_logger().info(f"Set acceleration to {value}, node: {self.node_id}")

    def set_position(self, value):  # value in radians        
        """모터 위치 명령 (라디안 단위)"""
        
        # # 목표 위치를 라디안 단위로 저장
        self.target_position = value
        
        # 라디안 값을 펄스로 변환한 후 zero_offset(펄스)을 더함
        position_pulse = self._convert_rad_to_pulse(value) + self.zero_offset
        self.node.rpdo[1]['Controlword'].phys = 0x103f        
        self.node.rpdo[1]['Target Position'].phys = position_pulse
        self.node.rpdo[1].transmit()

        # self.get_logger().debug(f"zero_offset: {self.zero_offset} pulse, target_position_pulse: {position_pulse}")

        # self.node.rpdo[1]['Controlword'].phys = 0x3f
        # self.node.rpdo[1].transmit()        

    def set_torque(self, value):        
        self.target_torque = value * 1000 / self.motor_rated_current # mA

        self.get_logger().info(f"Set torque to {self.target_torque}, node: {self.node_id}")
        self.node.rpdo[1]['Controlword'].phys = 0x2f
        self.target_torque = value
        self.node.rpdo[1]['Target torque'].phys = self.target_torque
        self.node.rpdo[1].transmit()

        self.node.rpdo[1]['Controlword'].phys = 0x3f
        self.node.rpdo[1].transmit()    

    def set_switchOn(self):
        self.get_logger().info(f"Set switch on, node: {self.node_id}")
        
        time.sleep(0.001)
        self.node.rpdo[1]['Controlword'].phys = 0x2f
        self.node.rpdo[1]['Target Position'].phys = self.node.sdo['Position actual value'].raw
        self.node.rpdo[1].transmit()
        time.sleep(0.001)

        self.node.rpdo[1]['Controlword'].phys = 0x103f
        self.node.rpdo[1].transmit()        
        time.sleep(0.001)

        pass

    def set_dt(self, dt):
        """시간 간격 설정 (초 단위)"""
        self.dt = dt
        self.get_logger().debug(f"Set dt to {dt} seconds for node {self.node_id}")

    def tpdo1_callback(self, message):
        # Statusword 읽기 (message.data의 첫 2바이트)
        statusword = int.from_bytes(message.data[0:2], byteorder='little')
        
        # 이전 상태와 현재 상태 비교를 위해 이전 statusword 가져오기
        previous_statusword = self.motor_status.get('statusword', None) if hasattr(self, 'motor_status') else None
        
        # statusword가 변경되었을 때만 비트 해석 및 상태 감지 수행
        if previous_statusword is None or previous_statusword != statusword:
            # Statusword 비트 해석
            is_ready_to_switch_on = bool(statusword & (1 << 0))  # Bit 0
            is_switched_on = bool(statusword & (1 << 1))        # Bit 1
            is_operation_enabled = bool(statusword & (1 << 2))  # Bit 2
            is_fault = bool(statusword & (1 << 3))              # Bit 3
            is_voltage_enabled = bool(statusword & (1 << 4))    # Bit 4
            is_quick_stop = bool(statusword & (1 << 5))         # Bit 5
            is_switch_on_disabled = bool(statusword & (1 << 6)) # Bit 6
            is_warning = bool(statusword & (1 << 7))            # Bit 7
            
            # 모터 상태 저장
            self.motor_status = {
                'statusword': statusword,
                'ready_to_switch_on': is_ready_to_switch_on,
                'switched_on': is_switched_on,
                'operation_enabled': is_operation_enabled,
                'fault': is_fault,
                'voltage_enabled': is_voltage_enabled,
                'quick_stop': is_quick_stop,
                'switch_on_disabled': is_switch_on_disabled,
                'warning': is_warning
            }
            
            # 상태 변화 감지 및 로그 출력
            if is_fault:
                self.get_logger().error(f"[MotorVendorZeroErr] Motor {self.node_id} Fault detected! Statusword: 0x{statusword:04X}")
                # 에러 발생 시 자동 리셋 시도
                #self.try_auto_reset()
            
            if is_switch_on_disabled:
                self.get_logger().warn(f"[MotorVendorZeroErr] Motor {self.node_id} Switch ON disabled! Statusword: 0x{statusword:04X}")
                # 비활성화 상태인 경우 활성화 시도
                #self.set_switchOn()
            
            if not is_operation_enabled:
                self.get_logger().warn(f"[MotorVendorZeroErr] Motor {self.node_id} Operation not enabled! Statusword: 0x{statusword:04X}")
                # 운영 모드가 비활성화된 경우 활성화 시도
                #if not is_fault and not is_quick_stop:  # 에러나 빠른 정지 상태가 아닌 경우에만 시도
                    #self.set_switchOn()
            
            if is_warning:
                self.get_logger().warn(f"[MotorVendorZeroErr] Motor {self.node_id} Warning bit set! Statusword: 0x{statusword:04X}")
        
        # statusword 변경 여부와 관계없이 항상 수행되는 위치 값 읽기 처리
        position = int.from_bytes(message.data[2:5], byteorder='little', signed=True)
        self.current_position = (position - self.zero_offset) * self.plusToRad  # rad로 변환
        #self.get_logger().debug(f'TPDO1 Position actual value: {self.current_position}')

    
    def tpdo2_callback(self, message):
        current_torque = int.from_bytes(message.data[0:3], byteorder='little', signed=True)  

        self.current_torque_sensor = current_torque / 1000 # mNm -> Nm        
        #self.get_logger().debug(f'TPDO2 Torque sensor: {self.current_torque_sensor}')


        pulse_velocity = int.from_bytes(message.data[4:7], byteorder='little', signed=True)

        self.velocity_actual_value = pulse_velocity * self.plusToRad  # rad/s로 변환
        #self.get_logger().debug(f'TPDO2 Velocity actual value: {self.velocity_actual_value} rad/s')
        
        self.current_acceleration = (self.velocity_actual_value - self.current_velocity_old) / self.dt
        self.current_velocity_old = self.velocity_actual_value
        #self.get_logger().debug(f'TPDO2 Acceleration: {self.current_acceleration} rad/s^2')

        # 로깅이 활성화된 경우 데이터 저장
        if hasattr(self, 'logging') and self.logging:
            current_time = (time.time() - self.start_time) * 1000  # ms 단위로 변환
            self.csv_writer.writerow([
                f"{current_time:.1f}",
                f"{self.current_position:.6f}",
                f"{self.current_torque_sensor:.6f}",
                f"{self.velocity_actual_value:.6f}",
                f"{self.current_acceleration:.6f}"
            ])


# 다음 개선 사항:
# 1. MotorVendorElmo 클래스에도 동일한 Status word 감시 기능 구현 필요
# 2. 구현 방법:
#    - motor_status 변수 초기화를 init()에 추가
#    - tpdo1_callback에서 Status word 비트 해석 코드 추가
#    - try_auto_reset(), get_motor_status(), get_status() 메소드 추가
#    - 에러 및 비활성화 시 자동 복구 로직 구현
