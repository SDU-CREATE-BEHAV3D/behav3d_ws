# behav3d_py/modbus_test.py
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class ModbusTest(Node):
    def __init__(self):
        super().__init__('modbus_test')

        self.declare_parameter('ip', '192.168.1.10')
        self.declare_parameter('port', 502)
        self.declare_parameter('unit_id', 1)
        self.declare_parameter('coil_index', 10)
        self.declare_parameter('poll_ms', 200)
        self.declare_parameter('extrude_on', False)  # ← default para evitar warning
        self.declare_parameter('speed_reg', 1)   # índice del holding register de velocidad
        self.declare_parameter('speed', 100)    # valor opcional a escribir al iniciar (int)

        ip   = self.get_parameter('ip').value
        port = self.get_parameter('port').value
        self.unit_id    = self.get_parameter('unit_id').value
        self.coil_index = self.get_parameter('coil_index').value
        self.speed_reg = self.get_parameter('speed_reg').value

        self.client = ModbusTcpClient(ip, port=port)
        if not self.client.connect():
            self.get_logger().error(f'No se pudo conectar a {ip}:{port}')
            raise SystemExit(1)
        self.get_logger().info(f'Conectado a Modbus TCP {ip}:{port} (unit {self.unit_id})')

        self.add_on_set_parameters_callback(self._on_params)

        self._last_state = None
        period = self.get_parameter('poll_ms').value / 1000.0
        self.timer = self.create_timer(period, self._poll)

        # Escritura inicial según parámetro
        init_param = self.get_parameter('extrude_on')
        if init_param.type_ == Parameter.Type.BOOL:
            self._write_coil(bool(init_param.value))

        speed_param = self.get_parameter('speed')
        if speed_param.type_ == Parameter.Type.INTEGER:
            self._write_speed(int(speed_param.value))

    def _on_params(self, params):
        for p in params:
            if p.name == 'extrude_on' and p.type_ == Parameter.Type.BOOL:
                self._write_coil(bool(p.value))
            elif p.name == 'speed' and p.type_ == Parameter.Type.INTEGER:
                self._write_speed(int(p.value))
        return SetParametersResult(successful=True)


    def _poll(self):
        # coil
        rr = self.client.read_coils(self.coil_index, count=1)
        if not rr.isError():
            state = bool(rr.bits[0])
            if state != self._last_state:
                self.get_logger().info(f'coil[{self.coil_index}] => {state}')
                self._last_state = state
        else:
            self.get_logger().warning(f'Error leyendo coil {self.coil_index}: {rr}')

        # holding register de velocidad
        rs = self.client.read_holding_registers(self.speed_reg, count=1)
        if not rs.isError():
            speed_val = rs.registers[0]
            # (opcional) loguea cada cierto tiempo o compara con último valor si quieres
            # self.get_logger().info(f'reg[{self.speed_reg}] => {speed_val}')
        else:
            self.get_logger().warning(f'Error leyendo reg {self.speed_reg}: {rs}')

    def _write_coil(self, value: bool):
        rr = self.client.write_coil(self.coil_index, value)  # usa slave en vez de unit
        if rr.isError():
            self.get_logger().error(f'Error escribiendo coil {self.coil_index}: {rr}')
        else:
            self.get_logger().info(f'coil[{self.coil_index}] <- {value}')
    def _write_speed(self, value: int):
        # Si tu firmware espera escala (p.ej. x10), aplica aquí
        wr = self.client.write_register(self.speed_reg, value)  # en pymodbus 3.x vale así
        if wr.isError():
            self.get_logger().error(f'Error escribiendo reg {self.speed_reg}: {wr}')
        else:
            self.get_logger().info(f'reg[{self.speed_reg}] <- {value}')

def main():
    rclpy.init()
    node = ModbusTest()
    rclpy.spin(node)
    node.client.close()
    node.destroy_node()
    rclpy.shutdown()
