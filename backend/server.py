#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS
from pymongo import MongoClient
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Timer

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')

        self.sub_odom = self.create_subscription(Odometry,"/odom",self.odom_callback,10)

        self.get_logger().info('WebServerNode iniciado')

        # Configurar Flask y MongoDB
        app = Flask(__name__)
        CORS(app, resources={r"/*": {"origins": "*"}})

        client = MongoClient('mongodb://localhost:27017/')
        db = client.RobotManager
        config_collection = db.configurations

        # *********************** SAVE CONFIGURATION ENDPOINTS ***********************
        @app.route('/config', methods=['GET'])
        def get_config():
            config = config_collection.find_one()
            if config:
                return jsonify({
                    'linear_vel': config.get('linear_vel', 1),
                    'angular_vel': config.get('angular_vel', 1)
                }), 200
            else:
                return jsonify({'message': 'No se encontró configuración'}), 404

        @app.route('/config', methods=['POST'])
        def save_config():
            data = request.json
            if not data or 'linear_vel' not in data or 'angular_vel' not in data:
                return jsonify({'message': 'Datos inválidos'}), 400

            config_collection.update_one({}, {'$set': {
                'linear_vel': data['linear_vel'],
                'angular_vel': data['angular_vel']
            }}, upsert=True)

            return jsonify({'message': 'Configuración guardada correctamente'}), 200
        

        # *********************** SHOW STATUS ENDPOINTS ***********************
        @app.route('/robot_status', methods=['GET'])
        def robot_status():
            status = 'Connected' if self.odom_active else 'Disconnected'
            return jsonify({'status': status}), 200

        '''
        @app.route('/cmd_vel', methods=['POST'])
        def cmd_vel():
            data = request.json
            if not data or 'linear_vel' not in data or 'angular_vel' not in data:
                return jsonify({'message': 'Datos inválidos'}), 400

            linear_vel = data['linear_vel']
            angular_vel = data['angular_vel']

            twist_message = Twist()
            twist_message.linear.x = data['linear_vel']
            twist_message.angular.z = data['angular_vel']
            self.cmd_vel_publisher.publish(twist_message)

            return jsonify({'message': 'Velocidad enviada correctamente'}), 200
        '''

        # Intervalo máximo permitido entre mensajes (en segundos)
        self.timeout_interval = 2.0  # Ajusta según tus necesidades

        # Estado de la conexión
        self.odom_active = False

        # Temporizador
        self.timer = None
        self.start_timer()

        # Ejecutar el servidor Flask en un hilo separado
        thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False))
        thread.daemon = True
        thread.start()

    def start_timer(self):
        """Inicia un temporizador para detectar pérdida de datos"""
        if self.timer:
            self.timer.cancel()
        self.timer = Timer(self.timeout_interval, self.check_odom_status)
        self.timer.start()

    def check_odom_status(self):
        """Función llamada cuando no se reciben datos en el tiempo esperado"""
        self.odom_active = False
        #self.get_logger().warn('No se reciben datos del tópico /odom.')

    def odom_callback(self, msg):
        """Callback ejecutado al recibir datos del tópico /odom"""
        self.odom_active = True
        #self.get_logger().info('Datos recibidos del tópico /odom.')
        # Reinicia el temporizador
        self.start_timer()


def main(args=None):
    rclpy.init(args=args)
    print("Servidor iniciado")
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
