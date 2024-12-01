#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS
from pymongo import MongoClient
import threading

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info('WebServerNode iniciado')

        # Configurar Flask y MongoDB
        app = Flask(__name__)
        CORS(app, resources={r"/*": {"origins": "*"}})

        client = MongoClient('mongodb://localhost:27017/')
        db = client.RobotManager
        config_collection = db.configurations

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

        # Ejecutar el servidor Flask en un hilo separado
        thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False))
        thread.daemon = True
        thread.start()

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
