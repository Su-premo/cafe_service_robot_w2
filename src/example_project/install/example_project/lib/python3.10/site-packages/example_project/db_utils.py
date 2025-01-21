import os
import yaml
import psycopg2
from psycopg2 import pool
import logging
# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
def load_db_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'resource', 'db_config.yaml')
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)['database']
config = load_db_config()
# 최소 1개, 최대 20개의 연결을 허용하는 연결 풀
connection_pool = pool.SimpleConnectionPool(1, 20, **config)
def get_connection():
    try:
        conn = connection_pool.getconn()
        logging.info("Connection obtained from the pool.")
        return conn
    except Exception as e:
        logging.error(f"Error getting connection from pool: {e}")
        return None
def return_connection(conn):
    try:
        connection_pool.putconn(conn)
        logging.info("Connection returned to the pool.")
    except Exception as e:
        logging.error(f"Error returning connection to pool: {e}")
def close_all_connections():
    try:
        connection_pool.closeall()
        logging.info("All connections closed.")
    except Exception as e:
        logging.error(f"Error closing all connections: {e}")
