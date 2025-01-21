import os
import yaml
import psycopg2
from psycopg2 import pool
import logging
import time

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def load_db_config():
    """데이터베이스 설정 파일 로드"""
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_path = os.path.join(script_dir, '..', 'resource', 'db_config.yaml')
        config_path = os.getenv('DB_CONFIG_PATH', default_path)

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")

        with open(config_path, 'r') as file:
            return yaml.safe_load(file)['database']
    except Exception as e:
        logging.error(f"설정 파일 로드 중 오류 발생: {e}")
        raise

def check_table_exists(conn, table_name):
    """테이블 존재 여부 및 데이터 확인"""
    try:
        with conn.cursor() as cursor:
            # 테이블 존재 여부 확인
            cursor.execute("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables 
                    WHERE table_schema = 'public' AND table_name = %s
                )
            """, (table_name,))
            exists = cursor.fetchone()[0]
            
            if exists:
                # 데이터 존재 여부 확인
                cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
                count = cursor.fetchone()[0]
                return count > 0
            return False
    except Exception as e:
        logging.error(f"테이블 {table_name} 확인 중 오류 발생: {e}")
        return False

def wait_for_table_initialization(conn, table_name, max_retries=30, delay=1):
    """테이블 초기화 완료 대기"""
    retries = 0
    while retries < max_retries:
        if check_table_exists(conn, table_name):
            logging.info(f"테이블 {table_name} 초기화 완료")
            return True
        logging.info(f"테이블 {table_name} 초기화 대기 중... ({retries}/{max_retries})")
        time.sleep(delay)
        retries += 1
    logging.error(f"테이블 {table_name} 초기화 타임아웃")
    return False

# 전역 연결 풀 변수
connection_pool = None

def init_connection_pool():
    """연결 풀 초기화"""
    global connection_pool
    if connection_pool is None:
        try:
            config = load_db_config()
            connection_pool = pool.SimpleConnectionPool(1, 20, **config)
            logging.info("연결 풀이 성공적으로 초기화되었습니다.")
        except Exception as e:
            logging.error(f"연결 풀 초기화 중 오류 발생: {e}")
            raise

def get_connection():
    """연결 풀에서 연결 가져오기"""
    global connection_pool
    if connection_pool is None:
        init_connection_pool()
    try:
        conn = connection_pool.getconn()
        logging.info("Connection obtained from the pool.")
        return conn
    except Exception as e:
        logging.error(f"Error getting connection from pool: {e}")
        return None

def return_connection(conn):
    """연결 반환"""
    global connection_pool
    try:
        if conn and connection_pool:
            connection_pool.putconn(conn)
            logging.info("Connection returned to the pool.")
    except Exception as e:
        logging.error(f"Error returning connection to pool: {e}")

def close_all_connections():
    """모든 연결 닫기"""
    global connection_pool
    try:
        if connection_pool:
            connection_pool.closeall()
            connection_pool = None
            logging.info("All connections closed.")
    except Exception as e:
        logging.error(f"Error closing all connections: {e}")

# 초기화 검증을 위한 테스트 코드
if __name__ == "__main__":
    try:
        conn = get_connection()
        if conn:
            # beans 테이블 초기화 대기
            if wait_for_table_initialization(conn, "beans"):
                with conn.cursor() as cursor:
                    cursor.execute("SELECT * FROM beans LIMIT 1")
                    result = cursor.fetchone()
                    if result:
                        logging.info("beans 테이블 데이터 확인 성공")
                    else:
                        logging.warning("beans 테이블에 데이터가 없습니다")
            else:
                logging.error("beans 테이블 초기화 실패")
            return_connection(conn)
        close_all_connections()
    except Exception as e:
        logging.error(f"데이터베이스 연결 테스트 실패: {e}")
