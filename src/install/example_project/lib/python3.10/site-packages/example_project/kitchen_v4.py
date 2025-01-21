import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import signal
from datetime import datetime
import sqlite3
import psycopg2
import json
import ast
from db_utils import get_connection, return_connection 
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QMessageBox
import matplotlib

class KitchenOrderNode(Node):
    def __init__(self):
        super().__init__('kitchen_order_node')
        self.subscription = self.create_subscription(
            String,
            'table_orders',
            self.order_callback,
            10)
        self.publisher = self.create_publisher(String, 'kitchen_messages', 10)
        self.emit_signal = None
        self.tables_data = self.get_cafe_tables_info()
        self.get_logger().info(f'Cafe_tables data: {self.tables_data}')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.order_data_dict = {}

    def order_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        if self.emit_signal is not None:
            try:
                table_info, order_data = msg.data.split(':', 1)
                table_number = table_info.replace('테이블 ', '').strip()
                order_dict = ast.literal_eval(order_data.strip())
                order_dict['table_number'] = int(table_number)
                self.order_data_dict[int(table_number)] = order_dict
                self.emit_signal(json.dumps(order_dict))
                self.get_logger().info(f"주문 처리됨: 테이블 {table_number}")
            except Exception as e:
                self.get_logger().error(f"주문 처리 중 오류 발생: {e}")
                self.get_logger().error(f"받은 데이터: {msg.data}")

    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func    

    def get_cafe_tables_info(self):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT * FROM cafe_tables")
                beans_info = cur.fetchall()
            return beans_info
        except psycopg2.Error as e:
            self.get_logger().error(f"Error fetching cafe_tables info: {e}")
            return []
        finally:
            return_connection(conn)

    def insert_order(self, order_data):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                cur.execute("""
                    INSERT INTO orders (
                        table_ordered, 
                        order_time, 
                        total_price, 
                        is_gifting, 
                        table_served
                    ) VALUES (
                        %s, 
                        NOW(), 
                        %s, 
                        FALSE, 
                        %s
                    ) RETURNING order_id
                """, (
                    order_data['table_number'], 
                    order_data['total_price'], 
                    order_data['table_number']
                ))
                order_id = cur.fetchone()[0]
            conn.commit()
            return order_id
        except psycopg2.Error as e:
            self.get_logger().error(f"Error inserting order: {e}")
        finally:
            return_connection(conn)

    def insert_order_details(self, order_id, order_items):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                for item in order_items:
                    bean_name, quantity = item.split(' X ')
                    cur.execute("SELECT bean_id, price FROM beans WHERE bean_name = %s", (bean_name,))
                    bean_info = cur.fetchone()
                    if bean_info:
                        bean_id, bean_price = bean_info
                        subtotal = bean_price * int(quantity)
                        cur.execute("""
                            INSERT INTO order_details (
                                order_id, 
                                bean_id, 
                                quantity, 
                                subtotal
                            ) VALUES (%s, %s, %s, %s)
                        """, (order_id, bean_id, int(quantity), subtotal))
            conn.commit()
        except psycopg2.Error as e:
            self.get_logger().error(f"Error inserting order details: {e}")
        finally:
            return_connection(conn)

    def send_robot_to_table(self, table_number):
        table_coordinates = {
            1: (1.0, 4.0, 0.0),
            2: (1.0, 7.0, 0.0),
            3: (4.0, 4.0, 0.0),
            4: (4.0, 7.0, 0.0)
        }

        if table_number not in table_coordinates:
            self.get_logger().error(f"Invalid table number: {table_number}")
            return
        
        x, y, z = table_coordinates[table_number]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
    
        self.nav_client.wait_for_server()
        self.get_logger().info(f"Sending robot to table {table_number}")
        self.nav_client.send_goal_async(goal_msg)

    def confirm_order(self, table):
        try:
            self.get_logger().info(f"Current order_data_dict: {self.order_data_dict}")
            
            order_data = self.order_data_dict.get(table)
            
            self.get_logger().info(f"Order data for table {table}: {order_data}")
            
            if not order_data:
                return "주문이 없습니다."

            total_price = order_data['total_price']
            people_count = order_data['people_count']

            order_id = self.insert_order(order_data)

            if order_id:
                self.insert_order_details(order_id, order_data['orders'])
                msg = String()
                msg.data = f"confirm_order,{table},{total_price},{people_count}"
                self.publisher.publish(msg)
                
                return f"테이블 {table}의 주문이 확인되었습니다.\n총 가격: {total_price}원\n인원 수: {people_count}명"

        except Exception as e:
            self.get_logger().error(f"Error in confirm_order: {e}")
            return f"주문 확인 실패: {str(e)}"

    def complete_payment(self, table):
        msg = String()
        msg.data = f"complete_payment,{table}"
        self.publisher.publish(msg)
        if table in self.order_data_dict:
            del self.order_data_dict[table]

    def get_daily_sales(self):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT COALESCE(SUM(total_price), 0) AS daily_sales
                    FROM orders
                    WHERE DATE(order_time) = CURRENT_DATE
                """)
                daily_sales = cur.fetchone()[0]
            return daily_sales
        except psycopg2.Error as e:
            self.get_logger().error(f"Error fetching daily sales: {e}")
            return 0
        finally:
            return_connection(conn)

    def get_menu_sales(self):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT b.bean_name, SUM(od.quantity * b.price) AS total_menu_sales
                    FROM order_details od
                    JOIN beans b ON od.bean_id = b.bean_id
                    GROUP BY b.bean_name
                    ORDER BY total_menu_sales DESC;
                """)
                menu_sales = cur.fetchall()
            return menu_sales
        except psycopg2.Error as e:
            self.get_logger().error(f"Error fetching menu sales: {e}")
            return []
        finally:
            return_connection(conn)

class MainWindow(QMainWindow):
    order_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("주문 목록 화면")
        self.setGeometry(100, 100, 1200, 800)
        
        self.connect_database()
        self.setup_ui()
        self.order_signal.connect(self.update_order)

    def log_error(self, message):
        print(f"ERROR: {message}")

    def log_info(self, message):
        print(f"INFO: {message}")

    def setup_ui(self):
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        self.list_widget = QListWidget()
        self.list_widget.addItems(["테이블 당 주문 내역", "로봇 제어"])
        self.list_widget.setStyleSheet("QListWidget::item { padding: 20px; }")
        self.list_widget.currentRowChanged.connect(self.display_page)
        
        self.stack_widget = QStackedWidget()
        
        order_page = QWidget()
        order_layout = QVBoxLayout()

        tables_layout = QHBoxLayout()
        self.text_browsers = []
        for i in range(1, 5):
            table_widget = QWidget()
            table_layout = QVBoxLayout()
            table_layout.addWidget(QLabel(f"테이블 {i}"))
            
            text_browser = QTextBrowser()
            self.text_browsers.append(text_browser)
            table_layout.addWidget(text_browser)
            
            btn_layout = QVBoxLayout()
            confirm_btn = QPushButton("주문확인")
            confirm_btn.clicked.connect(lambda _, table=i: self.confirm_order(table))
            payment_btn = QPushButton("결제 완료")
            payment_btn.clicked.connect(lambda _, table=i: self.complete_payment(table))
            btn_layout.addWidget(confirm_btn)
            btn_layout.addWidget(payment_btn)
            table_layout.addLayout(btn_layout)
            
            table_widget.setLayout(table_layout)
            tables_layout.addWidget(table_widget)
            
        order_layout.addLayout(tables_layout)
        
        bottom_layout = QHBoxLayout()
        daily_sales_btn = QPushButton("일일매출")
        menu_sales_btn = QPushButton("메뉴별매출")
        daily_sales_btn.clicked.connect(self.show_daily_sales)
        menu_sales_btn.clicked.connect(self.show_menu_sales)
        bottom_layout.addWidget(daily_sales_btn)
        bottom_layout.addWidget(menu_sales_btn)
        order_layout.addLayout(bottom_layout)
        
        order_page.setLayout(order_layout)
        
        robot_page = QWidget()
        robot_layout = QVBoxLayout()
        
        table_button_layout = QHBoxLayout()
        self.table_buttons = []
        for i in range(1, 5):
            button = QPushButton(f"테이블 {i}")
            button.clicked.connect(lambda _, table=i: self.send_robot_to_table(table))
            self.table_buttons.append(button)
            table_button_layout.addWidget(button)

        description_label = QLabel("테이블 버튼\n버튼 클릭시 서빙 로봇을 해당하는 테이블로 이동하는 명령을 보냅니다.")
        description_label.setStyleSheet("QLabel { background-color: lightgray; padding: 10px; }")

        robot_layout.addLayout(table_button_layout)
        robot_layout.addWidget(description_label)
        robot_page.setLayout(robot_layout)
        
        self.stack_widget.addWidget(order_page)
        self.stack_widget.addWidget(robot_page)
        
        main_layout.addWidget(self.list_widget, 1)
        main_layout.addWidget(self.stack_widget, 3)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

    def display_page(self, index):
        self.stack_widget.setCurrentIndex(index)
        if index == 1:  # 로봇 제어 페이지의 인덱스가 1이라고 가정
            self.setWindowTitle("로봇 서비스")
        else:
            self.setWindowTitle("주문 목록 화면")

    def send_robot_to_table(self, table):
        self.node.send_robot_to_table(table)
        QMessageBox.information(self, "로봇 이동", f"로봇이 테이블 {table}로 이동합니다.")

    def complete_payment(self, table):
        if 1 <= table <= len(self.text_browsers):
            self.text_browsers[table-1].clear()
            self.node.complete_payment(table)
            QMessageBox.information(self, "결제 완료", f"테이블 {table}의 결제가 완료되었습니다.")

    def show_daily_sales(self):
        daily_sales = self.node.get_daily_sales()
        QMessageBox.information(self, "일일 매출", f"오늘의 총 매출: {daily_sales}원")

    def show_menu_sales(self):
        try:
            menu_sales = self.node.get_menu_sales()
            self.node.get_logger().info(f"Retrieved menu sales data: {menu_sales}")
            
            if not menu_sales:
                raise ValueError("메뉴 매출 데이터가 없습니다.")

            bean_names, sales = zip(*menu_sales)
            
            fig, ax = plt.subplots(figsize=(10, 6))
            ax.bar(bean_names, sales)
            ax.set_xlabel('메뉴')
            ax.set_ylabel('매출 (원)')
            ax.set_title('메뉴별 매출')
            plt.xticks(rotation=45, ha='right')
            
            canvas = FigureCanvas(fig)
            
            dialog = QWidget()
            layout = QVBoxLayout()
            layout.addWidget(canvas)
            dialog.setLayout(layout)
            dialog.setWindowTitle('메뉴별 매출 그래프')
            dialog.show()
            
            self.node.get_logger().info("Menu sales graph displayed successfully")

        except ValueError as ve:
            self.node.get_logger().error(f"Data error: {ve}")
            QMessageBox.warning(self, "데이터 오류", str(ve))
        except matplotlib.pyplot.error as mpl_error:
            self.node.get_logger().error(f"Matplotlib error: {mpl_error}")
            QMessageBox.critical(self, "그래프 생성 오류", f"그래프를 생성하는 중 오류가 발생했습니다: {mpl_error}")
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error in show_menu_sales: {e}")
            QMessageBox.critical(self, "예상치 못한 오류", f"메뉴별 매출 표시 중 오류 발생: {e}")



    def update_order(self, order_data):
        try:
            if isinstance(order_data, str):
                order_data = ast.literal_eval(order_data)
            
            table_num = order_data['table_number']
            if 1 <= table_num <= len(self.text_browsers):
                current_text = self.text_browsers[table_num-1].toPlainText()
                new_order_info = f"인원: {order_data['people_count']}명\n주문:\n"
                for item in order_data['orders']:
                    new_order_info += f"{item}\n"
                
                updated_text = current_text + new_order_info
                self.text_browsers[table_num-1].setText(updated_text)
                QMessageBox.information(self, "새 주문", f"테이블 {table_num}에서 새로운 주문이 들어왔습니다.")
                
                self.log_info(f"주문 데이터 추가: 테이블 {table_num}, 데이터: {order_data}")
        except Exception as e:
            self.log_error(f"주문 처리 중 오류 발생: {e}")


    def confirm_order(self, table):
        result = self.node.confirm_order(table)
        QMessageBox.information(self, "주문 확인", result)

    def connect_database(self):
        self.conn = sqlite3.connect('database.db')
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            table_number INTEGER NOT NULL,
            coffee_bean TEXT NOT NULL,
            quantity INTEGER NOT NULL,
            time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        )
        ''')
        self.conn.commit()

    def insert_data(self, table_num, order_info):
        try:
            coffee_bean, quantity = order_info.split('x')
            coffee_bean = coffee_bean.strip()
            quantity = int(quantity.strip())
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            self.cursor.execute(
                "INSERT INTO orders (table_number, coffee_bean, quantity, time) VALUES (?, ?, ?, ?)",
                (table_num, coffee_bean, quantity, current_time)
            )
            self.conn.commit()
        except Exception as e:
            self.log_error(f"Error inserting data: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = KitchenOrderNode()
    
    app = QApplication(sys.argv)
    main_window = MainWindow(node)
    main_window.show()
    
    def run_ros_spin():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=run_ros_spin, daemon=True)
    ros_thread.start()
    
    node.set_emit_signal(main_window.order_signal.emit)
    
    exit_code = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()

