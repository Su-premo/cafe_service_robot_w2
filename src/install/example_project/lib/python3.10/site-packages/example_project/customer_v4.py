import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QMessageBox,
                           QHBoxLayout, QLabel, QPushButton, QTextEdit, QStackedWidget, QSpinBox, QLineEdit)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import signal
import os
# ---------- DB ---------------
import psycopg2
from db_utils import get_connection, return_connection 

class TableOrderNode(Node):
    def __init__(self, table_number):
        super().__init__(f'table_order_node_{table_number}')
        self.publisher = self.create_publisher(String, 'table_orders', 10)

        self.table_number = table_number
        # beans_data를 메서드 호출로 초기화
        self.beans_data = self.get_beans_info()
        self.subscription = self.create_subscription(String,
        'kitchen_messages', self.kitchen_message_callback, 10)


    def send_order(self, order_data):
        msg = String()
        msg.data = f"테이블 {self.table_number}: {order_data}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Published order: {msg.data}')

    # def send_order(self, order_data):
    #     # order_data는 딕셔너리 형태로 전달
    #     msg = String()
    #     # 딕셔너리를 문자열로 변환하여 전송
    #     msg.data = f"{self.table_number}|{order_data['total_price']}|{order_data['orders']}"
    #     self.publisher.publish(msg)
    #     self.get_logger().info(f'Published order: {msg.data}')

    def kitchen_message_callback(self, msg):
        if self.emit_signal is not None:
            self.emit_signal(msg.data)



    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func

    def get_beans_info(self):
        conn = get_connection()
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT bean_name, price FROM beans")
                beans_info = cur.fetchall()
                
                # 로깅 추가
                self.get_logger().info(f"Beans info: {beans_info}")
                
                return beans_info
        except psycopg2.Error as e:
            self.get_logger().error(f"Error fetching beans info: {e}")
            return []
        finally:
            return_connection(conn)


class MainWindow(QMainWindow):
    kitchen_message_signal = pyqtSignal(str)

    def __init__(self, node, table_number):
        super().__init__()
        self.node = node
        self.table_number = table_number
        self.setWindowTitle(f"테이블 {table_number} 주문 시스템")
        self.setGeometry(100, 100, 1200, 800)
        
        self.setup_ui()
        self.kitchen_message_signal.connect(self.handle_kitchen_message)
        node.set_emit_signal(self.kitchen_message_signal.emit)
        self.order_counts = {}

    def setup_ui(self):
            main_widget = QWidget()
            self.setCentralWidget(main_widget)
            main_layout = QVBoxLayout()
            
            header_layout = QHBoxLayout()
            self.header_buttons = []
            headers = ["인원수 입력", "에스프레소 주문"]  # "선물 보내기" 제거
            
            for text in headers:
                button = QPushButton(text)
                button.setStyleSheet("""
                    QPushButton {
                        background-color: gray;
                        color: white;
                        padding: 10px;
                        border: none;
                        font-size: 14px;
                    }
                """)
                self.header_buttons.append(button)
                header_layout.addWidget(button)
            
            main_layout.addLayout(header_layout)
            
            self.stack_widget = QStackedWidget()
            
            self.stack_widget.addWidget(self.create_people_page())
            self.stack_widget.addWidget(self.create_order_page())
            # self.stack_widget.addWidget(self.create_gift_page())  # 이 줄 제거
            
            main_layout.addWidget(self.stack_widget)
            main_widget.setLayout(main_layout)

            for i, button in enumerate(self.header_buttons):
                button.clicked.connect(lambda checked, index=i: self.change_page(index))

    def create_people_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        
        center_layout = QVBoxLayout()
        center_layout.setAlignment(Qt.AlignCenter)
        
        label = QLabel("인원수를 입력해주세요")
        label.setStyleSheet("font-size: 20px;")
        center_layout.addWidget(label)
        
        self.people_spinbox = QSpinBox()
        self.people_spinbox.setMinimum(1)
        self.people_spinbox.setMaximum(10)
        self.people_spinbox.setValue(1)
        self.people_spinbox.setFixedWidth(200)
        self.people_spinbox.setStyleSheet("""
            QSpinBox {
                font-size: 16px;
                padding: 5px;
            }
        """)
        center_layout.addWidget(self.people_spinbox)
        
        submit_btn = QPushButton("입력하기")
        submit_btn.setFixedWidth(200)
        submit_btn.setStyleSheet("""
            QPushButton {
                background-color: gray;
                color: white;
                padding: 10px;
                font-size: 16px;
            }
        """)
        submit_btn.clicked.connect(self.confirm_people_count)
        center_layout.addWidget(submit_btn)
        
        layout.addLayout(center_layout)
        page.setLayout(layout)
        return page

    def confirm_people_count(self):
        people_count = self.people_spinbox.value()
        QMessageBox.information(self, "인원수 확인", f"인원수 {people_count}명이 확인되었습니다.")
        self.change_page(1)  # 주문 페이지로 이동


    def create_order_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        coffee_layout = QHBoxLayout()
        # 하드코딩된 이미지 파일 이름과 원두 이름을 매핑하는 리스트
        coffee_types = {
            "과테말라 안티구아": "guatemala.jpg",
            "콜롬비아 수프리모": "colombia.jpg",
            "케냐 AA": "kenya.jpg",
            "코스타리카 따라주": "costarica.jpg"
        }
        # ROS2 패키지 경로 가져오기
        package_path = get_package_share_directory('example_project')
        images_path = os.path.join(package_path, 'images')
        print(f"Package path: {package_path}")
        print(f"Images path: {images_path}")
        # DB에서 원두 정보를 가져옵니다.
        beans_data = self.node.beans_data  # DB에서 가져온 원두 정보
        for bean in beans_data:
            coffee_name = bean[0]  # DB에서 가져온 원두 이름
            price = bean[1]  # DB에서 가져온 가격
            image_file = coffee_types.get(coffee_name)  # 이미지 파일 이름 가져오기
            coffee_widget = QWidget()
            coffee_v_layout = QVBoxLayout()
            image_label = QLabel()
            if image_file:  # 이미지 파일이 존재하는 경우
                image_path = os.path.join(images_path, image_file)
                print(f"Looking for image at: {image_path}")
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    scaled_pixmap = pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    image_label.setPixmap(scaled_pixmap)
                    image_label.setAlignment(Qt.AlignCenter)
                else:
                    image_label.setText(f"이미지를 찾을 수 없습니다\n{image_path}")
            else:
                image_label.setText("이미지 파일이 없습니다.")
            coffee_v_layout.addWidget(image_label)
            # 커피 이름과 가격 표시
            name_label1 = QLabel(f"{coffee_name}")
            name_label2 = QLabel(f"{price}원")  # 가격 추가
            name_label1.setAlignment(Qt.AlignCenter)
            name_label2.setAlignment(Qt.AlignCenter)
            coffee_v_layout.addWidget(name_label1)
            coffee_v_layout.addWidget(name_label2)
            coffee_btn = QPushButton("선택")
            coffee_btn.clicked.connect(lambda checked, c=coffee_name: self.add_to_order(c))
            coffee_btn.setStyleSheet("""
                QPushButton {
                    background-color: gray;
                    color: white;
                    padding: 10px;
                }
            """)
            coffee_v_layout.addWidget(coffee_btn)
            coffee_widget.setLayout(coffee_v_layout)
            coffee_layout.addWidget(coffee_widget)
        layout.addLayout(coffee_layout)
        # 주문 내역 및 버튼
        order_layout = QVBoxLayout()
        self.order_text = QTextEdit()
        self.order_text.setReadOnly(True)
        order_layout.addWidget(self.order_text)
        self.total_label = QLabel("합계: 0원")
        order_layout.addWidget(self.total_label)
        button_layout = QHBoxLayout()
        reset_btn = QPushButton("초기화")
        reset_btn.clicked.connect(self.reset_order)
        order_btn = QPushButton("주문하기")
        order_btn.clicked.connect(self.place_order)
        button_layout.addWidget(reset_btn)
        button_layout.addWidget(order_btn)
        order_layout.addLayout(button_layout)
        layout.addLayout(order_layout)
        page.setLayout(layout)
        return page

    def change_page(self, index):
        self.stack_widget.setCurrentIndex(index)
        
        for i, button in enumerate(self.header_buttons):
            if i == index:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: darkgray;
                        color: white;
                        padding: 10px;
                        border: none;
                        font-size: 14px;
                    }
                """)
            else:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: gray;
                        color: white;
                        padding: 10px;
                        border: none;
                        font-size: 14px;
                    }
                """)

    def add_to_order(self, coffee):
        # 해당 커피의 주문 수량 증가
        if coffee in self.order_counts:
            self.order_counts[coffee] += 1
        else:
            self.order_counts[coffee] = 1
            
        # 주문 텍스트 업데이트
        order_text = ""
        for coffee_name, count in self.order_counts.items():
            if count > 1:
                order_text += f"{coffee_name} X {count}\n"
            else:
                order_text += f"{coffee_name}\n"
                
        self.order_text.setPlainText(order_text)
        self.update_total()

    def reset_order(self):
        self.order_text.clear()
        self.total_label.setText("합계: 0원")
        self.order_counts = {}  # 주문 수량 초기화

    def update_total(self):
        total = 0
        for coffee_name, count in self.order_counts.items():
            # DB에서 해당 커피의 가격 찾기
            for bean in self.node.beans_data:
                if bean[0] == coffee_name:
                    price = bean[1]
                    total += price * count
                    break
        self.total_label.setText(f"합계: {total}원")

    def reset_order(self):
        self.order_text.clear()
        self.total_label.setText("합계: 0원")

    def place_order(self):
        try:
            # 주문 정보 구성
            order_lines = self.order_text.toPlainText().strip().split('\n')
            total_price = int(self.total_label.text().replace("합계: ", "").replace("원", ""))
            
            order_data = {
                'total_price': total_price,
                'orders': order_lines,
                'people_count': self.people_spinbox.value()
            }
            
            # 노드를 통해 주문 전송
            self.node.send_order(order_data)
            
            QMessageBox.information(self, "주문 완료", "주문이 전송되었습니다.")
            self.reset_order()
            
        except Exception as e:
            QMessageBox.warning(self, "주문 실패", f"주문 처리 중 오류가 발생했습니다: {str(e)}")

    def handle_kitchen_message(self, message):
        self.node.get_logger().info(f"Received kitchen message: {message}")
        try:
            action, table, *_ = message.split(',')
            if action == "confirm_order" and int(table) == self.table_number:
                QMessageBox.information(self, "주문 접수", "주문이 접수되었습니다.")
        except ValueError:
            self.node.get_logger().error(f"Invalid message format: {message}")


    def reset_order(self):
        self.order_text.clear()
        self.total_label.setText("합계: 0원")
        self.order_counts = {}



def main(args=None):
    rclpy.init(args=args)
    table_number = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    node = TableOrderNode(table_number)
    app = QApplication(sys.argv)
    window = MainWindow(node, table_number)
    
    def run_ros():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    thread = threading.Thread(target=run_ros, daemon=True)
    thread.start()

    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()