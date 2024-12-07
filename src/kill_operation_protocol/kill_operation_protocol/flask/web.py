from flask import Flask, render_template, request, redirect, url_for, session, flash, Response, jsonify
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
import cv2
import numpy as np
import os
import sqlite3
import hashlib
# Flask 선언
app = Flask(__name__, template_folder='/home/rokey/kill-operation-protocol/src/kill_operation_protocol/kill_operation_protocol/flask/templates')
app.secret_key = '2131ds12'  # 세션을 위한 비밀 키 설정

# SWATLogger 클래스
class SWATLogger:
    def __init__(self, db_name="src/db/swat_logs.db"):
        # 데이터베이스 파일 경로의 디렉토리가 없으면 생성
        directory = os.path.dirname(db_name)
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"디렉토리 생성: {directory}")
        
        # 데이터베이스 파일 연결 (없으면 새로 생성됨)
        self.connection = sqlite3.connect(db_name)
        self.cursor = self.connection.cursor()
        self.create_swat_table()

    def create_swat_table(self):
        # SWAT 테이블 생성
        create_table_query = '''
        CREATE TABLE IF NOT EXISTS SWAT (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            log_message VARCHAR(100) NOT NULL,
            timestamp DATETIME DEFAULT (strftime('%Y-%m-%d %H:%M:%S', 'now', 'localtime'))
        );
        '''
        
        self.cursor.execute(create_table_query)
        
        # 변경사항 저장
        self.connection.commit()
        print("SWAT 테이블이 성공적으로 생성되었습니다.")

    def insert_log(self, log_message):
        insert_query = '''
        INSERT INTO SWAT (log_message) VALUES (?);
        '''
        self.cursor.execute(insert_query, (log_message,))
        self.connection.commit()
        print(f"로그 삽입: {log_message}")

    def show_tables(self):
        # 테이블 목록 조회
        self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = self.cursor.fetchall()
        print("데이터베이스에 존재하는 테이블들:")
        for table in tables:
            print(table[0])

    def close_connection(self):
        # 연결 종료
        self.connection.close()

# SWATLogger 인스턴스 생성
swat_logger = SWATLogger()

class SimpleRosNode(Node):
    def __init__(self):
        super().__init__('Swat')
        self.get_logger().info('ROS2 running')
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Best Effort로 설정하여 네트워크 부하 감소
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE  # 데이터를 계속 유지하지 않음
        )

        # Publisher 및 Subscriber 설정
        self.dispatch_publisher = self.create_publisher(String, 'dispatch', qos_profile)
        self.subscription_worldview = self.create_subscription(
            CompressedImage,  # 메시지 유형 변경
            'world_view/annotated',  # 주제 이름 변경
            self.worldview_compressed_callback,  # 콜백 함수
            qos_profile
        )    
        self.subscription_worldview = self.create_subscription(
            CompressedImage,  # 메시지 유형 변경
            'camera/image/compressed',  # 주제 이름 변경
            self.robotview_compressed_callback,  # 콜백 함수
            qos_profile
        )
        self.subscription_log = self.create_subscription(String, 'LOG', self.log_callback, qos_profile)
        
        # init
        self.worldview_frame = None
        self.robotview_frame = None
        self.logs = []

   

    def worldview_compressed_callback(self, msg):
        try:
            # CompressedImage의 데이터를 NumPy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축 해제
            if frame is not None:
                self.worldview_frame = frame  # 그대로 프레임 저장
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed WorldView frame: {e}")

    def robotview_compressed_callback(self, msg):
        try:
            # CompressedImage의 데이터를 NumPy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축 해제
            if frame is not None:
                self.robotview_frame = frame  # 그대로 프레임 저장
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed RobotView frame: {e}")

                
    def log_callback(self, msg):
        self.get_logger().info(f"Received log: {msg.data}") 
        self.logs.append(msg.data)
        swat_logger.insert_log(msg.data)  # SWATLogger를 사용하여 로그 삽입
        if len(self.logs) > 50:  # If too many logs then delete
            self.logs.pop(0)

    def publish_dispatch(self):
        msg = String()
        msg.data = '1'
        self.dispatch_publisher.publish(msg)
        self.get_logger().info("Dispatch message published: 1")

# Flask route
@app.route('/')
def index():
    print("DEBUG - Session in index:", session)  # 세션 값 출력
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return render_template('index.html')
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form.get('username', '').strip()
        password = request.form.get('password', '').strip()
        hashed_password = hashlib.sha256(password.encode('utf-8')).hexdigest()

        print(f"DEBUG - Received Username: {username}")
        print(f"DEBUG - Received Hashed Password: {hashed_password}")

        try:
            # SQLite 데이터베이스 연결 및 사용자 검증
            with sqlite3.connect("src/db/swat_logs.db") as conn:
                cursor = conn.cursor()
                query = "SELECT * FROM LOGIN WHERE username = ? AND password = ?"
                cursor.execute(query, (username, hashed_password))
                result = cursor.fetchone()

            print(f"DEBUG - Query Result: {result}")

            if result:
                # 로그인 성공 시 세션 설정
                session['logged_in'] = True
                session['username'] = username
                flash('로그인 성공', 'success')
                print("DEBUG - Session after login:", session)
                return redirect(url_for('index'))
            else:
                # 로그인 실패 처리
                flash('로그인 실패: 사용자명 또는 비밀번호가 잘못되었습니다.', 'danger')
                print("DEBUG - Login failed: Incorrect credentials.")
        except sqlite3.Error as e:
            # 데이터베이스 오류 처리
            flash(f'로그인 실패: 데이터베이스 오류가 발생했습니다. {e}', 'danger')
            print(f"DEBUG - Database Error: {e}")

    # GET 요청 또는 로그인 실패 시 로그인 페이지 렌더링
    return render_template('login.html')


@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    flash('로그아웃 되었습니다.', 'info')
    return redirect(url_for('login'))

@app.route('/video_feed_1')
def video_feed_1():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return Response(generate_frames('worldview'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_2')
def video_feed_2():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return Response(generate_frames('robotview'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/logs')
def logs():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return jsonify({'logs': node.logs})

@app.route('/dispatch', methods=['POST'])
def dispatch():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    if request.method == 'POST':
        node.publish_dispatch()
        return "Dispatched", 200

def generate_frames(cam_type):
    while True:
        if cam_type == 'worldview':
            frame = node.worldview_frame
        elif cam_type == 'robotview':
            frame = node.robotview_frame
        else:
            frame = None

        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                node.get_logger().error('Failed to encode frame')
                continue
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def run_flask():
    app.run(host='0.0.0.0', port=5000)

# ROS2 관련 클래스 및 메인 함수 유지
def main(args=None):
    rclpy.init(args=args)
    
    global node
    node = SimpleRosNode()

    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ROS2 Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        swat_logger.close_connection()

if __name__ == '__main__':
    main()
