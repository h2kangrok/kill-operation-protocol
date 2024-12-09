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
import datetime
from swat_interface.msg import RobotStatus, TrackingMsg
# Flask 선언
app = Flask(__name__, template_folder='/home/hayang/kill_operation_ws/kill-operation-protocol/src/kill_operation_protocol/kill_operation_protocol/flask/templates')
app.secret_key = '2131ds12'

# SWATLogger 클래스
class SWATLogger:
    def __init__(self, db_name="src/db/swat_logs.db"):
        self.db_name = db_name
        self.create_tables()

    def create_tables(self):
        with sqlite3.connect(self.db_name) as conn:
            cursor = conn.cursor()

            # SWAT 테이블 생성
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS SWAT (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                log_message VARCHAR(100) NOT NULL,
                timestamp DATETIME DEFAULT (strftime('%Y-%m-%d %H:%M:%S', 'now', 'localtime'))
            );
            ''')

            # Operation_log 테이블 생성
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS Operation_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                operation_message VARCHAR(100) NOT NULL,
                timestamp DATETIME DEFAULT (strftime('%Y-%m-%d %H:%M:%S', 'now', 'localtime'))
            );
            ''')

            # LOGIN_HISTORY 테이블 생성
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS LOGIN_HISTORY (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT NOT NULL,
                login_time TEXT NOT NULL
            );
            ''')

            conn.commit()

    def insert_log(self, log_message):
        with sqlite3.connect(self.db_name) as conn:
            cursor = conn.cursor()
            cursor.execute('INSERT INTO SWAT (log_message) VALUES (?);', (log_message,))
            conn.commit()

    def insert_operation_log(self, operation_message):
        with sqlite3.connect(self.db_name) as conn:
            cursor = conn.cursor()
            cursor.execute('INSERT INTO Operation_log (operation_message) VALUES (?);', (operation_message,))
            conn.commit()

    def get_logs(self, limit=50):
        with sqlite3.connect(self.db_name) as conn:
            cursor = conn.cursor()
            cursor.execute(f"SELECT log_message, timestamp FROM SWAT ORDER BY timestamp DESC LIMIT {limit};")
            return [{'log_message': row[0], 'timestamp': row[1]} for row in cursor.fetchall()]

    def get_operation_logs(self, limit=50):
        with sqlite3.connect(self.db_name) as conn:
            cursor = conn.cursor()
            cursor.execute(f"SELECT operation_message, timestamp FROM Operation_log ORDER BY timestamp DESC LIMIT {limit};")
            return [{'operation_message': row[0], 'timestamp': row[1]} for row in cursor.fetchall()]

# SWATLogger 인스턴스 생성
swat_logger = SWATLogger()

# ROS2 노드 클래스
class SimpleRosNode(Node):
    def __init__(self):
        super().__init__('Swat')
        self.get_logger().info('ROS2 running')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription_log = self.create_subscription(
            String, 'LOG', self.log_callback, qos_profile
        )

        self.subscription_secondary_log = self.create_subscription(
            String, 'SECONDARY_LOG', self.secondary_log_callback, qos_profile
        )

        self.subscription_start = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.start_callback,
            qos_profile
        )
        self.robostus = False

        self.subscription_complete = self.create_subscription(
            TrackingMsg,
            '/boom/AMR/shut_down_cam',
            self.complete_callback,
            qos_profile
        )
        self.mission_complete = False

        self.subscription_worldview = self.create_subscription(
            CompressedImage,
            '/world_view/annotated',
            self.worldview_compressed_callback,
            qos_profile
        )
        self.subscription_robotview = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.robotview_compressed_callback,
            qos_profile
        )

        self.worldview_frame = None
        self.robotview_frame = None

    # 기존 LOG 콜백
    def log_callback(self, msg):
        self.get_logger().info(f"Received log: {msg.data}")
        swat_logger.insert_log(msg.data)

    # 추가된 SECONDARY_LOG 콜백
    def secondary_log_callback(self, msg):
        self.get_logger().info(f"Received secondary log: {msg.data}")
        swat_logger.insert_operation_log(msg.data)

    def worldview_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.worldview_frame = frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed WorldView frame: {e}")

    
    def complete_callback(self, msg):
        if msg.tracking_msg == "Shutdown initiated":
            self.mission_complete == True
            try:
                with sqlite3.connect("src/db/swat_logs.db") as conn:
                    cursor = conn.cursor()
                    cursor.execute("SELECT operation_message, timestamp FROM Operation_log ORDER BY timestamp DESC;")
                    records = cursor.fetchall()
                return jsonify([{'operation_message': record[0], 'timestamp': record[1]} for record in records])
            except sqlite3.Error as e:
                return jsonify({'error': f'Failed to fetch operation logs: {e}'}), 500


        self.get_logger().info(f"Mission Complete: {self.mission_complete}")

    def robotview_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.robotview_frame = frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed RobotView frame: {e}")

    def start_callback(self, msg):
        self.robostus = msg.move_status
        if self.robostus == 1:
            try:
                # 데이터베이스에 "ARM 시작!!" 메시지 기록
                with sqlite3.connect("src/db/swat_logs.db") as conn:
                    cursor = conn.cursor()
                    cursor.execute("INSERT INTO Operation_log (operation_message) VALUES (?);", ("ARM 시작!!",))
                    conn.commit()
                    self.get_logger().info("ARM 시작!! 기록 완료")
            except sqlite3.Error as e:
                self.get_logger().error(f"Failed to insert ARM start message: {e}")
        self.get_logger().info(f"Start status received: {self.robostus}")

# Flask route
@app.route('/')
def index():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return render_template('index.html')
    
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form.get('username', '').strip()
        password = request.form.get('password', '').strip()
        hashed_password = hashlib.sha256(password.encode('utf-8')).hexdigest()

        try:
            with sqlite3.connect("src/db/swat_logs.db") as conn:
                cursor = conn.cursor()

                # 사용자 인증 쿼리
                query = "SELECT * FROM LOGIN WHERE username = ? AND password = ?"
                cursor.execute(query, (username, hashed_password))
                result = cursor.fetchone()

                if result:
                    # 로그인 성공 처리
                    session['logged_in'] = True
                    session['username'] = username
                    flash('로그인 성공', 'success')

                    # 로그인 기록 삽입
                    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    log_query = "INSERT INTO LOGIN_HISTORY (username, login_time) VALUES (?, ?)"
                    cursor.execute(log_query, (username, timestamp))
                    conn.commit()  # 변경사항 저장

                    return redirect(url_for('index'))
                else:
                    flash('로그인 실패: 사용자명 또는 비밀번호가 잘못되었습니다.', 'danger')
        except sqlite3.Error as e:
            flash(f'로그인 실패: 데이터베이스 오류가 발생했습니다. {e}', 'danger')

    return render_template('login.html')


@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    flash('로그아웃 되었습니다.', 'info')
    return redirect(url_for('login'))
@app.route('/Database', methods=['GET'])
def handleDBOption():
    # 세션 확인: 로그인이 되어 있지 않으면 로그인 페이지로 리다이렉트
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))

    # 로그인 기록 가져오기
    with sqlite3.connect("src/db/swat_logs.db") as conn:
        cursor = conn.cursor()
        cursor.execute("SELECT username, login_time FROM LOGIN_HISTORY ORDER BY login_time DESC;")
        records = cursor.fetchall()

    # JSON 형태로 데이터 반환
    return jsonify([{'username': record[0], 'login_time': record[1]} for record in records])

@app.route('/start_status')
def start_status():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    return jsonify({'start': node.robostus})

@app.route('/mission_status')
def mission_status():
    return jsonify({'complete': node.mission_complete})

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
    logs = swat_logger.get_logs()
    return jsonify({'logs': logs})

@app.route('/secondary_logs')
def secondary_logs():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    operation_logs = swat_logger.get_operation_logs()
    return jsonify({'logs': operation_logs})

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
                continue
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def run_flask():
    app.run(host='0.0.0.0', port=5000)

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

if __name__ == '__main__':
    main()
