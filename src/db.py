import hashlib
import sqlite3
import os

class SWATLogger:
    def __init__(self, db_name="/home/yoonkangrok/kill_operation_ws/src/db/swat_logs.db"):
        # 데이터베이스 파일 경로의 디렉토리가 없으면 생성
        directory = os.path.dirname(db_name)
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"디렉토리 생성: {directory}")
        
        # 데이터베이스 파일 연결 (없으면 새로 생성됨)
        self.connection = sqlite3.connect(db_name)
        self.cursor = self.connection.cursor()
        self.create_swat_table()
        self.create_login_table()

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

    def create_login_table(self):
        # LOGIN 테이블 생성
        create_table_query = '''
        CREATE TABLE IF NOT EXISTS LOGIN (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username VARCHAR(50) NOT NULL,
            password VARCHAR(255) NOT NULL
        );
        '''
        
        self.cursor.execute(create_table_query)
        
        # 비밀번호 해시화하여 기본 사용자 추가
        insert_users_query = '''
        INSERT INTO LOGIN (username, password) VALUES
        ('swat', ?),
        ('1', ?)
        ON CONFLICT DO NOTHING;
        '''
        
        # 해시된 비밀번호 값
        hashed_swat_password = hashlib.sha256('rokey'.encode('utf-8')).hexdigest()
        hashed_user1_password = hashlib.sha256('1'.encode('utf-8')).hexdigest()
        
        self.cursor.execute(insert_users_query, (hashed_swat_password, hashed_user1_password))
        
        # 변경사항 저장
        self.connection.commit()
        print("LOGIN 테이블이 성공적으로 생성되었고 기본 사용자가 추가되었습니다.")

    def close_connection(self):
        # 연결 종료
        self.connection.close()

if __name__ == "__main__":
    logger = SWATLogger()
    logger.create_swat_table()
    logger.create_login_table()
    logger.close_connection()
