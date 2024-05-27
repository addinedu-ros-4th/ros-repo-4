import logging

# 로그 파일 읽어서 리스트 형태로 저장
log_file_path = 'app.log'

with open(log_file_path, 'r') as file:
    log_lines = file.readlines()

# 각 로그 메시지 앞뒤 공백 제거
log_lines = [line.strip() for line in log_lines]

# 결과 출력 (리스트 형태)
print(log_lines)
