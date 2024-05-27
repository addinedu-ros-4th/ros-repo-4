import yaml

# YAML 파일 읽기
with open('config.yaml', 'r') as file:
    config = yaml.safe_load(file)

# 읽어온 데이터 출력
print(config)
print(config['database']['host'])
print(config['server']['port'])

# 파이썬 객체를 YAML 파일로 저장하기
data = {
    'database': {
        'host': 'localhost',
        'port': 5432,
        'username': 'turtles',
        'password': 'pass'
    },
    'server': {
        'host': '0.0.0.0',
        'port': 8080
    }
}

with open('output.yaml', 'w') as file:
    yaml.dump(data, file)