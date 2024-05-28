import mysql.connector

class DBManager:

    def __init__(self, host,password, port, user, databases):
        #각자 컴퓨터에서 할때는 각자 user로 바꿔주기
        self.local = mysql.connector.connect(
            host = host,
            port = port,
            user = user,
            password = password,
            database = databases 
        )

    def getdata(self):
        result_list = []
        cursor = self.local.cursor()

        # 쿼리 실행
        sql = "SELECT * FROM UserData;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list
    
    def dbclose(self):
        self.local.close()

def main():
    pass

if __name__ == "__main__":
    main()
