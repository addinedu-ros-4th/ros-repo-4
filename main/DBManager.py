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

    
    def getAnimal(self):
        result_list = []
        cursor = self.local.cursor()

        sql = "SELECT * FROM Animal;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list

    def getAnimalRFID(self):
        result_list = []
        cursor = self.local.cursor()

        sql = "SELECT * FROM AnimalRFID;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list

    def getCameraPath(self):
        result_list = []
        cursor = self.local.cursor()

        sql = "SELECT * FROM CameraPath;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list

    def getFood(self):
        result_list = []
        cursor = self.local.cursor()

        sql = "SELECT * FROM Food;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list

    def getFoodRobotSchedule(self):
        result_list = []
        cursor = self.local.cursor()

        sql = "SELECT * FROM FoodRobotSchedule;"

        cursor.execute(sql)
        result = cursor.fetchall()

        for val in result:
            result_list.append(list(val))

        return result_list
    
    def getUserData(self):
        result_list = []
        cursor = self.local.cursor()

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
