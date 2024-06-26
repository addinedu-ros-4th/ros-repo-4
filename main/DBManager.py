import mysql.connector
import pandas as pd

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
        cursor = self.local.cursor()
        sql = "SELECT * FROM Animal;"
        cursor.execute(sql)
        result = cursor.fetchall()
        columns = [column[0] for column in cursor.description]
        animal_df = pd.DataFrame(result, columns=columns)

        return animal_df

    def getCameraPath(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM CameraPath;"
        cursor.execute(sql)
        result = cursor.fetchall()
        columns = [column[0] for column in cursor.description]
        camera_df = pd.DataFrame(result, columns=columns)
        camera_df['captured_date'] = pd.to_datetime(camera_df['captured_date']).dt.date

        return camera_df

    def getEmployeeData(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM EmployeeData;"
        cursor.execute(sql)
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        employee_df= pd.DataFrame(result, columns=columns)

        return employee_df

    def register_employee(self, employee_name, registered_date):
        # 데이터베이스에 새로운 직원 추가
        cursor = self.local.cursor()
        query = "INSERT INTO EmployeeData (name, registered_date) VALUES (%s, %s)"
        values = (employee_name, registered_date)
        cursor.execute(query, values)
        self.local.commit()
    

    def getFood(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM Food;"
        cursor.execute(sql)
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        food_df= pd.DataFrame(result, columns=columns)
        

        return food_df

    def getFoodRobotSchedule(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM FoodRobotSchedule;"
        cursor.execute(sql)
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        food_robot_schedule_df= pd.DataFrame(result, columns=columns)

        return food_robot_schedule_df

    def getHarmfulAnimal(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM HarmfulAnimal;"
        cursor.execute(sql)
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        harmful_animal_df= pd.DataFrame(result, columns=columns)

        return harmful_animal_df

    def register_harmful_animal(self, index_num, animal_name):
        # 데이터베이스에 새로운 유해동물 추가
        cursor = self.local.cursor()
        query = "INSERT INTO HarmfulAnimal (index_num, animal_name) VALUES (%s, %s)"
        values = (index_num, animal_name)
        cursor.execute(query, values)
        self.local.commit()
    
    def delete_harmful_animal(self, index_num, animal_name):
        # HarmfulAnimal 테이블에서 해당 동물 삭제
        cursor = self.local.cursor()
        query = "DELETE FROM HarmfulAnimal WHERE index_num = %s AND animal_name = %s"
        values = (index_num, animal_name)
        cursor.execute(query, values)
        self.local.commit()

    
    def getUserData(self):
        sql = "SELECT * FROM UserData;"
        cursor = self.local.cursor()
        cursor.execute(sql)
        
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        users_df=pd.DataFrame(result, columns=columns)
        #for val in result:
            #result_list.append(list(val))
        return users_df
    
    def dbclose(self):
        self.local.close()

def main():

    pass

if __name__ == "__main__":
    main()
