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
    def getSchedulenums(self):
        cursor = self.local.cursor()
        sql = "SELECT COUNT(DISTINCT time) AS total_unique_times FROM food_scheduled_time;"
        cursor.execute(sql)
        schedule_result = cursor.fetchone()  # fetchone으로 하나의 행을 가져옵니다.
        total_unique_times = schedule_result[0]  # 튜플의 첫 번째 요소로 접근하여 total_unique_times 값을 가져옵니다.
        return total_unique_times

        
    def getFoodIntake(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM FoodIntake;"
        cursor.execute(sql)
        result = cursor.fetchall()
        columns = [column[0] for column in cursor.description]
        intake_df = pd.DataFrame(result, columns=columns)
        return intake_df      

    def getAnimalPose(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM animalPosture;"
        cursor.execute(sql)
        result = cursor.fetchall()
        columns = [column[0] for column in cursor.description]
        pose_df = pd.DataFrame(result, columns=columns)
        return pose_df
    
    def getSensorData(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM SensorData;"
        cursor.execute(sql)
        result = cursor.fetchall()
        columns = [column[0] for column in cursor.description]
        sensor_df = pd.DataFrame(result, columns=columns)
        return sensor_df                
      
        
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

    def register_food(self, barcode_id, brand_name, weight, expiry_date, registered_date):
        # 데이터베이스에 새로운 음식 추가
        cursor = self.local.cursor()
        query = "INSERT INTO Food (barcode_id, brand_name, weight, expiry_date, registered_date) VALUES (%s, %s, %s, %s, %s)"
        values = (barcode_id, brand_name, weight, expiry_date, registered_date)
        cursor.execute(query, values)
        self.local.commit()
        
    def register_animal(self, animal_id, gender, age, food_brand, room , weight, registered_date, rfid_uid):
        # 데이터베이스에 새로운 동물 추가
        cursor = self.local.cursor()
        query = "INSERT INTO Animal (animal_id, gender, age, food_brand, room , weight, registered_date, rfid_uid) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
        values = (animal_id, gender, age, food_brand, room , weight, registered_date, rfid_uid)
        cursor.execute(query, values)
        self.local.commit()        
    
    
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

    def getFacilitySetting(self):
        cursor = self.local.cursor()
        sql = "SELECT * FROM FacilitySetting;"
        cursor.execute(sql)
        columns = [column[0] for column in cursor.description]
        result = cursor.fetchall()
        facility_setting_df= pd.DataFrame(result, columns=columns)
        return facility_setting_df
        
        
    def updateFacilitySetting(self, settings):
        cursor = self.local.cursor()
        for facility_name, level in settings.items():
            sql = '''
                UPDATE FacilitySetting
                SET level = %s
                WHERE facility_name = %s
            '''
            cursor.execute(sql, (level, facility_name))
        
        self.local.commit()
    
    def clearFoodScheduledTimes(self,room_number ):
        cursor = self.local.cursor()
        cursor.execute(f"DELETE FROM food_scheduled_time where room = {room_number}")
        self.local.commit()
        
    def clearScheduledTimes(self, table_name):
        cursor = self.local.cursor()
        cursor.execute(f"DELETE FROM {table_name}")
        self.local.commit()
    
    def addFacilityScheduledTimes(self, time):
        # 데이터베이스에 새로운 유해동물 추가
        cursor = self.local.cursor()
        query = "INSERT INTO HarmfulAnimal (time) VALUES (%s)"
        values = (time)
        cursor.execute(query, values)
        self.local.commit()


    def addScheduledTimes(self, table_name, times):
        cursor = self.local.cursor()
        sql = f"INSERT INTO {table_name} (time) VALUES (%s)"
        cursor.executemany(sql, [(time,) for time in times])
        self.local.commit()
        
    def getScheduledTimes(self, table_name):
        cursor = self.local.cursor()
        cursor.execute(f"SELECT time FROM {table_name}")
        result = cursor.fetchall()
        # Convert TIME to "HH:MM" format with leading zeros if necessary
        return [self.formatTime(row[0]) for row in result]

    def formatTime(self, time):
        # Convert time to string and split it by ':'
        time_str = str(time)
        parts = time_str.split(':')
        # Check if the hour part has only one digit
        if len(parts[0]) == 1:
            # Prepend '0' to the hour part
            parts[0] = '0' + parts[0]
        # Join the parts with ':'
        return ':'.join(parts[:2])  # Return only the hour and minute parts

    def get_reserved_times(self, room):
        cursor = self.local.cursor()
        query = "SELECT time FROM food_scheduled_time WHERE room = %s"
        result = cursor.execute(query, (room,))
        return cursor.fetchall()

    def get_reserved_times_facility(self):
        cursor = self.local.cursor()
        query = "SELECT time FROM facility_scheduled_time "
        cursor.execute(query)
        result = cursor.fetchall()
        return result
    
    def insert_food_schedule(self, room, time):
        cursor = self.local.cursor()
        query = "INSERT INTO food_scheduled_time (room, time) VALUES (%s, %s)"
        cursor.execute(query, (room, time))
        self.local.commit()

    def insert_facility_schedule(self, time):
        cursor = self.local.cursor()
        query = "INSERT INTO facility_scheduled_time (time) VALUES (%s)"
        cursor.execute(query, (time))
        self.local.commit()    
        
    def dbclose(self):
        self.local.close()

def main():
    pass
    

if __name__ == "__main__":
    main()
