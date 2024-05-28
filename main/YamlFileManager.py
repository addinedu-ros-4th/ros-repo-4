import yaml
import unittest

class YamlFileManager:
    def __init__(self, path):
        self.path = path
        # YAML 파일 읽기
        with open(self.path, 'r') as file:
            self.config = yaml.safe_load(file)

    def getLastTaskID(self):
        return self.config['robot']['last_task_id']
    
    def getTotalRobotNum(self):
        return self.config['robot']['total_robot_num']
    
    def getClientMoudule(self):
        client_section =self.config['client']
        return {
            'module1': client_section.get('module1', 1),
            'module2': client_section.get('module2', 2),
            'module3': client_section.get('module3', 3),
            'module4': client_section.get('module4', 4)
        }

    def getMysqlData(self):
        return self.config['mysql']['ip'], self.config['mysql']['password']
    
    def getTagData(self):
        return self.config['security']['ip_tag'], self.config['security']['password_tag']
    
    def saveYamlFile(self,task_id):

        data = {
            'robot': {
                'last_task_id': task_id,
                'total_robot_num': self.getTotalRobotNum()
            },
            'client': {
                'module1': 1,
                'module2': 2,
                'module3': 3,
                'module4': 4,
            }
        }

        with open(self.path, 'w') as file:
            yaml.dump(data, file)

class TestYamlFileManager(unittest.TestCase):
    
    def test_get_return_value_taskid(self):
        yaml_manager = YamlFileManager('config.yaml')
        result = yaml_manager.getLastTaskID()
        self.assertEqual(result,0)

    def test_get_return_value_robot_num(self):
        yaml_manager = YamlFileManager('config.yaml')
        result = yaml_manager.getTotalRobotNum()
        self.assertEqual(result,1)

if __name__ == "__main__":
    unittest.main()



    