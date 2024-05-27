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
    
    def saveYamlFile(self,task_id):

        data = {
            'robot': {
                'last_task_id': task_id,
                'total_robot_num': self.getTotalRobotNum()
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



    