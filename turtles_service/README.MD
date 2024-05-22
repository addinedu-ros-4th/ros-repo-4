 
* way point 서비스 실행하기 전에 navigation pinkbot에서 실행 한 상태여야 함!    
        
* way point service run     
       
![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/f0c24fe7-6746-4eff-9813-ea622023d375)

* way point service call     

![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/c015bfee-a667-420c-b611-8a2fc6685f49)

       * 코드 붙여서 쓸때 혹시 끊겼는지 확인하고 쓰기! (밑에꺼 복사해서 가져다 쓰기!)
``` ros2 service call /waypoint_service turtles_service_msgs/srv/WayPointPoses "{x1: 0.0, y1: 0.0, z1: 0.0, x2: 0.0, y2: 0.0, z2: 0.0, x3: 0.0, y3: 0.0, z3: 0.0}" ```

