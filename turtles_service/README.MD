# Navigation to pose service       

* nav_service 서비스 실행하기 전에 navigation pinkbot에서 실행한 상태여야 함! 

* nav_service run 
         
![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/23b811f0-7ecc-4514-b8f7-ff271bd8c6c8)
       
* nav_service call
       
![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/b3a1972d-bfb2-4396-bd6e-55e86f5a7f28)
       
``` ros2 service call /nav_service turtles_service_msgs/srv/NavToPose "{x: 0.6378898620605469, y: -0.9446680545806885, z: -0.005340576171875}" ```



          



# Way point following service      
* way point 서비스 실행하기 전에 navigation pinkbot에서 실행 한 상태여야 함!    
        
* way point service run     
       
![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/f0c24fe7-6746-4eff-9813-ea622023d375)

* way point service call     

![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/c015bfee-a667-420c-b611-8a2fc6685f49)

       * 코드 붙여서 쓸때 혹시 끊겼는지 확인하고 쓰기! (밑에꺼 복사해서 가져다 쓰기!)
``` ros2 service call /waypoint_service turtles_service_msgs/srv/WayPointPoses "{x1: 0.0, y1: 0.0, z1: 0.0, x2: 0.0, y2: 0.0, z2: 0.0, x3: 0.0, y3: 0.0, z3: 0.0}" ```
