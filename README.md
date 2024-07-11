# 🐮 WHAT 소 🤠  

<br/>

![image](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/28bcebe8-6ff4-46c1-8e56-78fd5c2c45be)

<br/>

## 프로젝트 기간 
2024.05.14 ~ 2024.06.13
<br/>
<br/>


# 📖 Introduction
이 프로젝트에서는 축산업 종사자의 고령화 및 노동 인구 감소 해결을 위해 축산업 유지에 요구되는 인력을 최소화시키는 스마트 축산 시스템을 구현하고자 하였다. 프로젝트의 주요 목표는 **자울주행 사료 급이 로봇**, **축사 관리 UI**, **IoT와 딥러닝 기술을 이용한 축사 및 가축 관리**다. 
      
 <br/>        
 <br/>       

# 🔧 Stack 

<br/>
        
<div align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"> <img src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white">   
</div>

<div align="center">
   <img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white"> <img src="https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=MySQL&logoColor=white">  <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white"> <img src="https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=Qt&logoColor=white">
</div>

<div align="center">
   <img src="https://img.shields.io/badge/Github-181717?style=for-the-badge&logo=Github&logoColor=white"> <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white">
<img src="https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white"> <img src="https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=Slack&logoColor=white">

</div>

<div align="center">
   <img src="https://img.shields.io/badge/Raspberrypi-A22846?style=for-the-badge&logo=Raspberrypi&logoColor=white"> <img src="https://img.shields.io/badge/Arduino-00878F?style=for-the-badge&logo=Arduino&logoColor=white">
</div>
<br/>

# Project 설계

## 🛠️ System Architecture

<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/819e9008-bb90-49b0-a062-910d3da7f3a4"></p>

## 📡 Interface Definition
<br/> 

* ROS2    : ROS2의 navigation 패키지로 로봇의 주행 제어
      
<br/>      

* TCP 통신 : ESP 보드에 연결된 센서 값을 수신하고 모터에 신호를 주어 제어
         
<br/>      

      
<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/cd14d4b7-a6c1-4e87-95bb-1ae914f35f48"></p>



# Project 환경
## 🗺️ Map
<br/>

* 실제 map 환경
<br/>       
<br/>
              
<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/6a3980ab-6016-4a4c-a513-46639c2714cd"></p>
<br/>

* slam으로 map 구축
<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/66fcca7c-13f8-4d2b-b079-029bcd0bcd45"></p>
<br/>




# 🗒️ Auto Feeding Scenario
<br/> 
가축에게 사료를 급이하기 위한 시나리오
<br/> 
<br/>


 

<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/0564c909-d6c7-459e-9540-3cecd9f6840d"></p>

# Position Feedback Sequence 
<br/>
로봇이 특정 위치로 이동할때 위치를 보정해주는 순서를 나타냄
<br/>
<br/>
<p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/a329df51-d41f-4f4d-b4ec-19490da596c6"></p>


# Deep learning 요소
## pose estimation
* YOLOv8-pose를 사용하여 가축의 자세를 확인한다
  <br/>
  <p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/310da08e-fd0f-42cc-a11b-ff4518938945"></p>
  <br/>
## temperature check
* OpenCV mask 기능을 사용하여 가축의 체온을 측정을 구현하였다
 <br/>
 <p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/d0e89fd3-9650-4d52-8422-12f134818b6c"></p>
 <br/>
 
## food remain check
* OpenCV mask 기능을 사용하여 가축의 사료 섭취량 확인을 구현하였다.
 <br/>
 <p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/85333305-2f7f-4e83-b4de-d3d0994720c8"></p>
 <br/>

# 결론
## 시연영상

## 🖼️ GUI
<br/>
사용자에게 축사 관리에 필요한 데이터를 제공하기 위한 GUI 어플리케이션
<br/>
<br/> 


 * Barn monitoring UI 

  <p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/647adf02-de9d-4ad9-b3b3-54928b01220b"></p>
  <br/>


   
 * Facility monitoring UI 
  
  <p align="center"><img src="https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/ab71eacb-e290-469b-8003-3970d3736459"></p>
  <br/>
## 회고 

# 🧑‍💻 Role  

|곽준|석승연|임대환|정다연|
|:--:|:--:|:--:|:--:|
|주행 파트 총괄|UI 파트 총괄|HW 파트 총괄|딥러닝 파트 총괄|
|Nav2 navigation|SLAM|다중 로봇 위치 확인|시스템 설계|
|ArUCo Marker 위치 보정|로봇 위치 디스플레이|Map 구축|통신 프로토콜 설계|
||ROS2 패키지 관리|로봇 원격 제어 구현|UI 와이어 프레임 설계|
||DB 구축 및 연동|OpenCV 객체 인식|ROS2 패키지 관리|
||소켓 통신 서버 구축|급이 로봇 HW 구축|YOLO 객체 인식|
||UI 기능 구현|급이 로봇 URDF 구현|YOLO 자세 추정|
||||Github 관리|

# 발표 자료
https://docs.google.com/presentation/d/1nAdl37Y_I2kckYBX1_BZfDcR490SVoEkmQdbZBEPiyI/edit?usp=sharing
