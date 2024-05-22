## URDF 적용하는 방법


1. base_link_trailer_join.stl 와 base_link_trailer_join.dae 파일을 다운로드 한다.
   
2. /home/dh/amr_ws/pinkbot/src/pinklab_minibot_robot/minibot_description/meshes/collision 경로에 들어가서 
  기존 base_link.stl를 다른 이름으로((base_link_1.stl) 바꾸고 base_link_trailer_join.stl의 이름을 base_link.stl로 바꿔준다.

3. /home/dh/amr_ws/pinkbot/src/pinklab_minibot_robot/minibot_description/meshes/visual 경로에 들어가서
  기존 base_link.dae를 다른 이름으로(base_link_1.dae) 바꾸고 base_link_trailer_join.dae의 이름을 base_link.dae로 바꿔준다.

4. /home/dh/amr_ws/pinkbot/ 워크스페이스로 돌아와서 colcon build를 한다.
