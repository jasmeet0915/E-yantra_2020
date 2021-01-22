## Navigation Stack:
  * Waypoints decide and Use more waypoints to prevent collision
  * Create a seperate parameters file (json, yaml etc.) with following format:
    * Pantry: [(12, 4), (12, 10)]
  * Final orientation for pick and place (refer task4 bot orientation)
  * Create 2 seperate waypoints for both tables in Pantry.
  
  Estimated Time: 26 Jan
  
## Pick and Place Stack:
  * Adjust/Change the Predefined Pose:
    * detect pose
    * drop pose 
    * rest/travel pose:
      * ajust bot orientation to use same pose
      * create room specific drop pose

  Estimated Time: 24 Jan
  
## Perception Stack:
  * Add objects for recognition taske photos from different angles and poses.
  * Close the find_object_2d node after required objects are detected (take snapshot if possible) for static centroid in bounding box.
  * Create 2 seperate waypoints for both tables in Pantry.
    * Future:
      * Use perception to detect which table has objects and which don't to save overall time.
      
Estimated Time: 26 Jan

Communication and Integration:
  * 2 types:
    * High level task specific communication messages(Topic):
      * Pick Up room: room_name
      * object pick: object_name
      * drop_room: room_name
  * 
