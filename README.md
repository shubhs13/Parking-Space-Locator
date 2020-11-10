
# Parking Space Detection

## Technologies Used

* Python
* OpenCv

# Pitch

  Finding a vacant parking space in urban areas is a tie consuming and thus not satisfying for potential visitor or costomer. Efficient car parking routing systems could support drivers to get an optimal parking space immediately . Therefore we are implementing a vision based "Parking Space Detection" system to detect space in Parking Lot.

## Proposed System
![alt text](https://github.com/shubhs13/Parking-Space-Locator/blob/master/pitch.png)
   
   * We'll pass each frame video through pipeline , one frame at a time.
   * The first step is to detect all the possible parking spaces in the frame of video .
   * The second step is to detect all the cars in each frame of video. This will let us track the moment of each car.
   * The third step is to determine which of the parking spaces are currently occupied by cars and which arent by combining the results of      first and second step.


## Conclusion

#### Even though our system is tested for outside parking lots, it can also be used in parking garages. In this case the Lightning conditions need to be adjusted and we are good to go. Improvements can be achieved by minimizing the influences of adjacent cars overlaying the labeled area due to camera perspective.




